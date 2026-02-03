/************************************************************************
norflash.v - NOR flasher for PS3

Copyright (C) 2010-2011, 2026

Hector Martin "marcan" <hector@marcansoft.com>
NORway.c (v0.8) - Teensy++ 2.0 port by judges@eEcho.com
RP2350B port by MikeM64

This code is licensed to you under the terms of the GNU GPL, version 2;
see file COPYING or http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*************************************************************************/


#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/structs/systick.h"
#include "pico/bootrom.h"

#include "tusb.h"

/* GPIO 0 through 21 are reserved for address lines */
#define ADDRESS_PIN_MASK    (0x3FFFFFull)

/* GPIO 22 though 38 are reserved for data lines */
#define DATA_PIN_SHIFT      (22)
#define DATA_PIN_MASK       (0x3FFFC00000ull)

/* GPIO 39 through 44 are for control lines */
#define RYBY_PIN            (39)
#define TRISTATE_PIN        (40)
#define CE_PIN              (41)
#define WE_PIN              (42)
#define OE_PIN              (43)
#define RESET_PIN           (44)

#define RYBY_PIN_MASK       (1ull << RYBY_PIN)
#define TRISTATE_PIN_MASK   (1ull << TRISTATE_PIN)
#define CE_PIN_MASK         (1ull << CE_PIN)
#define WE_PIN_MASK         (1ull << WE_PIN)
#define OE_PIN_MASK         (1ull << OE_PIN)
#define RESET_PIN_MASK      (1ull << RESET_PIN)

#define CONTROL_PIN_MASK    (RYBY_PIN_MASK | \
                             TRISTATE_PIN_MASK | \
                             CE_PIN_MASK | \
                             WE_PIN_MASK | \
                             OE_PIN_MASK | \
                             RESET_PIN_MASK)

#define ALL_PIN_MASK        (ADDRESS_PIN_MASK | \
                             DATA_PIN_MASK | \
                             CONTROL_PIN_MASK)

#define NUM_PINS            (45)


enum fsm_states_e {
    S_IDLE = 0,
    S_READING_BSS_4,
    S_READING_BSS_8,
    S_READING_BSS_64,
    S_READING_BSS_128,
    S_READING_BSS_WORD,
    S_ADDR2,
    S_ADDR3,
    S_WRITE,
    S_WRITE_INCREMENT,
};

enum external_commands_e {
    CMD_NOP = 0,
    CMD_READSTATE,
    CMD_PING1,
    CMD_PING2,
    CMD_BOOTLOADER,
    CMD_ADDR_INCREMENT,
    CMD_RELEASE_PORTS,
    CMD_INIT_PORTS,
    CMD_RESET_DISABLE,
    CMD_RESET_ENABLE,
    CMD_VERIFY_DISABLE, // Not implemented yet
    CMD_VERIFY_ENABLE, // Not implemented yet
    CMD_SPEEDTEST_READ,
    CMD_SPEEDTEST_WRITE,
    CMD_WAIT, // Not implemented yet
    CMD_WAIT_INCREMENT, // Not implemented yet
    CMD_READ_BSS_4 = 0x10,
    CMD_READ_BSS_8 = 0x11,
    CMD_READ_BSS_64 = 0x12,
    CMD_READ_BSS_128 = 0x13,
    CMD_READ_BSS_WORD = 0x14,
    CMD_WRITE = 0x18,
    CMD_WRITE_INCREMENT = 0x19,
};

// Define block/sector size for reading/writing
#define BSS_4       0x01000 //2Kwords = 4KB
#define BSS_8       0x02000 //4Kwords = 8KB
#define BSS_64      0x10000 //32Kwords = 64KB
#define BSS_128     0x20000 //64Kwords = 128KB
#define BSS_WORD    0x00002 //word = 2Bytes


/* 8ns per tick -> 13 ticks for 104ns delay */
#define DELAY_100_NS()  (systick_delay(13))


void systick_timer_init(void)
{
    // the clock is set to 125 MHz => 1 tick == 8 ns
    // init SysTick timer
    systick_hw->csr = 0x05; // enable systick at 1 cycle resolution (8ns)
    systick_hw->rvr = 0xffff; // 16 bit
}


void systick_delay(uint16_t ticks)
{
    uint16_t start = systick_hw->cvr;
    while((uint16_t)(start-systick_hw->cvr) < ticks); 
}


/*
 * Reset pins to a known good default state
 */
void init_pins(void)
{
    /* Assign all pins to SW I/O */
    gpio_set_function_masked64(ALL_PIN_MASK, GPIO_FUNC_SIO);

    /* Address pins are always output and default to 0 */
    gpio_set_dir_out_masked64(ADDRESS_PIN_MASK);
    gpio_clr_mask64(ADDRESS_PIN_MASK);

    /* Data Pins start as output and are not touched */
    gpio_set_dir_out_masked64(DATA_PIN_MASK);

    /* Control pins except RYBY are output */
    gpio_set_dir_out_masked64(CONTROL_PIN_MASK & (~RYBY_PIN_MASK));
    gpio_set_dir_in_masked64(RYBY_PIN_MASK);
}


/*
 * Release all pins to enable regular booting
 */
void release_pins(void)
{
    uint32_t i;

    gpio_set_dir_in_masked64(ALL_PIN_MASK);

    /* Disable pull up/down resistors on the pins */
    for (i = 0; i < NUM_PINS; i++) {
        gpio_set_pulls(i, false, false);
    }
}


uint16_t get_data_pins(void)
{
    uint64_t all_gpio_pins = gpio_get_all64();

    return (uint16_t)(all_gpio_pins >> DATA_PIN_SHIFT);
}


void set_data_pins_input(void)
{
    gpio_set_dir_in_masked64(DATA_PIN_MASK);
}


void set_data_pins_output(void)
{
    gpio_set_dir_out_masked64(DATA_PIN_MASK);
}


/*
 * To simplify porting the teensy code, the address is updated
 * byte-by-byte per the existing protocol.
 */
static uint32_t s_address = 0x0;
void update_address3(uint8_t addr3)
{
    s_address = (addr3 << 16) | (s_address & 0xffff);
}


void update_address2(uint8_t addr2)
{
    s_address = (s_address & 0xff00ff) | (addr2 << 8);
}


void update_address1(uint8_t addr1)
{
    s_address = (s_address & 0xffff00) | (addr1);
}


/*
 * Put the current address on the bus
 */
void update_address_pins(void)
{
    gpio_put_masked(
        ADDRESS_PIN_MASK,
        (s_address & ADDRESS_PIN_MASK)
    );
}


void address_increment_and_update_pins(void)
{
    s_address++;
    update_address_pins();
}


/* Helpers for single pins */
void CE_LOW(void)
{
    gpio_clr_mask64(CE_PIN_MASK);
}


void CE_HIGH(void)
{
    gpio_set_mask64(CE_PIN_MASK);
}


void OE_LOW(void)
{
    gpio_clr_mask64(OE_PIN_MASK);
}


void OE_HIGH(void)
{
    gpio_set_mask64(OE_PIN_MASK);
}


void RESET_LOW(void)
{
    gpio_clr_mask64(RESET_PIN_MASK);
}


void RESET_HIGH(void)
{
    gpio_set_mask64(RESET_PIN_MASK);
}


void WE_LOW(void)
{
    gpio_clr_mask64(WE_PIN_MASK);
}


void WE_HIGH(void)
{
    gpio_set_mask64(WE_PIN_MASK);
}


void __attribute__((noreturn)) enter_bootloader(void)
{
    reset_usb_boot(0, 0);
}


uint8_t state_byte(void)
{
    uint8_t state_byte = 0;

    if (gpio_get(TRISTATE_PIN)) {
        state_byte |= 0x20;
    }

    if (gpio_get(RESET_PIN)) {
        state_byte |= 0x10;
    }

    if (gpio_get(RYBY_PIN)) {
        state_byte |= 0x08;
    }

    if (gpio_get(CE_PIN)) {
        state_byte |= 0x04;
    }

    if (gpio_get(WE_PIN)) {
        state_byte |= 0x02;
    }

    if (gpio_get(OE_PIN)) {
        state_byte |= 0x01;
    }

    return (0);
}


void wait_for_usb_serial_connection(void)
{
    while(!tud_cdc_connected()) {
    }
}


void flush_uart_io(void)
{
    int rc;
    absolute_time_t end_time;

    /* Flush all output */
    stdio_flush();

    /* Flush any waiting input data */
    char flush_buf[8];

    /* Wait up to 10us since the last data to consider the
     * input buffer flushed */
    do {
        end_time = delayed_by_us(get_absolute_time(), 10);
        rc = stdio_get_until(flush_buf, sizeof(flush_buf), end_time);
    } while (rc != PICO_ERROR_TIMEOUT);
}


void usb_serial_putchar(char c)
{
    (void)stdio_putchar_raw(c);
}


/*
 * Waits for a character to be returned by the serial port.
 * Will wait up to 100ms for a character to be received -
 * needed because the python script on the host is much slower
 * to send data than the pico can receive it.
 */
int usb_serial_getchar(void)
{
    int rc;
    absolute_time_t end_time;
    char input_buf;

    end_time = delayed_by_us(get_absolute_time(), 100000);
    rc = stdio_get_until(&input_buf, sizeof(input_buf), end_time);

    if (rc != PICO_ERROR_TIMEOUT) {
        rc = input_buf;
    }

    return (rc);
}


int usb_serial_getbuf(char *input_buf, size_t input_buf_len)
{
    int rc;
    absolute_time_t end_time;

    end_time = delayed_by_us(get_absolute_time(), 100000);
    rc = stdio_get_until(input_buf, input_buf_len, end_time);

    return (rc);
}


size_t usb_serial_write(char *buf, size_t len)
{
    return stdio_put_string(
        buf, len,
        false /* newline */,
        false /* cr_translation */);
}


void speedtest_send()
{
    char buf_read[64];
    uint8_t buf_ix;
    uint32_t addr;

    addr = buf_ix = 0;
    while (1) {
        buf_read[buf_ix] = buf_ix;
        buf_ix++;
        buf_read[buf_ix] = buf_ix;
        buf_ix++;
        if (buf_ix == 64) {
            usb_serial_write(buf_read, buf_ix);
            buf_ix = 0;
        }
        if (addr++ == (BSS_128/2-1))
            break;
    }
}


void speedtest_receive()
{
    int rc;
    char buf_write[BSS_4];
    int i = 0;

    while (i < BSS_4) {
        rc = usb_serial_getbuf(&buf_write[i], 128);
        if (rc == PICO_ERROR_TIMEOUT) {
            usb_serial_putchar('T');
            return;
        }

        i += rc;
    }

    if (i != BSS_4) {
        usb_serial_putchar('R');
    } else {
        usb_serial_putchar('K');
    }
}


enum fsm_states_e run_idle_state(void)
{
    int rc;
    enum fsm_states_e next_state = S_IDLE;

    rc = usb_serial_getchar();

    if (rc != PICO_ERROR_TIMEOUT) {
        switch (rc) {
        case CMD_NOP:
            break;
        case CMD_READSTATE:
            usb_serial_putchar(state_byte());
            break;
        case CMD_PING1:
            usb_serial_putchar(0x42);
            break;
        case CMD_PING2:
            usb_serial_putchar(0xbd);
            break;
        case CMD_BOOTLOADER:
            enter_bootloader();
            break;
        case CMD_ADDR_INCREMENT:
            address_increment_and_update_pins();
            break;
        case CMD_RELEASE_PORTS:
            release_pins();
            break;
        case CMD_INIT_PORTS:
            init_pins();
            break;
        case CMD_RESET_DISABLE:
            RESET_HIGH();
            break;
        case CMD_RESET_ENABLE:
            RESET_LOW();
            break;
        case CMD_SPEEDTEST_READ:
            speedtest_send();
            break;
        case CMD_SPEEDTEST_WRITE:
            speedtest_receive();
            break;
        case CMD_READ_BSS_4:
            next_state = S_READING_BSS_4;
            break;
        case CMD_READ_BSS_8:
            next_state = S_READING_BSS_8;
            break;
        case CMD_READ_BSS_64:
            next_state = S_READING_BSS_64;
            break;
        case CMD_READ_BSS_128:
            next_state = S_READING_BSS_128;
            break;
        case CMD_READ_BSS_WORD:
            next_state = S_READING_BSS_WORD;
            break;
        case CMD_WRITE:
            OE_HIGH();
            CE_LOW();
            WE_LOW();
            next_state = S_WRITE;
            break;
        case CMD_WRITE_INCREMENT:
            OE_HIGH();
            CE_LOW();
            WE_LOW();
            next_state = S_WRITE_INCREMENT;
            break;
        default:
            /*
             * This is for commands that pack an argument into
             * the command byte itself.
             */
            if ((rc >> 7) == 1) {
                /*
                 * Address - Receive address byte 3
                 */
                update_address3((rc << 1) >> 1);
                next_state = S_ADDR2;
            } else if ((rc >> 6) == 1) {
                /*
                 * Delay - The teensy implementation would loop
                 * through the main FSM loop for as many iterations
                 * as sent. For the pico, 1 "cycle" == 1us.
                 */
                sleep_us(rc & 0x3f);
            }
            break;
        }
    }

    return (next_state);
}


enum fsm_states_e run_reading_state(enum fsm_states_e current_state)
{
    uint16_t data_pins;

    set_data_pins_input();

    if (current_state == S_READING_BSS_WORD) {
        OE_LOW();
        DELAY_100_NS();
        data_pins = get_data_pins();
        OE_HIGH();

        usb_serial_putchar((char)(data_pins & 0xff00 >> 8));
        usb_serial_putchar((char)(data_pins & 0xff));
    } else {
        uint32_t bss_size;
        char     buf_read[64];
        uint8_t  buf_ix = 0;
        uint32_t address = 0;

        if (current_state == S_READING_BSS_4) {
            bss_size = BSS_4;
        } else if (current_state == S_READING_BSS_8) {
            bss_size = BSS_8;
        } else if (current_state == S_READING_BSS_64) {
            bss_size = BSS_64;
        } else if (current_state == S_READING_BSS_128) {
            bss_size = BSS_128;
        } else {
            bss_size = BSS_WORD;
        }

        do {
            OE_LOW();
            DELAY_100_NS();

            data_pins = get_data_pins();
            buf_read[buf_ix++] = data_pins & 0xff00 >> 8;
            buf_read[buf_ix++] = data_pins & 0xff;
            OE_HIGH();

            address_increment_and_update_pins();
            if (buf_ix == 64) {
                usb_serial_write(buf_read, sizeof(buf_read));
                buf_ix = 0;
            }
        } while (address++ != (bss_size/2-1));
    }

    set_data_pins_output();

    return (S_IDLE);
}


enum fsm_states_e run_addr2_state(enum fsm_states_e current_state)
{
    int rc;
    enum fsm_states_e next_state = S_ADDR3;

    rc = usb_serial_getchar();

    if (rc != PICO_ERROR_TIMEOUT) {
        update_address2(rc);
    } else {
        next_state = S_ADDR2;
    }

    return (next_state);
}


enum fsm_states_e run_addr3_state(enum fsm_states_e current_state)
{
    int rc;
    enum fsm_states_e next_state = S_IDLE;

    rc = usb_serial_getchar();

    if (rc != PICO_ERROR_TIMEOUT) {
        update_address1(rc);
    } else {
        next_state = S_ADDR3;
    }

    return (next_state);
}


enum fsm_states_e run_write_state(enum fsm_states_e current_state)
{
    enum fsm_states_e next_state = S_IDLE;
    int rc;
    uint16_t data_word = 0;

    do {
    rc = usb_serial_getchar();
        if (rc != PICO_ERROR_TIMEOUT) {
            data_word |= ((uint8_t)rc) << 8;
        }
    } while (rc == PICO_ERROR_TIMEOUT);

    do {
    rc = usb_serial_getchar();
        if (rc != PICO_ERROR_TIMEOUT) {
            data_word |= ((uint8_t)rc);
        }
    } while (rc == PICO_ERROR_TIMEOUT);

    DELAY_100_NS();

    if (current_state == S_WRITE_INCREMENT) {
        address_increment_and_update_pins();
    }

    return (next_state);
}


enum fsm_states_e run_norway_state_machine(enum fsm_states_e current_state)
{
    enum fsm_states_e next_state = S_IDLE;

    switch (current_state) {
    case S_IDLE:
        next_state = run_idle_state();
        break;
    case S_READING_BSS_4:
    case S_READING_BSS_8:
    case S_READING_BSS_64:
    case S_READING_BSS_128:
    case S_READING_BSS_WORD:
        next_state = run_reading_state(current_state);
        break;
    case S_ADDR2:
        next_state = run_addr2_state(current_state);
        break;
    case S_ADDR3:
        next_state = run_addr3_state(current_state);
        break;
    case S_WRITE:
    case S_WRITE_INCREMENT:
        next_state = run_write_state(current_state);
        break;
    }

    return (next_state);
}


int main(void)
{
    enum fsm_states_e current_fsm_state = S_IDLE;

    stdio_init_all();
    systick_timer_init();

    /* Don't lock the flash bus unless we're connected to a computer */
    release_pins();
    wait_for_usb_serial_connection();
    init_pins();

    while (1) {
        flush_uart_io();

        while (tud_cdc_connected()) {
            current_fsm_state = run_norway_state_machine(current_fsm_state);
        }
    }

    return 0;
}
