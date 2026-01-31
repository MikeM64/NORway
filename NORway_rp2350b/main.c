/************************************************************************
  norflash.v - NOR flasher for PS3

Copyright (C) 2010-2011  Hector Martin "marcan" <hector@marcansoft.com>

This code is licensed to you under the terms of the GNU GPL, version 2;
see file COPYING or http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*************************************************************************
 NORway.c (v0.8) - Teensy++ 2.0 port by judges@eEcho.com
*************************************************************************
* RP2350B port by MikeM64
*************************************************************************/


#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/bootrom.h"

#include "tusb.h"

/* GPIO 0 through 21 are reserved for address lines */
#define ADDRESS_PIN_MASK    (0x3FFFFFull)

/* GPIO 22 though 38 are reserved for data lines */
#define DATA_PIN_MASK       (0x3FFFC00000ull)

/* GPIO 39 through 44 are for control lines */
#define RYBY_PIN_MASK       (1ull << 39)
#define TRISTATE_PIN_MASK   (1ull << 40)
#define CE_PIN_MASK         (1ull << 41)
#define WE_PIN_MASK         (1ull << 42)
#define OE_PIN_MASK         (1ull << 43)
#define RESET_PIN_MASK      (1ull << 44)

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
};

enum external_commands_e {
    CMD_NOP = 0,
    CMD_READSTATE,
    CMD_PING1,
    CMD_PING2,
    CMD_BOOTLOADER,
    CMD_SPEEDTEST_READ = 12,
    CMD_SPEEDTEST_WRITE = 13,
};

// Define block/sector size for reading/writing
#define BSS_4       0x01000 //2Kwords = 4KB
#define BSS_8       0x02000 //4Kwords = 8KB
#define BSS_64      0x10000 //32Kwords = 64KB
#define BSS_128     0x20000 //64Kwords = 128KB
#define BSS_WORD    0x00002 //word = 2Bytes

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


void __attribute__((noreturn)) enter_bootloader(void)
{
    reset_usb_boot(0, 0);
}


uint8_t state_byte(void)
{
    /* TODO: Use the real implementation */
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
        case CMD_SPEEDTEST_READ:
            speedtest_send();
            break;
        case CMD_SPEEDTEST_WRITE:
            speedtest_receive();
            break;
        }
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
    }

    return (next_state);
}


int main(void)
{
    enum fsm_states_e current_fsm_state = S_IDLE;

    stdio_init_all();

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
