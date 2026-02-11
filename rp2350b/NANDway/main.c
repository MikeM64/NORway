/************************************************************************
NANDway.c (v0.64) - Teensy++ 2.0 NAND Flasher for PS3/Xbox/Wii

Copyright (C) 2013  Effleurage
                    judges <judges@eEcho.com>
Copyright (C) 2026  MikeM64 - RPi Pico Ports

This code is licensed to you under the terms of the GNU GPL, version 2;
see file COPYING or http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*************************************************************************/

#include <stdio.h>
#include "pico/stdlib.h"

#include "bootloader.h"
#include "delay.h"
#include "mem_utils.h"
#include "usb_serial.h"

#define VERSION_MAJOR           0
#define VERSION_MINOR           65

#define BUILD_DUAL_NAND         1
#define BUILD_SIGNAL_BOOSTER    2

#if ((BUILD_VERSION != BUILD_DUAL_NAND) && (BUILD_VERSION != BUILD_SIGNAL_BOOSTER))
    #error BUILD_VERSION must be defined!
#endif

// Define commands
enum {
    CMD_PING1 = 0,
    CMD_PING2,
    CMD_BOOTLOADER,
    CMD_IO_LOCK,
    CMD_IO_RELEASE,
    CMD_PULLUPS_DISABLE,
    CMD_PULLUPS_ENABLE,
    CMD_NAND0_ID,
    CMD_NAND0_READPAGE,
    CMD_NAND0_WRITEPAGE,
    CMD_NAND0_ERASEBLOCK,
    CMD_NAND1_ID,
    CMD_NAND1_READPAGE,
    CMD_NAND1_WRITEPAGE,
    CMD_NAND1_ERASEBLOCK,
} cmd_t;


enum fsm_states_e {
    S_IDLE = 0,
};

/*! \brief NAND flash read page command start. */
#define NAND_COMMAND_READ1              0x00
/*! \brief NAND flash read page command end. */
#define NAND_COMMAND_READ2              0x30
/*! \brief NAND flash read ID command. */
#define NAND_COMMAND_READID             0x90
/*! \brief NAND flash reset command. */
#define NAND_COMMAND_RESET              0xFF
/*! \brief NAND flash program page command start. */
#define NAND_COMMAND_PAGEPROG1          0x80
/*! \brief NAND flash program page command end. */
#define NAND_COMMAND_PAGEPROG2          0x10
/*! \brief NAND flash erase block command start. */
#define NAND_COMMAND_ERASE1             0x60
/*! \brief NAND flash erase block command end. */
#define NAND_COMMAND_ERASE2             0xD0
/*! \brief NAND flash read status command. */
#define NAND_COMMAND_STATUS             0x70
/*! \brief NAND flash random program page command start. */
#define NAND_COMMAND_RANDOM_PAGEPROG    0x85
/*! \brief NAND flash random read page command start. */
#define NAND_COMMAND_RANDOM_READ1       0x05
/*! \brief NAND flash random read page command end. */
#define NAND_COMMAND_RANDOM_READ2       0xE0

#define NAND_STATUS_FAIL            (1<<0) /* HIGH - FAIL,  LOW - PASS */
#define NAND_STATUS_IDLE            (1<<5) /* HIGH - IDLE,  LOW - ACTIVE */
#define NAND_STATUS_READY           (1<<6) /* HIGH - READY, LOW - BUSY */
#define NAND_STATUS_NOT_PROTECTED   (1<<7) /* HIGH - NOT,   LOW - PROTECTED */

#define BUF_SIZE_RW     4320
#define BUF_SIZE_ADDR   3

uint16_t    PAGE_PLUS_RAS_SZ = 0; /* page size + Redundant Area Size */
uint8_t     IO_PULLUPS = 0xFF;
uint8_t     buf_rw[BUF_SIZE_RW];
uint8_t     buf_addr[BUF_SIZE_ADDR];

/*! \brief NAND flash information about maker, device, size and timing.
*/
typedef struct _nand_info {
    uint8_t raw_data[5];

    /*! \brief Out of bounce layout information. */
    uint16_t    oob_size;

    /*! \brief Maker code. */
    uint8_t     maker_code;
    /*! \brief Device code. */
    uint8_t     device_code;

    /*! \brief Page size in bytes. */
    uint32_t    page_size;
    /*! \brief Number of positions to shift when converting an offset in
     *         a block to page. Used when calculating NAND flash address.
     */
    //uint32_t  page_shift;
    /*! \brief Number of pages per block. */
    uint32_t    pages_per_block;
    /*! \brief Block size in bytes. */
    uint32_t    block_size;
    /*! \brief Number of positions to shift when converting block number
     *         to NAND flash address.
     */
    //uint32_t  block_shift;
    /*! \brief Number of blocks. */
    uint32_t    num_blocks;
    /*! \brief NAND flash I/O bus width in bits. */
    uint8_t     bus_width;

    /*! \brief NAND flash number of planes. */
    uint8_t     num_planes;
    /*! \brief NAND flash plane size. */
    uint32_t    plane_size;
} nand_info;


#if BUILD_VERSION == BUILD_DUAL_NAND
    #error Dual NAND is not yet supported!
#elif BUILD_VERSION == BUILD_SIGNAL_BOOSTER
    #define NAND0_IO_PIN_SHIFT (28)
    #define NAND0_IO_PIN_MASK (0xFFull << 28)

    #define ALL_PIN_MASK (NAND0_IO_PIN_MASK)
#endif /* BUILD_VERSION */


/*
 * Enable pullups on the NAND IO pins
 */
void nand_io_pullups_enable(void)
{
    uint32_t i;

    for (i = NAND0_IO_PIN_SHIFT; i < 8; i++) {
        gpio_set_pulls(i, true /* up */, false /* down */);
    }
}


/*
 * Disable pullups on the NAND IO pins
 */
void nand_io_pullups_disable(void)
{
    uint32_t i;

    for (i = NAND0_IO_PIN_SHIFT; i < 8; i++) {
        gpio_set_pulls(i, false /* up */, false /* down */);
    }
}


/*
 * Reset pins to a known good default state
 */
void init_pins(void)
{
    /* Assign all pins to SW I/O */
    gpio_set_function_masked64(ALL_PIN_MASK, GPIO_FUNC_SIO);

    /* NAND IO Pins are set to output by default */
    gpio_set_dir_out_masked64(NAND0_IO_PIN_MASK);
}


/*
 * Release all pins to enable regular booting
 */
void release_pins(void)
{
    gpio_set_dir_in_masked64(ALL_PIN_MASK);

    nand_io_pullups_disable();
}


enum fsm_states_e run_idle_state(void)
{
    int                 rc;
    enum fsm_states_e   next_state = S_IDLE;

    rc = usb_serial_getchar();

    if (rc != PICO_ERROR_TIMEOUT) {
        switch (rc) {
        case CMD_PING1:
            usb_serial_putchar(VERSION_MAJOR);
            break;
        case CMD_PING2:
            uint16_t freeram = get_free_heap();
            usb_serial_putchar(VERSION_MINOR);
            usb_serial_putchar((freeram >> 8) & 0xFF);
            usb_serial_putchar(freeram & 0xFF);
            break;
        case CMD_BOOTLOADER:
            enter_bootloader();
            break;
        case CMD_IO_LOCK:
            init_pins();
            break;
        case CMD_IO_RELEASE:
            release_pins();
            break;
        case CMD_PULLUPS_DISABLE:
            nand_io_pullups_disable();
            break;
        case CMD_PULLUPS_ENABLE:
            nand_io_pullups_enable();
            break;
        }
    }

    return (next_state);
}


enum fsm_states_e run_nandway_state_machine(enum fsm_states_e current_state)
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
    enum fsm_states_e   current_fsm_state = S_IDLE;

    stdio_init_all();
    systick_timer_init();

    release_pins();
    wait_for_usb_serial_connection();
    init_pins();

    while(1) {
        usb_serial_flush();

        while (usb_serial_connected()) {
            current_fsm_state = run_nandway_state_machine(current_fsm_state);
        }
    }

    return 0;
}
