#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/structs/ssi.h"
#include "hardware/structs/ioqspi.h"

const uint LED_PIN = 26;

static ssi_hw_t *const ssi = (ssi_hw_t *) XIP_SSI_BASE;

// ----------------------------------------------------------------------------------
// Low-level SSI in SPI mode flash routines lifted from bootrom 
// program_flash_generic.c:

// These are supported by almost any SPI flash
#define FLASHCMD_PAGE_PROGRAM     0x02
#define FLASHCMD_READ_DATA        0x03
#define FLASHCMD_READ_STATUS      0x05
#define FLASHCMD_WRITE_ENABLE     0x06
#define FLASHCMD_SECTOR_ERASE     0x20
#define FLASHCMD_READ_SFDP        0x5a
#define FLASHCMD_READ_JEDEC_ID    0x9f

#define FLASHCMD_CHIP_ERASE       0xc7
#define FLASHCMD_VOLATILE_SR_WRITE_ENABLE	0x50  
#define FLASHCMD_WRITE_SR1        0x01
#define FLASHCMD_WRITE_SR2        0x31
#define FLASHCMD_WRITE_SR3        0x11  

typedef enum {
    OUTOVER_NORMAL = 0,
    OUTOVER_INVERT,
    OUTOVER_LOW,
    OUTOVER_HIGH
} outover_t;

// Also allow any unbounded loops to check whether the above abort condition
// was asserted, and terminate early
int flash_was_aborted() {
    return *(io_rw_32 *) (IO_QSPI_BASE + IO_QSPI_GPIO_QSPI_SD1_CTRL_OFFSET)
           & IO_QSPI_GPIO_QSPI_SD1_CTRL_INOVER_BITS;
}

// Flash code may be heavily interrupted (e.g. if we are running USB MSC
// handlers concurrently with flash programming) so we control the CS pin
// manually
static void __noinline flash_cs_force(outover_t over) {
    io_rw_32 *reg = (io_rw_32 *) (IO_QSPI_BASE + IO_QSPI_GPIO_QSPI_SS_CTRL_OFFSET);
#ifndef GENERAL_SIZE_HACKS
    *reg = *reg & ~IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS
        | (over << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB);
#else
    // The only functions we want are FSEL (== 0 for XIP) and OUTOVER!
    *reg = over << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB;
#endif
    // Read to flush async bridge
    (void) *reg;
}

// Put bytes from one buffer, and get bytes into another buffer.
// These can be the same buffer.
// If tx is NULL then send zeroes.
// If rx is NULL then all read data will be dropped.
//
// If rx_skip is nonzero, this many bytes will first be consumed from the FIFO,
// before reading a further count bytes into *rx.
// E.g. if you have written a command+address just before calling this function.
void __noinline flash_put_get(const uint8_t *tx, uint8_t *rx, size_t count, size_t rx_skip) {
    // Make sure there is never more data in flight than the depth of the RX
    // FIFO. Otherwise, when we are interrupted for long periods, hardware
    // will overflow the RX FIFO.
    const uint max_in_flight = 16 - 2; // account for data internal to SSI
    size_t tx_count = count;
    size_t rx_count = count;
    while (tx_count || rx_skip || rx_count) {
        // NB order of reads, for pessimism rather than optimism
        uint32_t tx_level = ssi_hw->txflr;
        uint32_t rx_level = ssi_hw->rxflr;
        bool did_something = false; // Expect this to be folded into control flow, not register
        if (tx_count && tx_level + rx_level < max_in_flight) {
            ssi->dr0 = (uint32_t) (tx ? *tx++ : 0);
            --tx_count;
            did_something = true;
        }
        if (rx_level) {
            uint8_t rxbyte = ssi->dr0;
            did_something = true;
            if (rx_skip) {
                --rx_skip;
            } else {
                if (rx)
                    *rx++ = rxbyte;
                --rx_count;
            }
        }
        // APB load costs 4 cycles, so only do it on idle loops (our budget is 48 cyc/byte)
        if (!did_something && __builtin_expect(flash_was_aborted(), 0))
            break;
    }
    flash_cs_force(OUTOVER_HIGH);
}

// Convenience wrapper for above
// (And it's hard for the debug host to get the tight timing between
// cmd DR0 write and the remaining data)
void flash_do_cmd(uint8_t cmd, const uint8_t *tx, uint8_t *rx, size_t count) {
    flash_cs_force(OUTOVER_LOW);
    ssi->dr0 = cmd;
    flash_put_get(tx, rx, count, 1);
}


// Timing of this one is critical, so do not expose the symbol to debugger etc
static inline void flash_put_cmd_addr(uint8_t cmd, uint32_t addr) {
    flash_cs_force(OUTOVER_LOW);
    addr |= cmd << 24;
    for (int i = 0; i < 4; ++i) {
        ssi->dr0 = addr >> 24;
        addr <<= 8;
    }
}

// Poll the flash status register until the busy bit (LSB) clears
static inline void flash_wait_ready() {
    uint8_t stat;
    do {
        flash_do_cmd(FLASHCMD_READ_STATUS, NULL, &stat, 1);
    } while (stat & 0x1 && !flash_was_aborted());
}

// Set the WEL bit (needed before any program/erase operation)
static __noinline void flash_enable_write() {
    flash_do_cmd(FLASHCMD_WRITE_ENABLE, NULL, NULL, 0);
}

// ----------------------------------------------------------------------------------

static void init_flash_ssi() {
	// Linkage for BOOTROM functions
    void (*connect_internal_flash)(void) = (void(*)(void))rom_func_lookup(rom_table_code('I', 'F'));
    void (*flash_exit_xip)(void) = (void(*)(void))rom_func_lookup(rom_table_code('E', 'X'));
	
	// Configure flash memory pads and mux to SSI
	connect_internal_flash();
	// Jolt flash out of xip mode and enter SSI mstr mode, calls flash_init_spi()
	// to put SPI single wire with 8 bit frames
	flash_exit_xip();
}

static void erase_flash() {
	// clear non-volatile protection to their default settings
	flash_do_cmd(FLASHCMD_VOLATILE_SR_WRITE_ENABLE, NULL, NULL, 0);
	uint8_t tx_buf = 0x00;
	flash_do_cmd(FLASHCMD_WRITE_SR2, &tx_buf, NULL, 1);
	flash_wait_ready();

	// erase the entire flash
	flash_enable_write();
	flash_do_cmd(FLASHCMD_CHIP_ERASE, NULL, NULL, 0);
	flash_wait_ready();

	// clear non-volatile SR2 bits
	flash_enable_write();
	tx_buf = 0;
	flash_do_cmd(FLASHCMD_WRITE_SR2, &tx_buf, NULL, 1);
	flash_wait_ready();

	// set 75% read driver strength in SR3
	flash_enable_write();
	tx_buf = 0x20;		// 75% read driver strength
	flash_do_cmd(FLASHCMD_WRITE_SR3, &tx_buf, NULL, 1);
	flash_wait_ready();
}

static uint32_t read_flash_sr() {
	uint32_t sr;
	//uint8_t txbuf[2] = {0};
	uint8_t rxbuf[2] = {0xff, 0xee};
	//txbuf[0] = 0x05;
	flash_do_cmd(0x05, NULL, rxbuf, 1);
	sr = rxbuf[0];
	flash_do_cmd(0x35, NULL, rxbuf, 1);
	sr <<= 8;
	sr |= rxbuf[0];
	flash_do_cmd(0x15, NULL, rxbuf, 1);
	sr <<= 8;
	sr |= rxbuf[0];

	return sr;
}

int main() {
	bi_decl(bi_program_description("Erase and update flash read driver strength."));
	bi_decl(bi_1pin_with_name(LED_PIN, "My little red LED"));

	stdio_init_all();

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	init_flash_ssi();

	erase_flash();
	//uint32_t sr = read_flash_sr();
#if 1
	while (1) {
		gpio_put(LED_PIN, 0);
		sleep_ms(250);
		gpio_put(LED_PIN, 1);
		//printf("sr: %x\n", sr);
		sleep_ms(1000);
	}
#endif
}
