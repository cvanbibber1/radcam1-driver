// SPDX-License-Identifier: BSD-3-Clause
/*
 * Hardware-timed UART RX on an arbitrary GPIO, using the RP1's PIO block.
 *
 * Completes the EXTUART debug port: pio_uart.c gives TX on GPIO24, this gives
 * RX on GPIO23. Neither pin has a UART alt-function on a Pi 5, and userspace
 * bit-banging was measured to top out near 9600 baud (see softuart.c), so PIO
 * is the only way to reach 921600 on the finalised board.
 *
 * The PIO program is the standard 8N1 uart_rx with framing-error handling:
 *
 *     .program uart_rx
 *     start:
 *         wait 0 pin 0          ; stall until the start bit arrives
 *         set x, 7   [10]       ; preload bit counter, delay to mid-bit
 *     bitloop:
 *         in pins, 1            ; sample one data bit, LSB first
 *         jmp x-- bitloop [6]   ; 8 cycles per bit
 *         jmp pin good_stop     ; stop bit must be high
 *         irq 4 rel             ; framing error or break: raise a sticky flag
 *         wait 1 pin 0          ; wait for the line to return to idle
 *         jmp start             ; and drop the byte - do not push bad framing
 *     good_stop:
 *         push                  ; deliver the byte
 *
 * Eight PIO cycles per bit, so the state machine runs at 8 x baud, same as TX.
 *
 * SELF-TEST WITHOUT A JUMPER WIRE
 * ------------------------------
 * Verifying RX normally needs TX physically wired to RX. It does not here: PIO
 * can have one state machine driving a pin while another samples that same
 * pin. `--loopback` runs the TX program and the RX program on GPIO23 at once,
 * so the silicon wires them together internally and the round trip is a real
 * test of the RX program, timing included.
 *
 * Build: see softuart/Makefile
 * Test:  sudo ./pio_uart_rx --loopback --baud 921600
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern int atoi(const char *);
#include <time.h>
#include <unistd.h>

#include "piolib.h"

#define RX_GPIO          23
#define DEFAULT_BAUD     921600
#define CYCLES_PER_BIT   8

/*
 * Assembled uart_rx. Each word derived from the PIO instruction encoding:
 *   [15:13] opcode  [12:8] delay  [7:5] varies  [4:0] varies
 *
 * 0: wait 0 pin 0    WAIT=001, delay 0, pol=0, src=PIN(01), idx=0
 *                    -> 001 00000 0 01 00000 = 0x2020
 * 1: set x, 7 [10]   SET=111, delay 10=01010, dest X=001, data 7=00111
 *                    -> 111 01010 001 00111 = 0xEA27
 * 2: in pins, 1      IN=010, delay 0, src PINS=000, count 1=00001
 *                    -> 010 00000 000 00001 = 0x4001
 * 3: jmp x-- 2 [6]   JMP=000, delay 6=00110, cond X--=010, addr 2=00010
 *                    -> 000 00110 010 00010 = 0x0642
 * 4: jmp pin 8       JMP=000, delay 0, cond PIN=110, addr 8=01000
 *                    -> 000 00000 110 01000 = 0x00C8
 * 5: irq 4 rel       IRQ=110, delay 0, Clr=0, Wait=0, idx=4|rel(0x10)=10100
 *                    -> 110 00000 0 0 0 10100 = 0xC014
 * 6: wait 1 pin 0    WAIT=001, delay 0, pol=1, src=PIN(01), idx=0
 *                    -> 001 00000 1 01 00000 = 0x20A0
 * 7: jmp 0           JMP=000, delay 0, cond always=000, addr 0=00000
 *                    -> 0x0000
 * 8: push            PUSH=100, delay 0, [7]=0, IfFull=0, Blk=1
 *                    -> 100 00000 0 0 1 00000 = 0x8020
 */
static const uint16_t uart_rx_instructions[] = {
	0x2020, 0xEA27, 0x4001, 0x0642, 0x00C8, 0xC014, 0x20A0, 0x0000, 0x8020,
};

static const struct pio_program uart_rx_program = {
	.instructions = uart_rx_instructions,
	.length = 9,
	.origin = -1,
};

/* TX program, identical to pio_uart.c - used only by the loopback self-test. */
static const uint16_t uart_tx_instructions[] = {
	0x9FA0, 0xF727, 0x6001, 0x0642,
};

static const struct pio_program uart_tx_program = {
	.instructions = uart_tx_instructions,
	.length = 4,
	.origin = -1,
};

static float clkdiv_for(uint32_t sys_hz, uint baud)
{
	return (float)sys_hz / (float)(CYCLES_PER_BIT * baud);
}

/*
 * `own_dir` must be false when another state machine is driving this same pin
 * (the internal-loopback self-test). The pad direction is a property of the
 * pin, not of the state machine: if RX forces it back to input, TX can no
 * longer drive it and nothing is ever received. RX samples the pad either way.
 */
static int rx_init(PIO pio, int sm, uint offset, uint pin, uint baud,
		   bool own_dir)
{
	pio_sm_config c = pio_get_default_sm_config();

	if (own_dir) {
		pio_gpio_init(pio, pin);
		pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
	}

	sm_config_set_in_pins(&c, pin);       /* for IN  */
	sm_config_set_jmp_pin(&c, pin);       /* for JMP PIN, the stop-bit check */
	/* Shift right: a UART is LSB-first. Autopush a byte at a time. */
	sm_config_set_in_shift(&c, true, false, 32);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
	sm_config_set_wrap(&c, offset, offset + uart_rx_program.length - 1);
	sm_config_set_clkdiv(&c, clkdiv_for(clock_get_hz(clk_sys), baud));

	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);
	return 0;
}

static int tx_init(PIO pio, int sm, uint offset, uint pin, uint baud)
{
	pio_sm_config c = pio_get_default_sm_config();

	pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);    /* output */

	sm_config_set_sideset(&c, 2, true, false);
	sm_config_set_sideset_pins(&c, pin);
	sm_config_set_out_pins(&c, pin, 1);
	sm_config_set_out_shift(&c, true, false, 32);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
	sm_config_set_wrap(&c, offset, offset + uart_tx_program.length - 1);
	sm_config_set_clkdiv(&c, clkdiv_for(clock_get_hz(clk_sys), baud));

	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);
	return 0;
}

/*
 * The RX program shifts right into a 32-bit ISR and pushes when full, so the
 * received byte lands in the most significant byte of the word.
 */
static inline uint8_t rx_byte_from_fifo(uint32_t word)
{
	return (uint8_t)(word >> 24);
}

static int loopback(uint pin, uint baud)
{
	static const char msg[] = "radcam1 EXTUART loopback 0123456789";
	PIO pio;
	int sm_rx, sm_tx, errors = 0;
	uint off_rx, off_tx;
	size_t sent = 0, got = 0;
	uint8_t received[128];

	pio = pio_open(0);
	if (PIO_IS_ERR(pio)) {
		fprintf(stderr, "cannot open pio0: %d\n", PIO_ERR_VAL(pio));
		return 1;
	}

	sm_rx = pio_claim_unused_sm(pio, true);
	sm_tx = pio_claim_unused_sm(pio, true);
	if (sm_rx < 0 || sm_tx < 0) {
		fprintf(stderr, "need two free PIO state machines\n");
		return 1;
	}

	off_rx = pio_add_program(pio, &uart_rx_program);
	off_tx = pio_add_program(pio, &uart_tx_program);

	printf("PIO internal loopback on GPIO%u at %u baud\n", pin, baud);
	printf("  clk_sys %" PRIu32 " Hz, clkdiv %.4f\n",
	       clock_get_hz(clk_sys), clkdiv_for(clock_get_hz(clk_sys), baud));
	printf("  RX sm %d at offset %u, TX sm %d at offset %u\n",
	       sm_rx, off_rx, sm_tx, off_tx);

	/*
	 * Order matters. TX must come up FIRST so the line is driven to its idle
	 * high state before RX starts looking at it. Bringing RX up first means
	 * it samples an undriven pin, reads the first low as a start bit, and
	 * desynchronises the whole stream - which showed up as intermittent
	 * content mismatches that varied with baud rate.
	 */
	tx_init(pio, sm_tx, off_tx, pin, baud);
	usleep(5000);                     /* let the line settle at idle high */

	rx_init(pio, sm_rx, off_rx, pin, baud, false);
	usleep(2000);

	/* Discard anything latched while the line was coming up. */
	while (!pio_sm_is_rx_fifo_empty(pio, sm_rx))
		(void)pio_sm_get(pio, sm_rx);

	struct timespec t_start;
	clock_gettime(CLOCK_MONOTONIC, &t_start);

	while (sent < strlen(msg) || got < strlen(msg)) {
		if (sent < strlen(msg) && !pio_sm_is_tx_fifo_full(pio, sm_tx))
			pio_sm_put(pio, sm_tx, (uint32_t)(uint8_t)msg[sent++]);

		if (!pio_sm_is_rx_fifo_empty(pio, sm_rx)) {
			uint32_t word = pio_sm_get(pio, sm_rx);

			if (got < sizeof(received))
				received[got] = rx_byte_from_fifo(word);
			got++;
		}

		/* Bail out rather than spin forever if RX never delivers. */
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		if (now.tv_sec - t_start.tv_sec > 5) {
			printf("  TIMED OUT: sent %zu, received %zu\n", sent, got);
			errors++;
			break;
		}
	}

	pio_sm_set_enabled(pio, sm_tx, false);
	pio_sm_set_enabled(pio, sm_rx, false);

	printf("\n  sent     : \"%s\" (%zu bytes)\n", msg, strlen(msg));
	printf("  received : \"");
	for (size_t i = 0; i < got && i < sizeof(received); i++)
		putchar(received[i] >= 32 && received[i] < 127 ? received[i] : '?');
	printf("\" (%zu bytes)\n", got);

	if (got != strlen(msg)) {
		printf("  LENGTH MISMATCH\n");
		errors++;
	} else if (memcmp(received, msg, strlen(msg)) != 0) {
		printf("  CONTENT MISMATCH\n");
		errors++;
	}

	printf("\n  %s\n", errors ? "RX FAILED"
			           : "RX VERIFIED - byte-exact round trip");
	pio_close(pio);
	return errors ? 1 : 0;
}

static void usage(const char *argv0)
{
	fprintf(stderr,
		"usage: %s [--baud N] [--pin N] [--loopback] [--rx SECONDS]\n"
		"\n"
		"  --baud N       baud rate (default %d)\n"
		"  --pin N        RX GPIO (default %d)\n"
		"  --loopback     drive and sample the same pin with two PIO state\n"
		"                 machines - verifies RX with no jumper wire\n"
		"  --rx SECONDS   receive and print for this long\n",
		argv0, DEFAULT_BAUD, RX_GPIO);
}

int main(int argc, char **argv)
{
	uint baud = DEFAULT_BAUD, pin = RX_GPIO;
	bool do_loopback = false;
	int rx_seconds = 0;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--baud") && i + 1 < argc)
			baud = (uint)atoi(argv[++i]);
		else if (!strcmp(argv[i], "--pin") && i + 1 < argc)
			pin = (uint)atoi(argv[++i]);
		else if (!strcmp(argv[i], "--loopback"))
			do_loopback = true;
		else if (!strcmp(argv[i], "--rx") && i + 1 < argc)
			rx_seconds = atoi(argv[++i]);
		else {
			usage(argv[0]);
			return 2;
		}
	}

	if (do_loopback)
		return loopback(pin, baud);

	if (rx_seconds > 0) {
		PIO pio = pio_open(0);
		int sm;
		uint off;

		if (PIO_IS_ERR(pio)) {
			fprintf(stderr, "cannot open pio0\n");
			return 1;
		}
		sm = pio_claim_unused_sm(pio, true);
		off = pio_add_program(pio, &uart_rx_program);
		rx_init(pio, sm, off, pin, baud, true);

		fprintf(stderr, "receiving on GPIO%u at %u baud for %ds\n",
			pin, baud, rx_seconds);
		for (int t = 0; t < rx_seconds * 1000; t++) {
			while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
				uint8_t b = rx_byte_from_fifo(pio_sm_get(pio, sm));

				putchar(b);
				fflush(stdout);
			}
			usleep(1000);
		}
		pio_sm_set_enabled(pio, sm, false);
		pio_close(pio);
		return 0;
	}

	usage(argv[0]);
	return 2;
}
