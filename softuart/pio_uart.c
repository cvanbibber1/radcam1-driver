// SPDX-License-Identifier: BSD-3-Clause
/*
 * Hardware-timed UART TX on an arbitrary GPIO, using the RP1's PIO block.
 *
 * Why this exists: radcam1's EXTUART debug port is fixed at GPIO24 (TX) /
 * GPIO23 (RX) by a finalised board, and neither pin has a UART alt-function on
 * a Pi 5. Bit-banging from userspace was measured (see softuart.c) and tops out
 * near 9600 baud, because worst-case scheduler preemption is 12-20 us however
 * fast the loop itself is.
 *
 * PIO solves that properly. The state machine generates every edge from the
 * PIO clock in hardware, so Linux scheduling latency is irrelevant - the CPU
 * only has to keep a 4-deep FIFO fed, which it has ~87 us to do per byte at
 * 921600 baud. The full 921600 is therefore comfortable.
 *
 * The PIO program is the classic 8N1 uart_tx:
 *
 *     .program uart_tx
 *     .side_set 1 opt
 *         pull       side 1 [7]   ; stop bit / idle high, stall when empty
 *         set x, 7   side 0 [7]   ; start bit, preload bit counter
 *     bitloop:
 *         out pins, 1             ; shift out one data bit, LSB first
 *         jmp x-- bitloop   [6]   ; 8 cycles per bit
 *
 * Eight PIO cycles per bit, so the state machine runs at 8 x baud.
 *
 * Build: make -C softuart pio_uart
 * Test:  sudo ./pio_uart --baud 921600 --tx "hello"
 *        sudo ./pio_uart --selftest
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern int atoi(const char *);
#include <unistd.h>

#include "piolib.h"

#define TX_GPIO          24
#define DEFAULT_BAUD     921600
#define CYCLES_PER_BIT   8

/*
 * Assembled uart_tx program. Each word is derived from the RP2040/RP1 PIO
 * instruction encoding rather than copied on faith:
 *
 *   [15:13] opcode  [12:8] delay/side-set  [7:5] varies  [4:0] varies
 *
 * With ".side_set 1 opt" the top two bits of the delay field are the side-set
 * enable flag and its value, leaving 3 bits of delay.
 *
 * 0: pull block side 1 [7]
 *      opcode PULL      = 100
 *      side-set en=1, val=1, delay=7   -> 1 1 111 = 11111
 *      [7]=1 (PULL), IfEmpty=0, Blk=1  -> 101
 *      -> 100 11111 101 00000 = 0x9FA0
 *
 * 1: set x, 7 side 0 [7]
 *      opcode SET       = 111
 *      side-set en=1, val=0, delay=7   -> 1 0 111 = 10111
 *      dest X = 001, data = 7 = 00111
 *      -> 111 10111 001 00111 = 0xF727
 *
 * 2: out pins, 1
 *      opcode OUT       = 011
 *      no side-set, delay 0            -> 00000
 *      dest PINS = 000, count 1 = 00001
 *      -> 011 00000 000 00001 = 0x6001
 *
 * 3: jmp x-- 2 [6]
 *      opcode JMP       = 000
 *      no side-set, delay 6            -> 00110
 *      cond X-- = 010, addr 2 = 00010
 *      -> 000 00110 010 00010 = 0x0642
 */
static const uint16_t uart_tx_instructions[] = {
	0x9FA0,
	0xF727,
	0x6001,
	0x0642,
};

static const struct pio_program uart_tx_program = {
	.instructions = uart_tx_instructions,
	.length = 4,
	.origin = -1,
};

struct pio_uart {
	PIO pio;
	int sm;
	uint offset;
	uint pin;
	uint baud;
	uint32_t sm_clock_hz;
	float div_actual;
};

static int pio_uart_init(struct pio_uart *u, uint pin, uint baud)
{
	pio_sm_config c;
	uint32_t sys_hz;
	float div;

	memset(u, 0, sizeof(*u));
	u->pin = pin;
	u->baud = baud;

	u->pio = pio_open(0);
	if (PIO_IS_ERR(u->pio)) {
		fprintf(stderr, "cannot open pio0: %d "
			"(is the rp1-pio driver loaded and /dev/pio0 accessible?)\n",
			PIO_ERR_VAL(u->pio));
		return -1;
	}

	u->sm = pio_claim_unused_sm(u->pio, true);
	if (u->sm < 0) {
		fprintf(stderr, "no free PIO state machine\n");
		return -1;
	}

	u->offset = pio_add_program(u->pio, &uart_tx_program);

	pio_gpio_init(u->pio, pin);
	pio_sm_set_consecutive_pindirs(u->pio, u->sm, pin, 1, true);

	c = pio_get_default_sm_config();
	sm_config_set_sideset(&c, 2, true, false);	/* 1 bit, optional */
	sm_config_set_sideset_pins(&c, pin);
	sm_config_set_out_pins(&c, pin, 1);
	sm_config_set_wrap(&c, u->offset, u->offset + uart_tx_program.length - 1);
	/* Shift right, LSB first, no autopull - the program pulls explicitly. */
	sm_config_set_out_shift(&c, true, false, 32);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

	sys_hz = clock_get_hz(clk_sys);
	div = (float)sys_hz / (float)(CYCLES_PER_BIT * baud);
	sm_config_set_clkdiv(&c, div);

	/*
	 * The hardware divider is an integer plus an 8-bit fraction, so the
	 * divisor actually used is not the float we asked for. Quantise it the
	 * same way the hardware does, otherwise the reported baud error is
	 * optimistic and hides real drift.
	 */
	{
		uint32_t div_int = (uint32_t)div;
		uint32_t div_frac = (uint32_t)((div - (float)div_int) * 256.0f + 0.5f);

		if (div_frac > 255) {
			div_int += 1;
			div_frac = 0;
		}
		u->div_actual = (float)div_int + (float)div_frac / 256.0f;
		u->sm_clock_hz = (uint32_t)((float)sys_hz / u->div_actual);
	}

	pio_sm_init(u->pio, u->sm, u->offset, &c);
	pio_sm_set_enabled(u->pio, u->sm, true);

	printf("PIO UART: GPIO%u, %u baud\n", pin, baud);
	printf("  clk_sys      : %" PRIu32 " Hz\n", sys_hz);
	printf("  clkdiv       : %.4f requested -> %.4f after quantisation\n",
	       div, u->div_actual);
	printf("  sm clock     : %" PRIu32 " Hz  (want %u = 8 x baud)\n",
	       u->sm_clock_hz, CYCLES_PER_BIT * baud);
	printf("  actual baud  : %.1f  (error %+.3f%%)\n",
	       (double)u->sm_clock_hz / CYCLES_PER_BIT,
	       100.0 * ((double)u->sm_clock_hz / CYCLES_PER_BIT - baud) / baud);
	return 0;
}

static void pio_uart_putc(struct pio_uart *u, uint8_t ch)
{
	/*
	 * The program shifts right and takes the low 8 bits, so the byte goes
	 * in the least significant position.
	 */
	pio_sm_put_blocking(u->pio, u->sm, (uint32_t)ch);
}

static void pio_uart_write(struct pio_uart *u, const void *buf, size_t len)
{
	const uint8_t *p = buf;

	for (size_t i = 0; i < len; i++)
		pio_uart_putc(u, p[i]);
}

static void pio_uart_close(struct pio_uart *u)
{
	if (!PIO_IS_ERR(u->pio)) {
		pio_sm_set_enabled(u->pio, u->sm, false);
		pio_close(u->pio);
	}
}

/*
 * Baud accuracy is the thing worth checking without an oscilloscope: a UART
 * tolerates roughly +/-2% total clock error between the two ends, so anything
 * under about 1% here leaves usable margin.
 */
static int selftest(void)
{
	static const uint bauds[] = { 9600, 115200, 230400, 460800, 921600 };
	int failures = 0;

	printf("PIO UART baud accuracy self-test (GPIO%d)\n\n", TX_GPIO);

	for (size_t i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
		struct pio_uart u;
		double actual, err;

		if (pio_uart_init(&u, TX_GPIO, bauds[i]))
			return 1;

		actual = (double)u.sm_clock_hz / CYCLES_PER_BIT;
		err = 100.0 * (actual - bauds[i]) / bauds[i];

		printf("  verdict      : %s\n\n",
		       (err < 1.0 && err > -1.0) ? "OK (well inside UART tolerance)"
						 : "MARGINAL - check clock divider");
		if (err >= 1.0 || err <= -1.0)
			failures++;

		/* Send a byte pattern so the line actually toggles. */
		pio_uart_write(&u, "U", 1);
		usleep(20000);
		pio_uart_close(&u);
	}

	printf("%s\n", failures ? "SOME BAUD RATES MARGINAL"
			        : "all baud rates within tolerance");
	return failures ? 1 : 0;
}

static void usage(const char *argv0)
{
	fprintf(stderr,
		"usage: %s [--baud N] [--pin N] [--tx STRING] [--selftest]\n"
		"\n"
		"  --baud N      baud rate (default %d)\n"
		"  --pin N       TX GPIO (default %d)\n"
		"  --tx STRING   transmit STRING\n"
		"  --selftest    check baud accuracy across common rates\n",
		argv0, DEFAULT_BAUD, TX_GPIO);
}

int main(int argc, char **argv)
{
	uint baud = DEFAULT_BAUD, pin = TX_GPIO;
	const char *tx = NULL;
	bool do_selftest = false;
	struct pio_uart u;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--baud") && i + 1 < argc)
			baud = (uint)atoi(argv[++i]);
		else if (!strcmp(argv[i], "--pin") && i + 1 < argc)
			pin = (uint)atoi(argv[++i]);
		else if (!strcmp(argv[i], "--tx") && i + 1 < argc)
			tx = argv[++i];
		else if (!strcmp(argv[i], "--selftest"))
			do_selftest = true;
		else {
			usage(argv[0]);
			return 2;
		}
	}

	if (do_selftest)
		return selftest();

	if (!tx) {
		usage(argv[0]);
		return 2;
	}

	if (pio_uart_init(&u, pin, baud))
		return 1;

	pio_uart_write(&u, tx, strlen(tx));
	pio_uart_write(&u, "\r\n", 2);

	/* Let the FIFO drain before tearing the state machine down. */
	usleep(1000 + (strlen(tx) + 2) * 10 * 1000000 / baud);

	printf("\ntransmitted %zu bytes\n", strlen(tx) + 2);
	pio_uart_close(&u);
	return 0;
}
