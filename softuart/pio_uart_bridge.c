// SPDX-License-Identifier: BSD-3-Clause
/*
 * Full-duplex PIO UART bridge for the radcam1 EXTUART debug port.
 *
 * TX on GPIO24, RX on GPIO23, both hardware-timed by the RP1 PIO block, so the
 * full 921600 baud is reachable on pins that have no UART alt-function on a
 * Pi 5.
 *
 * The bridge exists because the PIO port is not a kernel tty: nothing in
 * userspace can open("/dev/...") on it. So this presents the simplest possible
 * interface instead - stdin goes out on the wire, whatever arrives on the wire
 * comes back on stdout - and radcam/piolink.py drives it as a subprocess,
 * exposing the same write()/read_bytes() interface the rest of the telemetry
 * code already expects.
 *
 * Both directions are non-blocking and the process never exits on transient
 * errors, because it sits underneath a payload that must not stop talking.
 *
 * Build: see softuart/Makefile
 * Test:  echo hello | sudo ./pio_uart_bridge --baud 921600
 */

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

extern int atoi(const char *);

#include "piolib.h"

#define TX_GPIO         24
#define RX_GPIO         23
#define DEFAULT_BAUD    921600
#define CYCLES_PER_BIT  8

/* uart_tx: see pio_uart.c for the derivation of each word. */
static const uint16_t uart_tx_instructions[] = {
	0x9FA0, 0xF727, 0x6001, 0x0642,
};
static const struct pio_program uart_tx_program = {
	.instructions = uart_tx_instructions, .length = 4, .origin = -1,
};

/* uart_rx: see pio_uart_rx.c for the derivation of each word. */
static const uint16_t uart_rx_instructions[] = {
	0x2020, 0xEA27, 0x4001, 0x0642, 0x00C8, 0xC014, 0x20A0, 0x0000, 0x8020,
};
static const struct pio_program uart_rx_program = {
	.instructions = uart_rx_instructions, .length = 9, .origin = -1,
};

static volatile sig_atomic_t running = 1;

static void on_signal(int sig)
{
	(void)sig;
	running = 0;
}

static float clkdiv_for(uint32_t sys_hz, uint baud)
{
	return (float)sys_hz / (float)(CYCLES_PER_BIT * baud);
}

static void tx_init(PIO pio, int sm, uint offset, uint pin, uint baud)
{
	pio_sm_config c = pio_get_default_sm_config();

	pio_gpio_init(pio, pin);
	pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

	sm_config_set_sideset(&c, 2, true, false);
	sm_config_set_sideset_pins(&c, pin);
	sm_config_set_out_pins(&c, pin, 1);
	sm_config_set_out_shift(&c, true, false, 32);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
	sm_config_set_wrap(&c, offset, offset + uart_tx_program.length - 1);
	sm_config_set_clkdiv(&c, clkdiv_for(clock_get_hz(clk_sys), baud));

	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);
}

static void rx_init(PIO pio, int sm, uint offset, uint pin, uint baud)
{
	pio_sm_config c = pio_get_default_sm_config();

	pio_gpio_init(pio, pin);
	pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);

	sm_config_set_in_pins(&c, pin);
	sm_config_set_jmp_pin(&c, pin);
	sm_config_set_in_shift(&c, true, false, 32);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
	sm_config_set_wrap(&c, offset, offset + uart_rx_program.length - 1);
	sm_config_set_clkdiv(&c, clkdiv_for(clock_get_hz(clk_sys), baud));

	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);
}

int main(int argc, char **argv)
{
	uint baud = DEFAULT_BAUD, tx_pin = TX_GPIO, rx_pin = RX_GPIO;
	PIO pio;
	int sm_tx, sm_rx;
	uint off_tx, off_rx;
	uint8_t inbuf[512];
	size_t in_len = 0, in_pos = 0;
	bool stdin_open = true;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--baud") && i + 1 < argc)
			baud = (uint)atoi(argv[++i]);
		else if (!strcmp(argv[i], "--tx-pin") && i + 1 < argc)
			tx_pin = (uint)atoi(argv[++i]);
		else if (!strcmp(argv[i], "--rx-pin") && i + 1 < argc)
			rx_pin = (uint)atoi(argv[++i]);
		else {
			fprintf(stderr,
				"usage: %s [--baud N] [--tx-pin N] [--rx-pin N]\n",
				argv[0]);
			return 2;
		}
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, SIG_IGN);

	pio = pio_open(0);
	if (PIO_IS_ERR(pio)) {
		fprintf(stderr, "cannot open pio0: %d\n", PIO_ERR_VAL(pio));
		return 1;
	}

	sm_tx = pio_claim_unused_sm(pio, true);
	sm_rx = pio_claim_unused_sm(pio, true);
	if (sm_tx < 0 || sm_rx < 0) {
		fprintf(stderr, "need two free PIO state machines\n");
		return 1;
	}

	off_tx = pio_add_program(pio, &uart_tx_program);
	off_rx = pio_add_program(pio, &uart_rx_program);

	/* TX first, so the line is idling high before RX starts sampling it. */
	tx_init(pio, sm_tx, off_tx, tx_pin, baud);
	usleep(5000);
	rx_init(pio, sm_rx, off_rx, rx_pin, baud);

	while (!pio_sm_is_rx_fifo_empty(pio, sm_rx))
		(void)pio_sm_get(pio, sm_rx);

	fprintf(stderr, "pio bridge: TX GPIO%u, RX GPIO%u, %u baud\n",
		tx_pin, rx_pin, baud);

	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

	while (running) {
		bool worked = false;

		/* stdin -> wire */
		if (in_pos >= in_len && stdin_open) {
			ssize_t n = read(STDIN_FILENO, inbuf, sizeof(inbuf));

			if (n > 0) {
				in_len = (size_t)n;
				in_pos = 0;
			} else if (n == 0) {
				stdin_open = false;   /* EOF: drain, then idle */
			} else if (errno != EAGAIN && errno != EWOULDBLOCK) {
				stdin_open = false;
			}
		}
		while (in_pos < in_len && !pio_sm_is_tx_fifo_full(pio, sm_tx)) {
			pio_sm_put(pio, sm_tx, (uint32_t)inbuf[in_pos++]);
			worked = true;
		}

		/* wire -> stdout */
		while (!pio_sm_is_rx_fifo_empty(pio, sm_rx)) {
			uint8_t b = (uint8_t)(pio_sm_get(pio, sm_rx) >> 24);

			if (write(STDOUT_FILENO, &b, 1) < 0 && errno != EAGAIN)
				break;
			worked = true;
		}

		/*
		 * Only sleep when there was nothing to do. Spinning would burn a
		 * core continuously on a power-constrained payload; sleeping
		 * unconditionally would add latency to every byte.
		 */
		if (!worked)
			usleep(200);
	}

	pio_sm_set_enabled(pio, sm_tx, false);
	pio_sm_set_enabled(pio, sm_rx, false);
	pio_close(pio);
	return 0;
}
