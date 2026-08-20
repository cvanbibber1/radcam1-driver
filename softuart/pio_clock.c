// SPDX-License-Identifier: BSD-3-Clause
/*
 * Generate a square-wave clock on any GPIO using the RP1 PIO block.
 *
 * Built for one specific problem: the AR1335 has no internal oscillator and
 * will not acknowledge its I2C address at all without an EXTCLK of 6-48 MHz.
 * The Pi 5's camera connector carries no clock, so if the module has no
 * oscillator of its own it can never respond. The connector's GPIO0 pin is the
 * only spare signal, and this board leaves it unused.
 *
 * RP1 GPCLK3 was tried first and would not reach the pad. PIO does, and gives
 * exact timing from the 200 MHz PIO clock:
 *
 *     set pins, 1 [3]     ; 4 cycles high
 *     set pins, 0 [3]     ; 4 cycles low
 *
 * Eight cycles per period, so 200 MHz / 8 = 25 MHz with clkdiv 1. That sits
 * comfortably inside the AR1335's range. Lower frequencies come from raising
 * the divider: f = 200e6 / (8 * clkdiv).
 *
 * Build: see softuart/Makefile
 * Run:   sudo ./pio_clock --pin 46 --hz 25000000 --seconds 30
 *        (GPIO46 = CAM1 connector IO0; GPIO34 = CAM0 connector IO0)
 */

#include <inttypes.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

extern int atoi(const char *);
extern long atol(const char *);

#include "piolib.h"

#define CYCLES_PER_PERIOD 8

/*
 * 0: set pins, 1 [3]   SET=111, delay 3=00011, dest PINS=000, data 1=00001
 *                      -> 111 00011 000 00001 = 0xE301
 * 1: set pins, 0 [3]   SET=111, delay 3=00011, dest PINS=000, data 0=00000
 *                      -> 111 00011 000 00000 = 0xE300
 */
static const uint16_t clock_instructions[] = { 0xE301, 0xE300 };

static const struct pio_program clock_program = {
	.instructions = clock_instructions, .length = 2, .origin = -1,
};

static volatile sig_atomic_t running = 1;

static void on_signal(int sig) { (void)sig; running = 0; }

int main(int argc, char **argv)
{
	uint pin = 46;
	double hz = 25000000.0;
	int seconds = 0;               /* 0 = run until signalled */
	PIO pio;
	int sm;
	uint offset;
	uint32_t sys_hz;
	float div;
	pio_sm_config c;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--pin") && i + 1 < argc)
			pin = (uint)atoi(argv[++i]);
		else if (!strcmp(argv[i], "--hz") && i + 1 < argc)
			hz = (double)atol(argv[++i]);
		else if (!strcmp(argv[i], "--seconds") && i + 1 < argc)
			seconds = atoi(argv[++i]);
		else {
			fprintf(stderr,
				"usage: %s [--pin N] [--hz N] [--seconds N]\n",
				argv[0]);
			return 2;
		}
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	pio = pio_open(0);
	if (PIO_IS_ERR(pio)) {
		fprintf(stderr, "cannot open pio0: %d\n", PIO_ERR_VAL(pio));
		return 1;
	}

	sm = pio_claim_unused_sm(pio, true);
	if (sm < 0) {
		fprintf(stderr, "no free PIO state machine\n");
		return 1;
	}
	offset = pio_add_program(pio, &clock_program);

	sys_hz = clock_get_hz(clk_sys);
	div = (float)sys_hz / (float)(CYCLES_PER_PERIOD * hz);
	if (div < 1.0f)
		div = 1.0f;             /* cannot run the SM faster than clk_sys */

	pio_gpio_init(pio, pin);
	pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

	c = pio_get_default_sm_config();
	sm_config_set_set_pins(&c, pin, 1);
	sm_config_set_wrap(&c, offset, offset + clock_program.length - 1);
	sm_config_set_clkdiv(&c, div);

	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);

	{
		uint32_t di = (uint32_t)div;
		uint32_t df = (uint32_t)((div - (float)di) * 256.0f + 0.5f);
		float actual_div = (float)di + (float)df / 256.0f;
		double actual = (double)sys_hz / (CYCLES_PER_PERIOD * actual_div);

		printf("PIO clock on GPIO%u\n", pin);
		printf("  clk_sys   : %" PRIu32 " Hz\n", sys_hz);
		printf("  clkdiv    : %.4f -> %.4f quantised\n", div, actual_div);
		printf("  output    : %.3f MHz (asked %.3f MHz, error %+.3f%%)\n",
		       actual / 1e6, hz / 1e6, 100.0 * (actual - hz) / hz);
		fflush(stdout);
	}

	if (seconds > 0) {
		for (int t = 0; t < seconds && running; t++)
			sleep(1);
	} else {
		while (running)
			sleep(1);
	}

	pio_sm_set_enabled(pio, sm, false);
	pio_close(pio);
	printf("clock stopped\n");
	return 0;
}
