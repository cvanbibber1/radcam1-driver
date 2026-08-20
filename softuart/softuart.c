// SPDX-License-Identifier: GPL-2.0
/*
 * Bit-banged soft UART for the radcam1 ground-debug port (EXTUART).
 *
 * The board is finalised with EXTUART on GPIO24 (TX) and GPIO23 (RX), and
 * neither pin has a hardware UART alt-function on a Pi 5 - they offer only
 * SD0/DPI/I2S/PIO. So the port has to be driven in software.
 *
 * Timing is the whole problem. One bit at 921600 baud lasts 1.085 us; at
 * 115200 it lasts 8.68 us. A userspace process can be preempted for tens of
 * microseconds at any moment, which corrupts a byte mid-transmission. This
 * implementation therefore:
 *
 *   - busy-waits on CLOCK_MONOTONIC rather than sleeping, so there is no
 *     scheduler round-trip per bit;
 *   - computes every bit edge from an absolute frame start time, so a late
 *     bit does not push all the following ones late (error does not
 *     accumulate across a byte);
 *   - optionally takes SCHED_FIFO and locks memory, which removes almost all
 *     of the preemption;
 *   - reports measured jitter, so the achievable baud is a measurement rather
 *     than a hope. Run `softuart --measure`.
 *
 * Build:  make -C softuart
 * Test:   ./softuart --measure
 *         ./softuart --baud 115200 --tx "hello world"
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <gpiod.h>

#define GPIO_CHIP   "/dev/gpiochip0"
#define TX_LINE     24
#define RX_LINE     23
#define DEFAULT_BAUD 115200

/* ---------------------------------------------------------------- timing */

static inline uint64_t now_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

/*
 * Spin until the deadline. Sleeping is not an option: nanosleep's granularity
 * and wake-up latency are both far larger than a bit period at these rates.
 */
static inline void busy_until(uint64_t deadline)
{
	while (now_ns() < deadline)
		; /* spin */
}

/* Ask for real-time scheduling; harmless (and expected) to fail as non-root. */
static bool go_realtime(void)
{
	struct sched_param sp = { .sched_priority = 50 };

	if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
		return false;
	mlockall(MCL_CURRENT | MCL_FUTURE);
	return true;
}

/* ------------------------------------------------------------------ gpio */

struct softuart {
	struct gpiod_chip *chip;
	struct gpiod_line_request *tx_req;
	unsigned int tx_offset;
	uint64_t bit_ns;
};

static int su_open(struct softuart *su, unsigned int baud)
{
	struct gpiod_line_settings *settings = NULL;
	struct gpiod_line_config *lcfg = NULL;
	struct gpiod_request_config *rcfg = NULL;
	int ret = -1;

	memset(su, 0, sizeof(*su));
	su->tx_offset = TX_LINE;
	su->bit_ns = 1000000000ULL / baud;

	su->chip = gpiod_chip_open(GPIO_CHIP);
	if (!su->chip) {
		fprintf(stderr, "cannot open %s: %s\n", GPIO_CHIP, strerror(errno));
		return -1;
	}

	settings = gpiod_line_settings_new();
	lcfg = gpiod_line_config_new();
	rcfg = gpiod_request_config_new();
	if (!settings || !lcfg || !rcfg)
		goto out;

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
	/* UART idles high, so come up high and never glitch a false start bit. */
	gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_ACTIVE);

	if (gpiod_line_config_add_line_settings(lcfg, &su->tx_offset, 1, settings))
		goto out;

	gpiod_request_config_set_consumer(rcfg, "radcam-softuart");

	su->tx_req = gpiod_chip_request_lines(su->chip, rcfg, lcfg);
	if (!su->tx_req) {
		fprintf(stderr, "cannot request GPIO%u: %s\n",
			su->tx_offset, strerror(errno));
		goto out;
	}
	ret = 0;

out:
	if (rcfg)
		gpiod_request_config_free(rcfg);
	if (lcfg)
		gpiod_line_config_free(lcfg);
	if (settings)
		gpiod_line_settings_free(settings);
	if (ret && su->chip)
		gpiod_chip_close(su->chip);
	return ret;
}

static void su_close(struct softuart *su)
{
	if (su->tx_req)
		gpiod_line_request_release(su->tx_req);
	if (su->chip)
		gpiod_chip_close(su->chip);
}

static inline void su_set(struct softuart *su, int value)
{
	gpiod_line_request_set_value(su->tx_req, su->tx_offset,
				     value ? GPIOD_LINE_VALUE_ACTIVE
					   : GPIOD_LINE_VALUE_INACTIVE);
}

/*
 * Transmit one byte, 8N1. Every edge is scheduled from `start`, the absolute
 * time the start bit began, so timing error cannot accumulate across the ten
 * bits of the frame.
 */
static void su_write_byte(struct softuart *su, uint8_t byte)
{
	uint64_t start = now_ns();
	int bit;

	su_set(su, 0);					/* start bit */
	busy_until(start + su->bit_ns);

	for (bit = 0; bit < 8; bit++) {			/* LSB first */
		su_set(su, (byte >> bit) & 1);
		busy_until(start + su->bit_ns * (uint64_t)(bit + 2));
	}

	su_set(su, 1);					/* stop bit */
	busy_until(start + su->bit_ns * 10ULL);
}

static void su_write(struct softuart *su, const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++)
		su_write_byte(su, buf[i]);
}

/* ------------------------------------------------------------- measuring */

/*
 * Measure how accurately this machine can hold a bit period, which is what
 * actually decides the usable baud. A UART sample is taken mid-bit, so timing
 * error must stay under half a bit period; in practice keep worst-case error
 * under ~25% of a bit to leave margin for the receiver's own tolerance.
 */
static int measure(unsigned int baud, unsigned int samples)
{
	struct softuart su;
	uint64_t bit_ns = 1000000000ULL / baud;
	uint64_t worst = 0, total = 0;
	uint64_t *err;
	bool rt;

	rt = go_realtime();
	printf("real-time scheduling: %s\n", rt ? "yes (SCHED_FIFO 50)"
					        : "NO - run as root for accurate results");

	if (su_open(&su, baud))
		return 1;

	err = calloc(samples, sizeof(*err));
	if (!err) {
		su_close(&su);
		return 1;
	}

	for (unsigned int i = 0; i < samples; i++) {
		uint64_t t0 = now_ns();
		uint64_t target = t0 + bit_ns;

		su_set(&su, i & 1);
		busy_until(target);

		uint64_t actual = now_ns();
		uint64_t e = actual > target ? actual - target : target - actual;

		err[i] = e;
		total += e;
		if (e > worst)
			worst = e;
	}

	su_set(&su, 1);

	printf("\nbaud %u  (bit period %" PRIu64 " ns)\n", baud, bit_ns);
	printf("  samples      : %u\n", samples);
	printf("  mean error   : %" PRIu64 " ns  (%.2f%% of a bit)\n",
	       total / samples, 100.0 * (double)(total / samples) / (double)bit_ns);
	printf("  worst error  : %" PRIu64 " ns  (%.2f%% of a bit)\n",
	       worst, 100.0 * (double)worst / (double)bit_ns);
	printf("  verdict      : %s\n",
	       worst < bit_ns / 4 ? "USABLE" :
	       worst < bit_ns / 2 ? "MARGINAL - expect occasional framing errors"
				  : "UNUSABLE at this baud");

	free(err);
	su_close(&su);
	return 0;
}

/* ------------------------------------------------------------------ main */

static void usage(const char *argv0)
{
	fprintf(stderr,
		"usage: %s [--baud N] [--tx STRING] [--measure] [--samples N]\n"
		"\n"
		"  --baud N      baud rate (default %d)\n"
		"  --tx STRING   transmit STRING on GPIO%d\n"
		"  --measure     report timing jitter and the usable baud\n"
		"  --samples N   measurement samples (default 20000)\n",
		argv0, DEFAULT_BAUD, TX_LINE);
}

int main(int argc, char **argv)
{
	unsigned int baud = DEFAULT_BAUD, samples = 20000;
	const char *tx = NULL;
	bool do_measure = false;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--baud") && i + 1 < argc)
			baud = (unsigned int)atoi(argv[++i]);
		else if (!strcmp(argv[i], "--tx") && i + 1 < argc)
			tx = argv[++i];
		else if (!strcmp(argv[i], "--samples") && i + 1 < argc)
			samples = (unsigned int)atoi(argv[++i]);
		else if (!strcmp(argv[i], "--measure"))
			do_measure = true;
		else {
			usage(argv[0]);
			return 2;
		}
	}

	if (baud == 0) {
		fprintf(stderr, "baud must be non-zero\n");
		return 2;
	}

	if (do_measure)
		return measure(baud, samples);

	if (tx) {
		struct softuart su;
		bool rt = go_realtime();

		if (su_open(&su, baud))
			return 1;
		fprintf(stderr, "transmitting %zu bytes at %u baud on GPIO%d%s\n",
			strlen(tx), baud, TX_LINE, rt ? " (real-time)" : "");
		su_write(&su, (const uint8_t *)tx, strlen(tx));
		su_write(&su, (const uint8_t *)"\r\n", 2);
		su_close(&su);
		return 0;
	}

	usage(argv[0]);
	return 2;
}
