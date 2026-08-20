# EXTUART tools. piolib comes from github.com/raspberrypi/utils.
PIOLIB  ?= /home/rad/driver-dev/rpi-utils/piolib
CFLAGS  ?= -O2 -Wall -Wextra
GPIOD   := $(shell pkg-config --libs libgpiod 2>/dev/null || echo -lgpiod)
PIOINC  := -I$(PIOLIB)/include -I$(PIOLIB)/include/pico
PIOLIBA := -L$(PIOLIB)/build -lpio

all: softuart pio_uart pio_uart_rx pio_uart_bridge pio_clock

softuart: softuart.c
	$(CC) $(CFLAGS) -o $@ $< $(GPIOD)

pio_uart pio_uart_rx pio_uart_bridge pio_clock: %: %.c
	$(CC) $(CFLAGS) -o $@ $< $(PIOINC) $(PIOLIBA)

clean:
	rm -f softuart pio_uart pio_uart_rx pio_uart_bridge pio_clock
.PHONY: all clean
