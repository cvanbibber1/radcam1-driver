# SPDX-License-Identifier: GPL-2.0
# Out-of-tree build of the AR1335 sensor driver.
#
#   make            # build ar1335.ko
#   make install    # install into /lib/modules/$(uname -r)/updates and depmod
#   make clean

KVER  ?= $(shell uname -r)
KDIR  ?= /lib/modules/$(KVER)/build
PWD   := $(shell pwd)

obj-m += ar1335.o

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

install: all
	$(MAKE) -C $(KDIR) M=$(PWD) INSTALL_MOD_DIR=updates modules_install
	depmod -a $(KVER)

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

.PHONY: all install clean
