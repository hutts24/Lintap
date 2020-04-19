obj-m += lintap.o

DESTDIR ?= /

KERNEL_VERSION := $(shell uname -r)
MODULE_DIR := /lib/modules/$(KERNEL_VERSION)

all:
	make -C $(MODULE_DIR)/build M=$(PWD) modules

debug:
	KCFLAGS=-DDEBUG	make all

clean:
	make -C $(MODULE_DIR)/build M=$(PWD) clean

install: all
	install -D -m 644 lintap.ko $(DESTDIR)$(MODULE_DIR)/lintap/lintap.ko
