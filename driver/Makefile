CONFIG_MODULE_SIG=n

ifeq ($(KERNELRELEASE), )
# 需要根据你的实际情况修改路径
KERNELDIR ?= /usr/src/linux-headers-$(shell uname -r)
PWD := $(shell pwd)

default:
    $(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
    $(MAKE) -C $(KERNELDIR) M=$(PWD) clean

load:
	insmod ch9344.ko

unload:
	rmmod ch9344

install: default
	insmod ch9344.ko || true
	mkdir -p /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial/ || true
	cp -f ./ch9344.ko /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial/ || true
	depmod -a

uninstall:
	rmmod ch9344 || true
	rm -rf /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial/ch9344.ko || true
	depmod -a

else
	obj-m := ch9344.o
endif
