ifeq ($(KERNELRELEASE), )
KERNELDIR := /lib/modules/$(shell uname -r)/build
PWD :=$(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR)  M=$(PWD)  
clean:
	rm -rf *.mk .tmp_versions Module.symvers *.mod.c *.o *.ko .*.cmd Module.markers modules.order *.a *.mod
load:
	insmod ch9344.ko
unload:
	rmmod ch9344
install: default
	mkdir -p /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial/
	cp -f ./ch9344.ko /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial/
	depmod -a
uninstall:
	rm -rf /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial/ch9344.ko
	depmod -a
else
	obj-m := ch9344.o
endif