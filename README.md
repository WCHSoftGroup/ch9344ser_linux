# ch9344 linux serial driver

## Description

​	This directory contains 2 parts, ch9344 driver and gpio testing utility. This driver and application only support USB to quad serial ports chip ch9344.

1. Open "Terminal"
2. Switch to "driver" directory
3. Compile the driver using "make", you will see the module "ch9344.ko" if successful
4. Type "sudo make load" or "sudo insmod ch9344.ko" to load the driver dynamically
5. Type "sudo make unload" or "sudo rmmod ch9344.ko" to unload the driver
6. Type "sudo make install" to make the driver work permanently
7. Type "sudo make uninstall" to remove the driver
8. You can refer to the link below to acquire uart application, you can use gcc or Cross-compile with cross-gcc
   https://github.com/WCHSoftGroup/tty_uart

​	Before the driver works, you should make sure that the ch9344 device has been plugged in and is working properly, you can use shell command "lsusb" or "dmesg" to confirm that, USB ID of ch9344 is [1A86]:[E018].

​	If ch9344 device works well, the driver will created tty devices named "ttyCH9344USBx" in /dev directory.

## Note

​	Any question, you can send feedback to mail: tech@wch.cn