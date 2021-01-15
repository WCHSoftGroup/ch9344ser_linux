# ch9344_linux

1. open "Terminal"

2. cd "driver" directory

3. compile the driver please enter "driver" directory

   1. BUILDING
    $ sudo make

   2. LOAD
    $ sudo make load
    or you can use
    $ sudo insmod ch9344.ko

   3. UNLOAD
    $ sudo make unload
    or you can use
    $ sudo rmmod ch9344.ko

   4. AUTOLOAD SINCE BOOT
       $ sudo make install
     
   5. CANCEL AUTOLOAD SINCE BOOT
       $ sudo make uninstall

4. cd "demo" to use uart application

   the tty device name of ch9344 is ttyWCHUSBx

   Note
     Any question, you can send feedback to mail: tech@wch.cn