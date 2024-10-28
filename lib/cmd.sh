#!/bin/sh

# generate libch9344
/usr/bin/gcc-7 ch9344_lib.c -Wall -fPIC -shared -o libch9344.so
mv libch9344.so x64/dynamic

/usr/bin/gcc-7 -m32 ch9344_lib.c -Wall -fPIC -shared -o libch9344.so
mv libch9344.so x86/dynamic

/home/rambo/Tools/toolchain/general/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc ch9344_lib.c -Wall -fPIC -shared -o libch9344.so
mv libch9344.so aarch64/dynamic

/home/rambo/Tools/toolchain/general/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-gcc ch9344_lib.c -Wall -fPIC -shared -o libch9344.so
mv libch9344.so arm-gnueabi/dynamic

/home/rambo/Tools/toolchain/general/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc ch9344_lib.c -Wall -fPIC -shared -o libch9344.so
mv libch9344.so arm-gnueabihf/dynamic

/home/rambo/Tools/toolchain/general/mips-loongson-gcc7.3-linux-gnu/2019.06-29/bin/mips-linux-gnu-gcc -mabi=64 ch9344_lib.c -Wall -fPIC -shared -o libch9344.so
mv libch9344.so mips64/dynamic

/home/rambo/Tools/toolchain/general/mips-loongson-gcc7.3-linux-gnu/2019.06-29/bin/mips-linux-gnu-gcc -mabi=32 ch9344_lib.c -Wall -fPIC -shared -o libch9344.so
mv libch9344.so mips32/dynamic

/usr/bin/gcc-7 -Wall -fPIC -c ch9344_lib.c
ar -cr libch9344.a ch9344_lib.o
ranlib libch9344.a
mv libch9344.a x64/static

/usr/bin/gcc-7 -m32 -Wall -fPIC -c ch9344_lib.c
ar -cr libch9344.a ch9344_lib.o
ranlib libch9344.a
mv libch9344.a x86/static

/home/rambo/Tools/toolchain/general/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc -Wall -fPIC -c ch9344_lib.c
ar -cr libch9344.a ch9344_lib.o
ranlib libch9344.a
mv libch9344.a aarch64/static

/home/rambo/Tools/toolchain/general/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-gcc -Wall -fPIC -c ch9344_lib.c
ar -cr libch9344.a ch9344_lib.o
ranlib libch9344.a
mv libch9344.a arm-gnueabi/static

/home/rambo/Tools/toolchain/general/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc -Wall -fPIC -c ch9344_lib.c
ar -cr libch9344.a ch9344_lib.o
ranlib libch9344.a
mv libch9344.a arm-gnueabihf/static

/home/rambo/Tools/toolchain/general/mips-loongson-gcc7.3-linux-gnu/2019.06-29/bin/mips-linux-gnu-gcc -mabi=64 -Wall -fPIC -c ch9344_lib.c
ar -cr libch9344.a ch9344_lib.o
ranlib libch9344.a
mv libch9344.a mips64/static

/home/rambo/Tools/toolchain/general/mips-loongson-gcc7.3-linux-gnu/2019.06-29/bin/mips-linux-gnu-gcc -mabi=32 -Wall -fPIC -c ch9344_lib.c
ar -cr libch9344.a ch9344_lib.o
ranlib libch9344.a
mv libch9344.a mips32/static

cp *.h aarch64/dynamic
cp *.h aarch64/static

cp *.h x64/dynamic
cp *.h x64/static

cp *.h x86/dynamic
cp *.h x86/static

cp *.h arm-gnueabi/dynamic
cp *.h arm-gnueabi/static

cp *.h arm-gnueabihf/dynamic
cp *.h arm-gnueabihf/static

cp *.h mips32/dynamic
cp *.h mips32/static

cp *.h mips64/dynamic
cp *.h mips64/static
