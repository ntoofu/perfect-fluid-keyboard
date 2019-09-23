FROM ubuntu:18.04
ARG prefix=/usr/local
ARG makeopts
ARG gcc_mirror_url=http://ftp.tsukuba.wide.ad.jp/software/gcc/releases/
ENV PATH $PATH:$prefix/bin
RUN \
        mkdir -p $prefix/bin && mkdir -p /root/work &&\
        apt update &&\
        apt install -y --no-install-recommends ca-certificates wget unzip build-essential &&\
        echo "Install binutils(2.32) for AVR" && cd /root/work &&\
        wget https://ftpmirror.gnu.org/binutils/binutils-2.32.tar.gz &&\
        tar zxvf binutils-2.32.tar.gz && cd binutils-2.32 &&\
        mkdir avr-build && cd avr-build &&\
        ../configure --prefix=$prefix --target=avr --disable-nls &&\
        make $makeopts && make $makeopts install &&\
        echo "Install GCC(8.3.0) for AVR" && cd /root/work &&\
        wget $gcc_mirror_url/gcc-8.3.0/gcc-8.3.0.tar.gz && tar zxvf gcc-8.3.0.tar.gz && cd gcc-8.3.0 &&\
        ./contrib/download_prerequisites && mkdir avr-build && cd avr-build &&\
        ../configure  --prefix=$prefix --target=avr --enable-languages=c,c++ --disable-nls --disable-libssp --with-dwarf2 &&\
        make $makeopts && make $makeopts install &&\
        echo "Install AVR Libc(2.0.0)" && cd /root/work &&\
        wget http://download.savannah.gnu.org/releases/avr-libc/avr-libc-2.0.0.tar.bz2 && tar jxvf avr-libc-2.0.0.tar.bz2 && cd avr-libc-2.0.0 &&\
        ./configure --prefix=$prefix --build=`./config.guess` --host=avr &&\
        make $makeopts && make $makeopts install &&\
        echo "Install DFP Pack for ATtiny202" && cd /root/work &&\
        wget http://packs.download.atmel.com/Atmel.ATtiny_DFP.1.3.229.atpack &&\
        mkdir atpack && cd atpack && unzip ../Atmel.ATtiny_DFP.1.3.229.atpack &&\
        mkdir -p $prefix/avr/lib/avrxmega3 &&\
        cp -r gcc/dev/attiny202/avrxmega3/* $prefix/avr/lib/avrxmega3/ &&\
        cp gcc/dev/attiny202/device-specs/specs-attiny202 $prefix/lib/gcc/avr/8.3.0/device-specs/ &&\
        cp include/avr/iotn202.h $prefix/avr/include/avr/ &&\
        echo "Install libraries for avrxmega3 not included in AVR Libc" && cd /root/work &&\
        wget -O avr8-gnu-toolchain-3.6.2.1759.linux.any.x86_64.tar.gz https://www.microchip.com/mymicrochip/filehandler.aspx?ddocname=en607660 &&\
        tar zxvf avr8-gnu-toolchain-3.6.2.1759.linux.any.x86_64.tar.gz &&\
        cp -r avr8-gnu-toolchain-linux_x86_64/avr/lib/avrxmega3/* $prefix/avr/lib/avrxmega3/ &&\
        echo "clean up" && cd / &&\
        rm -rf /var/lib/apt/lists/* && rm -rf /root/work
