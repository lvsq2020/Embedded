
* You may need to rebuild ludev first:
    * ./configure --host=arm-linux-gnueabihf CC=/home/jack/am5728/ti-processor-sdk-linux-am57xx-evm-03.01.00.06/linux-devkit/sysroots/x86_64-arago-linux/usr/bin/arm-linux-gnueabihf-gcc --prefix=/home/jack/am5728/demo/ipc-cmem-plate-recognize/ludev/__install
    * make clean && make

* Read bmp picture from ./data/1.bmp, ./data/2.bmp etc.

* Be careful of width and height of picture, set in dsp1/Server.c

* Copy dsp1/bin/release/server_dsp1.xe66, host/bin/release/app_host

* Check dsp log print: cat /sys/kernel/debug/remoteproc/remoteproc2/trace0
