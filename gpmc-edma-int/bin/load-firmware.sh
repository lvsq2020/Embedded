#!/bin/sh

if [ -f "server_dsp1.xe66" ];then
    path=$(cd "$(dirname "$0")"; pwd)"/server_dsp1.xe66"
    rm /lib/firmware/dra7-dsp1-fw.xe66
    ln -s $path /lib/firmware/dra7-dsp1-fw.xe66
    echo 40800000.dsp > /sys/bus/platform/drivers/omap-rproc/unbind
    echo 40800000.dsp > /sys/bus/platform/drivers/omap-rproc/bind
fi

if [ -f "server_dsp2.xe66" ];then
    path=$(cd "$(dirname "$0")"; pwd)"/server_dsp2.xe66"
    rm /lib/firmware/dra7-dsp2-fw.xe66
    ln -s $path /lib/firmware/dra7-dsp2-fw.xe66
    echo 41000000.dsp > /sys/bus/platform/drivers/omap-rproc/unbind
    echo 41000000.dsp > /sys/bus/platform/drivers/omap-rproc/bind
fi

if [ -f "server_ipu1.xem4" ];then
    path=$(cd "$(dirname "$0")"; pwd)"/server_ipu1.xem4"
    rm /lib/firmware/dra7-ipu1-fw.xem4
    ln -s $path /lib/firmware/dra7-ipu1-fw.xem4
    echo 58820000.ipu > /sys/bus/platform/drivers/omap-rproc/unbind
    echo 58820000.ipu > /sys/bus/platform/drivers/omap-rproc/bind
fi

if [ -f "server_ipu2.xem4" ];then
    path=$(cd "$(dirname "$0")"; pwd)"/server_ipu2.xem4"
    rm /lib/firmware/dra7-ipu2-fw.xem4
    ln -s $path /lib/firmware/dra7-ipu2-fw.xem4
    echo 55020000.ipu > /sys/bus/platform/drivers/omap-rproc/unbind
    echo 55020000.ipu > /sys/bus/platform/drivers/omap-rproc/bind
fi

