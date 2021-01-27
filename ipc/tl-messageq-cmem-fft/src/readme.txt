#
#  ======== readme.txt ========
#

MessageQ Example

Program Logic:
The slave creates a message to pass data around. The host fills with custom 
array from CMEM sends a message to the slave core. The slave calculates with 
FFT and fills datas into CMEM .Then sends the message back to the host. 
Then the host a shutdown message to the slave. The slave returns the message, 
shuts itself down and reinitializes itself for future runs.

Based on ipc/examples/DRA7XX_linux_elf/ex02_messageq

DSP1 and host are targets for build, makefiles are not changed.

make -j host
make -j dsp1

Folder contains device tree file for linux with related CMEM configuration.

