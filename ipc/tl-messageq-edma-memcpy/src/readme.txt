#
#  ======== readme.txt ========
#

tl-edma-with-ipc

Program Logic:
This routine mainly demonstrates that the DSP core uses EDMA to copy the data of the source address in the DDR memory to the target address of the DDR memory. The ARM reads the data of the target address in the DDR memory and calculates the total time. The command transmission between ARM and DSP is performed by IPC MessageQ.
