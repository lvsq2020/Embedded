vpath % /home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/ipc_3_47_01_00/packages/ti/sdo/
vpath %.c /home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/xdctools_3_50_03_33_core/packages/

XOPTS = -I"/home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/xdctools_3_50_03_33_core/packages/" -Dxdc_target_types__=/home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/bios_6_52_00_12/packages/ti/targets/elf/std.h -Dxdc_target_name__=C66

CCOPTS = -mv6600 --abi=eabi -q -mi10 -mo -pdr -pden -pds=238 -pds=880 -pds1110  --program_level_compile -g  -Dti_sysbios_family_c64p_Exception_enableExternalMPC__D=FALSE -Dti_sysbios_family_c64p_Exception_enablePrint__D=TRUE -Dti_sysbios_knl_Task_minimizeLatency__D=FALSE -Dti_sysbios_knl_Clock_stopCheckNext__D=FALSE -Dti_sysbios_hal_Core_numCores__D=1

XDC_ROOT  = /home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/xdctools_3_50_03_33_core/packages/

IPC_ROOT = /home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/ipc_3_47_01_00/packages/ti/sdo/

BUILD_DEFS = 

SYSBIOS_INC  = -I"/home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/bios_6_52_00_12/packages"

IPC_INC    = -I"/home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/ipc_3_47_01_00/packages"

UTILS_INC    = -I"/home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/ipc_3_47_01_00/packages"

CC = /home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/ti-cgt-c6000_8.2.2/bin/cl6x -c $(CCOPTS) -I /home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/ti-cgt-c6000_8.2.2/include
ASM = /home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/ti-cgt-c6000_8.2.2/bin/cl6x -c $(CCOPTS) -I /home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/ti-cgt-c6000_8.2.2/include
AR = /home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/ti-cgt-c6000_8.2.2/bin/ar6x rq

DEL = rm -f
CP = cp -f
MKDIR = mkdir -p

define RM
	$(if $(wildcard $1),$(DEL) $1,:)
endef

define ASSEMBLE
	@echo asme66 $< ...
	@$(ASM) $(BUILD_DEFS) $(XOPTS) $(UTILS_INC) $<
endef


all: utils.ae66


UTILS.obj: utils/UTILS.c utils/NameServer.c utils/NameServerRemoteNull.c utils/MultiProc.c utils/List.c makefile
	@-$(call RM, $@)
	@echo cle66 $< ...
	@$(CC) $(BUILD_DEFS) $(XOPTS) $(SYSBIOS_INC) \
	$(IPC_INC) $(UTILS_INC) \
	$(IPC_ROOT)utils/UTILS.c \
	$(IPC_ROOT)utils/NameServer.c \
	$(IPC_ROOT)utils/NameServerRemoteNull.c \
	$(IPC_ROOT)utils/MultiProc.c \
	$(IPC_ROOT)utils/List.c \

utils.ae66: UTILS.obj 
	@-$(call RM, $@)
	@echo are66 $^ ...
	@$(AR) $@ $^


clean:
	@-$(call RM, utils.ae66)
	@-$(call RM, UTILS.obj)
