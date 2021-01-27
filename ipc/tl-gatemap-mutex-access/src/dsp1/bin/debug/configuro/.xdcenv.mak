#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/bios_6_52_00_12/packages;/home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/ipc_3_47_01_00/packages;/home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/xdctools_3_50_03_33_core/packages
override XDCROOT = /home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/xdctools_3_50_03_33_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/bios_6_52_00_12/packages;/home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/ipc_3_47_01_00/packages;/home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/xdctools_3_50_03_33_core/packages;/home/cy/extern_disk/sdk/am5728/ti-processor-sdk-rtos-am57xx-evm-04.03.00.05/xdctools_3_50_03_33_core/packages;..
HOSTOS = Linux
endif
