#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /home/am57x/rtos/bios_6_52_00_12/packages;/home/am57x/rtos/ipc_3_47_01_00/packages;/home/am57x/rtos/xdctools_3_50_03_33_core/packages
override XDCROOT = /home/am57x/rtos/xdctools_3_50_03_33_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /home/am57x/rtos/bios_6_52_00_12/packages;/home/am57x/rtos/ipc_3_47_01_00/packages;/home/am57x/rtos/xdctools_3_50_03_33_core/packages;/home/am57x/rtos/xdctools_3_50_03_33_core/packages;..
HOSTOS = Linux
endif
