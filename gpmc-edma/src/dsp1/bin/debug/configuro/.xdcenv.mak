#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /opt/ti/bios_6_52_00_12/packages;/opt/ti/ipc_3_47_01_00/packages;/opt/ti/xdctools_3_50_03_33_core/packages;/opt/ti/edma3_lld_2_12_05_30C/packages;/opt/ti/pdk_am57xx_1_0_10/packages
override XDCROOT = /opt/ti/xdctools_3_50_03_33_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /opt/ti/bios_6_52_00_12/packages;/opt/ti/ipc_3_47_01_00/packages;/opt/ti/xdctools_3_50_03_33_core/packages;/opt/ti/edma3_lld_2_12_05_30C/packages;/opt/ti/pdk_am57xx_1_0_10/packages;/opt/ti/xdctools_3_50_03_33_core/packages;..
HOSTOS = Linux
endif
