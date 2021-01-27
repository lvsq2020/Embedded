## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,e66 linker.cmd package/cfg/Dsp1_pe66.oe66

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/Dsp1_pe66.xdl
	$(SED) 's"^\"\(package/cfg/Dsp1_pe66cfg.cmd\)\"$""\"/home/cy/extern_disk/git-store/am5728/IPC_Examples/Linux-4.03/Linux/tl-ipc-examples/tl-gatemap-mutex-access/dsp1/bin/debug/configuro/\1\""' package/cfg/Dsp1_pe66.xdl > $@
	-$(SETDATE) -r:max package/cfg/Dsp1_pe66.h compiler.opt compiler.opt.defs
