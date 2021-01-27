# invoke SourceDir generated makefile for Dsp1.pe66
Dsp1.pe66: .libraries,Dsp1.pe66
.libraries,Dsp1.pe66: package/cfg/Dsp1_pe66.xdl
	$(MAKE) -f package/cfg/Dsp1_pe66.src/makefile.libs

clean::
	$(MAKE) -f package/cfg/Dsp1_pe66.src/makefile.libs clean

