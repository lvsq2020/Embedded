/*
 * Do not modify this file; it is automatically generated from the template
 * linkcmd.xdt in the ti.targets.elf package and will be overwritten.
 */

/*
 * put '"'s around paths because, without this, the linker
 * considers '-' as minus operator, not a file name character.
 */


-l"/home/louis/Embedded/cmem-fft/dsp1/bin/debug/configuro/package/cfg/Dsp1_pe66.oe66"
-l"/home/louis/Embedded/cmem-fft/dsp1/bin/debug/configuro/package/cfg/Dsp1_pe66.src/ipc/ipc.ae66"
-l"/opt/ti/ipc_3_47_01_00/packages/ti/pm/lib/debug/ti.pm_null.ae66"
-l"/home/louis/Embedded/cmem-fft/dsp1/bin/debug/configuro/package/cfg/Dsp1_pe66.src/utils/utils.ae66"
-l"/opt/ti/ipc_3_47_01_00/packages/ti/deh/lib/debug/ti.deh_vayu.ae66"
-l"/opt/ti/bios_6_52_00_12/packages/ti/targets/rts6000/lib/ti.targets.rts6000.ae66"
-l"/opt/ti/bios_6_52_00_12/packages/ti/targets/rts6000/lib/boot.ae66"
-l"/home/louis/Embedded/cmem-fft/dsp1/bin/debug/configuro/package/cfg/Dsp1_pe66.src/sysbios/sysbios.ae66"
-l"/opt/ti/ipc_3_47_01_00/packages/ti/trace/lib/debug/ti.trace.ae66"

--retain="*(xdc.meta)"


--args 0x64
-heap  0x0
-stack 0x1000

MEMORY
{
    L2SRAM (RWX) : org = 0x800000, len = 0x40000
    OCMC_RAM1 (RWX) : org = 0x40300000, len = 0x80000
    OCMC_RAM2 (RWX) : org = 0x40400000, len = 0x100000
    OCMC_RAM3 (RWX) : org = 0x40500000, len = 0x100000
    EXT_CODE (RWX) : org = 0x95000000, len = 0x400000
    EXT_DATA (RW) : org = 0x95400000, len = 0x3000000
    EXT_HEAP (RW) : org = 0x98400000, len = 0x300000
    TRACE_BUF (RW) : org = 0x9f000000, len = 0x60000
    EXC_DATA (RW) : org = 0x9f060000, len = 0x10000
    PM_DATA (RWX) : org = 0x9f070000, len = 0x20000
    CMEM (RW) : org = 0xa0000000, len = 0xc000000
}

/*
 * Linker command file contributions from all loaded packages:
 */

/* Content from xdc.services.global (null): */

/* Content from xdc (null): */

/* Content from xdc.corevers (null): */

/* Content from xdc.shelf (null): */

/* Content from xdc.services.spec (null): */

/* Content from xdc.services.intern.xsr (null): */

/* Content from xdc.services.intern.gen (null): */

/* Content from xdc.services.intern.cmd (null): */

/* Content from xdc.bld (null): */

/* Content from ti.targets (null): */

/* Content from ti.targets.elf (null): */

/* Content from xdc.rov (null): */

/* Content from ti.sdo.ipc.family (null): */

/* Content from xdc.services.getset (null): */

/* Content from ti.catalog.c6000 (null): */

/* Content from ti.catalog (null): */

/* Content from ti.catalog.peripherals.hdvicp2 (null): */

/* Content from xdc.platform (null): */

/* Content from xdc.cfg (null): */

/* Content from ti.catalog.arp32 (null): */

/* Content from ti.catalog.arm.cortexm4 (null): */

/* Content from ti.catalog.arm.cortexa15 (null): */

/* Content from ti.platforms.evmDRA7XX (null): */

/* Content from ti.sysbios.hal (null): */

/* Content from ti.sysbios.interfaces (null): */

/* Content from xdc.runtime (null): */

/* Content from ti.trace (null): */

/* Content from ti.sysbios.knl (null): */

/* Content from ti.sysbios (null): */

/* Content from ti.sysbios.family.c64p (ti/sysbios/family/c64p/linkcmd.xdt): */

/* Content from ti.sysbios.family.c66 (ti/sysbios/family/c66/linkcmd.xdt): */
ti_sysbios_family_c66_Cache_l1dSize = 32768;
ti_sysbios_family_c66_Cache_l1pSize = 32768;
ti_sysbios_family_c66_Cache_l2Size = 0;

/* Content from ti.sysbios.rts (ti/sysbios/rts/linkcmd.xdt): */

/* Content from ti.sysbios.rts.ti (ti/sysbios/rts/ti/linkcmd.xdt): */

/* Content from ti.sysbios.family (null): */

/* Content from ti.targets.rts6000 (null): */

/* Content from ti.deh (null): */

/* Content from ti.sysbios.gates (null): */

/* Content from xdc.runtime.knl (null): */

/* Content from ti.sdo.utils (null): */

/* Content from ti.sysbios.heaps (null): */

/* Content from ti.sdo.ipc.interfaces (null): */

/* Content from ti.sysbios.syncs (null): */

/* Content from ti.sysbios.timers.dmtimer (null): */

/* Content from ti.sysbios.family.shared.vayu (null): */

/* Content from ti.sysbios.family.c62 (null): */

/* Content from ti.sysbios.xdcruntime (null): */

/* Content from ti.pm (null): */

/* Content from ti.sysbios.utils (null): */

/* Content from ti.ipc.remoteproc (ti/ipc/remoteproc/linkcmd.xdt): */


    /*
     *  Set entry point to the HWI reset vector 0 to automatically satisfy
     *  any alignment constraints for the boot vector.
     */
    -eti_sysbios_family_c64p_Hwi0

    /*
     * We just modified the entry point, so suppress "entry point symbol other
     * than _c_int00 specified" warning.
     */
    --diag_suppress=10063

/* Content from ti.sdo.ipc.family.vayu (null): */

/* Content from ti.sdo.ipc (ti/sdo/ipc/linkcmd.xdt): */

SECTIONS
{

    ti_sdo_ipc_init: load > EXT_DATA, type = NOINIT
}

/* Content from ti.sdo.ipc.transports (null): */

/* Content from ti.sdo.ipc.nsremote (null): */

/* Content from ti.ipc.ipcmgr (null): */

/* Content from ti.ipc.transports (null): */

/* Content from ti.ipc.rpmsg (null): */

/* Content from ti.ipc.family.vayu (null): */

/* Content from ti.ipc.namesrv (null): */

/* Content from ti.sdo.ipc.notifyDrivers (null): */

/* Content from ti.sdo.ipc.heaps (null): */

/* Content from ti.sdo.ipc.gates (null): */

/* Content from configuro (null): */

/* Content from xdc.services.io (null): */

/* Content from ti.sdo.ipc.family.ti81xx (null): */


/*
 * symbolic aliases for static instance objects
 */
xdc_runtime_Startup__EXECFXN__C = 1;
xdc_runtime_Startup__RESETFXN__C = 1;

SECTIONS
{
    .text: load >> EXT_CODE
    .ti.decompress: load > EXT_CODE
    .stack: load > EXT_DATA
    GROUP: load > EXT_DATA
    {
        .bss:
        .neardata:
        .rodata:
    }
    .cinit: load > EXT_DATA
    .pinit: load >> EXT_DATA
    .init_array: load > EXT_DATA
    .const: load >> EXT_DATA
    .data: load >> EXT_DATA
    .fardata: load >> EXT_DATA
    .switch: load >> EXT_DATA
    .sysmem: load > EXT_DATA
    .far: load >> EXT_DATA
    .args: load > EXT_DATA align = 0x4, fill = 0 {_argsize = 0x64; }
    .cio: load >> EXT_DATA
    .ti.handler_table: load > EXT_DATA
    .c6xabi.exidx: load > EXT_DATA
    .c6xabi.extab: load >> EXT_DATA
    .tracebuf: load > TRACE_BUF
    .errorbuf: load > EXC_DATA
    .vecs: load > EXT_CODE
    .resource_table: load > 0x95000000, type = NOINIT
    xdc.meta: load > EXT_DATA, type = COPY

}
