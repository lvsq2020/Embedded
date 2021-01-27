/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2010-2016
 *
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/** 
 * In the PCIe sample example two EVMs are used to test the PCIe driver. 
 * As described in the following figure, EVM RC is configured as a Root Complex
 * and EVM EP is configured as End Point.
 *
 *         EVM RC                                          EVM EP
 *   ------------------                             -------------------
 *   |                |                             |                 |
 *   |   Root         |          PCIe Link          |  End Point      |
 *   |   Complex      | <-------------------------->|                 |
 *   |                |                             |                 |
 *   ------------------                             -------------------
 *  
 * Once the PCIe link is established, the following sequence of actions will happen:
 *  - EVM RC sends data to EVM EP
 *  - EVM EP waits to receive all the data
 *  - EVM EP sends the data back to EVM RC
 *  - EVM RC waits to receive all the data
 *  - EVM RC verifies if the received data matches the sent data and declares test pass or fail.
 *  - EVM EP sends 10 MSI and 10 INTA's to EVM RC.
 *
 */

/* this define must precede inclusion of any xdc header file */
#define Registry_CURDESC Test__Desc
#define MODULE_NAME "Server"

/* xdctools header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Registry.h>

#include <stdio.h>

/* package header files */
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/MultiProc.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* local header files */
#include "../shared/AppCommon.h"

/* module header file */
#include "Server.h"


#define SOC_AM572x
#include "pcie_sample.h"
#include <ti/drv/pcie/soc/pcie_soc.h>

#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>

#include "TSC.h"
#include "MyTrace.h"
#include <ti/drv/pcie/soc/pcie_soc.h>

#include <ti/ipc/remoteproc/Resource.h>

#include <xdc/runtime/Types.h>
#include <xdc/runtime/Timestamp.h>

#include <c6x.h>

#include "xdma-core.h"

#include <stdint.h>

#define PRODUCT_EXAMPLE
#define SEND_BUF_ADDRESS 0xB0000000
#define RECVICE_BUF_ADDRESS 0xB0500000
#define H2C_DESC_ADDRESS 0xB0A00000
#define C2H_DESC_ADDRESS 0xB0B00000

#ifdef __ARM_ARCH_7A__
#include <ti/sysbios/family/arm/a15/Cache.h>
#include <ti/sysbios/family/arm/a15/Mmu.h>
#if defined(SOC_K2G)
#include <ti/csl/cslr_msmc.h>
#define COHERENT  /* Cache ops unnecessary */
#endif
#endif
#if defined(SOC_AM572x) || defined(SOC_AM571x) || defined(SOC_K2G) || (__ARM_ARCH_7A__)
#include "ti/board/board.h"
#endif

#if !defined(SOC_AM572x) && !defined(SOC_AM571x)
#include <ti/csl/csl_bootcfgAux.h>
#include <ti/csl/csl_xmcAux.h>
#include <ti/csl/csl_serdes_pcie.h>
#include <ti/csl/csl_pscAux.h>
#define PCIE_REV0_HW
#else
#include <ti/drv/pcie/example/sample/am57x/src/pcie_sample_board.h>
#define PCIE_REV1_HW
#endif
#ifdef _TMS320C6X 
#include <ti/csl/csl_cacheAux.h>
#endif
#include <ti/csl/csl_chip.h>

#ifdef _TMS320C6X 
#pragma DATA_SECTION(dstBuf, ".dstBufSec")
/* Cache coherence: Align must be a multiple of cache line size (L2=128 bytes, L1=64 bytes) to operate with cache enabled. */
/* Aligning to 256 bytes because the PCIe inbound offset register masks the last 8bits of the buffer address  */
#pragma DATA_ALIGN(dstBuf, 256) // TI way of aligning
#endif

/* last element in the buffer is a marker that indicates the buffer status: full/empty */
#define PCIE_EXAMPLE_MAX_CACHE_LINE_SIZE 128
#define PCIE_EXAMPLE_UINT32_SIZE           4 /* preprocessor #if requires a real constant, not a sizeof() */

#define PCIE_EXAMPLE_DSTBUF_BYTES ((PCIE_BUFSIZE_APP + 1) * PCIE_EXAMPLE_UINT32_SIZE)
#define PCIE_EXAMPLE_DSTBUF_REM (PCIE_EXAMPLE_DSTBUF_BYTES % PCIE_EXAMPLE_MAX_CACHE_LINE_SIZE)
#define PCIE_EXAMPLE_DSTBUF_PAD (PCIE_EXAMPLE_DSTBUF_REM ? (PCIE_EXAMPLE_MAX_CACHE_LINE_SIZE - PCIE_EXAMPLE_DSTBUF_REM) : 0)
#define PCIE_EDMA_EXAMPLE_DSTBUF_BYTES (PCIE_EXAMPLE_LINE_SIZE * PCIE_EXAMPLE_UINT32_SIZE)
typedef struct dstBuf_s {
  volatile uint32_t buf[PCIE_BUFSIZE_APP + 1];
  /* Cache coherence: Must pad to cache line size in order to enable cacheability */
#if PCIE_EXAMPLE_DSTBUF_PAD
  uint8_t padding[PCIE_EXAMPLE_DSTBUF_PAD];
#endif
#ifdef EDMA
  volatile uint32_t edma_buf[PCIE_EXAMPLE_LINE_SIZE];
#endif
#ifdef EDMAPKTBENCH
  edmaPktBenchBuf_t edmaPktBenchBuf;
#endif
} dstBuf_t;
dstBuf_t dstBuf
#ifdef __ARM_ARCH_7A__
__attribute__((aligned(256), section(".bss:dstBufSec"))) // GCC way of aligning
#endif
; // for dstBuf

/* module structure */
typedef struct {
    UInt16              hostProcId;         // host processor id
    MessageQ_Handle     slaveQue;           // created locally
} Server_Module;

/* private data */
Registry_Desc               Registry_CURDESC;
static Server_Module        Module;

extern uint32_t isr_count;
extern uint32_t msi_isr_index;
uint32_t totalInts = 48000;
extern uint32_t msi_index_count[32];

uint32_t *pcie_bar1_base_address = NULL;
struct xdma_desc *xdma_h2c_desc_ptr = NULL;
struct xdma_desc *xdma_c2h_desc_ptr = NULL;

uint8_t is_xdma_write = 1;

#define PCIE_EXAMPLE_BUF_EMPTY 0
#define PCIE_EXAMPLE_BUF_FULL  1

/* Does not need to be aligned (even for cache) since it is only accessed locally */
uint32_t srcBuf[PCIE_BUFSIZE_APP];

/* Global variable timers for throughput */
uint64_t totalDMATime = 0;

#ifdef EDMA
/* This is the data that will be used as a temporary space holder
 * for the data being transfered using DMA.
 *
 * This is done since EDMA cannot send a specific value or token
 * but instead it can send blocks of data.
 * */
 #ifdef _TMS320C6X 
#pragma DATA_SECTION(dataContainer, ".testData")
#pragma DATA_ALIGN(dataContainer, PCIE_EXAMPLE_LINE_SIZE)
#endif
UInt32 dataContainer[PCIE_EXAMPLE_LINE_SIZE]
#ifdef __ARM_ARCH_7A__
__attribute__((aligned(256))) // GCC way of aligning
#endif
; // for dstBuf
#endif

#ifdef _TMS320C6X 
extern volatile unsigned int cregister TSCL;
#endif

/* Global config variable that controls
   the PCIe mode. It is global so it can be poked
   from CCS. It should be set either to EP or RC. */
pcieMode_e PcieModeGbl = pcie_RC_MODE;

#ifndef CSL_PSC_PD_PCIEX
#ifndef CSL_PSC_PD_PCIE
#define CSL_PSC_PD_PCIE CSL_PSC_PD_PCIE_0
#endif
#else
#define CSL_PSC_PD_PCIE CSL_PSC_PD_PCIEX
#endif

#ifndef CSL_PSC_LPSC_PCIEX 
#ifndef CSL_PSC_LPSC_PCIE
#define CSL_PSC_LPSC_PCIE CSL_PSC_LPSC_PCIE_0
#endif
#else
#define CSL_PSC_LPSC_PCIE CSL_PSC_LPSC_PCIEX
#endif
void cache_invalidate (void *ptr, int size)
{
#ifdef _TMS320C6X 
  uint32_t key;
  /* Disable Interrupts */
  key = _disable_interrupts();

  /*  Cleanup the prefetch buffer also. */
  CSL_XMC_invalidatePrefetchBuffer();

  CACHE_invL1d (ptr, size, CACHE_FENCE_WAIT);
  CACHE_invL2  (ptr, size, CACHE_FENCE_WAIT);

  /* Reenable Interrupts. */
  _restore_interrupts(key);
#elif defined(__ARM_ARCH_7A__)
#ifndef COHERENT
  /*  while bios could have been used on c66 that device chose csl */
  Cache_inv (ptr, size, Cache_Type_ALLD, TRUE);
#endif
#else
/* #error dont know how to invalidate the cache */
#endif
}

void cache_writeback (void *ptr, int size)
{
#ifdef _TMS320C6X 
  uint32_t key;
  /* Disable Interrupts */
  key = _disable_interrupts();

  CACHE_wbL1d (ptr, size, CACHE_FENCE_WAIT);
  CACHE_wbL2  (ptr, size, CACHE_FENCE_WAIT);

  /* Reenable Interrupts. */
  _restore_interrupts(key);
#elif defined(__ARM_ARCH_7A__)
#ifndef COHERENT
  /*  while bios could have been used on c66 that device chose csl */
  Cache_wb (ptr, size, Cache_Type_ALLD, TRUE);
#endif
#else
/* #error dont know how to writeback the cache */
#endif
}

/*
 *  ======== Server_init ========
 */
Void Server_init(Void)
{
    Registry_Result result;

    /* register with xdc.runtime to get a diags mask */
    result = Registry_addModule(&Registry_CURDESC, MODULE_NAME);
    Assert_isTrue(result == Registry_SUCCESS, (Assert_Id)NULL);

    /* initialize module object state */
    Module.hostProcId = MultiProc_getId("HOST");
}

/*
 *  ======== Server_create ========
 */
Int Server_create()
{
    Int                 status = 0;
    MessageQ_Params     msgqParams;
    char                msgqName[32];

    /* enable some log events */
    Diags_setMask(MODULE_NAME"+EXF");

    /* create local message queue (inbound messages) */
    MessageQ_Params_init(&msgqParams);
    sprintf(msgqName, App_SlaveMsgQueName, MultiProc_getName(MultiProc_self()));
    Module.slaveQue = MessageQ_create(msgqName, &msgqParams);

    if (Module.slaveQue == NULL) {
        status = -1;
        goto leave;
    }

    Log_print0(Diags_INFO,"Server_create: server is ready");

leave:
    Log_print1(Diags_EXIT, "<-- Server_create: %d", (IArg)status);
    return (status);
}

/*
 *  ======== Server_delete ========
 */

Int Server_delete()
{
    Int         status;

    Log_print0(Diags_ENTRY, "--> Server_delete:");

    /* delete the video message queue */
    status = MessageQ_delete(&Module.slaveQue);

    if (status < 0) {
        goto leave;
    }

leave:
    if (status < 0) {
        Log_error1("Server_finish: error=0x%x", (IArg)status);
    }

    /* disable log events */
    Log_print1(Diags_EXIT, "<-- Server_delete: %d", (IArg)status);
    Diags_setMask(MODULE_NAME"-EXF");

    return(status);
}

/*
 *  ======== Server_exit ========
 */

Void Server_exit(Void)
{
    /*
     * Note that there isn't a Registry_removeModule() yet:
     *     https://bugs.eclipse.org/bugs/show_bug.cgi?id=315448
     *
     * ... but this is where we'd call it.
     */
}


/*****************************************************************************
 * Function: Converts a core local L2 address to a global L2 address 
 *   Input addr:  L2 address to be converted to global.
 *   return:  uint32_t   Global L2 address
 *****************************************************************************/
uint32_t pcieConvert_CoreLocal2GlobalAddr (uint32_t  addr)
{
#ifdef _TMS320C6X

  uint32_t coreNum;

  /* Get the core number. */
  coreNum = CSL_chipReadReg(CSL_CHIP_DNUM); 
  
#if defined(SOC_AM572x) || defined(SOC_AM571x)
  /* Compute the global address. */
  return ((1 << 30) | (coreNum << 24) | (addr & 0x00ffffff));

#else
  /* Compute the global address. */
  return ((1 << 28) | (coreNum << 24) | (addr & 0x00ffffff));
#endif
#else
  return addr;
#endif
}    

/*****************************************************************************
 * Function: Power domain configuration
 ****************************************************************************/
pcieRet_e pciePowerCfg(void)
{
  return pcie_RET_OK;
}

static uint32_t readTime32(void)
{
	uint32_t timeVal;
#if defined (_TMS320C6X)
	timeVal = TSCL;
#elif __ARM_ARCH_7A__
	__asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(timeVal));
#else
	/* M4 specific implementation*/
	static uint32_t simuTimer = 0;
	simuTimer++;
	timeVal = simuTimer;
#endif
	return timeVal;
}

/*****************************************************************************
  * Function: Utility function to introduce delay
  ****************************************************************************/
void cycleDelay (uint32_t count)
{
	uint32_t start = (uint32_t)readTime32();
	while (((uint32_t)readTime32() - start) < count);
}

/*****************************************************************************
  * Function: Enable/Disable DBI writes
  ****************************************************************************/
pcieRet_e pcieCfgDbi(Pcie_Handle handle, uint8_t enable)
{
	pcieRegisters_t        regs;
	pcieRet_e              retVal;
	pciePlconfDbiRoWrEnReg_t dbiRo;

	memset (&dbiRo, 0, sizeof(dbiRo));
	memset (&regs, 0, sizeof(regs));

	regs.plconfDbiRoWrEn = &dbiRo;

	if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &regs)) != pcie_RET_OK)
	{
		PCIE_logPrintf ("SET CMD STATUS register failed!\n");
		return retVal;
	}
	return pcie_RET_OK;
}


/*****************************************************************************
 * Function: Serdes configuration 
 ****************************************************************************/
pcieRet_e pcieSerdesCfg(void)
{
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(DEVICE_K2E) && !defined(DEVICE_K2L) && \
    !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2L) && !defined(SOC_K2E) && !defined(SOC_K2G) && \
    !defined(SOC_AM572x) && !defined(SOC_AM571x)
  uint16_t cfg;

  /* Provide PLL reference clock to SERDES inside PCIESS
     Program PLL settings and enable PLL from PCIe SERDES.*/
  cfg = 0x01C9; /* value based on PCIe userguide */

  CSL_BootCfgSetPCIEConfigPLL(cfg);
#else /* !DEVICE_K2K && !DEVICE_K2H && !DEVICE_K2E && !DEVICE_K2L */
#if defined(SOC_AM572x) || defined(SOC_AM571x)

  /*Set PCIE_PERSTn to out of reset state*/
  //PlatformPCIE_GPIO_Init();
  //PlatformPCIE_PERSTn_Reset(0);

  PlatformPCIESS1ClockEnable();
  PlatformPCIESS2ClockEnable();
  PlatformPCIESS1PllConfig();
  PlatformPCIESSSetPhyMode();

  PlatformPCIESS1CtrlConfig();
  PlatformPCIESS2CtrlConfig();
  PlatformPCIESS1Reset();
  PlatformPCIESS2Reset();
  PlatformPCIESS1PhyConfig();
  PlatformPCIESS2PhyConfig();
#else
#ifndef  SIMULATOR_SUPPORT

  uint32_t i;

#if defined(DEVICE_K2E) || defined(SOC_K2E)
  /* Configure 2 lanes of serdes with different config */

  CSL_SERDES_RESULT status1, status2;
  CSL_SERDES_LANE_ENABLE_STATUS lane_retval1 = CSL_SERDES_LANE_ENABLE_NO_ERR;
  CSL_SERDES_LANE_ENABLE_STATUS lane_retval2 = CSL_SERDES_LANE_ENABLE_NO_ERR;
  CSL_SERDES_LANE_ENABLE_PARAMS_T serdes_lane_enable_params1, serdes_lane_enable_params2;

  memset(&serdes_lane_enable_params1, 0, sizeof(serdes_lane_enable_params1));
  memset(&serdes_lane_enable_params2, 0, sizeof(serdes_lane_enable_params2));

  serdes_lane_enable_params1.base_addr = CSL_PCIE_0_SERDES_CFG_REGS;
  serdes_lane_enable_params1.peripheral_base_addr = CSL_PCIE_0_SLV_CFG_REGS;
  serdes_lane_enable_params1.ref_clock = CSL_SERDES_REF_CLOCK_100M;
  serdes_lane_enable_params1.linkrate = CSL_SERDES_LINK_RATE_5G;
  serdes_lane_enable_params1.num_lanes = 1;
  serdes_lane_enable_params1.phy_type = SERDES_PCIe;
  serdes_lane_enable_params1.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;
  serdes_lane_enable_params1.lane_mask = 0x1;
  for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
  {
      serdes_lane_enable_params1.loopback_mode[i] = CSL_SERDES_LOOPBACK_DISABLED;
      serdes_lane_enable_params1.lane_ctrl_rate[i] = CSL_SERDES_LANE_FULL_RATE; /* GEN2 */
  }

  serdes_lane_enable_params2.base_addr = CSL_PCIE_1_SERDES_CFG_REGS;
  serdes_lane_enable_params2.peripheral_base_addr = CSL_PCIE_1_SLV_CFG_REGS;
  serdes_lane_enable_params2.ref_clock = CSL_SERDES_REF_CLOCK_100M;
  serdes_lane_enable_params2.linkrate = CSL_SERDES_LINK_RATE_5G;
  serdes_lane_enable_params2.num_lanes = 1;
  serdes_lane_enable_params2.phy_type = SERDES_PCIe;
  serdes_lane_enable_params2.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;
  serdes_lane_enable_params2.lane_mask = 0x1;
  for(i=0; i< serdes_lane_enable_params2.num_lanes; i++)
  {
      serdes_lane_enable_params2.loopback_mode[i] = CSL_SERDES_LOOPBACK_DISABLED;
      serdes_lane_enable_params2.lane_ctrl_rate[i] = CSL_SERDES_LANE_FULL_RATE; /* GEN2 */
  }

  //SB CMU and COMLANE Setup
  status1 = CSL_PCIeSerdesInit(serdes_lane_enable_params1.base_addr, serdes_lane_enable_params1.ref_clock, serdes_lane_enable_params1.linkrate);

  if (status1 != 0)
    PCIE_logPrintf ("Debug: Invalid PCIE 0 Serdes Init Params\n");


  status2 = CSL_PCIeSerdesInit(serdes_lane_enable_params2.base_addr, serdes_lane_enable_params2.ref_clock, serdes_lane_enable_params2.linkrate);

  if (status2 != 0)
    PCIE_logPrintf ("Debug: Invalid PCIE 1 Serdes Init Params\n");

  /* Common Init Mode */
  /* Iteration Mode needs to be set to Common Init Mode first with a lane_mask value equal to the total number of lanes being configured */
  /* For example, if there are a total of 2 lanes being configured, lane mask needs to be set to 0x3 */
  serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
  serdes_lane_enable_params1.lane_mask = 0x1;
  lane_retval1 = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);

  /* Lane Init Mode */
  /* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be initialized by setting
     iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT with the lane_mask equal to the specific lane being configured */
  /* For example, if lane 0 is being configured, lane mask needs to be set to 0x1. if lane 1 is being configured, lane mask needs to be 0x2 etc */
  serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
  for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
  {
      serdes_lane_enable_params1.lane_mask = 1<<i;
      lane_retval1 = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);
  }

  if (lane_retval1 != 0)
  {
      PCIE_logPrintf ("Invalid Serdes Lane Enable Init\n");
      exit(0);
  }

  /* Common Init Mode */
  /* Iteration Mode needs to be set to Common Init Mode first with a lane_mask value equal to the total number of lanes being configured */
  /* For example, if there are a total of 2 lanes being configured, lane mask needs to be set to 0x3 */
  serdes_lane_enable_params2.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
  serdes_lane_enable_params2.lane_mask = 0x1;
  lane_retval2 = CSL_SerdesLaneEnable(&serdes_lane_enable_params2);

  /* Lane Init Mode */
  /* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be initialized by setting
     iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT with the lane_mask equal to the specific lane being configured */
  /* For example, if lane 0 is being configured, lane mask needs to be set to 0x1. if lane 1 is being configured, lane mask needs to be 0x2 etc */
  serdes_lane_enable_params2.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
  for(i=0; i< serdes_lane_enable_params2.num_lanes; i++)
  {
      serdes_lane_enable_params2.lane_mask = 1<<i;
      lane_retval2 = CSL_SerdesLaneEnable(&serdes_lane_enable_params2);
  }

  if (lane_retval2 != 0)
  {
      PCIE_logPrintf ("Invalid Serdes Lane Enable Init\n");
      exit(0);
  }

  PCIE_logPrintf ("Debug: Serdes Setup Successfully\n");
#else
  /* Configure all lane of serdes with common config */
  CSL_SERDES_RESULT status;
  CSL_SERDES_LANE_ENABLE_STATUS lane_retval = CSL_SERDES_LANE_ENABLE_NO_ERR;
  CSL_SERDES_LANE_ENABLE_PARAMS_T serdes_lane_enable_params;

  memset(&serdes_lane_enable_params, 0, sizeof(serdes_lane_enable_params));

#if defined(DEVICE_K2L) || defined(SOC_K2L)
  /* Check CSISC2_3_MUXSEL bit */
  if (CSL_FEXTR(*(volatile uint32_t *)(CSL_BOOT_CFG_REGS + 0x20), 28, 28) != 1)
  {
      PCIE_logPrintf ("PCIe Serdes Mux Not Selected!\n");
      exit(1);
  }

  serdes_lane_enable_params.base_addr = CSL_CSISC2_3_SERDES_CFG_REGS;
  serdes_lane_enable_params.peripheral_base_addr = CSL_PCIE_0_SLV_CFG_REGS;
#elif defined(SOC_K2G)
   serdes_lane_enable_params.base_addr = CSL_PCIE_0_SERDES_CFG_REGS;
   serdes_lane_enable_params.peripheral_base_addr = CSL_PCIE_0_SLV_CFG_REGS;
#else
   serdes_lane_enable_params.base_addr = CSL_PCIE_SERDES_CFG_REGS;
   serdes_lane_enable_params.peripheral_base_addr = CSL_PCIE_SLV_CFG_REGS;
#endif

   serdes_lane_enable_params.ref_clock = CSL_SERDES_REF_CLOCK_100M;
   serdes_lane_enable_params.linkrate = CSL_SERDES_LINK_RATE_5G;
   serdes_lane_enable_params.num_lanes = 1;
   serdes_lane_enable_params.phy_type = SERDES_PCIe;
   serdes_lane_enable_params.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;
   serdes_lane_enable_params.lane_mask = 0x1;
   for(i=0; i< serdes_lane_enable_params.num_lanes; i++)
   {
       serdes_lane_enable_params.loopback_mode[i] = CSL_SERDES_LOOPBACK_DISABLED;
       serdes_lane_enable_params.lane_ctrl_rate[i] = CSL_SERDES_LANE_FULL_RATE; /* GEN2 */
   }

   status = CSL_PCIeSerdesInit(serdes_lane_enable_params.base_addr,
                               serdes_lane_enable_params.ref_clock,
                               serdes_lane_enable_params.linkrate);

   if (status != 0)
   {
       PCIE_logPrintf ("Invalid Serdes Init Params\n");
   }

   /* Common Init Mode */
   /* Iteration Mode needs to be set to Common Init Mode first with a lane_mask value equal to the total number of lanes being configured */
   /* For example, if there are a total of 2 lanes being configured, lane mask needs to be set to 0x3 */
   serdes_lane_enable_params.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
   serdes_lane_enable_params.lane_mask = 0x1;
   lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params);

   /* Lane Init Mode */
   /* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be initialized by setting
      iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT with the lane_mask equal to the specific lane being configured */
   /* For example, if lane 0 is being configured, lane mask needs to be set to 0x1. if lane 1 is being configured, lane mask needs to be 0x2 etc */
   serdes_lane_enable_params.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
   for(i=0; i< serdes_lane_enable_params.num_lanes; i++)
   {
       serdes_lane_enable_params.lane_mask = 1<<i;
       lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params);
   }

   if (lane_retval != 0)
   {
       PCIE_logPrintf ("Invalid Serdes Lane Enable Init\n");
       exit(0);
   }

   PCIE_logPrintf ("Debug: Serdes Setup Successfully\n");
  #endif
#endif
#endif
#endif
  /*Wait for PLL to lock (3000 CLKIN1 cycles) */
  cycleDelay(10000);
  
  return pcie_RET_OK;
}

/*****************************************************************************
 * Function: Enable/Disable LTSSM (Link Training)
 * This function demonstrates how one can write one binary to use either
 * rev of PCIE
 ****************************************************************************/
pcieRet_e pcieLtssmCtrl(Pcie_Handle handle, uint8_t enable)
{
  pcieCmdStatusReg_t       cmdStatus;
  pcieTiConfDeviceCmdReg_t deviceCmd;
  pcieRegisters_t          regs;
  pcieRet_e retVal;

  memset (&cmdStatus,    0, sizeof(cmdStatus));
  memset (&deviceCmd,    0, sizeof(deviceCmd));
  memset (&regs,         0, sizeof(regs));

  regs.cmdStatus = &cmdStatus;
  if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs)) != pcie_RET_OK)
  {
    if (retVal == pcie_RET_INV_REG)
    {
      /* The cmdStatus register doesn't exist; try the deviceCmd instead */
      regs.cmdStatus       = NULL;
      regs.tiConfDeviceCmd = &deviceCmd;
      if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs)) != pcie_RET_OK)
      {
        PCIE_logPrintf ("Read CMD STATUS and DEVICE CMD registers failed!\n");
        return retVal;
      }
    }
    else
    {
      PCIE_logPrintf ("Read CMD STATUS register failed!\n");
      return retVal;
    }
  }
  
  if(enable)
    deviceCmd.ltssmEn = cmdStatus.ltssmEn = 1;
  else  
    deviceCmd.ltssmEn = cmdStatus.ltssmEn = 0;

  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &regs)) != pcie_RET_OK)
  {
    PCIE_logPrintf ("SET CMD STATUS register failed!\n");
    return retVal;
  }

  return pcie_RET_OK;
}

/*****************************************************************************
 * Function: Configure PCIe in Gen1 vs Gen2 mode
 ****************************************************************************/
pcieRet_e pcieSetGen2(Pcie_Handle handle)
{
  pcieRet_e              retVal;

  pcieRegisters_t        regs;
  pcieLinkCapReg_t       linkCap;
  pcieGen2Reg_t          gen2;

  uint8_t                targetGen, dirSpd;

#ifdef GEN2
  targetGen = 2;
  dirSpd    = 1;
#else
  targetGen = 1;
  dirSpd    = 0;
#endif

  memset (&gen2,             0, sizeof(gen2));
  memset (&linkCap,          0, sizeof(linkCap));
  memset (&regs,             0, sizeof(regs));

  /* Set gen1/gen2 in link cap */
  regs.linkCap = &linkCap;
  if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs)) != pcie_RET_OK)
  {
    PCIE_logPrintf ("GET linkCap register failed!\n");
    return retVal;
  }

  if (linkCap.maxLinkSpeed != targetGen)
  {
    PCIE_logPrintf ("PowerUP linkCap gen=%d change to %d\n", linkCap.maxLinkSpeed, targetGen);
    linkCap.maxLinkSpeed = targetGen;
  }
  else
  {
    regs.linkCap = NULL; /* Nothing to write back */
  }

  /* Setting PL_GEN2 */
  gen2.numFts = 0xF;
  gen2.dirSpd = dirSpd;
  gen2.lnEn   = 1;
#ifdef PCIESS1_X2
  gen2.lnEn = 2;
#endif
  regs.gen2 = &gen2;

  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &regs)) != pcie_RET_OK)
  {
    PCIE_logPrintf ("SET GEN2/link cap register failed!\n");
    return retVal;
  }

  return retVal;
}

/*****************************************************************************
 * Function: Configure PCIe in Root Complex Mode
 ****************************************************************************/
pcieRet_e pcieCfgRC(Pcie_Handle handle)
{
  pcieRet_e retVal;

  pcieObSizeReg_t        obSize;
  pcieType1Bar32bitIdx_t type1Bar32bitIdx;
  pcieStatusCmdReg_t     statusCmd;
  pcieDevStatCtrlReg_t   devStatCtrl;
  pcieAccrReg_t          accr;
                 
  pcieRegisters_t        setRegs;
  pcieRegisters_t        getRegs;

  memset (&obSize,           0, sizeof(obSize));
  memset (&type1Bar32bitIdx, 0, sizeof(type1Bar32bitIdx));
  memset (&statusCmd,        0, sizeof(statusCmd));
  memset (&devStatCtrl,      0, sizeof(devStatCtrl));
  memset (&accr,             0, sizeof(accr));

#if 0
  /*Disable link training*/
  if ((retVal = pcieLtssmCtrl(handle, FALSE)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("Failed to disable Link Training!\n");
    return retVal;
  }
#endif

  /* Configure the size of the translation regions */   
  memset (&setRegs, 0, sizeof(setRegs));
  memset (&getRegs, 0, sizeof(getRegs));
  
#ifdef PCIE_REV0_HW
  /* Only required for v0 hw */
  obSize.size = pcie_OB_SIZE_8MB;
  setRegs.obSize = &obSize;
  
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET OB_SIZE register failed!\n");
    return retVal;
  }
#endif

  /* Set gen2/link cap */
  if ((retVal = pcieSetGen2(handle)) != pcie_RET_OK)
  {
    PCIE_logPrintf ("pcieSetGen2 failed!\n");
    return retVal;
  }
  
  /* Configure BAR Masks */   
  /* First need to enable writing on BAR mask registers */
  if ((retVal = pcieCfgDbi (handle, 1)) != pcie_RET_OK)
  {
    return retVal;
  }
  
  /* Configure Masks*/
  memset (&setRegs, 0, sizeof(setRegs));
  memset (&getRegs, 0, sizeof(getRegs));

  type1Bar32bitIdx.reg.reg32 = PCIE_BAR_MASK;
  setRegs.type1BarMask32bitIdx = &type1Bar32bitIdx;

  /* BAR 0 */
  type1Bar32bitIdx.idx = 0; /* configure BAR 0*/
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET BAR MASK register failed!\n");
    return retVal;
  }
  
  /* BAR 1 */
  type1Bar32bitIdx.idx = 1; /* configure BAR 1*/
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET BAR MASK register failed!\n");
    return retVal;
  }
  
  /* Disable writing on BAR Masks */
  if ((retVal = pcieCfgDbi (handle, 0)) != pcie_RET_OK)
  {
    return retVal;
  }

  /* Enable memory access and mastership of the bus */
  memset (&setRegs, 0, sizeof(setRegs));
  memset (&getRegs, 0, sizeof(getRegs));

  getRegs.statusCmd = &statusCmd;
  if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &getRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("Read Status Comand register failed!\n");
    return retVal;
  }
  statusCmd.memSp  = 1;
  statusCmd.busMs  = 1;
  statusCmd.resp   = 1;
  statusCmd.serrEn = 1;
  setRegs.statusCmd = &statusCmd;   
  
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET Status Command register failed!\n");
    return retVal;
  }

  /* Enable Error Reporting */
  memset (&setRegs, 0, sizeof(setRegs));
  memset (&getRegs, 0, sizeof(getRegs));

  getRegs.devStatCtrl = &devStatCtrl;
  if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &getRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("Regad Device Status Control register failed!\n");
    return retVal;
  }
 
  devStatCtrl.reqRp = 1;
  devStatCtrl.fatalErRp = 1;
  devStatCtrl.nFatalErRp = 1;
  devStatCtrl.corErRp = 1;
  setRegs.devStatCtrl = &devStatCtrl;   
  
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET Device Status Control register failed!\n");
    return retVal;
  }

#ifdef PCIE_REV0_HW
  /* Enable ECRC */
  memset (&setRegs, 0, sizeof(setRegs));
  
  accr.chkEn=1;
  accr.chkCap=1;
  accr.genEn=1;
  accr.genCap=1;
  setRegs.accr = &accr;

  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET ACCR register failed!\n");
    return retVal;
  }
#endif

  return pcie_RET_OK;
}

/*****************************************************************************
 * Function: Configure and enable Outbound Address Translation for rev 1
 ****************************************************************************/
pcieRet_e pcieObTransCfg(Pcie_Handle handle, uint32_t obAddrLo, uint32_t obAddrHi, uint8_t region)
{
  pcieAtuRegionParams_t regionParams;
  pcieRet_e             retVal;
  uint32_t              resSize;

  if ((retVal = Pcie_getMemSpaceReserved (handle, &resSize)) != pcie_RET_OK) {
    PCIE_logPrintf ("getMemSpaceReserved failed (%d)\n", (int)retVal);
    return retVal;
  }

  PCIE_logPrintf ("pcieObTransCfg resSize (0x%x)\n", resSize);

  if(PcieModeGbl == pcie_RC_MODE)
  {
    /*Configure OB region for remote configuration access space*/
    regionParams.regionDir    = PCIE_ATU_REGION_DIR_OUTBOUND;
    regionParams.tlpType      = PCIE_TLP_TYPE_CFG;
    regionParams.enableRegion = 1;

    regionParams.lowerBaseAddr    = PCIE_WINDOW_CFG_BASE + resSize;
    regionParams.upperBaseAddr    = 0; /* only 32 bits needed given data area size */
    regionParams.regionWindowSize = PCIE_WINDOW_CFG_MASK;

    regionParams.lowerTargetAddr = 0U;
    regionParams.upperTargetAddr = 0U;

    if ( (retVal = Pcie_atuRegionConfig(
                     handle,
                     pcie_LOCATION_LOCAL,
                     (uint32_t) 0U,
                     &regionParams)) != pcie_RET_OK) 
    {
      return retVal;
    }
  }

  /*Configure OB region for memory transfer*/
  regionParams.regionDir    = PCIE_ATU_REGION_DIR_OUTBOUND;
  regionParams.tlpType      = PCIE_TLP_TYPE_MEM;
  regionParams.enableRegion = 1;

  regionParams.lowerBaseAddr    = PCIE_WINDOW_MEM_BASE + resSize;
  regionParams.upperBaseAddr    = 0; /* only 32 bits needed given data area size */
  regionParams.regionWindowSize = PCIE_WINDOW_MEM_MASK;

  regionParams.lowerTargetAddr = obAddrLo;
  regionParams.upperTargetAddr = obAddrHi;

  return Pcie_atuRegionConfig(
           handle,
           pcie_LOCATION_LOCAL,
           (uint32_t) 1U,
           &regionParams);
}

/*****************************************************************************
 * Function: Configure and enable Inbound Address Translation for rev 1
 ****************************************************************************/
pcieRet_e pcieIbTransCfg(Pcie_Handle handle, pcieIbTransCfg_t *ibCfg)
{
  pcieAtuRegionParams_t regionParams;

  /*Configure IB region for memory transfer*/
  regionParams.regionDir    = PCIE_ATU_REGION_DIR_INBOUND;
  regionParams.tlpType      = PCIE_TLP_TYPE_MEM;
  regionParams.enableRegion = 1;
  regionParams.matchMode    = PCIE_ATU_REGION_MATCH_MODE_ADDR;

  regionParams.lowerBaseAddr    = ibCfg->ibStartAddrLo;
  regionParams.upperBaseAddr    = ibCfg->ibStartAddrHi;
  regionParams.regionWindowSize = PCIE_INBOUND_MASK;

  /* This aligns the buffer to 4K, which needs to be compensated by the application */
  regionParams.lowerTargetAddr = (ibCfg->ibOffsetAddr & ~0xfffU) ;
  regionParams.upperTargetAddr = 0;

  return Pcie_atuRegionConfig(
           handle,
           pcie_LOCATION_LOCAL,
           (uint32_t) 0U,
           &regionParams);
}


/*****************************************************************************
 * Function: Initialize application buffers
 ****************************************************************************/
void pcieInitAppBuf(void)
{
  uint32_t i;
  
  for (i=0; i<PCIE_BUFSIZE_APP; i++)
  {
    dstBuf.buf[i] = 0;
    srcBuf[i] = i;
  }
  
  dstBuf.buf[PCIE_BUFSIZE_APP] = PCIE_EXAMPLE_BUF_EMPTY;
  cache_writeback ((void *)dstBuf.buf, PCIE_EXAMPLE_DSTBUF_BYTES);
  
#ifdef EDMA
  for (i = 0; i < PCIE_EXAMPLE_LINE_SIZE - 1U; i++) 
  {
    dstBuf.edma_buf[i] = 0;
  }
  dstBuf.edma_buf[PCIE_EXAMPLE_LINE_SIZE - 1U] = PCIE_EXAMPLE_BUF_EMPTY;
  cache_writeback ((void *)dstBuf.edma_buf, PCIE_EDMA_EXAMPLE_DSTBUF_BYTES);
#endif
}

/*****************************************************************************
 * Function: Check LTSSM status and wait for the link to be up
 ****************************************************************************/
void pcieWaitLinkUp(Pcie_Handle handle)
{
  pcieRegisters_t  getRegs;

  memset (&getRegs, 0, sizeof(getRegs));

#ifdef PCIE_REV0_HW
  pcieDebug0Reg_t            ltssmStateReg;
  getRegs.debug0 =          &ltssmStateReg;
#else
  pcieTiConfDeviceCmdReg_t   ltssmStateReg;
  getRegs.tiConfDeviceCmd = &ltssmStateReg;
#endif
  
  memset (&ltssmStateReg,  0, sizeof(ltssmStateReg));
  
  uint8_t ltssmState = 0;
 
  while(ltssmState != pcie_LTSSM_L0)
  {
    cycleDelay(100);
    if (Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &getRegs) != pcie_RET_OK) 
    {
      PCIE_logPrintf ("Read LTSSM state failed!\n");
      return;
    }
    ltssmState = ltssmStateReg.ltssmState;
  }
}

pcieRet_e pcieCheckLinkParams(Pcie_Handle handle)
{
  pcieRet_e retVal = pcie_RET_OK;
  pcieRegisters_t regs;
  pcieLinkStatCtrlReg_t linkStatCtrl;
  int32_t expLanes = 1, expSpeed = 1;
  const char *pass = "PASS", *fail = "FAIL";
  const char *result = pass;

#ifdef GEN2
  expSpeed = 2;
#endif
#ifdef PCIESS1_X2
  expLanes = 2;
#endif
  /* Get link status */
  memset (&regs, 0, sizeof(regs));
  regs.linkStatCtrl = &linkStatCtrl;

  PCIE_logPrintf ("Checking link speed and # of lanes\n");
  retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs);
  if (retVal != pcie_RET_OK) {
     PCIE_logPrintf ("Failed to read linkStatCtrl: %d\n", retVal);
  } else {
    /* Check number of lanes */
    if (expLanes != linkStatCtrl.negotiatedLinkWd) {
       result = fail;
       retVal = pcie_RET_UNSUPPORTED;
    } else {
       result = pass;
    }
    PCIE_logPrintf ("Expect %d lanes, found %d lanes (%s)\n",
                    (int)expLanes, (int)linkStatCtrl.negotiatedLinkWd, result);

    /* Check speed */
    if (expSpeed != linkStatCtrl.linkSpeed) {
       result = fail;
       retVal = pcie_RET_UNSUPPORTED;
    } else {
       result = pass;
    }
    PCIE_logPrintf ("Expect gen %d speed, found gen %d speed (%s)\n",
                    (int)expSpeed, (int)linkStatCtrl.linkSpeed, result);
  }

  return retVal;
}


#ifdef PCIE_REV1_HW
uint32_t count = 0;
Void ClkMain(UArg arg)
{
    count++;
    PCIE_logPrintf ("ClkMain: count: %d\n", count);
}

void pcieRcWaitInts (Pcie_Handle handle, SemaphoreP_Handle sem, void *pcieBase)
{
  int32_t numInts = 0;
  volatile unsigned long long start_time_val;
  volatile unsigned long long end_time_val;
  uint8_t i = 0;


  PCIE_logPrintf ("RC waiting for %d interrupts\n", totalInts);
  //while (numInts < 2*PCIE_NUM_INTS)
  //while (numInts < totalInts)
  while (1)
  {
    /* Wait on the semaphore */
    SemaphoreP_pend (sem, SemaphoreP_WAIT_FOREVER);

    /* Count it */
    numInts ++;
  }

  PCIE_logPrintf ("RC got all %d interrupts\n", (int)totalInts);
  for (i = 0; i < 32; i++)
      PCIE_logPrintf ("MSI %d count: %d\n", i, msi_index_count[i]);
}
#endif

void* malloc_aligned(size_t required_bytes, size_t alignment){
    void* p1;
    void** p2;
    int offset = alignment - 1 + sizeof(void*);
    if ((p1 = (void*)malloc(required_bytes + offset)) == NULL){
        return NULL;
    }
    p2 = (void**)(((size_t)(p1)+offset) & ~(alignment - 1));
    p2[-1] = p1;
    return p2;
}

void free_aligned(void *p2){
    void* p1 = ((void**)p2)[-1];
    free(p1);
}

extern uint32_t isr_flag;
extern IHeap_Handle systemHeap;

uint32_t *send_buf = NULL;
uint32_t *recvice_buf = NULL;

uint32_t *send_buf_address = NULL;
uint32_t *recvice_buf_address = NULL;
uint32_t *h2c_desc_address = NULL;
uint32_t *c2h_desc_address = NULL;

void pcie_set_dma_status(uint32_t *pcie_bar1_base_address)
{
    *(uint32_t *) (pcie_bar1_base_address + 0x90/4) = 0x00F83E1E;
    *(uint32_t *) (pcie_bar1_base_address + 0x1090/4) = 0x00F83E1E;
    *(uint32_t *) (pcie_bar1_base_address + 0x2004/4) = 0x1;
    *(uint32_t *) (pcie_bar1_base_address + 0x2010/4) = 0x3;
}

void buffer_init(uint32_t *send_buf,uint32_t *recvice_buf,uint32_t len)
{
    uint32_t i = 0;
    for (i = 0; i < len; i++){
        send_buf[i] = 0xabcdef00 + i;
        recvice_buf[i] = 0x00000000;
    }
}

void H2C_clear_status(void)
{
    uint32_t engine_int_req = 0;
    uint32_t val = 0;

    engine_int_req = *(uint32_t *) (pcie_bar1_base_address + 0x2044/4);
    *(uint32_t *) (pcie_bar1_base_address + 0x2018/4) = engine_int_req;
    val = *(uint32_t *) (pcie_bar1_base_address + 0x0044/4);
    val = *(uint32_t *) (pcie_bar1_base_address + 0x0048/4);
    *(uint32_t *) (pcie_bar1_base_address + 0x0004/4) = 0x00f83e1e;
    *(uint32_t *) (pcie_bar1_base_address + 0x2014/4) = 0x3;
}

void C2H_clear_status(void)
{
    uint32_t engine_int_req = 0;
    uint32_t val = 0;

    engine_int_req = *(uint32_t *) (pcie_bar1_base_address + 0x2044/4);
    *(uint32_t *) (pcie_bar1_base_address + 0x2018/4) = engine_int_req;
    val = *(uint32_t *) (pcie_bar1_base_address + 0x1044/4);
    val = *(uint32_t *) (pcie_bar1_base_address + 0x1048/4);
    *(uint32_t *) (pcie_bar1_base_address + 0x1004/4) = 0x00f83e1e;
    *(uint32_t *) (pcie_bar1_base_address + 0x2014/4) = 0x3;
}

void start_DMA_write(void)
{
    /* start H2C engines, start DMA write */
    *(uint32_t *) (pcie_bar1_base_address + 0x0004/4) = 0x00f83e1f;
}

void start_DMA_read(void)
{
    /* start C2H engines, start DMA read */
    *(uint32_t *) (pcie_bar1_base_address + 0x1004/4) = 0x00f83e1f;
}

/*
*	======== Server_exec ========
*/
Int Server_exec()
{
    Int                 status;
    Bool                running = TRUE;
    App_Msg *           msg;
    MessageQ_QueueId    queId;

    Log_print0(Diags_ENTRY | Diags_INFO, "--> Server_exec:");

	while (running) {

	/* wait for inbound message */
	status = MessageQ_get(Module.slaveQue, (MessageQ_Msg *)&msg,MessageQ_FOREVER);
	if (status < 0) {
		goto leave;
	}

	if (msg->cmd == App_CMD_SHUTDOWN){
		running = FALSE;
	}
	else if(msg->cmd == App_CMD_SETUP){
		send_buf_address = msg->send_buf_address;
		recvice_buf_address = msg->recvice_buf_address;
		h2c_desc_address = msg->h2c_desc_address;
		c2h_desc_address = msg->c2h_desc_address;
	}
	else if (msg->cmd == APP_CMD_TRANSFER) {
#ifdef _TMS320C6X 
	  TSCL = 1;
#endif
	  pcieRet_e        retVal;
	  pcieIbTransCfg_t ibCfg;
	  pcieBarCfg_t     barCfg;
	  Pcie_Handle      handle = NULL;
	  void            *pcieBase;
	  dstBuf_t        *pciedstBufBase;
	  uint32_t         i;
#ifdef PCIE_REV1_HW
	  SemaphoreP_Handle sem = NULL;
#endif

	  /* Get remote buffer out of cache */
	  cache_writeback ((void *)&dstBuf, sizeof(dstBuf));

#ifdef EDMA
	  EDMA3_DRV_Handle hEdma = NULL;
	  hEdma = edmaInit(hEdma);
	  if (hEdma==NULL) PCIE_logPrintf("ERROR: EDMA handle not initialized!\n");
#endif

#ifndef IO_CONSOLE
	  //Console_printf ("IO_CONSOLE not defined.  Most output will go to UART\n");
#endif

	  PCIE_logPrintf ("**********************************************\n");
	  PCIE_logPrintf ("*             PCIe Test Start                *\n");
	  
	  PCIE_logPrintf ("*                RC mode                     *\n");
	  
	  PCIE_logPrintf ("**********************************************\n\n");
	  
	  PCIE_logPrintf ("Version #: 0x%08x; string %s\n\n", (unsigned)Pcie_getVersion(), Pcie_getVersionStr());
	  

	  /* Pass device config to LLD */
	  if ((retVal = Pcie_init (&pcieInitCfg)) != pcie_RET_OK)
	  {
	    PCIE_logPrintf ("LLD device configuration failed\n");
	    exit(1);
	  }

	  /* Initialize application buffers */
	  pcieInitAppBuf();

	  /* Power up PCIe Module */
	  if ((retVal = pciePowerCfg()) != pcie_RET_OK) {
	    PCIE_logPrintf ("PCIe Power Up failed (%d)\n", (int)retVal);
	    exit(1);
	  }

	  PCIE_logPrintf ("PCIe Power Up.\n");

	  if ((retVal = Pcie_open(0, &handle)) != pcie_RET_OK)
	  {
	    PCIE_logPrintf ("Open failed (%d)\n", (int)retVal);
	    exit(1);
	  }

	  /* Configure SERDES*/
	  if ((retVal = pcieSerdesCfg()) != pcie_RET_OK) {
	    PCIE_logPrintf ("PCIe Serdes config failed (%d)\n", (int)retVal);
	    exit(1);
	  }

	  /* Set the PCIe mode*/
	  if ((retVal = Pcie_setInterfaceMode(handle, PcieModeGbl)) != pcie_RET_OK) {
	    PCIE_logPrintf ("Set PCIe Mode failed (%d)\n", (int)retVal);
	    exit(1);
	  }

	  PCIE_logPrintf ("PLL configured.\n");

	  if(PcieModeGbl == pcie_RC_MODE)
	  {
	    /* Configure application registers for Root Complex*/
	    if ((retVal = pcieCfgRC(handle)) != pcie_RET_OK) 
	    {
	      PCIE_logPrintf ("Failed to configure PCIe in RC mode (%d)\n", (int)retVal);
	      exit(1);
	    }

	    /* Configure Address Translation */
	    
	    barCfg.location = pcie_LOCATION_LOCAL;
	    barCfg.mode     = pcie_RC_MODE;
	    barCfg.base     = PCIE_IB_LO_ADDR_RC;
	    barCfg.prefetch = pcie_BAR_NON_PREF;
	    barCfg.type     = pcie_BAR_TYPE32;
	    barCfg.memSpace = pcie_BAR_MEM_MEM;
	    barCfg.idx      = PCIE_BAR_IDX_RC;
	    
	    if ((retVal = Pcie_cfgBar(handle, &barCfg)) != pcie_RET_OK) 
	    {
	      PCIE_logPrintf ("Failed to configure BAR (%d)\n", (int)retVal);
	      exit(1);
	    }


	    ibCfg.ibBar         = PCIE_BAR_IDX_RC; /* Match BAR that was configured above*/
	    ibCfg.ibStartAddrLo = PCIE_IB_LO_ADDR_RC;
	    ibCfg.ibStartAddrHi = PCIE_IB_HI_ADDR_RC;
	    ibCfg.ibOffsetAddr  = (uint32_t)pcieConvert_CoreLocal2GlobalAddr ((uint32_t)dstBuf.buf);
	    ibCfg.region        = PCIE_IB_REGION_RC;       

	    if ((retVal = pcieIbTransCfg(handle, &ibCfg)) != pcie_RET_OK) 
	    {
	      PCIE_logPrintf ("Failed to configure Inbound Translation (%d)\n", (int)retVal);
	      exit(1);
	    }
	    else
	    {
	      PCIE_logPrintf ("Successfully configured Inbound Translation!\n");
	    }

	    if ((retVal = pcieObTransCfg (handle, PCIE_OB_LO_ADDR_RC, PCIE_OB_HI_ADDR_RC, PCIE_OB_REGION_RC)) != pcie_RET_OK) 
	    {
	      PCIE_logPrintf ("Failed to configure Outbound Address Translation (%d)\n", (int)retVal);
	      exit(1);
	    }
	    else
	    {
	      PCIE_logPrintf ("Successfully configured Outbound Translation!\n");
	    }
	  }

	  PCIE_logPrintf ("Starting link training...\n");

	  /*Enable link training*/
	  if ((retVal = pcieLtssmCtrl(handle, TRUE)) != pcie_RET_OK) 
	  {
	    PCIE_logPrintf ("Failed to Enable Link Training! (%d)\n", (int)retVal);
	    exit(1);
	  }

	  /* Wait for link to be up */
	  pcieWaitLinkUp(handle);

	  PCIE_logPrintf ("Link is up.\n");
	  if ((retVal = pcieCheckLinkParams(handle)) != pcie_RET_OK)
	  {
	    PCIE_logPrintf ("Link width/speed verification FAILed: %d\n", retVal);
	    /* This exit() can be removed if this example is being used as
	     * template with non TI card that supports slower or narrower connections
	     */
	    exit(1);
	  }

	  if ((retVal = Pcie_getMemSpaceRange (handle, &pcieBase, NULL)) != pcie_RET_OK) {
	    PCIE_logPrintf ("getMemSpaceRange failed (%d)\n", (int)retVal);
	    exit(1);
	  }
	  
#ifdef PCIE_REV1_HW
	  /* Adjust PCIE base to point at remote target buffer */
	  pcieBase = (char *)pcieBase + 
	                     PCIE_WINDOW_MEM_BASE +  /* data area doesn't start at low address */
	                     (((uint32_t)&dstBuf) & 0xfff); /* dstBuf needs to be 4K aligned in addr tran */
#endif
	  pciedstBufBase = (dstBuf_t *)pcieBase;  // 0x21000000  BAR1 base address
	  pcie_bar1_base_address = (uint32_t *)pcieBase;

	  pcieVndDevIdReg_t idregs;
	  pcieType0BarIdx_t barregs;
	  pcieRegisters_t   regs;
	  idregs.devId = 0;
	  idregs.raw = 0;
	  idregs.vndId = 0;

	  memset(&regs, 0, sizeof(pcieRegisters_t));
	  regs.vndDevId = &idregs;

	  if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_REMOTE, &regs)) != pcie_RET_OK)
	  {
	       PCIE_logPrintf ("Read DEVICE_ID failed!\n");
	       exit(1);
	   }

	   regs.type0BarIdx = &barregs;
	   for (i = 0; i < 2; i++)
	   {
	      barregs.idx = i;
	      if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_REMOTE, &regs)) != pcie_RET_OK)
	      {
	          PCIE_logPrintf("Read BAR %d error!\n", i);
	          exit(1);
	      }
	   }

	   regs.type0BarIdx->reg.base = 0xFFF0000;
	   for (i = 0; i < 2; i++)
	   {
	       barregs.idx = i;
	       if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_REMOTE, &regs)) != pcie_RET_OK)
	       {
	           PCIE_logPrintf("Write BAR %d error!\n", i);
	           exit(1);
	        }

	   }

	   pcieStatusCmdReg_t statuscmd;
	   regs.statusCmd = &statuscmd;
	   for (i = 0; i < 2; i++)
	   {
	       barregs.idx = i;
	       if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_REMOTE, &regs)) != pcie_RET_OK)
	       {
	           PCIE_logPrintf("Read BAR %d error!\n", i);
	           exit(1);
	       }
	   }

	   // WRITE BAR 0 ADDRESS and ENABLE MEM SPACE DECODING / BUS MASTER
	   barregs.idx = 0;
	   regs.statusCmd->memSp = 1;
	   regs.statusCmd->busMs = 1;
	   regs.type0BarIdx->reg.base = PCIE_OB_LO_ADDR_RC / 16;
	   if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_REMOTE, &regs)) != pcie_RET_OK)
	   {
	       PCIE_logPrintf("Write type0BarIdx->reg.base failed!\n");
	       exit(1);
	   }

	    // WRITE BAR 1 ADDRESS
	    barregs.idx = 1;
	    regs.type0BarIdx->reg.base = PCIE_OB_HI_ADDR_RC / 16;
	    if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_REMOTE, &regs)) != pcie_RET_OK)
	    {
	        PCIE_logPrintf("Write type0BarIdx->reg.base failed!\n");
	        exit(1);
	    }

	    for ( i = 0; i < 2; i++)
	    {
	        barregs.idx = i;
	        if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_REMOTE, &regs)) != pcie_RET_OK)
	        {
	            PCIE_logPrintf("Read BAR%d failed!\n", i);
	            exit(1);
	        }
	    }

	  if(PcieModeGbl == pcie_RC_MODE)
	  {
#ifdef PCIE_REV1_HW
	    sem = PlatformSetupMSIAndINTX (handle);
#endif

	    uint32_t frep = 750000000;          //DSP frequency is 750MHz
	    uint32_t kb_val = 4;
	    uint32_t len = 1024 * kb_val / 4; //（32位数据是4个字节所以要除以4）
	    uint32_t error_count = 0;
	    int32_t loop_count = 0;

	    struct xdma_desc * xdma_h2c_desc_ptr_pa;
	    struct xdma_desc * xdma_h2c_desc_ptr_da;
	    struct xdma_desc * xdma_c2h_desc_ptr_pa;
	    struct xdma_desc * xdma_c2h_desc_ptr_da;
	    uint64_t         speed_sum_w = 0;
	    uint64_t         speed_sum_r = 0;

	    unsigned long long t1, t2;
	    unsigned long long cycles, speed;

	    pcie_set_dma_status(pcie_bar1_base_address);

	    while (1)
	    {
	    send_buf = send_buf_address;
	    msg->send_buf = send_buf;
	    recvice_buf = recvice_buf_address;
	    msg->recvice_buf = recvice_buf;
	    loop_count = 0;

	    while (loop_count++ < 64)
	    {
	        uint8_t wait_for_irq = 0;
	        cache_invalidate ((void *)send_buf, len * 4 * 4);
	        cache_invalidate ((void *)recvice_buf, len * 4 * 4);

	        buffer_init(send_buf,recvice_buf,len);

	        /************************************* host to card set up ******************************************/
		if((loop_count % 16) == 0){
		PCIE_logPrintf("*******************************Start PCIE DMA Write Test************************\n");
		}
		xdma_h2c_desc_ptr = h2c_desc_address;
	        xdma_h2c_desc_ptr_da = xdma_h2c_desc_ptr;
	        if (Resource_virtToPhys((UInt32)xdma_h2c_desc_ptr_da, &xdma_h2c_desc_ptr_pa) !=
	                        Resource_S_SUCCESS) {
	                PCIE_logPrintf("Resource_virtToPhys: Failed to translate buffer address");
	        }
	        xdma_desc_set(xdma_h2c_desc_ptr, (uint64_t)send_buf, 0x0, len * 4, 1);

	        *(uint32_t *) (pcie_bar1_base_address + 0x4080/4) = PCI_DMA_L((uint64_t)xdma_h2c_desc_ptr_pa);      //将描述符低8位地址写入BAR空间的0x4080
	        *(uint32_t *) (pcie_bar1_base_address + 0x4084/4) = PCI_DMA_H((uint64_t)xdma_h2c_desc_ptr_pa);      //将描述符的高8位地址写入BAR空间的0x4084

	        TSCH = 0;
	        TSCL = 0;
	        t1 = _itoll(TSCH, TSCL);

	        start_DMA_write();

	        wait_for_irq = 0;

	        do
	        {
	            if (isr_flag)
	            {
	                t2 = _itoll(TSCH, TSCL);
	                cycles = t2 - t1;
	                speed = (unsigned long long)len * 4 * 8 * frep / 1024 / 1024 / cycles;
	                speed_sum_w += speed;

	                isr_flag = 0;
	                wait_for_irq = 1;

	                H2C_clear_status();
	           }
	       } while(wait_for_irq == 0);
	       if((loop_count % 16) == 0){
	       PCIE_logPrintf("pcie dma write is done!\n");
	       PCIE_logPrintf("pcie dma write speed: %dMB/s\n",speed/8);
	       PCIE_logPrintf("*******************************End PCIE DMA Write Test**************************\n\n");
	       }
	       /************************************* end of host to card set up ************************************/


	       /************************************* card to host set up *******************************************/
	       if((loop_count % 16) == 0){
	       PCIE_logPrintf("*******************************Start PCIE DMA Read Test**************************\n");
	       }
	       xdma_c2h_desc_ptr = c2h_desc_address;
	       xdma_c2h_desc_ptr_da = xdma_c2h_desc_ptr;
	       if (Resource_virtToPhys((UInt32)xdma_c2h_desc_ptr_da, &xdma_c2h_desc_ptr_pa) !=
	                       Resource_S_SUCCESS) {
	               PCIE_logPrintf("Server_exec: Failed to translate buffer address");
	       }
	       xdma_desc_set(xdma_c2h_desc_ptr, (uint64_t)recvice_buf, 0x0, len * 4, 0);

	       *(uint32_t *) (pcie_bar1_base_address + 0x5080/4) = PCI_DMA_L((uint64_t)xdma_c2h_desc_ptr_pa);
	       *(uint32_t *) (pcie_bar1_base_address + 0x5084/4) = PCI_DMA_H((uint64_t)xdma_c2h_desc_ptr_pa);

	       TSCH = 0;
	       TSCL = 0;
	       t1 = _itoll(TSCH, TSCL);

	       start_DMA_read();

	       wait_for_irq = 0;
	       do
	       {
	           if (isr_flag)
	           {
	               t2 = _itoll(TSCH, TSCL);
	               cycles = t2 - t1;
	               speed = (unsigned long long)len * 4 * 8 * frep / 1024 / 1024 / cycles;
	               speed_sum_r += speed;

	               isr_flag = 0;
	               wait_for_irq = 1;

	               C2H_clear_status();
	           }
	       } while(wait_for_irq == 0);

	       for (i = 0; i < len; i++){
	       		if (recvice_buf[i] != send_buf[i]){
				error_count += 1;
				PCIE_logPrintf ("data error: send_buf[%d]: 0x%x recvice_buf[%d]: 0x%x\n", i, send_buf[i], i, recvice_buf[i]);
			}
	       }

	       if((loop_count % 16) == 0){
	       PCIE_logPrintf("pcie dma read is done!\n");
	       PCIE_logPrintf("pcie dma read speed: %dMB/s\n",speed/8);
	       PCIE_logPrintf("data_error_count: %d\n",error_count);
	       PCIE_logPrintf("*******************************End PCIE DMA Read Test**************************\n\n\n");
	       }
	       /************************************* end of card to host set up *******************************************/
	           send_buf += len;
	           recvice_buf += len;
	       }

	       PCIE_logPrintf ("=================== average ======================\n");
	       PCIE_logPrintf ("average write speed: %ldMB/s\n",speed_sum_w / (loop_count - 1) / 8);
	       PCIE_logPrintf ("average read speed: %ldMB/s\n",speed_sum_r / (loop_count - 1) / 8);
	       PCIE_logPrintf ("total byte size: %d KB\n", len * 4 * (loop_count - 1));
	       PCIE_logPrintf ("data type: uint32_t error_count: %d\n", error_count);
	       speed_sum_w = 0;
	       speed_sum_r = 0;
	       error_count = 0;
	       msg->buf_size = len * 4 * 1 *(loop_count - 1);
	       queId = MessageQ_getReplyQueue(msg); /* type-cast not needed */
	       MessageQ_put(queId, (MessageQ_Msg)msg);
	       exit(0);
	   }
	}
	}
	/* process the message */
	Log_print1(Diags_INFO, "Server_exec: processed cmd=0x%x", msg->cmd);

	/* send message back */
	queId = MessageQ_getReplyQueue(msg); /* type-cast not needed */
	MessageQ_put(queId, (MessageQ_Msg)msg);
}
leave:
	Log_print1(Diags_EXIT, "<-- Server_exec: %d", (IArg)status);
	return(status);
}

