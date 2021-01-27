/*
 * Copyright (C) 2009-2016 Texas Instruments Incorporated - http://www.ti.com/
 *
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

/* xdctools header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Registry.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/hal/Hwi.h>

/* package header files */
#include <ti/ipc/SharedRegion.h>
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/remoteproc/Resource.h>

#include <ti/osal/HwiP.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/csl_edma.h>
#include <ti/csl/arch/c66x/interrupt.h>
#include <ti/csl/soc/am572x/src/cslr_soc_dsp_baseaddress.h>
#include <ti/csl/src/ip/edma/V1/edma.h>
#include <ti/csl/src/ip/edma/V1/hw_edma_tpcc.h>

/* local header files */
#include "../shared/AppCommon.h"

/* module header file */
#include "edma_test.h"
#include "sin_data.h"









/* L2SRAM : 0x00800000 ~ 0x00847FFF */
/* half of L2SRAM used for test, write/read buffer take max half of test address */
#define EDMA_MAX_TRANS_SIZE   (64 * 1024)

/* 32 KB:  ACNT is a 16-bit unsigned value with valid values between 0 and 65 535 */
#define EDMA_ACNT_MAX_SIZE       (32768)

/*****************************************************************************
 * Choose the type of EDMA transfer (Current options are "DMA" and "QDMA")
 *****************************************************************************/
/* DSP1 EDMA3 transfer event number */
#define DSP1_EDMA3_CC_XFER_COMPLETION_INT 19
#define DSP1_EDMA3_CC_ERROR_INT 27

/* DSP1 EDAM3 region number */
#define DSP1_EDMA3_REGION 2

/* DSP1 EDMA3 channel number */
#define DSP1_EDMA3_CHANNEL 0

/* DSP1 EDMA3 tcc number */
#define DSP1_EDMA3_TCC 0

/* DSP1 EDMA3 evtq number */
#define DSP1_EDMA3_EVTQ 0

/* OPT Field specific defines */
#define OPT_SYNCDIM_SHIFT                   (0x00000002u)
#define OPT_TCC_MASK                        (0x0003F000u)
#define OPT_TCC_SHIFT                       (0x0000000Cu)
#define OPT_ITCINTEN_SHIFT                  (0x00000015u)
#define OPT_TCINTEN_SHIFT                   (0x00000014u)
#define OPT_TCCMOD_SHIFT                    (0x0000000Bu)

/* edma_debug: 0 -- close the debug info, 1 -- open the debug info */
int edma_debug = 0;

/* Flag variable to check transfer completion on channel 1 */
volatile short edma_irqRaised1 = 0;

UInt32	transStart, transCost;

void edma_cb_isr(UInt32 event_id)
{
    switch(event_id) {
    case DSP1_EDMA3_CC_XFER_COMPLETION_INT:
        /* Transfer completed successfully */
        edma_irqRaised1 = 1;
        break;
    case DSP1_EDMA3_CC_ERROR_INT:
        /* Transfer resulted in error. */
        edma_irqRaised1 = -1;
        break;
    default:
        break;
    };

    transCost = Timestamp_get32() - transStart;
    EDMA3ClrIntr(CSL_DSP_DSP_EDMA_CC_REGS, DSP1_EDMA3_TCC);
}

/**
 *  \brief  EDMA3 init function
 *
 *  \param  hEdma       [IN]      EDMA handle
 *
 *  \return EDMA handle
 */
Int edma3_init(void)
{
    Hwi_Params hwiParams;
    Error_Block eb;

    EDMAsetRegion(DSP1_EDMA3_REGION);

    EDMA3Init(CSL_DSP_DSP_EDMA_CC_REGS, 0);

    Hwi_Params_init(&hwiParams);
    Error_init(&eb);

    /* set the event id of the peripheral assigned to this interrupt */
    hwiParams.eventId = DSP1_EDMA3_CC_XFER_COMPLETION_INT;

    /* set the argument passed to ISR function */
    hwiParams.arg = DSP1_EDMA3_CC_XFER_COMPLETION_INT;

    hwiParams.maskSetting = Hwi_MaskingOption_SELF;

    Hwi_create(11, edma_cb_isr, &hwiParams, &eb);
    if (Error_check(&eb)) {
        Log_print0(Diags_INFO, "Hwi create failed \n");
        return -1;
    }

    /* set the event id of the peripheral assigned to this interrupt */
    hwiParams.eventId = DSP1_EDMA3_CC_ERROR_INT;

    /* set the argument passed to ISR function */
    hwiParams.arg = DSP1_EDMA3_CC_ERROR_INT;

    hwiParams.maskSetting = Hwi_MaskingOption_SELF;

    Hwi_create(12, edma_cb_isr, &hwiParams, &eb);
    if (Error_check(&eb)) {
        Log_print0(Diags_INFO, "Hwi create failed \n");
        return -1;
    }

    Hwi_enable();

    EDMA3RequestChannel(CSL_DSP_DSP_EDMA_CC_REGS, EDMA3_CHANNEL_TYPE_DMA, DSP1_EDMA3_CHANNEL, \
                        DSP1_EDMA3_TCC, DSP1_EDMA3_EVTQ);

    EDMA3EnableEvtIntr(CSL_DSP_DSP_EDMA_CC_REGS, DSP1_EDMA3_CHANNEL);

    return 0;
}

/**
 *  \brief  EDMA3 deinit function
 *
 *  \param  hEdma       [IN]      EDMA handle
 *
 *  \return
 */
void edma3_deinit(void)
{
    EDMA3FreeChannel(CSL_DSP_DSP_EDMA_CC_REGS, EDMA3_CHANNEL_TYPE_DMA, DSP1_EDMA3_CHANNEL, \
                    EDMA3_TRIG_MODE_MANUAL, DSP1_EDMA3_TCC, DSP1_EDMA3_EVTQ);

    EDMA3Deinit(CSL_DSP_DSP_EDMA_CC_REGS, 0);
}




Int32 A15ReadfromFPGA(App_Msg* const msg)
{
    Int32 Status = 0;
    UInt16 acnt, bcnt, ccnt;
    UInt32 *devmem_virt_addr;
    UInt32 timeout;
    EDMA3CCPaRAMEntry edmaParam;
    UInt32 *edma_dest;

    Resource_physToVirt(msg->devmem_phy_addr, (UInt32*)&devmem_virt_addr);	// convert phy address to virtual address 0x01000000


    edma_dest = Memory_alloc(SharedRegion_getHeap(0), EDMA_MAX_TRANS_SIZE, 256, NULL);
    if (edma_dest == NULL) {
        Log_error0("Failed to alloc edma write memory !\n");
    }

    acnt = msg->transfer_size;
    bcnt = 1;
    ccnt = 1;
    /* Wait edma complete time set as 130ms, base on cpu freq as 750MHz */
    timeout = 100000000;

    /* 1.2 edma write, write_buff -> virt_address */
    edma_irqRaised1 = 0;

    edmaParam.opt = 0;
    edmaParam.srcAddr = (uint32_t)devmem_virt_addr;
    edmaParam.destAddr = (uint32_t)edma_dest;
    edmaParam.aCnt = acnt;
    edmaParam.bCnt = bcnt;
    edmaParam.cCnt = ccnt;
    edmaParam.srcBIdx = acnt * bcnt;
    edmaParam.destBIdx = acnt * bcnt;
    edmaParam.srcCIdx = acnt * bcnt;
    edmaParam.destCIdx = acnt * bcnt;
    edmaParam.linkAddr = 0xFFFFu;
    edmaParam.opt &= 0xFFFFFFFCu;
    /* Program the TCC */
    edmaParam.opt |= ((DSP1_EDMA3_TCC << OPT_TCC_SHIFT) & OPT_TCC_MASK);
    /* Clear TCCMODE to make sure edma does not interrupt till finished */
    edmaParam.opt |= (0 << OPT_TCCMOD_SHIFT);
    /* Enable Intermediate & Final transfer completion interrupt */
    edmaParam.opt |= (1 << OPT_ITCINTEN_SHIFT);
    edmaParam.opt |= (1 << OPT_TCINTEN_SHIFT);
    /* AB Sync Transfer Mode */
    edmaParam.opt |= (1 << OPT_SYNCDIM_SHIFT);

    EDMA3SetPaRAM(CSL_DSP_DSP_EDMA_CC_REGS, DSP1_EDMA3_CHANNEL, &edmaParam);

    edma_irqRaised1 = 0;

    /* Start edma transfer */
    EDMA3EnableTransfer(CSL_DSP_DSP_EDMA_CC_REGS, DSP1_EDMA3_CHANNEL, EDMA3_TRIG_MODE_MANUAL);

    /* Wait for a transfer completion interrupt occur */
    while (timeout)
    {
        if (edma_irqRaised1 > 0)
        {
            break;
        }
        else if (edma_irqRaised1 < 0)
        {
            Log_print1(Diags_INFO, "EDMA transmit have interrupt error %d\r\n", edma_irqRaised1);
            /* If interrupt error occurs, return error status */
        }
        else
        {
            timeout--;
            /* Delay 1 cpu cyle */
            asm(" nop");
        }
    }

    if (timeout == 0)
    {
        Log_error0("EDMA transmit time out !\r\n");
	Status = -1;
    }
   return Status;
}



