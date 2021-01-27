/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2015-2016
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
 * @file pcie_example_board.h
 *
 */

#include <stdint.h>

#include <ti/csl/soc.h>
#define MEM_BARRIER_DISABLE
#include <ti/csl/hw_types.h>

#include "pcie_sample.h"
#include <ti/drv/pcie/example/sample/am57x/src/pcie_sample_board.h>
#include <ti/csl/soc.h>
#include <ti/osal/osal.h>
#include <ti/drv/pcie/pcie.h>
#ifdef _TMS320C6X
#include <ti/csl/csl_chipAux.h>
#endif
#include <string.h>

#if defined(IDK_AM572x)
#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_v1.h>
#endif
#include <ti/csl/cslr_device.h>
#include <ti/csl/soc/am572x/src/cslr_control_core_pad_io.h>
#include <ti/csl/soc/am572x/src/cslr_soc.h>

#if defined(IDK_AM572x)
#define GPIO_PIN_VAL_LOW        (0U)
#define GPIO_PIN_VAL_HIGH       (1U)

/* Port and pin number mask for GPIO_PCIE_RSTDRVn and GPIO_PCIE_SWRSTn.
   Bits 7-0: Pin number  and Bits 15-8: Port number */
#define GPIO_PCIE_RSTDRVn         (0x0316) /*VIN1A_D18/GPIO3_22*/
#define GPIO_PCIE_SWRSTn          (0x0317) /*VIN1A_D19/GPIO3_23*/

#define CTRL_CORE_PAD_GPIO_PIN    (0x20000 | 0x0E)



/* GPIO Driver call back functions */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    NULL,
    NULL
};

/* GPIO Driver board specific pin configuration structure */
GPIO_PinConfig gpioPinConfigs[] = {
	GPIO_PCIE_RSTDRVn | GPIO_CFG_OUTPUT,
	GPIO_PCIE_SWRSTn | GPIO_CFG_OUTPUT,
};

/* GPIO Driver configuration structure */
GPIO_v1_Config GPIO_v1_config = {
    gpioPinConfigs,
    gpioCallbackFunctions,
    sizeof(gpioPinConfigs) / sizeof(GPIO_PinConfig),
    sizeof(gpioCallbackFunctions) / sizeof(GPIO_CallbackFxn),
    0,
};
#endif


void PlatformPCIESS1ClockEnable(void)
{
    uint32_t regVal;

    /*OCP2SCP1 enables accessing the PCIe PHY serial configuration*/
    HW_WR_FIELD32(SOC_L3INIT_CM_CORE_BASE + CM_L3INIT_OCP2SCP1_CLKCTRL,
                  CM_L3INIT_OCP2SCP1_CLKCTRL_MODULEMODE,
                  CM_L3INIT_OCP2SCP1_CLKCTRL_MODULEMODE_AUTO);

    /*OCP2SCP3 enables accessing the PCIe PHY serial configuration*/
    HW_WR_FIELD32(SOC_L3INIT_CM_CORE_BASE + CM_L3INIT_OCP2SCP3_CLKCTRL,
                  CM_L3INIT_OCP2SCP3_CLKCTRL_MODULEMODE,
                  CM_L3INIT_OCP2SCP3_CLKCTRL_MODULEMODE_AUTO);

    /*PCIeSS CLKSTCTRL SW WakeUp*/
    HW_WR_FIELD32(SOC_L3INIT_CM_CORE_BASE + CM_PCIE_CLKSTCTRL,
                  CM_PCIE_CLKSTCTRL_CLKTRCTRL,
                  CM_PCIE_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

    /*L3 Init PCIeSS1 CLKCTRL SW Enable*/
    HW_WR_FIELD32(SOC_L3INIT_CM_CORE_BASE + CM_PCIE_PCIESS1_CLKCTRL,
                  CM_PCIE_PCIESS1_CLKCTRL_MODULEMODE,
                  CM_PCIE_PCIESS1_CLKCTRL_MODULEMODE_ENABLED);

    while ((HW_RD_REG32(SOC_L3INIT_CM_CORE_BASE + CM_PCIE_PCIESS1_CLKCTRL) &
            CM_PCIE_PCIESS1_CLKCTRL_IDLEST_MASK) !=
           CM_PCIE_PCIESS1_CLKCTRL_IDLEST_FUNC)
    {
        ;
    }

    /*Enable PCIe PHY optional clk*/
    regVal = HW_RD_REG32(SOC_L3INIT_CM_CORE_BASE + CM_PCIE_PCIESS1_CLKCTRL);

    HW_SET_FIELD(regVal, CM_PCIE_PCIESS1_CLKCTRL_OPTFCLKEN_PCIEPHY_CLK_DIV,
                 CM_PCIE_PCIESS1_CLKCTRL_OPTFCLKEN_PCIEPHY_CLK_DIV_FCLK_EN);

    HW_SET_FIELD(regVal, CM_PCIE_PCIESS1_CLKCTRL_OPTFCLKEN_PCIEPHY_CLK,
                 CM_PCIE_PCIESS1_CLKCTRL_OPTFCLKEN_PCIEPHY_CLK_FCLK_EN);

    HW_SET_FIELD(regVal, CM_PCIE_PCIESS1_CLKCTRL_OPTFCLKEN_32KHZ,
                 CM_PCIE_PCIESS1_CLKCTRL_OPTFCLKEN_32KHZ_FCLK_EN);

    HW_WR_REG32(SOC_L3INIT_CM_CORE_BASE + CM_PCIE_PCIESS1_CLKCTRL, regVal);
}

void PlatformPCIESS1PllConfig(void)
{
    uint32_t regVal;

    /*OCP2SCP_SYSCONFIG[1] Soft Reset*/
    regVal = HW_RD_REG32(SOC_OCP2SCP3_BASE + 0x10U) & 0xFFFFFFFDU;

    regVal |= 0x02U;

    HW_WR_REG32(SOC_OCP2SCP3_BASE + 0x10U, regVal);

    /*OCP2SCP_SYSSTATUS[0] Reset Done*/
    while ((HW_RD_REG32(SOC_OCP2SCP3_BASE + 0x14U) & 0x01U) != 0x01U)
    {
        ;
    }

    /*OCP2SCP_TIMING[9:7] Division Ratio = 1*/
    regVal = HW_RD_REG32(SOC_OCP2SCP3_BASE + 0x18U) & 0xFFFFFC7FU;

    regVal |= (uint32_t) 0x8U << 4U;

    HW_WR_REG32(SOC_OCP2SCP3_BASE + 0x18U, regVal);

    /*OCP2SCP_TIMING[3:0] (SYNC2) = 0xF*/
    regVal = HW_RD_REG32(SOC_OCP2SCP3_BASE + 0x18U) & 0xFFFFFFF0U;

    regVal |= 0xFU;

    HW_WR_REG32(SOC_OCP2SCP3_BASE + 0x18U, regVal);

#ifdef SOC_AM571x
    /*OCP2SCP_SYSCONFIG[1] Soft Reset*/
    regVal = HW_RD_REG32(SOC_OCP2SCP1_BASE + 0x10U) & 0xFFFFFFFDU;

    regVal |= 0x02U;

    HW_WR_REG32(SOC_OCP2SCP1_BASE + 0x10U, regVal);

    /*OCP2SCP_SYSSTATUS[0] Reset Done*/
    while ((HW_RD_REG32(SOC_OCP2SCP1_BASE + 0x14U) & 0x01U) != 0x01U)
    {
        ;
    }

    /*OCP2SCP_TIMING[9:7] Division Ratio = 1*/
    regVal = HW_RD_REG32(SOC_OCP2SCP1_BASE + 0x18U) & 0xFFFFFC7FU;

    regVal |= (uint32_t) 0x8U << 4U;

    HW_WR_REG32(SOC_OCP2SCP1_BASE + 0x18U, regVal);

    /*OCP2SCP_TIMING[3:0] (SYNC2) = 0xF*/
    regVal = HW_RD_REG32(SOC_OCP2SCP1_BASE + 0x18U) & 0xFFFFFFF0U;

    regVal |= 0xFU;

    HW_WR_REG32(SOC_OCP2SCP1_BASE + 0x18U, regVal);
#endif

    /*PCIe DPLL - M&N programming; CLKSEL*/
    regVal = HW_RD_REG32(SOC_CKGEN_CM_CORE_BASE + CM_CLKSEL_DPLL_PCIE_REF);

    HW_SET_FIELD(regVal, CM_CLKSEL_DPLL_PCIE_REF_DPLL_DIV, 0x09U);

    HW_SET_FIELD(regVal, CM_CLKSEL_DPLL_PCIE_REF_DPLL_MULT, 0x2EEU);

    HW_WR_REG32(SOC_CKGEN_CM_CORE_BASE + CM_CLKSEL_DPLL_PCIE_REF, regVal);

    /*SigmaDelta SD DIV programming */
    HW_WR_FIELD32(SOC_CKGEN_CM_CORE_BASE + CM_CLKSEL_DPLL_PCIE_REF,
                  CM_CLKSEL_DPLL_PCIE_REF_DPLL_SD_DIV, 0x06U);

    /*PCIe DPLL - M2 programming*/
    HW_WR_FIELD32(SOC_CKGEN_CM_CORE_BASE + CM_DIV_M2_DPLL_PCIE_REF,
                  CM_DIV_M2_DPLL_PCIE_REF_DIVHS, 0x0FU);

    /*DPLL Enable*/
    HW_WR_FIELD32(SOC_CKGEN_CM_CORE_BASE + CM_CLKMODE_DPLL_PCIE_REF,
                  CM_CLKMODE_DPLL_PCIE_REF_DPLL_EN,
                  CM_CLKMODE_DPLL_PCIE_REF_DPLL_EN_DPLL_LOCK_MODE);

    /* Check for DPLL lock status */
    while (((HW_RD_REG32(SOC_CKGEN_CM_CORE_BASE + CM_IDLEST_DPLL_PCIE_REF) &
             CM_IDLEST_DPLL_PCIE_REF_ST_DPLL_CLK_MASK) <<
            CM_IDLEST_DPLL_PCIE_REF_ST_DPLL_CLK_SHIFT) !=
           CM_IDLEST_DPLL_PCIE_REF_ST_DPLL_CLK_DPLL_LOCKED)
    {
        ;
    }

    /*PCIe Tx and Rx Control of ACSPCIe*/
    HW_WR_FIELD32(SOC_SEC_EFUSE_REGISTERS_BASE + CSL_CONTROL_CORE_SEC_SMA_SW_6,
                  CSL_CONTROL_CORE_SEC_SMA_SW_6_PCIE_TX_RX_CONTROL, 0x02U);

    /*Locking APLL to 2.5GHz with 100MHz input*/
    regVal = HW_RD_REG32(SOC_CKGEN_CM_CORE_BASE + CM_CLKMODE_APLL_PCIE);

    HW_SET_FIELD(regVal, CM_CLKMODE_APLL_PCIE_CLKDIV_BYPASS,
                 CM_CLKMODE_APLL_PCIE_CLKDIV_BYPASS_PCIEDIVBY2_BYPASS_1);

    HW_SET_FIELD(regVal, CM_CLKMODE_APLL_PCIE_REFSEL,
                 CM_CLKMODE_APLL_PCIE_REFSEL_CLKREF_ADPLL);

    HW_WR_REG32(SOC_CKGEN_CM_CORE_BASE + CM_CLKMODE_APLL_PCIE, regVal);

    HW_WR_FIELD32(SOC_CKGEN_CM_CORE_BASE + CM_CLKMODE_APLL_PCIE,
                  CM_CLKMODE_APLL_PCIE_MODE_SELECT,
                  CM_CLKMODE_APLL_PCIE_MODE_SELECT_APLL_FORCE_LOCK_MODE);

    /*Wait for APLL lock*/
    while (((HW_RD_REG32(SOC_CKGEN_CM_CORE_BASE + CM_IDLEST_APLL_PCIE) &
             CM_IDLEST_APLL_PCIE_ST_APLL_CLK_MASK) <<
            CM_IDLEST_APLL_PCIE_ST_APLL_CLK_SHIFT) !=
           CM_IDLEST_APLL_PCIE_ST_APLL_CLK_APLL_LOCKED)
    {
        ;
    }
}

void PlatformPCIESSSetPhyMode (void)
{
/* Set the PHY mode as PCIE and not USB */
#if defined(SOC_AM571x)
  /* OCP2SCP1_USB3TX_PHY_USB.MEM_ENTESTCLK = 1 */
  *(volatile unsigned int*)0x4a08482c |= (1<<31);   //PCIE mode
  /* USB3PHYRX_ANA_PROGRAMMABILITY_REG1.MEM_EN_PLLBYP = 1 */
  *(volatile unsigned int*)0x4a08440c |= (1<<7);    //PCIE mode
#endif
/* Set the PHY mode as X2 */
#if defined(PCIESS1_X2)
  {
    uint32_t regVal;
    /* CTRL_CORE_CONTROL_IO_2 */
    *(volatile unsigned int*)0x4a002558 |= (1<<13);   /* 0: 1-lane; 1: 2-lane */

    /* B1C0 Mode selection, bit 3:2 of 0x4a003c3c, reset = 0x2?
     * 0: PCIESS1x1 and/or PCIESS2x1
     * 1: PCIESS2x2, PCIESS2 not used */
    regVal  = *(volatile unsigned int *)0x4a003c3cU;
    regVal &= 0xFFFFFFF2U;
    regVal |= (1U << 2);   //PCIESS1 X2 mode
    regVal |= (1U << 0);   //PCIE_B0_B1_TSYNCEN
    *(volatile unsigned int *)0x4a003c3cU = regVal;
  }
#endif
}

void PlatformPCIESS1Reset(void)
{
    /*Reset PCIeSS1*/
    HW_WR_FIELD32(SOC_L3INIT_PRM_BASE + RM_PCIESS_RSTCTRL,
                  RM_PCIESS_RSTCTRL_RST_LOCAL_PCIE1,
                  RM_PCIESS_RSTCTRL_RST_LOCAL_PCIE1_CLEAR);

    /* Wait till PCIeSS1 is out of reset */
    while (((HW_RD_REG32(SOC_L3INIT_PRM_BASE + RM_PCIESS_RSTST) &
             RM_PCIESS_RSTST_RST_LOCAL_PCIE1_MASK) <<
            RM_PCIESS_RSTST_RST_LOCAL_PCIE1_SHIFT) !=
           RM_PCIESS_RSTST_RST_LOCAL_PCIE1_RESET_YES)
    {
        ;
    }
}

void PlatformPCIESS1CtrlConfig(void)
{
    uint32_t regVal;

    /*CONTROL MODULE PWR CTL REG status of PCIeSS1*/
    regVal = HW_RD_REG32(
        SOC_SEC_EFUSE_REGISTERS_BASE + CSL_CONTROL_CORE_SEC_PHY_POWER_PCIESS1);

    HW_SET_FIELD(regVal, CSL_CONTROL_CORE_SEC_PHY_POWER_PCIESS1_PCIESS1_PWRCTL_CMD,
                 0x03U);

    HW_SET_FIELD(regVal, CSL_CONTROL_CORE_SEC_PHY_POWER_PCIESS1_PCIESS1_PWRCTL_CLKFREQ,
                 0x1AU);

    HW_WR_REG32(SOC_SEC_EFUSE_REGISTERS_BASE + CSL_CONTROL_CORE_SEC_PHY_POWER_PCIESS1,
                regVal);

    /*Set PCIeSS1/2 delay count*/
    HW_WR_FIELD32(SOC_SEC_EFUSE_REGISTERS_BASE + CSL_CONTROL_CORE_SEC_PCIE_PCS,
                  CSL_CONTROL_CORE_SEC_PCIE_PCS_PCIESS2_PCS_RC_DELAY_COUNT, 0x96U);
}

void PlatformPCIESS1PhyConfig(void)
{
    uint32_t regVal;

    /*Program for Analog circuits in the IP.*/
    regVal  = HW_RD_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE1_BASE + 0x0CU);
    regVal &= 0x07FFFFFFU;
    regVal |= ((uint32_t) 0x10U << 24U);
    HW_WR_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE1_BASE + 0x0CU, regVal);

    regVal  = HW_RD_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE1_BASE + 0x0CU);
    regVal &= 0xFFFC3FFFU;
    regVal |= ((uint32_t) 0x10U << 12U);
    HW_WR_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE1_BASE + 0x0CU, regVal);

    /*Program for digital section of the IP.*/
    regVal  = HW_RD_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE1_BASE + 0x28U);
    regVal &= 0xE30007FFU;
    regVal |= 0x001B3000U;
    HW_WR_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE1_BASE + 0x28U, regVal);

    regVal  = HW_RD_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE1_BASE + 0x0CU);
    regVal &= 0xFFFFFF9FU;
    regVal |= ((uint32_t) 0x0U << 4U);
    HW_WR_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE1_BASE + 0x0CU, regVal);

    /*Determines which of the 4 EFUSE registers. Selects dll_rate2_coarsetrim*/
    regVal  = HW_RD_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE1_BASE + 0x1CU);
    regVal &= 0x3FFFFFFFU;
    regVal |= ((uint32_t) 0x8U << 28U);
    HW_WR_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE1_BASE + 0x1CU, regVal);

    /*
     * Programs the DLL and the Phase Interpolator analog RW 0x3
     * circuits to work with different clock frequencies
     */
    regVal  = HW_RD_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE1_BASE + 0x24U);
    regVal &= 0x3FFFFFFFU;
    regVal |= ((uint32_t) 0xCU << 28U);
    HW_WR_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE1_BASE + 0x24U, regVal);

    /*Program IP Equalizer*/
    regVal  = HW_RD_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE1_BASE + 0x38U);
    regVal &= 0x0U;
    regVal |= 0x0000F80FU;
    HW_WR_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE1_BASE + 0x38U, regVal);
}

static uint32_t msi_ints = 0, intx_ints = 0, unknown_ints = 0;
uint32_t isr_count = 0;
uint32_t isr_flag = 0;
uint32_t msi_isr_index = 0;
uint32_t msi_index_count[32] = {0};
SemaphoreP_Handle semaphoreHandle;
static void PlatformMsiIntxIsr (uintptr_t vhandle)
{
    pcieTiConfIrqStatusMsiReg_t pendingBits;
    pciePlconfMsiCtrlIntStatusReg_t msiBits[8];
    pciePlconfMsiCtrlIntStatusReg_t *clearMsiBits = NULL;
    int32_t clearMsiBitsSize = 0;
    Pcie_Handle handle = (Pcie_Handle)vhandle;
    int32_t msiError = 0, i;

    isr_count++;
    isr_flag = 1;
#if 1
    /* Figure out which type if interrupt */
    if (Pcie_getPendingFuncInts (handle, &pendingBits, sizeof(msiBits), msiBits) == pcie_RET_OK)
    {
#if 0
            msi_isr_index = 0;
            for (i = 1; i < 32; i++)
            {
                if (msiBits[0].msiCtrlIntStatus & (1<<i))
                {
                    msi_isr_index = i;
                    msi_index_count[msi_isr_index] += 1;
                    break;
                }
            }
#endif
            /* clear it */
            clearMsiBitsSize = sizeof(msiBits);
            clearMsiBits = msiBits;
    }
#endif
    /* Tell user task ISR happend */
    //SemaphoreP_postFromISR (semaphoreHandle);
    /* Clear/acknowledge the interrupt */
    Pcie_clrPendingFuncInts (handle, &pendingBits, clearMsiBitsSize, clearMsiBits);
}


HwiP_Handle pcieHwi;
SemaphoreP_Handle PlatformSetupMSIAndINTX (Pcie_Handle handle)
{
    HwiP_Params                        hwiInputParams;
    CSL_XbarIrqCpuId                   cpu;
    uint32_t                           cpuEvent;
    uint32_t                           xbarIndex;
    int32_t                            vector;
    pcieRet_e                          retVal;
    pcieRegisters_t                    regs;
    pcieRegisters_t                    epRegs;
    pcieTiConfIrqStatusMsiReg_t        rcMsiStat;
    pcieTiConfIrqEnableSetMsiReg_t     rcMsiEn;
    pciePlconfMsiCtrlAddressReg_t      rcMsiLowAddress;
    pciePlconfMsiCtrlUpperAddressReg_t rcMsiUpAddress;
    pciePlconfMsiCtrlIntEnableReg_t    rcMsiIntEnable;
    pciePlconfMsiCtrlIntStatusReg_t    rcMsiBits[8];
    pcieMsiCapReg_t                    epMsiCap;
    int32_t                            i;
    pcieCapPtrReg_t         capPtr;
    uint16_t  fpga_msi_ctrl_val;
    uint8_t  fpga_msi_ctrl_val_lsb;
    uint8_t  fpga_msi_ctrl_val_msb;

    /* Create a semaphore for user task to wait for interrupt */
    semaphoreHandle = SemaphoreP_create (0, NULL);
    if (!semaphoreHandle)
    {
        PCIE_logPrintf("Failed to create semaphore\n");
        exit(1);
    }
    memset (&regs, 0, sizeof(regs));
    memset (&epRegs, 0, sizeof(epRegs));
    memset (&rcMsiEn, 0, sizeof(rcMsiEn));
    memset (&rcMsiLowAddress, 0, sizeof(rcMsiLowAddress));
    memset (&rcMsiStat, 0, sizeof(rcMsiStat));
    memset (&rcMsiUpAddress, 0, sizeof(rcMsiUpAddress));
    memset (&rcMsiIntEnable, 0, sizeof(rcMsiIntEnable));
    memset (&epMsiCap, 0, sizeof(epMsiCap));

    cpu = CSL_XBAR_IRQ_CPU_ID_DSP1;

    cpuEvent = 48;
    xbarIndex = cpuEvent - 31;
    vector = 12;

    /* Configure xbar */
    CSL_xbarIrqConfigure (cpu, xbarIndex, CSL_XBAR_PCIe_SS1_IRQ_INT1);

    /* Construct Hwi object for PCIe MSI */
    HwiP_Params_init (&hwiInputParams);
    hwiInputParams.name = "PCIE_MSI_AND_INTX";
    hwiInputParams.arg  = (uintptr_t)handle;
    hwiInputParams.priority = 0;
    hwiInputParams.evtId = cpuEvent;
    pcieHwi = HwiP_create(vector, PlatformMsiIntxIsr, &hwiInputParams);

    if (! pcieHwi)
    {
        PCIE_logPrintf("Hwi create failed\n");
        exit(1);
    }

    /* Enable MSI on EP (FPGA A7) */
    epMsiCap.capId = *(volatile uint32_t*)(PCIE_EP_CFG_SPACE_ADDR + FPGA_CFG_MSI_CAP_OFFSET);
    fpga_msi_ctrl_val_lsb = (*(volatile uint32_t*)(PCIE_EP_CFG_SPACE_ADDR + FPGA_CFG_MSI_CAP_OFFSET) >> 16) & 0xff;
    fpga_msi_ctrl_val_msb = (*(volatile uint32_t*)(PCIE_EP_CFG_SPACE_ADDR + FPGA_CFG_MSI_CAP_OFFSET) >> 24) & 0xff;
    fpga_msi_ctrl_val = ((fpga_msi_ctrl_val_msb & 0xff) << 8) | (fpga_msi_ctrl_val_lsb & 0xff);

    PCIE_logPrintf ("epMsiCap.capId: (0x%x)\n", epMsiCap.capId);
    PCIE_logPrintf ("before fpga_msi_ctrl_val_msb: (0x%x) fpga_msi_ctrl_val_lsb: (0x%x) fpga_msi_ctrl_val: (0x%x)\n",
                    fpga_msi_ctrl_val_msb, fpga_msi_ctrl_val_lsb, fpga_msi_ctrl_val);

    *(volatile uint32_t*)(PCIE_EP_CFG_SPACE_ADDR + FPGA_CFG_MSI_UP32_ADDR_OFFSET) = 0x0;
    *(volatile uint32_t*)(PCIE_EP_CFG_SPACE_ADDR + FPGA_CFG_MSI_LOW32_ADDR_OFFSET) = PCIE_WINDOW_MSI_ADDR; /* When EP is FPGA A7, the value must equal with RC msiCtrlAddress */
    *(volatile uint32_t*)(PCIE_EP_CFG_SPACE_ADDR + FPGA_CFG_MSI_DATA_OFFSET)  = PCIE_WINDOW_MSI_DATA;

    /* Enable MSI on EP (FPGA A7), support 32 vectors */
    *(volatile uint32_t*)(PCIE_EP_CFG_SPACE_ADDR+FPGA_CFG_MSI_CAP_OFFSET) |= (1 << 16) | (5 << 20);
    fpga_msi_ctrl_val_lsb = (*(volatile uint32_t*)(PCIE_EP_CFG_SPACE_ADDR + FPGA_CFG_MSI_CAP_OFFSET) >> 16) & 0xff;
    fpga_msi_ctrl_val_msb = (*(volatile uint32_t*)(PCIE_EP_CFG_SPACE_ADDR + FPGA_CFG_MSI_CAP_OFFSET) >> 24) & 0xff;
    fpga_msi_ctrl_val = ((fpga_msi_ctrl_val_msb & 0xff) << 8) | (fpga_msi_ctrl_val_lsb & 0xff);

    PCIE_logPrintf ("after fpga_msi_ctrl_val_msb: (0x%x) fpga_msi_ctrl_val_lsb: (0x%x) fpga_msi_ctrl_val: (0x%x)\n",
                     fpga_msi_ctrl_val_msb, fpga_msi_ctrl_val_lsb, fpga_msi_ctrl_val);

    /* Clear any pending interrupts inside PCIE */
    regs.tiConfIrqStatusMsi = &rcMsiStat;
    rcMsiStat.inta = rcMsiStat.intb = rcMsiStat.intc = rcMsiStat.intd = 1;
    rcMsiStat.msi = 1;
    /* Clear any MSIs */
    for (i = 0; i < 8; i++)
    {
        rcMsiBits[i].msiCtrlIntStatus = ~0U;
        regs.plconfMsiCtrlIntStatus[i] = &rcMsiBits[i];
    }

    /* Enable the interrupts inside PCIE */
    regs.tiConfIrqEnableSetMsi = &rcMsiEn;
    rcMsiEn.inta = rcMsiEn.intb = rcMsiEn.intc = rcMsiEn.intd = 1;
    rcMsiEn.msi = 1;
    /* Set RC's data address */
    regs.plconfMsiCtrlAddress = &rcMsiLowAddress;
    regs.plconfMsiCtrlUpperAddress = &rcMsiUpAddress;
    rcMsiLowAddress.msiCtrlAddress = PCIE_WINDOW_MSI_ADDR;
    rcMsiUpAddress.msiCtrlUpperAddress  = 0;
    /* Set RC's interrupt enable in plconf */
    regs.plconfMsiCtrlIntEnable[0] = &rcMsiIntEnable;
    /* Enable 32 MSI vector */
    rcMsiIntEnable.msiCtrlIntEnable = 0xffffffff;
    //rcMsiIntEnable.msiCtrlIntEnable = 0x1;
    retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &regs);
    if (retVal != pcie_RET_OK)
    {
        PCIE_logPrintf ("write of RC interrupt regs failed (%d)\n", retVal);
        exit(1);
    }

    return semaphoreHandle;
}

void PlatformGetInts (uint32_t *msis, uint32_t *intx, uint32_t *unknowns)
{
    if (msis)
    {
        *msis = msi_ints;
    }
    if (intx)
    {
        *intx = intx_ints;
    }
    if (unknowns)
    {
        *unknowns = unknown_ints;
    }
}

void PlatformPCIE_GPIO_Init(void)
{
#if defined(IDK_AM572x)
	GPIO_init();
#endif
}

/*
 * API: PlatformPCIE_PERSTn_Reset
 * Variable: resetOnOffF - Reset On/OFF Flag 1-ON/RESET , 0- OFF/Out of RESET
 * Set PCIE_PERSTn to out of reset state
 * GPIO_PCIE_RSTDRV - LOW to select the Reset driver
 * GPIO_PCIE_SWRST - HIGH to set to out of RESET state
 */ 
void PlatformPCIE_PERSTn_Reset(uint32_t resetOnOffF)
{
#if defined(IDK_AM572x)
	GPIO_write(0, GPIO_PIN_VAL_HIGH); //Select Reset Driver

	if(resetOnOffF)	
	{
		GPIO_write(1, GPIO_PIN_VAL_LOW);
	}
	else
	{
		GPIO_write(1, GPIO_PIN_VAL_HIGH);
	}

#endif
}

void PlatformPCIESS2ClockEnable(void) 
{
    /* This example enables portions of PCISS2 to use its serdes lane
     * for dual operation of PCIESS1 */
#ifdef PCIESS1_X2
    uint32_t regVal;

    /*L3 Init PCIeSS2 CLKCTRL SW Enable*/
    HW_WR_FIELD32(SOC_L3INIT_CM_CORE_BASE + CM_PCIE_PCIESS2_CLKCTRL,
                  CM_PCIE_PCIESS2_CLKCTRL_MODULEMODE,
                  CM_PCIE_PCIESS2_CLKCTRL_MODULEMODE_ENABLED);

    while ((HW_RD_REG32(SOC_L3INIT_CM_CORE_BASE + CM_PCIE_PCIESS2_CLKCTRL) &
            CM_PCIE_PCIESS2_CLKCTRL_IDLEST_MASK) !=
           CM_PCIE_PCIESS2_CLKCTRL_IDLEST_FUNC)
    {
        ;
    }

    /*Enable PCIe PHY optional clk*/
    regVal = HW_RD_REG32(SOC_L3INIT_CM_CORE_BASE + CM_PCIE_PCIESS2_CLKCTRL);

    HW_SET_FIELD(regVal, CM_PCIE_PCIESS2_CLKCTRL_OPTFCLKEN_PCIEPHY_CLK_DIV,
                 CM_PCIE_PCIESS1_CLKCTRL_OPTFCLKEN_PCIEPHY_CLK_DIV_FCLK_EN);

    HW_SET_FIELD(regVal, CM_PCIE_PCIESS2_CLKCTRL_OPTFCLKEN_PCIEPHY_CLK,
                 CM_PCIE_PCIESS1_CLKCTRL_OPTFCLKEN_PCIEPHY_CLK_FCLK_EN);

    HW_SET_FIELD(regVal, CM_PCIE_PCIESS2_CLKCTRL_OPTFCLKEN_32KHZ,
                 CM_PCIE_PCIESS1_CLKCTRL_OPTFCLKEN_32KHZ_FCLK_EN);

    HW_WR_REG32(SOC_L3INIT_CM_CORE_BASE + CM_PCIE_PCIESS2_CLKCTRL, regVal);
#endif
}


void PlatformPCIESS2Reset(void)
{
    /* This example enables portions of PCISS2 to use its serdes lane
     * for dual operation of PCIESS1 */
#ifdef PCIESS1_X2
    /*Reset PCIeSS2*/
    HW_WR_FIELD32(SOC_L3INIT_PRM_BASE + RM_PCIESS_RSTCTRL,
                  RM_PCIESS_RSTCTRL_RST_LOCAL_PCIE2,
                  RM_PCIESS_RSTCTRL_RST_LOCAL_PCIE2_CLEAR);

    /* Wait till PCIeSS2 is out of reset */
    while (((HW_RD_REG32(SOC_L3INIT_PRM_BASE + RM_PCIESS_RSTST) &
             RM_PCIESS_RSTST_RST_LOCAL_PCIE2_MASK) >>
            RM_PCIESS_RSTST_RST_LOCAL_PCIE2_SHIFT) !=
           RM_PCIESS_RSTST_RST_LOCAL_PCIE2_RESET_YES)
    {
        ;
    }
#endif
}

void PlatformPCIESS2CtrlConfig(void)
{
    /* This example enables portions of PCISS2 to use its serdes lane
     * for dual operation of PCIESS1 */
#ifdef PCIESS1_X2
    uint32_t regVal;

#ifdef SOC_AM572x
    /*CONTROL MODULE PWR CTL REG status of PCIeSS2*/
    regVal = HW_RD_REG32(
        SOC_SEC_EFUSE_REGISTERS_BASE + CSL_CONTROL_CORE_SEC_PHY_POWER_PCIESS2);

    HW_SET_FIELD(regVal, CSL_CONTROL_CORE_SEC_PHY_POWER_PCIESS2_PCIESS2_PWRCTL_CMD,
                 0x03U);

    HW_SET_FIELD(regVal, CSL_CONTROL_CORE_SEC_PHY_POWER_PCIESS2_PCIESS2_PWRCTL_CLKFREQ,
                 0x1AU);

    HW_WR_REG32(SOC_SEC_EFUSE_REGISTERS_BASE + CSL_CONTROL_CORE_SEC_PHY_POWER_PCIESS2,
                regVal);

    /*Set PCIeSS2 delay count*/
    HW_WR_FIELD32(SOC_SEC_EFUSE_REGISTERS_BASE + CSL_CONTROL_CORE_SEC_PCIE_PCS,
                  CSL_CONTROL_CORE_SEC_PCIE_PCS_PCIESS2_PCS_RC_DELAY_COUNT, 0x96U);
#endif

#ifdef SOC_AM571x
    /*CONTROL MODULE PWR CTL REG status of CTRL_CORE_PHY_POWER_USB: 0x4A002370 */
    regVal = HW_RD_REG32(
    		SOC_CTRL_MODULE_CORE_CORE_REGISTERS_BASE + CTRL_CORE_PHY_POWER_USB);

    HW_SET_FIELD(regVal, CTRL_CORE_PHY_POWER_USB_USB_PWRCTL_CLK_CMD,
                 0x13U);

    HW_SET_FIELD(regVal, CTRL_CORE_PHY_POWER_USB_USB_PWRCTL_CLK_FREQ,
                 0x1AU);

    HW_WR_REG32(SOC_CTRL_MODULE_CORE_CORE_REGISTERS_BASE + CTRL_CORE_PHY_POWER_USB,
                regVal);

    /*Set PCIeSS2 delay count*/
    HW_WR_FIELD32(SOC_SEC_EFUSE_REGISTERS_BASE + CSL_CONTROL_CORE_SEC_PCIE_PCS,
                  CSL_CONTROL_CORE_SEC_PCIE_PCS_PCIESS2_PCS_RC_DELAY_COUNT, 0x96U);
#endif
#endif
}

void PlatformPCIESS2PhyConfig(void)
{
#ifdef PCIESS1_X2
    uint32_t regVal;

#ifdef SOC_AM572x
    /*Program for Analog circuits in the IP.*/
    regVal  = HW_RD_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE2_BASE + 0x0CU);
    regVal &= 0x07FFFFFFU;
    regVal |= ((uint32_t) 0x10U << 24U);
    HW_WR_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE2_BASE + 0x0CU, regVal);

    regVal  = HW_RD_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE2_BASE + 0x0CU);
    regVal &= 0xFFFC3FFFU;
    regVal |= ((uint32_t) 0x10U << 12U);
    HW_WR_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE2_BASE + 0x0CU, regVal);

    /*Program for digital section of the IP.*/
    regVal  = HW_RD_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE2_BASE + 0x28U);
    regVal &= 0xE30007FFU;
    regVal |= 0x001B3000U;
    HW_WR_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE2_BASE + 0x28U, regVal);

    regVal  = HW_RD_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE2_BASE + 0x0CU);
    regVal &= 0xFFFFFF9FU;
    regVal |= ((uint32_t) 0x0U << 4U);
    HW_WR_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE2_BASE + 0x0CU, regVal);

    /*Determines which of the 4 EFUSE registers. Selects dll_rate2_coarsetrim*/
    regVal  = HW_RD_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE2_BASE + 0x1CU);
    regVal &= 0x3FFFFFFFU;
    regVal |= ((uint32_t) 0x8U << 28U);
    HW_WR_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE2_BASE + 0x1CU, regVal);

    /*
     * Programs the DLL and the Phase Interpolator analog RW 0x3
     * circuits to work with different clock frequencies
     */
    regVal  = HW_RD_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE2_BASE + 0x24U);
    regVal &= 0x3FFFFFFFU;
    regVal |= ((uint32_t) 0xCU << 28U);
    HW_WR_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE2_BASE + 0x24U, regVal);

    /*Program IP Equalizer*/
    regVal  = HW_RD_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE2_BASE + 0x38U);
    regVal &= 0x0U;
    regVal |= 0x0000F80FU;
    HW_WR_REG32(SOC_OCP2SCP3_USB3RX_PHY_PCIE2_BASE + 0x38U, regVal);
#endif

#ifdef SOC_AM571x
    /*Program for Analog circuits in the IP.*/
    regVal  = HW_RD_REG32(SOC_OCP2SCP1_USB3RX_PHY_USB_BASE + 0x0CU);
    regVal &= 0x07FFFFFFU;
    regVal |= ((uint32_t) 0x10U << 24U);
    HW_WR_REG32(SOC_OCP2SCP1_USB3RX_PHY_USB_BASE + 0x0CU, regVal);

    regVal  = HW_RD_REG32(SOC_OCP2SCP1_USB3RX_PHY_USB_BASE + 0x0CU);
    regVal &= 0xFFFC3FFFU;
    regVal |= ((uint32_t) 0x10U << 12U);
    HW_WR_REG32(SOC_OCP2SCP1_USB3RX_PHY_USB_BASE + 0x0CU, regVal);

    /*Program for digital section of the IP.*/
    regVal  = HW_RD_REG32(SOC_OCP2SCP1_USB3RX_PHY_USB_BASE + 0x28U);
    regVal &= 0xE30007FFU;
    regVal |= 0x001B3000U;
    HW_WR_REG32(SOC_OCP2SCP1_USB3RX_PHY_USB_BASE + 0x28U, regVal);

    regVal  = HW_RD_REG32(SOC_OCP2SCP1_USB3RX_PHY_USB_BASE + 0x0CU);
    regVal &= 0xFFFFFF9FU;
    regVal |= ((uint32_t) 0x0U << 4U);
    HW_WR_REG32(SOC_OCP2SCP1_USB3RX_PHY_USB_BASE + 0x0CU, regVal);

    /*Determines which of the 4 EFUSE registers. Selects dll_rate2_coarsetrim*/
    regVal  = HW_RD_REG32(SOC_OCP2SCP1_USB3RX_PHY_USB_BASE + 0x1CU);
    regVal &= 0x3FFFFFFFU;
    regVal |= ((uint32_t) 0x8U << 28U);
    HW_WR_REG32(SOC_OCP2SCP1_USB3RX_PHY_USB_BASE + 0x1CU, regVal);

    /*
     * Programs the DLL and the Phase Interpolator analog RW 0x3
     * circuits to work with different clock frequencies
     */
    regVal  = HW_RD_REG32(SOC_OCP2SCP1_USB3RX_PHY_USB_BASE + 0x24U);
    regVal &= 0x3FFFFFFFU;
    regVal |= ((uint32_t) 0xCU << 28U);
    HW_WR_REG32(SOC_OCP2SCP1_USB3RX_PHY_USB_BASE + 0x24U, regVal);

    /*Program IP Equalizer*/
    regVal  = HW_RD_REG32(SOC_OCP2SCP1_USB3RX_PHY_USB_BASE + 0x38U);
    regVal &= 0x0U;
    regVal |= 0x0000F80FU;
    HW_WR_REG32(SOC_OCP2SCP1_USB3RX_PHY_USB_BASE + 0x38U, regVal);
#endif
#endif
}

