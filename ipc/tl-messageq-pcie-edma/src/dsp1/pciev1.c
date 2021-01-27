/*
 *
 * Copyright (C) 2010-2016 Texas Instruments Incorporated - http://www.ti.com/ 
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

/*
 *  File Name: pciev1.c
 *
 *  Processing/configuration functions for the PCIe driver.
 *
 */

#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/pcieloc.h>
#include <ti/drv/pcie/src/v1/pcieloc.h>

#include <string.h>
/*****************************************************************************
 * Set the mode of one interface without depending directly on device 
 * dependant registers (via device.c)
 ****************************************************************************/
static void pcie_set_mode (Pciev1_DeviceCfgBaseAddrs *iface, pcieMode_e mode); /*for misra warning*/
static void pcie_set_mode (Pciev1_DeviceCfgBaseAddrs *iface, pcieMode_e mode)
{
  pcieTiConfDeviceTypeReg_t typeReg;
  uint32_t regVal;

  memset (&typeReg, 0, sizeof(typeReg));
  switch (mode)
  {
    case pcie_EP_MODE:
      regVal = 0;
      break;
    case pcie_LEGACY_EP_MODE:
      regVal = 1U;
      break;
    case pcie_RC_MODE:
    default:
      regVal = 4U;
      break;
  }
  typeReg.type = regVal;
  pciev1_write_tiConfDeviceType_reg (iface->tiConf, &typeReg);
  do {
    /* Poll until complete */
    pciev1_read_tiConfDeviceType_reg (iface->tiConf, &typeReg);
  } while (typeReg.type != regVal);
} /* pcie_set_mode */

/*****************************************************************************
 **********  External APIs **********************
 ****************************************************************************/

/*********************************************************************
 * FUNCTION PURPOSE: Sets PCIe mode to RC or EP for interface
 * specified by handle
 *********************************************************************/
pcieRet_e Pciev1_setInterfaceMode
(
  Pcie_Handle handle,     /**< [in]  The PCIE LLD instance identifier */
  pcieMode_e  mode        /**< [in] PCIE Mode */
)
{
  Pcie_DeviceCfgBaseAddr *cfg = pcie_handle_to_cfg (handle);
  Pciev1_DeviceCfgBaseAddrs *bases = cfg->cfgBase;

  if (bases) {
     pcie_set_mode (bases, mode);
     return pcie_RET_OK;
  }

  return pcie_RET_INV_HANDLE;
} /* Pciev1_setInterfaceMode */

/*********************************************************************
 * FUNCTION PURPOSE: Returns amount of reserved space between beginning
 *                   of hardware's data area and the base returned
 *                   by @ref Pcie_getMemSpaceRange.  This enables
 *                   sw to position windows correctly
 *********************************************************************/
pcieRet_e Pciev1_getMemSpaceReserved 
(
  Pcie_Handle  handle,     /**< [in]  The PCIE LLD instance identifier */
  uint32_t    *resSize     /**< [out] Reserved space */
)
{
  pcieRet_e retVal = pcie_RET_OK;

  if (pcieLObjIsValid == 0) {
    return pcie_RET_NO_INIT;
  }

  pcie_check_handle(handle);

  if (resSize) {
    Pcie_DeviceCfgBaseAddr *bases = pcie_handle_to_cfg (handle);
    if (bases) {
      *resSize = bases->dataReserved;
    } else {
      retVal = pcie_RET_INV_HANDLE;
    }
  }

  return retVal;
} /* Pciev1_getMemSpaceReserved */

/*********************************************************************
 * FUNCTION PURPOSE: Returns the PCIe Internal Address Range for the 
 *                   Memory Space. This range is used for accessing memory.
 *********************************************************************/
pcieRet_e Pciev1_getMemSpaceRange 
(
  Pcie_Handle  handle,     /**< [in]  The PCIE LLD instance identifier */
  void         **base,     /**< [out] Memory Space base address */
  uint32_t      *size      /**< [out] Memory Space total size */
)
{
  pcieRet_e retVal = pcie_RET_OK;

  if (pcieLObjIsValid == 0) {
    return pcie_RET_NO_INIT;
  }

  pcie_check_handle(handle);

  if (base) {
    Pcie_DeviceCfgBaseAddr *bases = pcie_handle_to_cfg (handle);
    if (bases) {
      *base = bases->dataBase;
    } else {
      retVal = pcie_RET_INV_HANDLE;
    }
  }

  if (size) {
    *size = (uint32_t)0x10000000; /* 256 MB */
  }

  return retVal;
} /* Pciev1_getMemSpaceRange */

/*********************************************************************
 * FUNCTION PURPOSE: Reads any register
 ********************************************************************/
pcieRet_e Pciev1_readRegs 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *readRegs  /**< [in/out] List of registers to read */
)
{
  Pcie_DeviceCfgBaseAddr *cfg = pcie_handle_to_cfg (handle);
  Pciev1_DeviceCfgBaseAddrs *bases = cfg->cfgBase;

  /* Base Address for the Config Space
     These registers can be Local/Remote and Type0(EP)/Type1(RC) */
  CSL_RcCfgDbIcsRegs  *baseCfgRcRegs     = bases->rcDbics;
  CSL_EpCfgDbIcsRegs  *baseCfgEpRegs     = bases->rcDbics;  
  CSL_RcCfgDbIcsRegs  *baseCfgRcCS2Regs  = bases->rcDbics2;
  CSL_EpCfgDbIcsRegs  *baseCfgEpCS2Regs  = bases->rcDbics2;  
  CSL_PcieRegs        *baseCfgTiConfRegs = bases->tiConf;
  CSL_PlConfRegs      *baseCfgPlRegs     = bases->plConf;
  
  pcieRet_e retVal = pcie_RET_OK;
  int32_t i;

  if (pcieLObjIsValid == 0) {
    return pcie_RET_NO_INIT;
  }

  pcie_check_handle(handle);

  /* Get base address for Local or Remote config space */
  if (location != pcie_LOCATION_LOCAL) 
  {
    char *remoteBase  = (char *)cfg->dataBase + bases->remoteOffset;
    uint32_t delta    = 0;
    baseCfgRcRegs     = (CSL_RcCfgDbIcsRegs *)(remoteBase + delta);
    baseCfgEpRegs     = (CSL_EpCfgDbIcsRegs *)(remoteBase + delta);
    delta             = (char *)bases->plConf - (char *)bases->rcDbics;
    baseCfgPlRegs     = (CSL_PlConfRegs *)    (remoteBase + delta);
  }

  /*****************************************************************************************
  * Reject hw rev 0 app registers (these are similar but not identical to TI CONF on rev 1)
  *****************************************************************************************/
  if (readRegs->pid) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->cmdStatus) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->cfgTrans) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->ioBase) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->tlpCfg) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->rstCmd) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->pmCmd) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->pmCfg) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->actStatus) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->obSize) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->diagCtrl) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->endian) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->priority) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->irqEOI) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->msiIrq) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->epIrqSet) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->epIrqClr) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->epIrqStatus) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  for (i = 0; i < 4; i++) {
    if (readRegs->genPurpose[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
  }
  for (i = 0; i < 8; i++) {
    if (readRegs->msiIrqStatusRaw[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (readRegs->msiIrqStatus[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (readRegs->msiIrqEnableSet[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (readRegs->msiIrqEnableClr[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
  }
  for (i = 0; i < 4; i++) {
    if (readRegs->legacyIrqStatusRaw[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (readRegs->legacyIrqStatus[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (readRegs->legacyIrqEnableSet[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (readRegs->legacyIrqEnableClr[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
  }
  if (readRegs->errIrqStatusRaw) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->errIrqStatus) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->errIrqEnableSet) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->errIrqEnableClr) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }

  if (readRegs->pmRstIrqStatusRaw) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->pmRstIrqStatus) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->pmRstIrqEnableSet) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->pmRstIrqEnableClr) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }

  for (i = 0; i < 8; i ++) {
    if (readRegs->obOffsetLo[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (readRegs->obOffsetHi[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
  }

  for (i = 0; i < 4; i ++) {
      if (readRegs->ibBar[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (readRegs->ibStartLo[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (readRegs->ibStartHi[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (readRegs->ibOffset[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
  }

  if (readRegs->pcsCfg0) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->pcsCfg1) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->pcsStatus) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }

  if (readRegs->serdesCfg0) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->serdesCfg1) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }

  /*****************************************************************************************
  *Configuration Registers
  *****************************************************************************************/

  /*Type 0, Type1 Common Registers*/

  if (readRegs->vndDevId) {
    pcie_check_result(retVal, pciev1_read_vndDevId_reg (&baseCfgEpRegs->DEVICE_VENDORID, readRegs->vndDevId));
  }
  if (readRegs->statusCmd) {
    pcie_check_result(retVal, pciev1_read_statusCmd_reg (&baseCfgEpRegs->STATUS_COMMAND_REGISTER, readRegs->statusCmd));
  }
  if (readRegs->revId) {
    pcie_check_result(retVal, pciev1_read_revId_reg (&baseCfgEpRegs->CLASSCODE_REVISIONID, readRegs->revId));
  }

  if (readRegs->bist) {
    pcie_check_result(retVal, pciev1_read_bist_reg (baseCfgEpRegs, readRegs->bist));
  }

  /*Type 0 Registers*/
  if (readRegs->type0BarIdx) {
    pcie_check_result(retVal, pciev1_read_type0Bar_reg (baseCfgEpRegs, &(readRegs->type0BarIdx->reg), 
                                                                       readRegs->type0BarIdx->idx));
  }
  if (readRegs->type0Bar32bitIdx) {
    pcie_check_result(retVal, pciev1_read_type0Bar32bit_reg (baseCfgEpRegs, &(readRegs->type0Bar32bitIdx->reg),
                                                                            readRegs->type0Bar32bitIdx->idx));
  }
  if (readRegs->type0BarMask32bitIdx) {
    pcie_check_result(retVal, pciev1_read_type0Bar32bit_reg (baseCfgEpCS2Regs, &(readRegs->type0BarMask32bitIdx->reg),
                                                                                 readRegs->type0BarMask32bitIdx->idx));
  }
  if (readRegs->subId) {
    pcie_check_result(retVal, pciev1_read_subId_reg (baseCfgEpRegs, readRegs->subId));
  }
  if (readRegs->cardbusCisPointer) {
    pcie_check_result(retVal, pciev1_read_cardbusCisPointer_reg (baseCfgEpRegs, readRegs->cardbusCisPointer));
  }
  if (readRegs->expRom) {
    pcie_check_result(retVal, pciev1_read_expRom_reg (baseCfgEpRegs, readRegs->expRom));
  }
  if (readRegs->capPtr) {
    pcie_check_result(retVal, pciev1_read_capPtr_reg (baseCfgEpRegs, readRegs->capPtr));
  }
  if (readRegs->intPin) {
    pcie_check_result(retVal, pciev1_read_intPin_reg (baseCfgEpRegs, readRegs->intPin));
  }

  /*Type 1 Registers*/
  if (readRegs->type1BistHeader) {
    pcie_check_result(retVal, pciev1_read_type1BistHeader_reg (baseCfgRcRegs, readRegs->type1BistHeader));
  }
  if (readRegs->type1BarIdx) {
    pcie_check_result(retVal, pciev1_read_type1Bar_reg (baseCfgRcRegs, &(readRegs->type1BarIdx->reg), 
                                                                       readRegs->type1BarIdx->idx));
  }
  if (readRegs->type1Bar32bitIdx) {
    pcie_check_result(retVal, pciev1_read_type1Bar32bit_reg (baseCfgRcRegs, &(readRegs->type1Bar32bitIdx->reg),
                                                                            readRegs->type1Bar32bitIdx->idx));
  }
  if (readRegs->type1BarMask32bitIdx) {
    pcie_check_result(retVal, pciev1_read_type1Bar32bit_reg (baseCfgRcCS2Regs, &(readRegs->type1BarMask32bitIdx->reg),
                                                                               readRegs->type1BarMask32bitIdx->idx));
  }
  if (readRegs->type1BusNum) {
    pcie_check_result(retVal, pciev1_read_type1BusNum_reg (baseCfgRcRegs, readRegs->type1BusNum));
  }
  if (readRegs->type1SecStat) {
    pcie_check_result(retVal, pciev1_read_type1SecStat_reg (baseCfgRcRegs, readRegs->type1SecStat));
  }
  if (readRegs->type1Memspace) {
    pcie_check_result(retVal, pciev1_read_type1Memspace_reg (baseCfgRcRegs, readRegs->type1Memspace));
  }
  if (readRegs->prefMem) {
    pcie_check_result(retVal, pciev1_read_prefMem_reg (baseCfgRcRegs, readRegs->prefMem));
  }
  if (readRegs->prefBaseUpper) {
    pcie_check_result(retVal, pciev1_read_prefBaseUpper_reg (baseCfgRcRegs, readRegs->prefBaseUpper));
  }
  if (readRegs->prefLimitUpper) {
    pcie_check_result(retVal, pciev1_read_prefLimitUpper_reg (baseCfgRcRegs, readRegs->prefLimitUpper));
  }
  if (readRegs->type1IOSpace) {
    pcie_check_result(retVal, pciev1_read_type1IOSpace_reg (baseCfgRcRegs, readRegs->type1IOSpace));
  }
  if (readRegs->type1CapPtr) {
    pcie_check_result(retVal, pciev1_read_type1CapPtr_reg (baseCfgRcRegs, readRegs->type1CapPtr));
  }
  if (readRegs->type1ExpnsnRom) {
    pcie_check_result(retVal, pciev1_read_type1ExpnsnRom_reg (baseCfgRcRegs, readRegs->type1ExpnsnRom));
  }
  if (readRegs->type1BridgeInt) {
    pcie_check_result(retVal, pciev1_read_type1BridgeInt_reg (baseCfgRcRegs, readRegs->type1BridgeInt));
  }

  /* Power Management Capabilities Registers */
  if (readRegs->pmCap) {
    pcie_check_result(retVal, pciev1_read_pmCap_reg (baseCfgEpRegs, readRegs->pmCap));
  }
  if (readRegs->pmCapCtlStat) {
    pcie_check_result(retVal, pciev1_read_pmCapCtlStat_reg (baseCfgEpRegs, readRegs->pmCapCtlStat));
  }

  /*MSI Registers*/
  if (readRegs->msiCap) {
    pcie_check_result(retVal, pciev1_read_msiCap_reg (baseCfgEpRegs, readRegs->msiCap));
  }
  if (readRegs->msiLo32) {
    pcie_check_result(retVal, pciev1_read_msiLo32_reg (baseCfgEpRegs, readRegs->msiLo32));
  }
  if (readRegs->msiUp32) {
    pcie_check_result(retVal, pciev1_read_msiUp32_reg (baseCfgEpRegs, readRegs->msiUp32));
  }
  if (readRegs->msiData) {
    pcie_check_result(retVal, pciev1_read_msiData_reg (baseCfgEpRegs, readRegs->msiData));
  }

  /*Capabilities Registers*/
  if (readRegs->pciesCap) {
    pcie_check_result(retVal, pciev1_read_pciesCap_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.PCIE_CAP, readRegs->pciesCap));
  }
  if (readRegs->deviceCap) {
    pcie_check_result(retVal, pciev1_read_deviceCap_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAP, readRegs->deviceCap));
  }
  if (readRegs->devStatCtrl) {
    pcie_check_result(retVal, pciev1_read_devStatCtrl_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAS, readRegs->devStatCtrl));
  }
  if (readRegs->linkCap) {
    pcie_check_result(retVal, pciev1_read_linkCap_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAP, readRegs->linkCap));
  }
  if (readRegs->linkStatCtrl) {
    pcie_check_result(retVal, pciev1_read_linkStatCtrl_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAS, readRegs->linkStatCtrl));
  }
  if (readRegs->slotCap) {
    pcie_check_result(retVal, pciev1_read_slotCap_reg (baseCfgRcRegs, readRegs->slotCap));
  }
  if (readRegs->slotStatCtrl) {
    pcie_check_result(retVal, pciev1_read_slotStatCtrl_reg (baseCfgRcRegs, readRegs->slotStatCtrl));
  }
  if (readRegs->rootCtrlCap) {
    pcie_check_result(retVal, pciev1_read_rootCtrlCap_reg (baseCfgRcRegs, readRegs->rootCtrlCap));
  }
  if (readRegs->rootStatus) {
    pcie_check_result(retVal, pciev1_read_rootStatus_reg (baseCfgRcRegs, readRegs->rootStatus));
  }
  if (readRegs->devCap2) {
    pcie_check_result(retVal, pciev1_read_devCap2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAP_2, readRegs->devCap2));
  }
  if (readRegs->devStatCtrl2) {
    pcie_check_result(retVal, pciev1_read_devStatCtrl2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAS_2, readRegs->devStatCtrl2));
  }
  if (readRegs->linkCap2) {
    pcie_check_result(retVal, pciev1_read_linkCap2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAP_2, readRegs->linkCap2));
  }
  if (readRegs->linkCtrl2) {
    pcie_check_result(retVal, pciev1_read_linkCtrl2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAS_2, readRegs->linkCtrl2));
  }


  /*Capabilities Extended Registers*/
  if (readRegs->extCap) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->uncErr) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->uncErrMask) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->uncErrSvrty) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->corErr) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->corErrMask) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->accr) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  for (i = 0; i < 4; i ++) {
    if (readRegs->hdrLog[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
  }
  if (readRegs->rootErrCmd) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->rootErrSt) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->errSrcID) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }

  /*Port Logic Registers*/
  if (readRegs->plAckTimer) {
    pcie_check_result(retVal, pciev1_read_plAckTimer_reg (baseCfgPlRegs, readRegs->plAckTimer));
  }
  if (readRegs->plOMsg) {
    pcie_check_result(retVal, pciev1_read_plOMsg_reg (baseCfgPlRegs, readRegs->plOMsg));
  }
  if (readRegs->plForceLink) {
    pcie_check_result(retVal, pciev1_read_plForceLink_reg (baseCfgPlRegs, readRegs->plForceLink));
  }
  if (readRegs->ackFreq) {
    pcie_check_result(retVal, pciev1_read_ackFreq_reg (baseCfgPlRegs, readRegs->ackFreq));
  }
  if (readRegs->lnkCtrl) {
    pcie_check_result(retVal, pciev1_read_lnkCtrl_reg (baseCfgPlRegs, readRegs->lnkCtrl));
  }
  if (readRegs->laneSkew) {
    pcie_check_result(retVal, pciev1_read_laneSkew_reg (baseCfgPlRegs, readRegs->laneSkew));
  }
  if (readRegs->symNum) {
    pcie_check_result(retVal, pciev1_read_symNum_reg (baseCfgPlRegs, readRegs->symNum));
  }
  if (readRegs->symTimerFltMask) {
    pcie_check_result(retVal, pciev1_read_symTimerFltMask_reg (baseCfgPlRegs, readRegs->symTimerFltMask));
  }
  if (readRegs->fltMask2) {
    pcie_check_result(retVal, pciev1_read_fltMask2_reg (baseCfgPlRegs, readRegs->fltMask2));
  }
  if (readRegs->debug0) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->debug1) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (readRegs->gen2) {
    pcie_check_result(retVal, pciev1_read_gen2_reg (baseCfgPlRegs, readRegs->gen2));
  }

  /* hw rev 1 PLCONF registers */
  if (readRegs->plconfObnpSubreqCtrl) {
    pcie_check_result(retVal, pciev1_read_plconfObnpSubreqCtrl_reg (baseCfgPlRegs, readRegs->plconfObnpSubreqCtrl));
  }
  if (readRegs->plconfTrPStsR) {
    pcie_check_result(retVal, pciev1_read_plconfTrPStsR_reg (baseCfgPlRegs, readRegs->plconfTrPStsR));
  }
  if (readRegs->plconfTrNpStsR) {
    pcie_check_result(retVal, pciev1_read_plconfTrNpStsR_reg (baseCfgPlRegs, readRegs->plconfTrNpStsR));
  }
  if (readRegs->plconfTrCStsR) {
    pcie_check_result(retVal, pciev1_read_plconfTrCStsR_reg (baseCfgPlRegs, readRegs->plconfTrCStsR));
  }
  if (readRegs->plconfQStsR) {
    pcie_check_result(retVal, pciev1_read_plconfQStsR_reg (baseCfgPlRegs, readRegs->plconfQStsR));
  }
  if (readRegs->plconfVcTrAR1) {
    pcie_check_result(retVal, pciev1_read_plconfVcTrAR1_reg (baseCfgPlRegs, readRegs->plconfVcTrAR1));
  }
  if (readRegs->plconfVcTrAR2) {
    pcie_check_result(retVal, pciev1_read_plconfVcTrAR2_reg (baseCfgPlRegs, readRegs->plconfVcTrAR2));
  }
  if (readRegs->plconfVc0PrQC) {
    pcie_check_result(retVal, pciev1_read_plconfVc0PrQC_reg (baseCfgPlRegs, readRegs->plconfVc0PrQC));
  }
  if (readRegs->plconfVc0NprQC) {
    pcie_check_result(retVal, pciev1_read_plconfVc0NprQC_reg (baseCfgPlRegs, readRegs->plconfVc0NprQC));
  }
  if (readRegs->plconfVc0CrQC) {
    pcie_check_result(retVal, pciev1_read_plconfVc0CrQC_reg (baseCfgPlRegs, readRegs->plconfVc0CrQC));
  }
  if (readRegs->plconfPhyStsR) {
    pcie_check_result(retVal, pciev1_read_plconfPhyStsR_reg (baseCfgPlRegs, readRegs->plconfPhyStsR));
  }
  if (readRegs->plconfPhyCtrlR) {
    pcie_check_result(retVal, pciev1_read_plconfPhyCtrlR_reg (baseCfgPlRegs, readRegs->plconfPhyCtrlR));
  }
  if (readRegs->plconfMsiCtrlAddress) {
    pcie_check_result(retVal, pciev1_read_plconfMsiCtrlAddress_reg (baseCfgPlRegs, readRegs->plconfMsiCtrlAddress));
  }
  if (readRegs->plconfMsiCtrlUpperAddress) {
    pcie_check_result(retVal, pciev1_read_plconfMsiCtrlUpperAddress_reg (baseCfgPlRegs, readRegs->plconfMsiCtrlUpperAddress));
  }
  for (i = 0; i < 8; i++) {
    if (readRegs->plconfMsiCtrlIntEnable[i]) {
      pcie_check_result(retVal, pciev1_read_plconfMsiCtrlIntEnable_reg (baseCfgPlRegs, readRegs->plconfMsiCtrlIntEnable[i], i));
    }
    if (readRegs->plconfMsiCtrlIntMask[i]) {
      pcie_check_result(retVal, pciev1_read_plconfMsiCtrlIntMask_reg (baseCfgPlRegs, readRegs->plconfMsiCtrlIntMask[i], i));
    }
    if (readRegs->plconfMsiCtrlIntStatus[i]) {
      pcie_check_result(retVal, pciev1_read_plconfMsiCtrlIntStatus_reg (baseCfgPlRegs, readRegs->plconfMsiCtrlIntStatus[i], i));
    }
  }
  if (readRegs->plconfMsiCtrlGpio) {
    pcie_check_result(retVal, pciev1_read_plconfMsiCtrlGpio_reg (baseCfgPlRegs, readRegs->plconfMsiCtrlGpio));
  }
  if (readRegs->plconfPipeLoopback) {
    pcie_check_result(retVal, pciev1_read_plconfPipeLoopback_reg (baseCfgPlRegs, readRegs->plconfPipeLoopback));
  }
  if (readRegs->plconfDbiRoWrEn) {
    pcie_check_result(retVal, pciev1_read_plconfDbiRoWrEn_reg (baseCfgPlRegs, readRegs->plconfDbiRoWrEn));
  }
  if (readRegs->plconfAxiSlvErrResp) {
    pcie_check_result(retVal, pciev1_read_plconfAxiSlvErrResp_reg (baseCfgPlRegs, readRegs->plconfAxiSlvErrResp));
  }
  if (readRegs->plconfAxiSlvTimeout) {
    pcie_check_result(retVal, pciev1_read_plconfAxiSlvTimeout_reg (baseCfgPlRegs, readRegs->plconfAxiSlvTimeout));
  }
  if (readRegs->plconfIatuIndex) {
    pcie_check_result(retVal, pciev1_read_plconfIatuIndex_reg (baseCfgPlRegs, readRegs->plconfIatuIndex));
  }
  if (readRegs->plconfIatuRegCtrl1) {
    pcie_check_result(retVal, pciev1_read_plconfIatuRegCtrl1_reg (baseCfgPlRegs, readRegs->plconfIatuRegCtrl1));
  }
  if (readRegs->plconfIatuRegCtrl2) {
    pcie_check_result(retVal, pciev1_read_plconfIatuRegCtrl2_reg (baseCfgPlRegs, readRegs->plconfIatuRegCtrl2));
  }
  if (readRegs->plconfIatuRegLowerBase) {
    pcie_check_result(retVal, pciev1_read_plconfIatuRegLowerBase_reg (baseCfgPlRegs, readRegs->plconfIatuRegLowerBase));
  }
  if (readRegs->plconfIatuRegUpperBase) {
    pcie_check_result(retVal, pciev1_read_plconfIatuRegUpperBase_reg (baseCfgPlRegs, readRegs->plconfIatuRegUpperBase));
  }
  if (readRegs->plconfIatuRegLimit) {
    pcie_check_result(retVal, pciev1_read_plconfIatuRegLimit_reg (baseCfgPlRegs, readRegs->plconfIatuRegLimit));
  }
  if (readRegs->plconfIatuRegLowerTarget) {
    pcie_check_result(retVal, pciev1_read_plconfIatuRegLowerTarget_reg (baseCfgPlRegs, readRegs->plconfIatuRegLowerTarget));
  }
  if (readRegs->plconfIatuRegUpperTarget) {
    pcie_check_result(retVal, pciev1_read_plconfIatuRegUpperTarget_reg (baseCfgPlRegs, readRegs->plconfIatuRegUpperTarget));
  }
  if (readRegs->plconfIatuRegCtrl3) {
    pcie_check_result(retVal, pciev1_read_plconfIatuRegCtrl3_reg (baseCfgPlRegs, readRegs->plconfIatuRegCtrl3));
  }


  /* TI CONF registers */
  if (readRegs->tiConfRevision) {
    pcie_check_result(retVal, pciev1_read_tiConfRevision_reg (baseCfgTiConfRegs, readRegs->tiConfRevision));
  }
  if (readRegs->tiConfSysConfig) {
    pcie_check_result(retVal, pciev1_read_tiConfSysConfig_reg (baseCfgTiConfRegs, readRegs->tiConfSysConfig));
  }
  if (readRegs->tiConfIrqEoi) {
    pcie_check_result(retVal, pciev1_read_tiConfIrqEoi_reg (baseCfgTiConfRegs, readRegs->tiConfIrqEoi));
  }
  if (readRegs->tiConfIrqStatusRawMain) {
    pcie_check_result(retVal, pciev1_read_tiConfIrqStatusRawMain_reg (baseCfgTiConfRegs, readRegs->tiConfIrqStatusRawMain));
  }
  if (readRegs->tiConfIrqStatusMain) {
    pcie_check_result(retVal, pciev1_read_tiConfIrqStatusMain_reg (baseCfgTiConfRegs, readRegs->tiConfIrqStatusMain));
  }
  if (readRegs->tiConfIrqEnableSetMain) {
    pcie_check_result(retVal, pciev1_read_tiConfIrqEnableSetMain_reg (baseCfgTiConfRegs, readRegs->tiConfIrqEnableSetMain));
  }
  if (readRegs->tiConfIrqEnableClrMain) {
    pcie_check_result(retVal, pciev1_read_tiConfIrqEnableClrMain_reg (baseCfgTiConfRegs, readRegs->tiConfIrqEnableClrMain));
  }
  if (readRegs->tiConfIrqStatusRawMsi) {
    pcie_check_result(retVal, pciev1_read_tiConfIrqStatusRawMsi_reg (baseCfgTiConfRegs, readRegs->tiConfIrqStatusRawMsi));
  }
  if (readRegs->tiConfIrqStatusMsi) {
    pcie_check_result(retVal, pciev1_read_tiConfIrqStatusMsi_reg (baseCfgTiConfRegs, readRegs->tiConfIrqStatusMsi));
  }
  if (readRegs->tiConfIrqEnableSetMsi) {
    pcie_check_result(retVal, pciev1_read_tiConfIrqEnableSetMsi_reg (baseCfgTiConfRegs, readRegs->tiConfIrqEnableSetMsi));
  }
  if (readRegs->tiConfIrqEnableClrMsi) {
    pcie_check_result(retVal, pciev1_read_tiConfIrqEnableClrMsi_reg (baseCfgTiConfRegs, readRegs->tiConfIrqEnableClrMsi));
  }
  if (readRegs->tiConfDeviceType) {
    pcie_check_result(retVal, pciev1_read_tiConfDeviceType_reg (baseCfgTiConfRegs, readRegs->tiConfDeviceType));
  }
  if (readRegs->tiConfDeviceCmd) {
    pcie_check_result(retVal, pciev1_read_tiConfDeviceCmd_reg (baseCfgTiConfRegs, readRegs->tiConfDeviceCmd));
  }
  if (readRegs->tiConfPmCtrl) {
    pcie_check_result(retVal, pciev1_read_tiConfPmCtrl_reg (baseCfgTiConfRegs, readRegs->tiConfPmCtrl));
  }
  if (readRegs->tiConfPhyCs) {
    pcie_check_result(retVal, pciev1_read_tiConfPhyCs_reg (baseCfgTiConfRegs, readRegs->tiConfPhyCs));
  }
  if (readRegs->tiConfIntxAssert) {
    pcie_check_result(retVal, pciev1_read_tiConfIntxAssert_reg (baseCfgTiConfRegs, readRegs->tiConfIntxAssert));
  }
  if (readRegs->tiConfIntxDeassert) {
    pcie_check_result(retVal, pciev1_read_tiConfIntxDeassert_reg (baseCfgTiConfRegs, readRegs->tiConfIntxDeassert));
  }
  if (readRegs->tiConfMsiXmt) {
    pcie_check_result(retVal, pciev1_read_tiConfMsiXmt_reg (baseCfgTiConfRegs, readRegs->tiConfMsiXmt));
  }
  if (readRegs->tiConfDebugCfg) {
    pcie_check_result(retVal, pciev1_read_tiConfDebugCfg_reg (baseCfgTiConfRegs, readRegs->tiConfDebugCfg));
  }
  if (readRegs->tiConfDebugData) {
    pcie_check_result(retVal, pciev1_read_tiConfDebugData_reg (baseCfgTiConfRegs, readRegs->tiConfDebugData));
  }
  if (readRegs->tiConfDiagCtrl) {
    pcie_check_result(retVal, pciev1_read_tiConfDiagCtrl_reg (baseCfgTiConfRegs, readRegs->tiConfDiagCtrl));
  }

  return retVal;
} /* Pciev1_readRegs */


/*********************************************************************
 * FUNCTION PURPOSE: Writes any register
 ********************************************************************/
pcieRet_e Pciev1_writeRegs 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *writeRegs /**< [in] List of registers to write */
)
{
  Pcie_DeviceCfgBaseAddr *cfg = pcie_handle_to_cfg (handle);
  Pciev1_DeviceCfgBaseAddrs *bases = cfg->cfgBase;

  /* Base Address for the Config Space
     These registers can be Local/Remote and Type0(EP)/Type1(RC) */
  CSL_RcCfgDbIcsRegs  *baseCfgRcRegs     = bases->rcDbics;
  CSL_EpCfgDbIcsRegs  *baseCfgEpRegs     = bases->rcDbics;  
  CSL_RcCfgDbIcsRegs  *baseCfgRcCS2Regs  = bases->rcDbics2;
  CSL_EpCfgDbIcsRegs  *baseCfgEpCS2Regs  = bases->rcDbics2;  
  CSL_PcieRegs        *baseCfgTiConfRegs = bases->tiConf;
  CSL_PlConfRegs      *baseCfgPlRegs     = bases->plConf;
  
  pcieRet_e retVal = pcie_RET_OK;
  int32_t i;

  if (pcieLObjIsValid == 0) {
    return pcie_RET_NO_INIT;
  }

  pcie_check_handle(handle);

  /* Get base address for Local/Remote config space */
  if (location != pcie_LOCATION_LOCAL) 
  {
    char *remoteBase  = (char *)cfg->dataBase + bases->remoteOffset;
    uint32_t delta    = 0;
    baseCfgRcRegs     = (CSL_RcCfgDbIcsRegs *)(remoteBase + delta);
    baseCfgEpRegs     = (CSL_EpCfgDbIcsRegs *)(remoteBase + delta);
    delta             = (char *)bases->plConf - (char *)bases->rcDbics;
    baseCfgPlRegs     = (CSL_PlConfRegs *)    (remoteBase + delta);
  }

  /*****************************************************************************************
  * Reject hw rev 0 app registers (these are similar but not identical to TI CONF on rev 1)
  *****************************************************************************************/
  if (writeRegs->cmdStatus) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->cfgTrans) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->ioBase) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->tlpCfg) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->rstCmd) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->pmCmd) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->pmCfg) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->obSize) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->diagCtrl) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->endian) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->priority) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->irqEOI) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->msiIrq) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->epIrqSet) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->epIrqClr) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->epIrqStatus) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  for (i = 0; i < 4; i++) {
    if (writeRegs->genPurpose[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
  }
  for (i = 0; i < 8; i++) {
    if (writeRegs->msiIrqStatusRaw[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (writeRegs->msiIrqStatus[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (writeRegs->msiIrqEnableSet[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (writeRegs->msiIrqEnableClr[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
  }
  for (i = 0; i < 4; i++) {
    if (writeRegs->legacyIrqStatusRaw[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (writeRegs->legacyIrqStatus[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (writeRegs->legacyIrqEnableSet[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (writeRegs->legacyIrqEnableClr[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
  }
  if (writeRegs->errIrqStatusRaw) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->errIrqStatus) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->errIrqEnableSet) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->errIrqEnableClr) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }

  if (writeRegs->pmRstIrqStatusRaw) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->pmRstIrqStatus) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->pmRstIrqEnableSet) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->pmRstIrqEnableClr) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }

  for (i = 0; i < 8; i ++) {
    if (writeRegs->obOffsetLo[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (writeRegs->obOffsetHi[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
  }

  for (i = 0; i < 4; i ++) {
    if (writeRegs->ibBar[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (writeRegs->ibStartLo[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (writeRegs->ibStartHi[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
    if (writeRegs->ibOffset[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
  }

  if (writeRegs->pcsCfg0) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->pcsCfg1) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }

  if (writeRegs->serdesCfg0) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->serdesCfg1) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }

  /*****************************************************************************************
  *Configuration Registers
  *****************************************************************************************/

  /*Type 0, Type1 Common Registers*/

  if (writeRegs->vndDevId) {
    pcie_check_result(retVal, pciev1_write_vndDevId_reg (&baseCfgEpRegs->DEVICE_VENDORID, writeRegs->vndDevId));
  }
  if (writeRegs->statusCmd) {
    pcie_check_result(retVal, pciev1_write_statusCmd_reg (&baseCfgEpRegs->STATUS_COMMAND_REGISTER, writeRegs->statusCmd));
  }
  if (writeRegs->revId) {
    pcie_check_result(retVal, pciev1_write_revId_reg (&baseCfgEpRegs->CLASSCODE_REVISIONID, writeRegs->revId));
  }

  if (writeRegs->bist) {
    pcie_check_result(retVal, pciev1_write_bist_reg (baseCfgEpRegs, writeRegs->bist));
  }

  /*Type 0 Registers*/
  if (writeRegs->type0BarIdx) {
    pcie_check_result(retVal, pciev1_write_type0Bar_reg (baseCfgEpRegs, &(writeRegs->type0BarIdx->reg), 
                                                                       writeRegs->type0BarIdx->idx));
  }
  if (writeRegs->type0BarMask32bitIdx) {
    pcie_check_result(retVal, pciev1_write_type0Bar32bit_reg (baseCfgEpCS2Regs, &(writeRegs->type0BarMask32bitIdx->reg),
                                                                                writeRegs->type0BarMask32bitIdx->idx));
  }
  if (writeRegs->type0Bar32bitIdx) {
    pcie_check_result(retVal, pciev1_write_type0Bar32bit_reg (baseCfgEpRegs, &(writeRegs->type0Bar32bitIdx->reg),
                                                                            writeRegs->type0Bar32bitIdx->idx));
  }
  if (writeRegs->subId) {
    pcie_check_result(retVal, pciev1_write_subId_reg (baseCfgEpRegs, writeRegs->subId));
  }
  if (writeRegs->cardbusCisPointer) {
    pcie_check_result(retVal, pciev1_write_cardbusCisPointer_reg (baseCfgEpRegs, writeRegs->cardbusCisPointer));
  }
  if (writeRegs->expRom) {
    pcie_check_result(retVal, pciev1_write_expRom_reg (baseCfgEpRegs, writeRegs->expRom));
  }
  if (writeRegs->capPtr) {
    pcie_check_result(retVal, pciev1_write_capPtr_reg (baseCfgEpRegs, writeRegs->capPtr));
  }
  if (writeRegs->intPin) {
    pcie_check_result(retVal, pciev1_write_intPin_reg (baseCfgEpRegs, writeRegs->intPin));
  }

  /*Type 1 Registers*/
  if (writeRegs->type1BistHeader) {
    pcie_check_result(retVal, pciev1_write_type1BistHeader_reg (baseCfgRcRegs, writeRegs->type1BistHeader));
  }
  if (writeRegs->type1BarIdx) {
    pcie_check_result(retVal, pciev1_write_type1Bar_reg (baseCfgRcRegs, &(writeRegs->type1BarIdx->reg), 
                                                                       writeRegs->type1BarIdx->idx));
  }
  if (writeRegs->type1BarMask32bitIdx) {
    pcie_check_result(retVal, pciev1_write_type1Bar32bit_reg (baseCfgRcCS2Regs, &(writeRegs->type1BarMask32bitIdx->reg),
                                                                                writeRegs->type1BarMask32bitIdx->idx));
  }
  if (writeRegs->type1Bar32bitIdx) {
    pcie_check_result(retVal, pciev1_write_type1Bar32bit_reg (baseCfgRcRegs, &(writeRegs->type1Bar32bitIdx->reg),
                                                                            writeRegs->type1Bar32bitIdx->idx));
  }
  if (writeRegs->type1BusNum) {
    pcie_check_result(retVal, pciev1_write_type1BusNum_reg (baseCfgRcRegs, writeRegs->type1BusNum));
  }
  if (writeRegs->type1SecStat) {
    pcie_check_result(retVal, pciev1_write_type1SecStat_reg (baseCfgRcRegs, writeRegs->type1SecStat));
  }
  if (writeRegs->type1Memspace) {
    pcie_check_result(retVal, pciev1_write_type1Memspace_reg (baseCfgRcRegs, writeRegs->type1Memspace));
  }
  if (writeRegs->prefMem) {
    pcie_check_result(retVal, pciev1_write_prefMem_reg (baseCfgRcRegs, writeRegs->prefMem));
  }
  if (writeRegs->prefBaseUpper) {
    pcie_check_result(retVal, pciev1_write_prefBaseUpper_reg (baseCfgRcRegs, writeRegs->prefBaseUpper));
  }
  if (writeRegs->prefLimitUpper) {
    pcie_check_result(retVal, pciev1_write_prefLimitUpper_reg (baseCfgRcRegs, writeRegs->prefLimitUpper));
  }
  if (writeRegs->type1IOSpace) {
    pcie_check_result(retVal, pciev1_write_type1IOSpace_reg (baseCfgRcRegs, writeRegs->type1IOSpace));
  }
  if (writeRegs->type1CapPtr) {
    pcie_check_result(retVal, pciev1_write_type1CapPtr_reg (baseCfgRcRegs, writeRegs->type1CapPtr));
  }
  if (writeRegs->type1ExpnsnRom) {
    pcie_check_result(retVal, pciev1_write_type1ExpnsnRom_reg (baseCfgRcRegs, writeRegs->type1ExpnsnRom));
  }
  if (writeRegs->type1BridgeInt) {
    pcie_check_result(retVal, pciev1_write_type1BridgeInt_reg (baseCfgRcRegs, writeRegs->type1BridgeInt));
  }

  /* Power Management Capabilities Registers */
  if (writeRegs->pmCap) {
    pcie_check_result(retVal, pciev1_write_pmCap_reg (baseCfgEpRegs, writeRegs->pmCap));
  }
  if (writeRegs->pmCapCtlStat) {
    pcie_check_result(retVal, pciev1_write_pmCapCtlStat_reg (baseCfgEpRegs, writeRegs->pmCapCtlStat));
  }

  /*MSI Registers*/
  if (writeRegs->msiCap) {
    pcie_check_result(retVal, pciev1_write_msiCap_reg (baseCfgEpRegs, writeRegs->msiCap));
  }
  if (writeRegs->msiLo32) {
    pcie_check_result(retVal, pciev1_write_msiLo32_reg (baseCfgEpRegs, writeRegs->msiLo32));
  }
  if (writeRegs->msiUp32) {
    pcie_check_result(retVal, pciev1_write_msiUp32_reg (baseCfgEpRegs, writeRegs->msiUp32));
  }
  if (writeRegs->msiData) {
    pcie_check_result(retVal, pciev1_write_msiData_reg (baseCfgEpRegs, writeRegs->msiData));
  }

  /*Capabilities Registers*/
  if (writeRegs->pciesCap) {
    pcie_check_result(retVal, pciev1_write_pciesCap_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.PCIE_CAP, writeRegs->pciesCap));
  }
  if (writeRegs->deviceCap) {
    pcie_check_result(retVal, pciev1_write_deviceCap_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAP, writeRegs->deviceCap));
  }

  if (writeRegs->devStatCtrl) {
    pcie_check_result(retVal, pciev1_write_devStatCtrl_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAS, writeRegs->devStatCtrl));
  }
  if (writeRegs->linkCap) {
    pcie_check_result(retVal, pciev1_write_linkCap_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAP, writeRegs->linkCap));
  }
  if (writeRegs->linkStatCtrl) {
    pcie_check_result(retVal, pciev1_write_linkStatCtrl_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAS, writeRegs->linkStatCtrl));
  }
  if (writeRegs->slotCap) {
    pcie_check_result(retVal, pciev1_write_slotCap_reg (baseCfgRcRegs, writeRegs->slotCap));
  }
  if (writeRegs->slotStatCtrl) {
    pcie_check_result(retVal, pciev1_write_slotStatCtrl_reg (baseCfgRcRegs, writeRegs->slotStatCtrl));
  }
  if (writeRegs->rootCtrlCap) {
    pcie_check_result(retVal, pciev1_write_rootCtrlCap_reg (baseCfgRcRegs, writeRegs->rootCtrlCap));
  }
  if (writeRegs->rootStatus) {
    pcie_check_result(retVal, pciev1_write_rootStatus_reg (baseCfgRcRegs, writeRegs->rootStatus));
  }
  if (writeRegs->devCap2) {
    pcie_check_result(retVal, pciev1_write_devCap2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAP_2, writeRegs->devCap2));
  }
  if (writeRegs->devStatCtrl2) {
    pcie_check_result(retVal, pciev1_write_devStatCtrl2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.DEV_CAS_2, writeRegs->devStatCtrl2));
  }
  if (writeRegs->linkCap2) {
    pcie_check_result(retVal, pciev1_write_linkCap2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAP_2, writeRegs->linkCap2));
  }
  if (writeRegs->linkCtrl2) {
    pcie_check_result(retVal, pciev1_write_linkCtrl2_reg (&baseCfgEpRegs->PCIE_CAP_STRUC.LNK_CAS_2, writeRegs->linkCtrl2));
  }

  /*Capabilities Extended Registers*/
  if (writeRegs->extCap) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->uncErr) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->uncErrMask) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->uncErrSvrty) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->corErr) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->corErrMask) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->accr) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  for (i = 0; i < 4; i ++) {
    if (writeRegs->hdrLog[i]) {
      /* Not supported on rev 1 */
      pcie_check_result(retVal, pcie_RET_INV_REG);
    }
  }
  if (writeRegs->rootErrCmd) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->rootErrSt) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->errSrcID) {
    /* Not supported on rev 1 */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }

  /*Port Logic Registers*/
  if (writeRegs->plAckTimer) {
    pcie_check_result(retVal, pciev1_write_plAckTimer_reg (baseCfgPlRegs, writeRegs->plAckTimer));
  }
  if (writeRegs->plOMsg) {
    pcie_check_result(retVal, pciev1_write_plOMsg_reg (baseCfgPlRegs, writeRegs->plOMsg));
  }
  if (writeRegs->plForceLink) {
    pcie_check_result(retVal, pciev1_write_plForceLink_reg (baseCfgPlRegs, writeRegs->plForceLink));
  }
  if (writeRegs->ackFreq) {
    pcie_check_result(retVal, pciev1_write_ackFreq_reg (baseCfgPlRegs, writeRegs->ackFreq));
  }
  if (writeRegs->lnkCtrl) {
    pcie_check_result(retVal, pciev1_write_lnkCtrl_reg (baseCfgPlRegs, writeRegs->lnkCtrl));
  }
  if (writeRegs->laneSkew) {
    pcie_check_result(retVal, pciev1_write_laneSkew_reg (baseCfgPlRegs, writeRegs->laneSkew));
  }
  if (writeRegs->symNum) {
    pcie_check_result(retVal, pciev1_write_symNum_reg (baseCfgPlRegs, writeRegs->symNum));
  }
  if (writeRegs->symTimerFltMask) {
    pcie_check_result(retVal, pciev1_write_symTimerFltMask_reg (baseCfgPlRegs, writeRegs->symTimerFltMask));
  }
  if (writeRegs->fltMask2) {
    pcie_check_result(retVal, pciev1_write_fltMask2_reg (baseCfgPlRegs, writeRegs->fltMask2));
  }
  if (writeRegs->gen2) {
    pcie_check_result(retVal, pciev1_write_gen2_reg (baseCfgPlRegs, writeRegs->gen2));
  }

  /* hw rev 1 PLCONF registers */
  if (writeRegs->plconfObnpSubreqCtrl) {
    pcie_check_result(retVal, pciev1_write_plconfObnpSubreqCtrl_reg (baseCfgPlRegs, writeRegs->plconfObnpSubreqCtrl));
  }
  if (writeRegs->plconfTrPStsR) {
    /* Pure RO register */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->plconfTrNpStsR) {
    /* Pure RO register */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->plconfTrCStsR) {
    /* Pure RO register */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->plconfQStsR) {
    pcie_check_result(retVal, pciev1_write_plconfQStsR_reg (baseCfgPlRegs, writeRegs->plconfQStsR));
  }
  if (writeRegs->plconfVcTrAR1) {
    /* Pure RO register */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->plconfVcTrAR2) {
    /* Pure RO register */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->plconfVc0PrQC) {
    pcie_check_result(retVal, pciev1_write_plconfVc0PrQC_reg (baseCfgPlRegs, writeRegs->plconfVc0PrQC));
  }
  if (writeRegs->plconfVc0NprQC) {
    pcie_check_result(retVal, pciev1_write_plconfVc0NprQC_reg (baseCfgPlRegs, writeRegs->plconfVc0NprQC));
  }
  if (writeRegs->plconfVc0CrQC) {
    pcie_check_result(retVal, pciev1_write_plconfVc0CrQC_reg (baseCfgPlRegs, writeRegs->plconfVc0CrQC));
  }
  if (writeRegs->plconfPhyStsR) {
    /* Pure RO register */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->plconfPhyCtrlR) {
    pcie_check_result(retVal, pciev1_write_plconfPhyCtrlR_reg (baseCfgPlRegs, writeRegs->plconfPhyCtrlR));
  }
  if (writeRegs->plconfMsiCtrlAddress) {
    pcie_check_result(retVal, pciev1_write_plconfMsiCtrlAddress_reg (baseCfgPlRegs, writeRegs->plconfMsiCtrlAddress));
  }
  if (writeRegs->plconfMsiCtrlUpperAddress) {
    pcie_check_result(retVal, pciev1_write_plconfMsiCtrlUpperAddress_reg (baseCfgPlRegs, writeRegs->plconfMsiCtrlUpperAddress));
  }
  for (i = 0; i < 8; i++) {
    if (writeRegs->plconfMsiCtrlIntEnable[i]) {
      pcie_check_result(retVal, pciev1_write_plconfMsiCtrlIntEnable_reg (baseCfgPlRegs, writeRegs->plconfMsiCtrlIntEnable[i], i));
    }
    if (writeRegs->plconfMsiCtrlIntMask[i]) {
      pcie_check_result(retVal, pciev1_write_plconfMsiCtrlIntMask_reg (baseCfgPlRegs, writeRegs->plconfMsiCtrlIntMask[i], i));
    }
    if (writeRegs->plconfMsiCtrlIntStatus[i]) {
      pcie_check_result(retVal, pciev1_write_plconfMsiCtrlIntStatus_reg (baseCfgPlRegs, writeRegs->plconfMsiCtrlIntStatus[i], i));
    }
  }
  if (writeRegs->plconfMsiCtrlGpio) {
    pcie_check_result(retVal, pciev1_write_plconfMsiCtrlGpio_reg (baseCfgPlRegs, writeRegs->plconfMsiCtrlGpio));
  }
  if (writeRegs->plconfPipeLoopback) {
    pcie_check_result(retVal, pciev1_write_plconfPipeLoopback_reg (baseCfgPlRegs, writeRegs->plconfPipeLoopback));
  }
  if (writeRegs->plconfDbiRoWrEn) {
    pcie_check_result(retVal, pciev1_write_plconfDbiRoWrEn_reg (baseCfgPlRegs, writeRegs->plconfDbiRoWrEn));
  }
  if (writeRegs->plconfAxiSlvErrResp) {
    pcie_check_result(retVal, pciev1_write_plconfAxiSlvErrResp_reg (baseCfgPlRegs, writeRegs->plconfAxiSlvErrResp));
  }
  if (writeRegs->plconfAxiSlvTimeout) {
    pcie_check_result(retVal, pciev1_write_plconfAxiSlvTimeout_reg (baseCfgPlRegs, writeRegs->plconfAxiSlvTimeout));
  }
  if (writeRegs->plconfIatuIndex) {
    pcie_check_result(retVal, pciev1_write_plconfIatuIndex_reg (baseCfgPlRegs, writeRegs->plconfIatuIndex));
  }
  if (writeRegs->plconfIatuRegCtrl2) {
    pcie_check_result(retVal, pciev1_write_plconfIatuRegCtrl2_reg (baseCfgPlRegs, writeRegs->plconfIatuRegCtrl2));
  }
  if (writeRegs->plconfIatuRegLowerBase) {
    pcie_check_result(retVal, pciev1_write_plconfIatuRegLowerBase_reg (baseCfgPlRegs, writeRegs->plconfIatuRegLowerBase));
  }
  if (writeRegs->plconfIatuRegUpperBase) {
    pcie_check_result(retVal, pciev1_write_plconfIatuRegUpperBase_reg (baseCfgPlRegs, writeRegs->plconfIatuRegUpperBase));
  }
  if (writeRegs->plconfIatuRegLimit) {
    pcie_check_result(retVal, pciev1_write_plconfIatuRegLimit_reg (baseCfgPlRegs, writeRegs->plconfIatuRegLimit));
  }
  if (writeRegs->plconfIatuRegLowerTarget) {
    pcie_check_result(retVal, pciev1_write_plconfIatuRegLowerTarget_reg (baseCfgPlRegs, writeRegs->plconfIatuRegLowerTarget));
  }
  if (writeRegs->plconfIatuRegUpperTarget) {
    pcie_check_result(retVal, pciev1_write_plconfIatuRegUpperTarget_reg (baseCfgPlRegs, writeRegs->plconfIatuRegUpperTarget));
  }
  if (writeRegs->plconfIatuRegCtrl3) {
    /* Pure RO register */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  /* Ctrl1 is done last since it has enable bit */
  if (writeRegs->plconfIatuRegCtrl1) {
    pcie_check_result(retVal, pciev1_write_plconfIatuRegCtrl1_reg (baseCfgPlRegs, writeRegs->plconfIatuRegCtrl1));
  }

  /* TI CONF registers */
  if (writeRegs->tiConfRevision) {
    /* Pure RO register */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->tiConfSysConfig) {
    pcie_check_result(retVal, pciev1_write_tiConfSysConfig_reg (baseCfgTiConfRegs, writeRegs->tiConfSysConfig));
  }
  if (writeRegs->tiConfIrqEoi) {
    pcie_check_result(retVal, pciev1_write_tiConfIrqEoi_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqEoi));
  }
  if (writeRegs->tiConfIrqStatusRawMain) {
    pcie_check_result(retVal, pciev1_write_tiConfIrqStatusRawMain_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqStatusRawMain));
  }
  if (writeRegs->tiConfIrqStatusMain) {
    pcie_check_result(retVal, pciev1_write_tiConfIrqStatusMain_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqStatusMain));
  }
  if (writeRegs->tiConfIrqEnableSetMain) {
    pcie_check_result(retVal, pciev1_write_tiConfIrqEnableSetMain_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqEnableSetMain));
  }
  if (writeRegs->tiConfIrqEnableClrMain) {
    pcie_check_result(retVal, pciev1_write_tiConfIrqEnableClrMain_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqEnableClrMain));
  }
  if (writeRegs->tiConfIrqStatusRawMsi) {
    pcie_check_result(retVal, pciev1_write_tiConfIrqStatusRawMsi_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqStatusRawMsi));
  }
  if (writeRegs->tiConfIrqStatusMsi) {
    pcie_check_result(retVal, pciev1_write_tiConfIrqStatusMsi_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqStatusMsi));
  }
  if (writeRegs->tiConfIrqEnableSetMsi) {
    pcie_check_result(retVal, pciev1_write_tiConfIrqEnableSetMsi_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqEnableSetMsi));
  }
  if (writeRegs->tiConfIrqEnableClrMsi) {
    pcie_check_result(retVal, pciev1_write_tiConfIrqEnableClrMsi_reg (baseCfgTiConfRegs, writeRegs->tiConfIrqEnableClrMsi));
  }
  if (writeRegs->tiConfDeviceType) {
    pcie_check_result(retVal, pciev1_write_tiConfDeviceType_reg (baseCfgTiConfRegs, writeRegs->tiConfDeviceType));
  }
  if (writeRegs->tiConfDeviceCmd) {
    pcie_check_result(retVal, pciev1_write_tiConfDeviceCmd_reg (baseCfgTiConfRegs, writeRegs->tiConfDeviceCmd));
  }
  if (writeRegs->tiConfPmCtrl) {
    pcie_check_result(retVal, pciev1_write_tiConfPmCtrl_reg (baseCfgTiConfRegs, writeRegs->tiConfPmCtrl));
  }
  if (writeRegs->tiConfPhyCs) {
    pcie_check_result(retVal, pciev1_write_tiConfPhyCs_reg (baseCfgTiConfRegs, writeRegs->tiConfPhyCs));
  }
  if (writeRegs->tiConfIntxAssert) {
    pcie_check_result(retVal, pciev1_write_tiConfIntxAssert_reg (baseCfgTiConfRegs, writeRegs->tiConfIntxAssert));
  }
  if (writeRegs->tiConfIntxDeassert) {
    pcie_check_result(retVal, pciev1_write_tiConfIntxDeassert_reg (baseCfgTiConfRegs, writeRegs->tiConfIntxDeassert));
  }
  if (writeRegs->tiConfMsiXmt) {
    pcie_check_result(retVal, pciev1_write_tiConfMsiXmt_reg (baseCfgTiConfRegs, writeRegs->tiConfMsiXmt));
  }
  if (writeRegs->tiConfDebugCfg) {
    pcie_check_result(retVal, pciev1_write_tiConfDebugCfg_reg (baseCfgTiConfRegs, writeRegs->tiConfDebugCfg));
  }
  if (writeRegs->tiConfDebugData) {
    /* Pure RO register */
    pcie_check_result(retVal, pcie_RET_INV_REG);
  }
  if (writeRegs->tiConfDiagCtrl) {
    pcie_check_result(retVal, pciev1_write_tiConfDiagCtrl_reg (baseCfgTiConfRegs, writeRegs->tiConfDiagCtrl));
  }

  return retVal;
} /* Pciev1_writeRegs */


/*********************************************************************
 * FUNCTION PURPOSE: Configures a BAR Register (32bits)
 ********************************************************************/
pcieRet_e Pciev1_cfgBar 
(
  Pcie_Handle            handle,   /**< [in] The PCIE LLD instance identifier */
  const pcieBarCfg_t    *barCfg    /**< [in] BAR configuration parameters */
)
{
  pcieRet_e          retVal = pcie_RET_OK;
  pcieType0BarIdx_t  type0BarIdx;  
  pcieType1BarIdx_t  type1BarIdx;  
  pcieRegisters_t    setRegs;
  uint32_t           barAddrField = 0;

  if (pcieLObjIsValid == 0) {
    return pcie_RET_NO_INIT;
  }

  pcie_check_handle(handle);
  
  memset (&setRegs,     0, sizeof(setRegs));
  memset (&type0BarIdx, 0, sizeof(type0BarIdx));
  memset (&type1BarIdx, 0, sizeof(type1BarIdx));

  if(barCfg->mode == pcie_RC_MODE)
  {
    pcie_getbits(barCfg->base, PCIE_RC_BAR_BASE_FULL, barAddrField);

    type1BarIdx.reg.base     = barAddrField;
    type1BarIdx.reg.prefetch = barCfg->prefetch;
    type1BarIdx.reg.type     = barCfg->type;
    type1BarIdx.reg.memSpace = barCfg->memSpace;
    type1BarIdx.idx          = barCfg->idx;

    setRegs.type1BarIdx = &type1BarIdx;   
  }
  else
  {
    pcie_getbits(barCfg->base, PCIE_EP_BAR_BASE_FULL, barAddrField);

    type0BarIdx.reg.base     = barAddrField;
    type0BarIdx.reg.prefetch = barCfg->prefetch;
    type0BarIdx.reg.type     = barCfg->type;
    type0BarIdx.reg.memSpace = barCfg->memSpace;
    type0BarIdx.idx          = barCfg->idx;

    setRegs.type0BarIdx = &type0BarIdx;   
  }

  retVal = Pciev1_writeRegs (handle, barCfg->location, &setRegs);

  return retVal;
} /* Pciev1_cfgBar */


/*********************************************************************
 * FUNCTION PURPOSE: Configures an ATU (address translation) region
 ********************************************************************/
pcieRet_e Pciev1_atuRegionConfig 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] local/remote */
  uint32_t         atuRegionIndex, /* [in] index number to configure */
  const            pcieAtuRegionParams_t *atuRegionParams /* [in] config structure */
)
{
  pcieRet_e                         retVal = pcie_RET_OK;
  pciePlconfIatuIndexReg_t          index;
  pciePlconfIatuRegCtrl1Reg_t       ctrl1;
  pciePlconfIatuRegCtrl2Reg_t       ctrl2;
  pciePlconfIatuRegLowerBaseReg_t   lowerBase;
  pciePlconfIatuRegUpperBaseReg_t   upperBase;
  pciePlconfIatuRegLimitReg_t       limit;
  pciePlconfIatuRegLowerTargetReg_t lowerTarget;
  pciePlconfIatuRegUpperTargetReg_t upperTarget;
  pcieRegisters_t                   regs;

  /* Set up register pointer for interesting registers */
  memset (&regs, 0, sizeof(regs));
  regs.plconfIatuIndex       = &index;

  /* Read current values for index */
  retVal = Pciev1_readRegs (handle, location, &regs);
  if (retVal == pcie_RET_OK)
  {
    /* Update ATU index register with new region direction and region index.
    **/
    switch (atuRegionParams->regionDir)
    {
      /* translate arguments to avoid CSL in public header files */
      case PCIE_ATU_REGION_DIR_OUTBOUND:
        index.regionDirection = CSL_PLCONF_IATU_INDEX_REGION_DIRECTION_OUTBOUND;
        break;
      case PCIE_ATU_REGION_DIR_INBOUND:
      default:
        index.regionDirection = CSL_PLCONF_IATU_INDEX_REGION_DIRECTION_INBOUND;
        break;
    }
    index.regionIndex = atuRegionIndex;

    /* Writeback the new values for index */
    retVal = Pciev1_writeRegs (handle, location, &regs);
    if (retVal == pcie_RET_OK)
    {
      regs.plconfIatuIndex          = NULL;
      regs.plconfIatuRegCtrl1       = &ctrl1;
      regs.plconfIatuRegCtrl2       = &ctrl2;
      regs.plconfIatuRegLowerBase   = &lowerBase;
      regs.plconfIatuRegUpperBase   = &upperBase;
      regs.plconfIatuRegLimit       = &limit;
      regs.plconfIatuRegLowerTarget = &lowerTarget;
      regs.plconfIatuRegUpperTarget = &upperTarget;

      /* Read current values of rest of registers for this index */
      retVal = Pciev1_readRegs (handle, location, &regs);
      if (retVal == pcie_RET_OK)
      {
        /* Set TLP(Transaction Layer packet) type. */
        switch (atuRegionParams->tlpType)
        {
          case PCIE_TLP_TYPE_IO:
            ctrl1.type = 2U;
            break;
          case PCIE_TLP_TYPE_CFG:
            ctrl1.type = 4U;
            break;
          case PCIE_TLP_TYPE_MEM:
          default:
            ctrl1.type = 0U;
            break;
        }

        /* Configure ATU control2 register. */
        /* Enable region. */
        ctrl2.regionEnable = atuRegionParams->enableRegion;
        if (PCIE_ATU_REGION_DIR_INBOUND == atuRegionParams->regionDir)
        {
          /* Set match mode. */
          switch (atuRegionParams->matchMode)
          {
            case PCIE_ATU_REGION_MATCH_MODE_ADDR:
             ctrl2.matchMode = CSL_PLCONF_IATU_REG_CTRL_2_MATCH_MODE__0;
             break;
            case PCIE_ATU_REGION_MATCH_MODE_BAR:
            default:
             ctrl2.matchMode = CSL_PLCONF_IATU_REG_CTRL_2_MATCH_MODE__1;
             break;
          }

          /* Set BAR number. */
          ctrl2.barNumber = atuRegionParams->barNumber;
        }

        /* Configure lower base. */
        lowerBase.iatuRegLowerBase = atuRegionParams->lowerBaseAddr >> 12;

        /* Configure upper base. */
        upperBase.iatuRegUpperBase = atuRegionParams->upperBaseAddr;

        /* Configure window size. */
        limit.iatuRegLimit = (atuRegionParams->lowerBaseAddr +
                  atuRegionParams->regionWindowSize) >> 12;

        /* Configure lower target. */
        lowerTarget.iatuRegLowerTarget = atuRegionParams->lowerTargetAddr >> 12;

        /* Configure Upper target. */
        upperTarget.iatuRegUpperTarget = atuRegionParams->upperTargetAddr;

        /* Writeback the new values */
        retVal = Pciev1_writeRegs (handle, location, &regs);
      }
    }
  }
  return retVal;
} /* Pciev1_atuRegionConfig */

/*********************************************************************
 * FUNCTION PURPOSE: Get pending functional (MSI/legacy) interrupts
 ********************************************************************/
pcieRet_e Pciev1_getPendingFuncInts
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  void            *pendingBits,/**< [out] rev-specific pending bits */
  int32_t          sizeMsiBits,/**< [in] size of msiBits in MAU */
  void            *msiBits /**< [out] rev-specific msi pending bits to check */
)
{
  pcieRet_e retVal;
  pcieTiConfIrqStatusMsiReg_t *statusReg = 
    (pcieTiConfIrqStatusMsiReg_t *)pendingBits;

  /* Get register pointer */
  Pcie_DeviceCfgBaseAddr    *cfg               = pcie_handle_to_cfg (handle);
  Pciev1_DeviceCfgBaseAddrs *bases             = cfg->cfgBase;

  /* Get the pending bits */
  retVal = pciev1_read_tiConfIrqStatusMsi_reg (bases->tiConf, statusReg);
  if (retVal == pcie_RET_OK)
  {
    /* If MSI is pending find which one(s) */
    if ((statusReg->msi != 0) && (msiBits != 0)) 
    {
      int32_t i;
      int32_t n = sizeMsiBits / sizeof(pciePlconfMsiCtrlIntStatusReg_t);
      pciePlconfMsiCtrlIntStatusReg_t *swRegs = 
        (pciePlconfMsiCtrlIntStatusReg_t *)msiBits;
      for (i = 0; i < n; i++)
      {
        retVal = pciev1_read_plconfMsiCtrlIntStatus_reg (
                   bases->plConf, swRegs + i, i);
        if (retVal != pcie_RET_OK)
        {
          break;
        }
      }
    }
  }

  return retVal;
} /* Pciev1_getPendingFuncInts */

/*********************************************************************
 * FUNCTION PURPOSE: Clear pending functional (MSI/legacy) interrupts
 ********************************************************************/
pcieRet_e Pciev1_clrPendingFuncInts
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  void            *pendingBits,/**< [in] rev-specific pending bits */
  int32_t          sizeMsiBits,/**< [in] size of msiBits in MAU */
  void            *msiBits /**< [in] rev-specific msi pending bits to ack */
)
{
  pcieRet_e retVal = pcie_RET_OK;
  pcieTiConfIrqStatusMsiReg_t *statusReg = 
    (pcieTiConfIrqStatusMsiReg_t *)pendingBits;

  /* Get register pointer */
  Pcie_DeviceCfgBaseAddr    *cfg               = pcie_handle_to_cfg (handle);
  Pciev1_DeviceCfgBaseAddrs *bases             = cfg->cfgBase;

  /* If MSI are provided, clear them (write 1 to clear) */
  if (msiBits) 
  {
    int32_t i;
    int32_t n = sizeMsiBits / sizeof(pciePlconfMsiCtrlIntStatusReg_t);
    pciePlconfMsiCtrlIntStatusReg_t *swRegs = 
      (pciePlconfMsiCtrlIntStatusReg_t *)msiBits;
    for (i = 0; i < n; i++)
    {
      retVal = pciev1_write_plconfMsiCtrlIntStatus_reg (
                 bases->plConf, swRegs + i, i);
      if (retVal != pcie_RET_OK)
      {
        break;
      }
    }
  }

  /* Clear the specified pending bits */
  if (retVal == pcie_RET_OK)
  {
    retVal = pciev1_write_tiConfIrqStatusMsi_reg (bases->tiConf, statusReg);
  }

  return retVal;
} /* Pciev1_clrPendingFuncInts */

/* Nothing past this point */

