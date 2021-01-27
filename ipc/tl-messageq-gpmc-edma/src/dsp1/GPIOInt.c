#include <stdio.h>
/* TI-RTOS Header files */
#include <ti/csl/soc.h>
#include <ti/osal/osal.h>
#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>
#include <ti/drv/gpio/soc/GPIO_v1.h>
#include <ti/board/board.h>

/* CSL Header files */
#include <ti/csl/csl_chip.h>
#include <ti/csl/soc/am572x/src/cslr_interrupt.h>
#include <ti/csl/soc/am572x/src/csl_device_xbar.h>
#include <ti/csl/soc/am572x/src/cslr_soc_dsp_baseaddress.h>
#include <ti/csl/src/ip/gpio/V1/gpio_v2.h>

#include "GPIOInt.h"


uint32_t intr_count = 0;
extern char sem_post_flag;
extern SemaphoreP_Handle semaphoreHandle;

#define GPIO_INT_PIN_NUM            (28)
#define GPIO_INT_PORT_NUM           (2)

#define GPIO2_SYSCONFIG             (CSL_DSP_GPIO2_REGS + 0x10)
#define CTRL_CORE_MPU_IRQ_142_143   0x4A002B54   // GPIO2 MPU IRQ, get the address by "omapconf dump crossbar irq mpu | grep GPIO2"
#define CM_L4PER_GPIO2_CLKCTRL      0x4A009760


/* GPIO Driver board specific pin configuration structure */
GPIO_PinConfig gpioPinConfigs[] = {
    GPIO_DEVICE_CONFIG(GPIO_INT_PORT_NUM, GPIO_INT_PIN_NUM) |
    GPIO_CFG_IN_INT_RISING | GPIO_CFG_INPUT,
};

/* GPIO Driver call back functions */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    NULL,
};

/* GPIO Driver configuration structure */
GPIO_v1_Config GPIO_v1_config = {
    gpioPinConfigs,
    gpioCallbackFunctions,
    sizeof(gpioPinConfigs) / sizeof(GPIO_PinConfig),
    sizeof(gpioCallbackFunctions) / sizeof(GPIO_CallbackFxn),
    0x20,
};

/* Callback function */
void AppGpioCallbackFxn(void);
void GPIO2_CrossbarConfigure(void);

/*
 *  ======== Board_initGPIO ========
 */
void Board_initGPIO(void)
{
    Board_initCfg boardCfg;
	uint32_t val;

	do {
		*(uint32_t *)CM_L4PER_GPIO2_CLKCTRL |= 0x1; // enable the gpio2 module clock
		val = *(uint32_t *)CM_L4PER_GPIO2_CLKCTRL;
	} while((val & 0x1) != 1);

    do {
    	val = *(uint32_t *)CTRL_CORE_MPU_IRQ_142_143;
    	val &= 0xff00ffff;                           //disable GPIO2_IRQ_1 to MPU, must do it!
    	*(uint32_t *)CTRL_CORE_MPU_IRQ_142_143 = val;
		val = *(uint32_t *)CTRL_CORE_MPU_IRQ_142_143;
    } while (((val >> 15) & 0xff) != 0);

	//GPIO2_CrossbarConfigure();

	/* GPIO initialization */
    GPIO_init();

    *(uint32_t *)GPIO2_SYSCONFIG = 0x00000008;

    /* Set the callback function */
    GPIO_setCallback(0, AppGpioCallbackFxn);

    /* Enable GPIO interrupt on the specific gpio pin */
    GPIO_enableInt(0);
}

/*
 *  ======== Callback function ========
 */
void AppGpioCallbackFxn(void)
{
	intr_count++;
	/* Tell user task ISR happend */
	if (sem_post_flag != 0) {
		sem_post_flag = 0;
		SemaphoreP_postFromISR (semaphoreHandle);
	}
}

void GPIO2_CrossbarConfigure(void)
{
    /*
     * AM5 DSP does not have a default Xbar connection for GPIO2
     * interrupt, need the following Xbar interrupt configuration
     * CSL_XBAR_INST_DSP1_IRQ_56 must be same as GPIO_v1_hwAttrs[1].line1EventId!!!!!!
     * GPIO_v1_hwAttrs is defined on pdk_am57xx_1_0_10/packages/ti/drv/gpio/soc/am572x/GPIO_soc.c
     */
    CSL_xbarDspIrqConfigure(1,
                            CSL_XBAR_INST_DSP1_IRQ_56,
                            CSL_XBAR_GPIO2_IRQ_1);
}
