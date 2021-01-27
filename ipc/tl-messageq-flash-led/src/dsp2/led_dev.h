#include <stdio.h>
#include <stdlib.h>

#define HWREG(reg, val)  \
    do { \
        (*((volatile unsigned int *)(reg))) = val; \
    } while(0)

#define GPIO3_BASE_ADDR           0x48057000
#define GPIO_OE_OFFSET            0x134
#define GPIO_DATAIN_OFFSET        0x138
#define GPIO_DATAOUT_OFFSET       0x13c
#define GPIO_CLEARDATAOUT_OFFSET  0x190
#define GPIO_SETDATAOUT_OFFSET    0x194

#define LED0_INDEX                (19)
#define LED1_INDEX                (20)
#define LED2_INDEX                (21)
