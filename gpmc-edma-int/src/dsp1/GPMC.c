/*
 * GPMC.c
 *
 *  Created on: 2018Äê6ÔÂ1ÈÕ
 *      Author: vefone
 */
#include <stdint.h>

#include "GPMC.h"


void gpmc_write_u16(char buf_id, uint32_t addr, uint16_t val)
{
    switch (buf_id) {
    case DATA_BUF0_ID:
        *(uint16_t *)(GPMC_DATA_BUFF0_ADDR + (addr<<1)) = val;
        break;
    case DATA_BUF1_ID:
        *(uint16_t *)(GPMC_DATA_BUFF1_ADDR + (addr<<1)) = val;
        break;
	case DATA_BUF2_ID:
        *(uint16_t *)(GPMC_DATA_BUFF2_ADDR + (addr<<1)) = val;
        break;
    case CMD_BUF_ID:
        *(uint16_t *)(GPMC_CMD_BUF_ADDR + (addr<<1)) = val;
        break;
    default:
        break;
    }
}

uint16_t  gpmc_read_u16(char buf_id, uint32_t addr)
{
    switch (buf_id) {
    case DATA_BUF0_ID:
        return *(uint16_t *)(GPMC_DATA_BUFF0_ADDR + (addr<<1));
        break;
    case DATA_BUF1_ID:
        return *(uint16_t *)(GPMC_DATA_BUFF1_ADDR + (addr<<1));
        break;
	case DATA_BUF2_ID:
        return *(uint16_t *)(GPMC_DATA_BUFF2_ADDR + (addr<<1));
        break;
    case CMD_BUF_ID:
		return *(uint16_t *)(GPMC_CMD_BUF_ADDR + (addr<<1));
        break;
    default:
        break;
    }
}
