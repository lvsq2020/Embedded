/*
 * gpmc.h
 *
 *  Created on: 2018年6月1日
 *      Author: vefone
 */

#ifndef GPMC_H_
#define GPMC_H_

#define GPMC_CS0_START_ADDR    0x12000000
#define GPMC_CS0_MEM_SIZE      0x01000000  // 16MB

#define GPMC_CMD_BUF_ADDR      0x12000000  // cmd buffer start address
#define GPMC_DATA_BUFF0_ADDR   0x12002000  // data buffer 0 start address
#define GPMC_DATA_BUFF1_ADDR   0x12004000  // data buffer 1 start address
#define GPMC_DATA_BUFF2_ADDR   0x12006000  // data buffer 2 start address


#define DATA_BUF0_ID           0
#define DATA_BUF1_ID           1
#define DATA_BUF2_ID           2
#define CMD_BUF_ID             3

#define BUFF_ID_OFFSET         0x0
#define BUFF_SIZE_OFFSET       0x1

void gpmc_write_u16(char buf_id, uint32_t addr, uint16_t val);
uint16_t  gpmc_read_u16(char buf_id, uint32_t addr);


#endif /* GPMC_H_ */
