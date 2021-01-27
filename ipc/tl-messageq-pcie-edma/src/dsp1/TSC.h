/*
 * TSC.h
 *
 *  Created on: 2017Äê11ÔÂ1ÈÕ
 *      Author: Lijy
 */

#ifndef LIB_TSC_H_
#define LIB_TSC_H_

#include <xdc/std.h>
#include <c6x.h>


#define TSC_getDelay(startTSC) 	((unsigned int)((0xFFFFFFFFl+TSCL)- (unsigned long long)startTSC)+ 1)
#define TSC_count_cycle_from(startTSC) 	(TSC_getDelay(startTSC)- cycle_measure_overhead)

#ifdef __cplusplus
extern "C" {
#endif

extern unsigned int gDSP_Core_Speed_Hz;

void TSC_init();
void calc_cycle_measure_overhead();
void TSC_delay_ms(Uint32 ms);
void TSC_delay_us(Uint32 us);

#ifdef __cplusplus
}
#endif

#endif /* LIB_TSC_H_ */
