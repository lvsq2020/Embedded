/*
 * TSC.c
 *
 *  Created on: 2017Äê11ÔÂ1ÈÕ
 *      Author: Lijy
 */

//#include <C6x.h>
#include "TSC.h"

/*===============================TSC===================================*/
unsigned int cycle_measure_overhead=50;
unsigned int gDSP_Core_Speed_Hz= 700000000; 	//DSP core clock speed in Hz

/*****************************************************************************
 Prototype    : calc_cycle_measure_overhead
 Description  : calclucate the cycles measurement overhead
 Input        : None
 Output       : None
 Return Value :

  History        :
  1.Date         : 2010/12/12
    Author       : Brighton Feng
    Modification : Created function

*****************************************************************************/
void calc_cycle_measure_overhead()
{
	unsigned int cycle_cold, cycle_warm;
	cycle_cold= TSCL;
	cycle_cold = TSC_getDelay(cycle_cold);
	cycle_warm= TSCL;
	cycle_warm = TSC_getDelay(cycle_warm);
	cycle_measure_overhead = (cycle_cold + cycle_warm)/2;
}

/*****************************************************************************
 Prototype    : TSC_init
 Description  : Initialize Time stamp counter to measure cycles
 Input        : None
 Output       : None
 Return Value :

  History        :
  1.Date         : 2010/12/12
    Author       : Brighton Feng
    Modification : Created function

*****************************************************************************/
void TSC_init()
{
	TSCL = 0; 	/* Enable the TSC */
	calc_cycle_measure_overhead();
}

/*****************************************************************************
 Prototype    : TSC_delay_ms
 Description  : Implement the delay function in millisecond
 Input        : Uint32 ms
 Output       : None
 Return Value :

  History        :
  1.Date         : 2010/12/12
    Author       : Brighton Feng
    Modification : Created function

*****************************************************************************/
void TSC_delay_ms(Uint32 ms)
{
	volatile unsigned long long startTSC, currentTSC;
	unsigned long long delay_cycles;
	Uint32 tscl, tsch;

	tscl= TSCL;
	tsch= TSCH;
	startTSC= _itoll(tsch,tscl);

	delay_cycles= ((unsigned long long)ms*gDSP_Core_Speed_Hz/1000);

	do
	{
		tscl= TSCL;
		tsch= TSCH;
		currentTSC= _itoll(tsch,tscl);
	}
	while((currentTSC-startTSC)<delay_cycles);
}

/*****************************************************************************
 Prototype    : TSC_delay_us
 Description  : Implement the delay function in microsecond
 Input        : Uint32 us
 Output       : None
 Return Value :
*****************************************************************************/
void TSC_delay_us(Uint32 us)
{
	volatile unsigned long long startTSC, currentTSC;
	unsigned long long delay_cycles;
	Uint32 tscl, tsch;

	tscl= TSCL;
	tsch= TSCH;
	startTSC= _itoll(tsch,tscl);

	delay_cycles= ((unsigned long long)us*gDSP_Core_Speed_Hz/1000000);

	do
	{
		tscl= TSCL;
		tsch= TSCH;
		currentTSC= _itoll(tsch,tscl);
	}
	while((currentTSC-startTSC)<delay_cycles);
}

