/* Copyright (C) 2016 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-1J/1E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT VERSION 1).                                         */

#ifndef  PULSE_COUNTER_INCLUDED
#define  PULSE_COUNTER_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include "sensor_driver.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef struct {
	// protected
	bool_t bBusy;          // should block going into sleep

	// public
	uint16 u16Result0; // PC0
	uint16 u16Result1; // PC1

	// private
	uint8 u8TickCount, u8TickWait;
} tsObjData_PulseCounter;

/****************************************************************************/
/***        Exported Functions (state machine)                            ***/
/****************************************************************************/
void vPulseCounter_Init(tsObjData_PulseCounter *pData, tsSnsObj *pSnsObj);
void vPulseCounter_Final(tsObjData_PulseCounter *pData, tsSnsObj *pSnsObj);

/****************************************************************************/
/***        Exported Functions (primitive funcs)                          ***/
/****************************************************************************/
PUBLIC bool_t bPulseCounter_reset();
PUBLIC void   vPulseCounter_setting();
PUBLIC bool_t bPulseCounetr_startRead();
PUBLIC uint16 u16PulseCounter_readResult(uint16 ch);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* PULSE_COUNTER_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

