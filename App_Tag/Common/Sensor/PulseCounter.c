/* Copyright (C) 2016 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-1J/1E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT VERSION 1).                                         */

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "jendefs.h"
#include "AppHardwareApi.h"
#include "string.h"

#include "sensor_driver.h"
#include "PulseCounter.h"

#include "ccitt8.h"

#undef SERIAL_DEBUG
#ifdef SERIAL_DEBUG
# include <serial.h>
# include <fprintf.h>
extern tsFILE sDebugStream;
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define PLS_CONVTIME   0 // No wait

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vProcessSnsObj_PulseCounter(void *pvObj, teEvent eEvent);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
void vPulseCounter_Init(tsObjData_PulseCounter *pData, tsSnsObj *pSnsObj) {
	vSnsObj_Init(pSnsObj);

	pSnsObj->pvData = (void*)pData;
	pSnsObj->pvProcessSnsObj = (void*)vProcessSnsObj_PulseCounter;

	memset((void*)pData, 0, sizeof(tsObjData_PulseCounter));
}

void vPulseCounter_Final(tsObjData_PulseCounter *pData, tsSnsObj *pSnsObj) {
	pSnsObj->u8State = E_SNSOBJ_STATE_INACTIVE;
}

// カウンターの設定
void vPulseCounter_setting() {
	// カウンタ0の設定  入力:AI4端子(Twe-Lite-DIP)
	bAHI_PulseCounterConfigure(
		E_AHI_PC_0,
		1,      // 0:RISE, 1:FALL EDGE
		3,      // Debounce 0:off, 1:2samples, 2:4samples, 3:8samples
		FALSE,   // Combined Counter (32bitカウンタ)
		FALSE);  // Interrupt (割り込み)

	// カウンタ0のセット
	bAHI_SetPulseCounterRef(
		E_AHI_PC_0,
		0x0); // 初期値

	// カウンタ0のスタート
	bAHI_StartPulseCounter(E_AHI_PC_0); // start it

	// カウンタ1の設定  入力:PWM4端子(Twe-Lite-DIP)
	bAHI_PulseCounterConfigure(
		E_AHI_PC_1,
		1,      // 0:RISE, 1:FALL EDGE
		3,      // Debounce 0:off, 1:2samples, 2:4samples, 3:8samples
		FALSE,   // Combined Counter (32bitカウンタ)
		FALSE);  // Interrupt (割り込み)

	// カウンタ1のセット
	bAHI_SetPulseCounterRef(
		E_AHI_PC_1,
		0x0); // 初期値

	// カウンタ1のスタート
	bAHI_StartPulseCounter(E_AHI_PC_1); // start it

}

/****************************************************************************
 *
 * NAME: bPulseCounterreset
 *
 * DESCRIPTION:
 *   to Set PulseCounter and Start.
 *
 * RETURNS:
 * bool_t	fail or success
 *
 ****************************************************************************/
PUBLIC bool_t bPulseCounter_reset()
{
	bool_t bOk = TRUE;

	// カウンターの設定
	vPulseCounter_setting();

	return bOk;
}

/****************************************************************************
 *
 * NAME: vHTSstartReadTemp
 *
 * DESCRIPTION:
 * Wrapper to start a read of the temperature sensor.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC bool_t bPulseCounter_startRead()
{
	bool_t bOk = TRUE;

	// なにもしない
	// カウンターのスタートは値の読み込み直後に行う。

	return bOk;
}

/****************************************************************************
 *
 * NAME: u16PulseCounter_readResult
 *
 *
 * RETURNS:
 * uint16: Counts per sleep piriod
 *        0xFFFF, error
 *
 * NOTES:
 * 読み込んだらすぐにカウンタをクリアし、次回読み込みとためにスタートする。
 *
 ****************************************************************************/
PUBLIC uint16 u16PulseCounter_readResult(uint16 ch)
{
	bool_t bOk;
	uint16 u16result0;
	uint16 u16result1;
    uint16 count;

    if ( ch == 0 )
    {
    	// カウンタ0の値を読み込む
    	bOk = bAHI_Read16BitCounter(E_AHI_PC_0, &u16result0);
    	if ( bOk ){
    		count = (int16)u16result0;
    	}
    	else
    		count = 0xffff;		// Error

    	// パルス数のクリア
    	bAHI_Clear16BitPulseCounter(E_AHI_PC_0); // 16bitの場合

    	// カウンターの再スタート
    	bAHI_StartPulseCounter(E_AHI_PC_0); // start it
    }
    else
    {
    // カウンタ1の値を読み込む
    	bOk = bAHI_Read16BitCounter(E_AHI_PC_1, &u16result1);
    	if ( bOk ){
    		count = (int16)u16result1;
    	}
    	else
    		count = 0xffff;		// Error

    	// パルス数のクリア
    	bAHI_Clear16BitPulseCounter(E_AHI_PC_1); // 16bitの場合

    	// カウンターの再スタート
    	bAHI_StartPulseCounter(E_AHI_PC_1); // start it
    }

#ifdef SERIAL_DEBUG
vfPrintf(&sDebugStream, "\n\rPulse DATA %d", count );
#endif

    return count;  // 周期スリープ間のカウント数
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
// the Main loop
PRIVATE void vProcessSnsObj_PulseCounter(void *pvObj, teEvent eEvent) {
	tsSnsObj *pSnsObj = (tsSnsObj *)pvObj;
	tsObjData_PulseCounter *pObj = (tsObjData_PulseCounter *)pSnsObj->pvData;

	// general process (independent from each state)
	switch (eEvent) {
		case E_EVENT_TICK_TIMER:
			if (pObj->u8TickCount < 100) {
				pObj->u8TickCount += pSnsObj->u8TickDelta;
#ifdef SERIAL_DEBUG
vfPrintf(&sDebugStream, "+");
#endif
			}
			break;
		case E_EVENT_START_UP:
			pObj->u8TickCount = 100; // expire immediately
#ifdef SERIAL_DEBUG
vfPrintf(&sDebugStream, "\n\rCounter WAKEUP");
#endif
			break;
		default:
			break;
	}

	// state machine
	switch(pSnsObj->u8State)
	{
	case E_SNSOBJ_STATE_INACTIVE:
		// do nothing until E_ORDER_INITIALIZE event
		break;

	case E_SNSOBJ_STATE_IDLE:
		switch (eEvent) {
		case E_EVENT_NEW_STATE:
			break;

		case E_ORDER_KICK:
			vSnsObj_NewState(pSnsObj, E_SNSOBJ_STATE_MEASURING);

			#ifdef SERIAL_DEBUG
			vfPrintf(&sDebugStream, "\n\rCounter KICKED");
			#endif

			break;

		default:
			break;
		}
		break;

	case E_SNSOBJ_STATE_MEASURING:
		switch (eEvent) {
		case E_EVENT_NEW_STATE:
			pObj->u16Result0 = SENSOR_TAG_DATA_ERROR;
			pObj->u16Result1 = SENSOR_TAG_DATA_ERROR;
			pObj->u8TickWait = PLS_CONVTIME;
			pObj->bBusy = TRUE;
			if (!bPulseCounter_startRead()) { // kick I2C communication
				vSnsObj_NewState(pSnsObj, E_SNSOBJ_STATE_COMPLETE);
			}

			pObj->u8TickCount = 0;
			break;

		default:
			break;
		}

		// wait until completion
		if (pObj->u8TickCount > pObj->u8TickWait) {
			pObj->u16Result0 = u16PulseCounter_readResult(0);
			pObj->u16Result1 = u16PulseCounter_readResult(1);
			// data arrival
			pObj->bBusy = FALSE;
			vSnsObj_NewState(pSnsObj, E_SNSOBJ_STATE_COMPLETE);
		}
		break;

	case E_SNSOBJ_STATE_COMPLETE:
		switch (eEvent) {
		case E_EVENT_NEW_STATE:
			#ifdef SERIAL_DEBUG
			vfPrintf(&sDebugStream, "\n\rCounter_CP: %d", pObj->i16Result);
			#endif

			break;

		case E_ORDER_KICK:
			// back to IDLE state
			vSnsObj_NewState(pSnsObj, E_SNSOBJ_STATE_IDLE);
			break;

		default:
			break;
		}
		break;

	default:
		break;
	}
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
