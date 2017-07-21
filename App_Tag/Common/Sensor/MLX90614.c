/* Copyright (C) 2016 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-1J/1E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT VERSION 1).                                         */

/* #Mikimasa */
/* MLX90614赤外線温度センサー */

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "jendefs.h"
#include "AppHardwareApi.h"
#include "string.h"

#include "sensor_driver.h"
#include "MLX90614.h"
#include "SMBus.h"

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
#define MLX90614_ADDRESS     (0x5A)  // Default

#define MLX90614_REG_TANV    (0x06)  // Th_anv
#define MLX90614_REG_TOBJ    (0x07)  // Th_obj
#define MLX90614_REG_PWMCTRL (0x22)  // PWMCTRL

/* #Mikimasa */
/* センサーの変換時間が長い（AD変換より長い）の場合は以下の数字では変換時間を設定できず、*/
/* ProcessEv_MLX90614.cでのスリープ時間で調整する必要がある             */
#define MLX90614_CONVTIME    (50) 	  // 充分な大きい数字として設定。事実上あまり意味がない(max100)

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vProcessSnsObj_MLX90614(void *pvObj, teEvent eEvent);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
void vMLX90614_Init(tsObjData_MLX90614 *pData, tsSnsObj *pSnsObj) {
	vSnsObj_Init(pSnsObj);

	pSnsObj->pvData = (void*)pData;
	pSnsObj->pvProcessSnsObj = (void*)vProcessSnsObj_MLX90614;

	memset((void*)pData, 0, sizeof(tsObjData_MLX90614));
}

void vMLX90614_Final(tsObjData_MLX90614 *pData, tsSnsObj *pSnsObj) {
	pSnsObj->u8State = E_SNSOBJ_STATE_INACTIVE;
}


/****************************************************************************
 *
 * NAME: bMLX90614reset
 *
 * DESCRIPTION:
 *   to reset ML90614 device
 *
 * RETURNS:
 * bool_t	fail or success
 *
 ****************************************************************************/
PUBLIC bool_t bMLX90614reset( bool_t bMode16 )
{
	bool_t bOk = TRUE;
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
PUBLIC bool_t bMLX90614startRead()
{
	bool_t bOk = TRUE;

	//  POWER ON RESETから走り始めているので、不要。
	//
	//
	return bOk;
}

/****************************************************************************
 *
 * NAME: u16MLX90614readResult
 *
 * RETURNS:
 * int16: , 12345 -> 123.45[c]
 *
 ****************************************************************************/
PUBLIC int16 i16MLX90614readResult()
{
	bool_t bOk = TRUE;
	uint16 u16result;
    int16 temp;
    uint16 H_data;
    uint8 au8data[2];

    // MLX90614_REG_TOBJ番地から２バイト読み込む

    // 読み出しアドレス指定  (RepeatSTART対応のため、ストップコンディションを出ないようにした)
	bOk &= bSMBusWrite_WO_STOP( MLX90614_ADDRESS, MLX90614_REG_TOBJ, 0, NULL );
	// 最後はNACKを返す。
	bOk &= bSMBusSequentialRead_NACK(MLX90614_ADDRESS, 2, au8data);
    if (bOk == FALSE) {
    	return( SENSOR_TAG_DATA_ERROR );
    }
    H_data = au8data[1];

    u16result = ((H_data << 8) | au8data[0]);	//	読み込んだ数値を代入
	temp = ( ( (float)u16result * 2.0 ) - 27315.0 );

#ifdef SERIAL_DEBUG
vfPrintf(&sDebugStream, "\n\rMLX90614 DATA %x", *((uint16*)au8data) );
#endif

    return temp;
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
// the Main loop
PRIVATE void vProcessSnsObj_MLX90614(void *pvObj, teEvent eEvent) {
	tsSnsObj *pSnsObj = (tsSnsObj *)pvObj;
	tsObjData_MLX90614 *pObj = (tsObjData_MLX90614 *)pSnsObj->pvData;

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
vfPrintf(&sDebugStream, "\n\rMLX90614 WAKEUP");
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
			vfPrintf(&sDebugStream, "\n\rMLX90614 KICKED");
			#endif

			break;

		default:
			break;
		}
		break;

	case E_SNSOBJ_STATE_MEASURING:
		switch (eEvent) {
		case E_EVENT_NEW_STATE:
			pObj->i16Result = SENSOR_TAG_DATA_ERROR;
			pObj->u8TickWait = MLX90614_CONVTIME;

			pObj->bBusy = TRUE;
#ifdef MLX90614_ALWAYS_RESET
			u8reset_flag = TRUE;
			if (!bMLX90614reset(TURE)) {
				vSnsObj_NewState(pSnsObj, E_SNSOBJ_STATE_COMPLETE);
			}
#else
			if (!bMLX90614startRead()) { // kick I2C communication
				vSnsObj_NewState(pSnsObj, E_SNSOBJ_STATE_COMPLETE);
			}
#endif
			pObj->u8TickCount = 0;
			break;

		default:
			break;
		}

		// wait until completion
		if (pObj->u8TickCount > pObj->u8TickWait) {
#ifdef MLX90614_ALWAYS_RESET
			if (u8reset_flag) {
				u8reset_flag = 0;
				if (!bMLX90614startRead()) {
					vMLX90614_new_state(pObj, E_SNSOBJ_STATE_COMPLETE);
				}

				pObj->u8TickCount = 0;
				pObj->u8TickWait = MLX90614_CONVTIME;
				break;
			}
#endif
			pObj->i16Result = i16MLX90614readResult();

			// data arrival
			pObj->bBusy = FALSE;
			vSnsObj_NewState(pSnsObj, E_SNSOBJ_STATE_COMPLETE);
		}
		break;

	case E_SNSOBJ_STATE_COMPLETE:
		switch (eEvent) {
		case E_EVENT_NEW_STATE:
			#ifdef SERIAL_DEBUG
			vfPrintf(&sDebugStream, "\n\rMLX90614_CP: %d", pObj->i16Result);
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
