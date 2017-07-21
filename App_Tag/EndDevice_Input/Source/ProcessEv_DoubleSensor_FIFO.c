/* Copyright (C) 2016 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-1J/1E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT VERSION 1).                                         */

#include <jendefs.h>

#include "utils.h"

#include "ccitt8.h"

#include "sprintf.h"

#include "Interactive.h"
#include "EndDevice_Input.h"

#include "sensor_driver.h"
#include "ADXL345.h"
#include "MPL115A2.h"
#include "BME280.h"

static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);
static void vStoreSensorValue();
static void vProcessDoubleSensor_FIFO(uint8 u8SensorName, teEvent eEvent);
static uint8 u8sns_cmplt = 0;

static tsSnsObj sSnsObj;
static tsObjData_ADXL345 sObjADXL345;

static tsSnsObj sSnsObj_Sub;
static tsObjData_MPL115A2 sObjMPL115A2;
static tsObjData_BME280 sObjBME280;

uint8 u8SubSensor;

enum {
	E_SNS_ADC_CMP_MASK = 1,
	E_SNS_ADXL345_CMP = 2,
	E_SNS_MPL115A2_CMP = 4,
	E_SNS_BME280_CMP = 4,
	E_SNS_ALL_CMP = 7
};

/*
 * ADC 計測をしてデータ送信するアプリケーション制御
 */
PRSEV_HANDLER_DEF(E_STATE_IDLE, tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	static bool_t bFirst = TRUE;
	if (eEvent == E_EVENT_START_UP) {
		if (u32evarg & EVARG_START_UP_WAKEUP_RAMHOLD_MASK) {
			// Warm start message
			V_PRINTF(LB "*** Warm starting woke by %s. ***", sAppData.bWakeupByButton ? "DIO" : "WakeTimer");
		} else {
			// 開始する
			// start up message
			vSerInitMessage();

			V_PRINTF(LB "*** Cold starting");
			V_PRINTF(LB "* start end device[%d]", u32TickCount_ms & 0xFFFF);
			// ADXL345 の初期化
		}

		// RC クロックのキャリブレーションを行う
		ToCoNet_u16RcCalib(sAppData.sFlash.sData.u16RcClock);

		// センサーがらみの変数の初期化
		u8sns_cmplt = 0;

		vADXL345_FIFO_Init( &sObjADXL345, &sSnsObj );
		if( bFirst ){
			V_PRINTF(LB "*** ADXL345 FIFO Setting...");

			//	文字数の判定
			uint8 u8Idx = sAppData.sFlash.sData.uParam.au8Param[0]==0 ? 0:
						  sAppData.sFlash.sData.uParam.au8Param[1]==0 ? 1:
						  sAppData.sFlash.sData.uParam.au8Param[2]==0 ? 2:
						  sAppData.sFlash.sData.uParam.au8Param[3]==0 ? 3 : 4;
			// センサのサンプリング周波数の指定
			uint16 u16SamplingHz = u32string2dec(sAppData.sFlash.sData.uParam.au8Param, u8Idx);

			// 初期化
			bADXL345reset();
			// 設定
			bADXL345_FIFO_Setting( u16SamplingHz );
		}
		vSnsObj_Process(&sSnsObj, E_ORDER_KICK);
		if (bSnsObj_isComplete(&sSnsObj)) {
			// 即座に完了した時はセンサーが接続されていない、通信エラー等
			u8sns_cmplt |= E_SNS_ADXL345_CMP;
			V_PRINTF(LB "*** ADXL345 comm err?");
			ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEP); // スリープ状態へ遷移
			return;
		}

		if( sAppData.sFlash.sData.i16param == 1 ){
			V_PRINTF(LB "*** Start BME280 ");
			vBME280_Init( &sObjBME280, &sSnsObj_Sub );
			if( bFirst ){
				V_PRINTF(LB "*** BME280 Setting...");
				// BME280 の初期化
				bBME280_Setting();
			}
			vSnsObj_Process(&sSnsObj_Sub, E_ORDER_KICK);
			if (bSnsObj_isComplete(&sSnsObj_Sub)) {
				// 即座に完了した時はセンサーが接続されていない、通信エラー等
				u8sns_cmplt |= E_SNS_BME280_CMP;
				V_PRINTF(LB "*** BME280 comm err?");
					ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEP); // スリープ状態へ遷移
					return;
			}
			u8SubSensor = PKT_ID_BME280;
		}else{
			V_PRINTF(LB "*** Start MPL115A2 ");
			vMPL115A2_Init( &sObjMPL115A2, &sSnsObj_Sub );
			vSnsObj_Process(&sSnsObj_Sub, E_ORDER_KICK);
			if (bSnsObj_isComplete(&sSnsObj_Sub)) {
				// 即座に完了した時はセンサーが接続されていない、通信エラー等
				u8sns_cmplt |= E_SNS_MPL115A2_CMP;
				V_PRINTF(LB "*** MPL115A2 comm err?");
				ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEP); // スリープ状態へ遷移
				return;
			}
			u8SubSensor = PKT_ID_MPL115A2;
		}

		bFirst = FALSE;

			// ADC の取得
		vADC_WaitInit();
		vSnsObj_Process(&sAppData.sADC, E_ORDER_KICK);

		// RUNNING 状態
		ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);

	} else {
		V_PRINTF(LB "*** unexpected state.");
		ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEP); // スリープ状態へ遷移
	}
}

PRSEV_HANDLER_DEF(E_STATE_RUNNING, tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	if ((eEvent == E_EVENT_START_UP) && (u32evarg & EVARG_START_UP_WAKEUP_RAMHOLD_MASK)) {
		V_PRINTF("#");
		vProcessDoubleSensor_FIFO( PKT_ID_ADXL345, E_EVENT_START_UP);
		vProcessDoubleSensor_FIFO( u8SubSensor, E_EVENT_START_UP);
	}

	// 送信処理に移行
	if (u8sns_cmplt == E_SNS_ALL_CMP) {
		ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_TX);
	}

	// タイムアウト
	if (ToCoNet_Event_u32TickFrNewState(pEv) > 100 ) {
		V_PRINTF(LB"! TIME OUT (E_STATE_RUNNING)");
		ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEP); // スリープ状態へ遷移
	}
}

PRSEV_HANDLER_DEF(E_STATE_APP_WAIT_TX, tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	if (eEvent == E_EVENT_NEW_STATE) {
		// ネットワークの初期化
		if (!sAppData.pContextNwk) {
			// 初回のみ
			sAppData.sNwkLayerTreeConfig.u8Role = TOCONET_NWK_ROLE_ENDDEVICE;
			sAppData.pContextNwk = ToCoNet_NwkLyTr_psConfig_MiniNodes(&sAppData.sNwkLayerTreeConfig);
			if (sAppData.pContextNwk) {
				ToCoNet_Nwk_bInit(sAppData.pContextNwk);
				ToCoNet_Nwk_bStart(sAppData.pContextNwk);
			} else {
				ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEP); // スリープ状態へ遷移
				return;
			}
		} else {
			// 一度初期化したら RESUME
			ToCoNet_Nwk_bResume(sAppData.pContextNwk);
		}

		if( sAppData.bWakeupByButton ){
			uint8	i;
			uint8	au8Data[100];
			uint8*	q = au8Data;
			S_OCTET(sAppData.sSns.u8Batt);
			S_BE_WORD(sAppData.sSns.u16Adc1);
			S_BE_WORD(sAppData.sSns.u16Adc2);
			S_OCTET( PKT_ID_ADXL345 );
			S_OCTET( 0xFA );
			S_OCTET( sObjADXL345.u8FIFOSample );
			for( i=0; i<sObjADXL345.u8FIFOSample; i++ ){
				S_BE_WORD(sObjADXL345.ai16ResultX[i]);
				S_BE_WORD(sObjADXL345.ai16ResultY[i]);
				S_BE_WORD(sObjADXL345.ai16ResultZ[i]);
			}
			if( sAppData.sFlash.sData.i16param == 1 ){
				S_OCTET( PKT_ID_BME280 );
				S_BE_WORD(sObjBME280.i16Temp);
				S_BE_WORD(sObjBME280.u16Hum);
				S_BE_WORD(sObjBME280.u16Pres);
			}else{
				S_OCTET( PKT_ID_MPL115A2 );
				S_BE_WORD(sObjMPL115A2.i16Result);
			}

			sAppData.u16frame_count++;

			if ( bTransmitToParent( sAppData.pContextNwk, au8Data, q-au8Data ) ) {
				ToCoNet_Tx_vProcessQueue();
			} else {
				ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEP); // 送信失敗
			}
		}else{
			V_PRINTF(LB"First");
			ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEP); // 送信失敗
		}

		vPortSetHi(LED);
		V_PRINTF(" FR=%04X", sAppData.u16frame_count);
	}

	if (eEvent == E_ORDER_KICK) { // 送信完了イベントが来たのでスリープする
		ToCoNet_Event_SetState(pEv, E_STATE_APP_RETRY); // スリープ状態へ遷移
	}

	// タイムアウト
	if (ToCoNet_Event_u32TickFrNewState(pEv) > 100) {
		V_PRINTF(LB"! TIME OUT (E_STATE_APP_WAIT_TX)");
		ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEP); // スリープ状態へ遷移
	}
}

PRSEV_HANDLER_DEF(E_STATE_APP_SLEEP, tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	if (eEvent == E_EVENT_NEW_STATE) {
		// Sleep は必ず E_EVENT_NEW_STATE 内など１回のみ呼び出される場所で呼び出す。
		V_PRINTF(LB"! Sleeping...");
		V_FLUSH();

		// Mininode の場合、特別な処理は無いのだが、ポーズ処理を行う
		ToCoNet_Nwk_bPause(sAppData.pContextNwk);

		// センサー用の電源制御回路を Hi に戻す
		vPortSetSns(FALSE);

		vPortSetLo(LED);

		vAHI_DioWakeEnable(PORT_INPUT_MASK_ADXL345, 0);		// ENABLE DIO WAKE SOURCE
		(void)u32AHI_DioInterruptStatus();					// clear interrupt register
		vAHI_DioWakeEdge(PORT_INPUT_MASK_ADXL345, 0);		// 割り込みエッジ(立上がりに設定)

		//u8Read_Interrupt();
		ToCoNet_vSleep( E_AHI_WAKE_TIMER_0, 0, FALSE, FALSE);
	}
}

PRSEV_HANDLER_DEF(E_STATE_APP_RETRY, tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	if (eEvent == E_EVENT_NEW_STATE) {
		V_PRINTF(LB"! Retry");
		vPortSetLo(LED);
	}

	if (eEvent == E_ORDER_KICK) { // 割込みが入ったため、計測して送信する。
		V_PRINTF(LB"! Int");

		// センサーがらみの変数の初期化
		u8sns_cmplt = 0;

		sSnsObj.u8State = 0;
		vSnsObj_Process(&sSnsObj, E_ORDER_KICK);

		sSnsObj_Sub.u8State = 0;
		vSnsObj_Process(&sSnsObj_Sub, E_ORDER_KICK);

		// ADC の取得
		vADC_WaitInit();
		sAppData.sADC.u8State = 0;
		vSnsObj_Process(&sAppData.sADC, E_ORDER_KICK);

		V_PRINTF(LB"! NEXT");
		// RUNNING 状態
		ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
	}
}

/**
 * イベント処理関数リスト
 */
static const tsToCoNet_Event_StateHandler asStateFuncTbl[] = {
	PRSEV_HANDLER_TBL_DEF(E_STATE_IDLE),
	PRSEV_HANDLER_TBL_DEF(E_STATE_RUNNING),
	PRSEV_HANDLER_TBL_DEF(E_STATE_APP_WAIT_TX),
	PRSEV_HANDLER_TBL_DEF(E_STATE_APP_SLEEP),
	PRSEV_HANDLER_TBL_DEF(E_STATE_APP_RETRY),
	PRSEV_HANDLER_TBL_TRM
};

/**
 * イベント処理関数
 * @param pEv
 * @param eEvent
 * @param u32evarg
 */
void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	ToCoNet_Event_StateExec(asStateFuncTbl, pEv, eEvent, u32evarg);
}

#if 0
/**
 * ハードウェア割り込み
 * @param u32DeviceId
 * @param u32ItemBitmap
 * @return
 */
static uint8 cbAppToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap) {
	uint8 u8handled = FALSE;

	switch (u32DeviceId) {
	case E_AHI_DEVICE_ANALOGUE:
		break;

	case E_AHI_DEVICE_SYSCTRL:
		break;

	case E_AHI_DEVICE_TIMER0:
		break;

	case E_AHI_DEVICE_TICK_TIMER:
		break;

	default:
		break;
	}

	return u8handled;
}
#endif

/**
 * ハードウェアイベント（遅延実行）
 * @param u32DeviceId
 * @param u32ItemBitmap
 */
static void cbAppToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap) {
	switch (u32DeviceId) {
	case E_AHI_DEVICE_TICK_TIMER:
		vProcessDoubleSensor_FIFO( PKT_ID_ADXL345,E_EVENT_TICK_TIMER );
		vProcessDoubleSensor_FIFO( u8SubSensor,E_EVENT_TICK_TIMER );
		break;

	case E_AHI_DEVICE_ANALOGUE:
		/*
		 * ADC完了割り込み
		 */
		V_PUTCHAR('@');
		vSnsObj_Process(&sAppData.sADC, E_ORDER_KICK);
		if (bSnsObj_isComplete(&sAppData.sADC)) {
			u8sns_cmplt |= E_SNS_ADC_CMP_MASK;
			vStoreSensorValue();
		}
		break;

	case E_AHI_DEVICE_SYSCTRL:
		ToCoNet_Event_Process(E_ORDER_KICK, 0, vProcessEvCore);
		break;

	case E_AHI_DEVICE_TIMER0:
		break;

	default:
		break;
	}
}

#if 0
/**
 * メイン処理
 */
static void cbAppToCoNet_vMain() {
	/* handle serial input */
	vHandleSerialInput();
}
#endif

#if 0
/**
 * ネットワークイベント
 * @param eEvent
 * @param u32arg
 */
static void cbAppToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg) {
	switch(eEvent) {
	case E_EVENT_TOCONET_NWK_START:
		break;

	default:
		break;
	}
}
#endif


#if 0
/**
 * RXイベント
 * @param pRx
 */
static void cbAppToCoNet_vRxEvent(tsRxDataApp *pRx) {

}
#endif

/**
 * TXイベント
 * @param u8CbId
 * @param bStatus
 */
static void cbAppToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus) {
	// 送信完了
	V_PRINTF(LB"! Tx Cmp = %d", bStatus);
	ToCoNet_Event_Process(E_ORDER_KICK, 0, vProcessEvCore);
}
/**
 * アプリケーションハンドラー定義
 *
 */
static tsCbHandler sCbHandler = {
	NULL, // cbAppToCoNet_u8HwInt,
	cbAppToCoNet_vHwEvent,
	NULL, // cbAppToCoNet_vMain,
	NULL, // cbAppToCoNet_vNwkEvent,
	NULL, // cbAppToCoNet_vRxEvent,
	cbAppToCoNet_vTxEvent
};

/**
 * アプリケーション初期化
 */
void vInitAppDoubleSensor_FIFO() {
	psCbHandler = &sCbHandler;
	pvProcessEv1 = vProcessEvCore;
}

static void vProcessDoubleSensor_FIFO(uint8 u8SensorName, teEvent eEvent) {
	if (bSnsObj_isComplete(&sSnsObj) && bSnsObj_isComplete(&sSnsObj_Sub) ) {
		 return;
	}

	// イベントの処理
	vSnsObj_Process(&sSnsObj, eEvent); // ポーリングの時間待ち
	if (bSnsObj_isComplete(&sSnsObj)) {
		u8sns_cmplt |= E_SNS_ADXL345_CMP;

		V_PRINTF(LB"!ADXL345: X : %d, Y : %d, Z : %d",
			sObjADXL345.ai16ResultX[0],
			sObjADXL345.ai16ResultY[0],
			sObjADXL345.ai16ResultZ[0]
		);

	}

	if( u8SensorName == PKT_ID_MPL115A2 ){
		// イベントの処理
		vSnsObj_Process(&sSnsObj_Sub, eEvent); // ポーリングの時間待ち
		if (bSnsObj_isComplete(&sSnsObj_Sub)) {
			if(!(u8sns_cmplt&E_SNS_MPL115A2_CMP)){
				V_PRINTF(LB"!MPL115A2: %dhPa", sObjMPL115A2.i16Result );
			}
			u8sns_cmplt |= E_SNS_MPL115A2_CMP;
		}
	}

	if( u8SensorName == PKT_ID_BME280 ){
		// イベントの処理
		vSnsObj_Process(&sSnsObj_Sub, eEvent); // ポーリングの時間待ち
		if (bSnsObj_isComplete(&sSnsObj_Sub)) {
			if(!(u8sns_cmplt&E_SNS_BME280_CMP)){
				V_PRINTF(LB"!BME280: %d.%02dC %d.%02d%% %dhPa", sObjBME280.i16Temp/100, sObjBME280.i16Temp%100, sObjBME280.u16Hum/100, sObjBME280.u16Hum%100, sObjBME280.u16Pres );
			}
			u8sns_cmplt |= E_SNS_BME280_CMP;
		}
	}

	if (bSnsObj_isComplete(&sSnsObj) && bSnsObj_isComplete(&sSnsObj_Sub) ) {
		// 完了時の処理
		if (u8sns_cmplt == E_SNS_ALL_CMP) {
			ToCoNet_Event_Process(E_ORDER_KICK, 0, vProcessEvCore);
		}
	}
}

/**
 * センサー値を格納する
 */
static void vStoreSensorValue() {
	// センサー値の保管
	sAppData.sSns.u16Adc1 = sAppData.sObjADC.ai16Result[TEH_ADC_IDX_ADC_1];
	sAppData.sSns.u16Adc2 = sAppData.sObjADC.ai16Result[TEH_ADC_IDX_ADC_2];
	sAppData.sSns.u8Batt = ENCODE_VOLT(sAppData.sObjADC.ai16Result[TEH_ADC_IDX_VOLT]);

	// ADC1 が 1300mV 以上(SuperCAP が 2600mV 以上)である場合は SUPER CAP の直結を有効にする
	if (sAppData.sSns.u16Adc1 >= VOLT_SUPERCAP_CONTROL) {
		vPortSetLo(DIO_SUPERCAP_CONTROL);
	}
}
