/* Copyright (C) 2016 Mono Wireless Inc. All Rights Reserved.    *
 * Released under MW-SLA-1J/1E (MONO WIRELESS SOFTWARE LICENSE   *
 * AGREEMENT VERSION 1).                                         */

/** @file
 *
 * @defgroup MBUSA modbus ascii 形式の系列入力処理を行う。
 * modbus ascii 形式の系列入力を行う。
 * ※ 書式のみの利用であって modbus 互換のシステムではない。
 *
 * - 入力の仕様
 *   - :([0-9A-F][0-9A-F])+[0-9A-F][0-9A-F][CR][LF]
 *   - 最後の 2 バイトは LRC となっていてLRCを含む 16進変換したバイトを加算すると 0 になる。\n
 *     つまり LRC は全データの和を取り、符号を反転（二の補数）させたもの。
 *   - + + + を間を空けて入力すると verbose モードへ遷移する判定を行う
 *   - 非 verbose モードでは、入力完了までのタイムアウトを設定する。
 *
 * - API 利用方法
 *   - 事前に tsModbusCmd 構造体を用意し、ゼロクリアし、u16maxlen, au8data を設定しておく。
 *   - UART からの入力バイトを ModBusAscii_u8Parse() に入力する
 *   - ModBusAscii_u8Parse() の戻り値が E_MODBUS_CMD_EMPTY であれば、その系列は処理されていない
 *   - 系列が処理中になった後 E_MODBUS_CMD_COMPLETE/E_MODBUS_CMD_ERROR/E_MODBUS_CMD_LRCERROR/
 *     E_MODBUS_CMD_VERBOSE_ON/E_MODBUS_CMD_VERBOSE_OFFの状態を取り得る。
 *   - (input_string とは多少振る舞いが違うので注意)
 *
 */

#ifndef MODBUS_ASCII_H_
#define MODBUS_ASCII_H_

#include <jendefs.h>
// Serial options
#include <serial.h>
#include <fprintf.h>

#define MODBUS_CMD_TIMEOUT_ms 1000 //!< 入力タイムアウト @ingroup MBUSA

/** @ingroup MBUSA
 * 内部状態
 */
typedef enum {
	E_MODBUS_CMD_EMPTY = 0,      //!< 入力されていない
	E_MODBUS_CMD_READCOLON,      //!< E_MODBUS_CMD_READCOLON
	E_MODBUS_CMD_READPAYLOAD,    //!< E_MODBUS_CMD_READPAYLOAD
	E_MODBUS_CMD_READCR,         //!< E_MODBUS_CMD_READCR
	E_MODBUS_CMD_READLF,         //!< E_MODBUS_CMD_READLF
	E_MODBUS_CMD_PLUS1,          //!< E_MODBUS_CMD_PLUS1
	E_MODBUS_CMD_PLUS2,          //!< E_MODBUS_CMD_PLUS2
	E_MODBUS_CMD_COMPLETE = 0x80,//!< 入力が完結した(LCRチェックを含め)
	E_MODBUS_CMD_ERROR,          //!< 入力エラー
	E_MODBUS_CMD_LRCERROR,       //!< LRCが間違っている
	E_MODBUS_CMD_VERBOSE_ON,     //!< verbose モード ON になった
	E_MODBUS_CMD_VERBOSE_OFF     //!< verbose モード OFF になった
} teModbusCmdState;

/** @ingroup MBUSA
 * 管理構造体
 */
typedef struct {
	uint32 u32timestamp; //!< タイムアウトを管理するためのカウンタ
	uint16 u16len; //!< 系列長
	uint16 u16pos; //!< 入力位置
	uint8 u8state; //!< 状態
	uint8 u8lrc; //!< LRC

	uint16 u16maxlen; //!< バッファの最大長
	uint8 *au8data; //!< バッファ（初期化時に固定配列へのポインタを格納しておく)

	bool_t bverbose; //!< TRUE: verbose モード
	uint32 u32timevbs; //!< verbose モードになったタイムスタンプ
} tsModbusCmd;


/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

uint8 ModBusAscii_u8Parse(tsModbusCmd *pCmd, uint8 u8byte);
#define ModBusAscii_bVerbose(pCmd) (pCmd->bverbose) //!< VERBOSEモード判定マクロ @ingroup MBUSA
void vSerOutput_ModbusAscii(tsFILE *psSerStream, uint8 u8addr, uint8 u8cmd, uint8 *pDat, uint16 u16len);

#endif /* MODBUS_ASCII_H_ */
