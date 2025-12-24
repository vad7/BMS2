/*
 * Copyright (c) 2025 by Vadim Kulakov vad7@yahoo.com, vad711
 *
 * This file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.

For ATMega328PB

Connections (Arduino Nano):
Two active balancers JK-DZ11B2A24S RS485 are supported.
UART0 (TX,RX) - Debug, setup port
UART1 (D11,D12) - BMS 1/2
pin A0(D14) - Debug to UART0 (TX,RX) - connect pin to GND before power on (115200 bps), buzzer will be off
pin A1(D15) - Key1 (to GND)
pin D8+D9+D10 -> resistor 0..50 Om -> Buzzer [+] (40 mA MAX), all pins must be on the same port!
pin D2 - LCD_RS, D3 - LCD_E, D4..D7 - LCD_DB4..7 - LCD 2004 (20x4) 5V
pin SDA(A4), SCL(A5) - I2C MAP
pin A2(D16) - Cell Undervoltage, Active LOW
pin A3(D17) - Cell Overvoltage, Active LOW, A2+A3 - BMS Error
pin D13 - slow sofware UART (default 333 Hz) (BMS errors, voltages, +CRC16(Modbus))
pin D25(PE0) - LED

Microart connector RJ-11 (6P6C):
1 - BMS_DISCHARGE / I2C_SLC_buf_iso (brown-white)
2 - GND								(brown)
3 - TEPM							(blue-white)
4 - BMS_CHARGE / I2C_SDA_buf_iso	(blue)
5 - +5v_iso							(orange)
6 - not connected on back side, on front side: +12V if enabled (orange-white)

*/

#define VERSION F("2.2")

#include "Arduino.h"
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <Wire.h>
extern "C" {
	#include <utility/twi.h>
}
#include "crc16.h"

#define DEBUG_ALWAYS_ON				// DEBUG to UART0(USB) - ON
//#define DEBUG_BMS_SEND

// PINS definition
#define BUZZER_PD1					8
#define BUZZER_PD2					9
#define BUZZER_PD3					10
#define KEY1						15
#define DEBUG_ACTIVE_PD				14	// activate - connect pin to GND before power on, buzzer -> off
#define LED_PD						25
#define LCD_ENABLED
#ifdef LCD_ENABLED
#include "LiquidCrystal.h"
// LCD ---------- rs, en, d4, d5, d6, d7
LiquidCrystal lcd( 2,  3,  4,  5,  6,  7);	// LCD 20x4
//#define LCD_Backlite_pin	13
#endif
//
#define OUT_CELL_UNDER_V			16	// A2, active LOW
#define OUT_CELL_OVER_V				17	// A3, active LOW, OUT_CELL_UNDER_V+OUT_CELL_OVER_V = BMS ERROR
#define RWARN_PULSE_PIN				13
#define RWARN_PULSE_LEVEL			0	// UART active level
#define RWARN_PULSE_QT				3000// microsec, bit length
#define RWARN_PULSE_HIGH_LEN		25	// *1msec
#define RWARN_PULSE_LOW_LEN			25	// *1msec
#define RWARN_PULSE_END_LOW_LEN		40	// *1msec

#define I2C_FREQ					2500
#define BMS_NUM_MAX					2		// max number of connected BMS
#define BMS_CELLS_QTY_MAX			24
#define MAIN_LOOP_PERIOD			1		// msec, not less than this, actually +848 us
#define BMS_NO_TEMP					255
#define WATCHDOG_NO_CONN			30UL	// sec
#define BMS_MIN_PAUSE_BETWEEN_READS	20UL	// msec, defaults
#define BMS_CHANGE_DELTA_PAUSE_MIN  1800	// sec
#define BMS_CHANGE_DELTA_EQUALIZER  60		// attempts (* ~1 sec)
#define BMS_CHANGE_DELTA_DISCHARGE  30		// sec, When I2C_MAP_MODE = M_ON balance delta => BalansDelta[max]
#define BEEP_DURATION				5		// *0.1 sec
#define BEEP_PAUSE					40		// *0.1 sec
const uint8_t BMS_Cmd_Head[] PROGMEM = { 0x55, 0xAA };
const uint8_t BMS_Cmd_Request[] PROGMEM = { 0xFF, 0x00, 0x00 };
const uint8_t BMS_Cmd_ChangeDelta[] PROGMEM = { 0xF2 };
#define BMS_OFFSET_Cmd				3
#define BMS_OFFSET_TotalV			4	// 10mV
#define BMS_OFFSET_Cells			8
#define BMS_OFFSET_HighV_Cell		9
#define BMS_OFFSET_LowV_Cell		10
#define BMS_OFFSET_BalansDir		11	// BIT0 - charging, BIT1 - discharging
#define BMS_OFFSET_Alarm			12
#define BMS_OFFSET_MaxDiffV			13	// mV
#define BMS_OFFSET_BalansI			15	// mA
#define BMS_OFFSET_TriggerV			17	// mV
#define BMS_OFFSET_BalansOn			21
#define BMS_OFFSET_CellsV_Array		23	// mV
#define BMS_OFFSET_Temperature		71	// C
#define BMS_Temperature_Correct		50

#if defined(__AVR_ATmega328PB__)
#define BMS_SERIAL 					Serial1	// BMS
#else
#define BMS_SERIAL 					Serial	// BMS
#endif
#define BMS_SERIAL_RATE				9600
#define DEBUG_TO_SERIAL				// use serial for debug or setup
#define DEBUG_TO_SERIAL_RATE		115200
#define DEBUG_READ_PERIOD			1000UL	// ms
#ifdef DEBUG_TO_SERIAL
#define DebugSerial 				Serial
#endif

#define BLINK_ALARM		{ error_alarm_time = 50; }
#ifdef DEBUG_TO_SERIAL
#define DEBUG(s) DebugSerial.print(s)
#define DEBUGH(s) DebugSerial.print(s, 16)
#define DEBUGN(s) DebugSerial.println(s)
#define DEBUGIF(d,s) { if(debug >= d) DebugSerial.print(s); }
#define DEBUGIFN(d,s) { if(debug >= d) DebugSerial.println(s); }
char	debug_read_buffer[64];
uint8_t debug_read_idx = 0;
const char dbg_debug[] PROGMEM = "debug";
const char dbg_bms[] PROGMEM = "bms";
const char dbg_cells[] PROGMEM = "cells";
const char dbg_period[] PROGMEM = "period";
const char dbg_wait_answer[] PROGMEM = "wait";
const char dbg_rotate[] PROGMEM = "rotate";
const char dbg_round[] PROGMEM = "Vround";
const char dbg_cell_min_V[] PROGMEM = "cellmin";
const char dbg_cell_max_V[] PROGMEM = "cellmax";
const char dbg_cell_delta_max_V[] PROGMEM = "deltamax";
const char dbg_correct[] PROGMEM = "Vcorr";
const char dbg_options[] PROGMEM = "options";
const char dbg_temp_correct[] PROGMEM = "tempcorr";
const char dbg_delta_default[] PROGMEM = "deltadef";
const char dbg_delta_pause[] PROGMEM = "deltapause";
const char dbg_watchdog[] PROGMEM = "watchdog";
const char dbg_vmaxhyst[] PROGMEM = "Vmaxhyst";
const char dbg_seterr[] PROGMEM = "ERR";
const char dbg_I2C_WRITE_BMS[] PROGMEM = "I2C_WRITE_BMS";
const char dbg_I2C_READ_BMS[] PROGMEM = "I2C_READ_BMS";
const char dbg_temp[] PROGMEM = "temp";
const char dbg_delta_change_pause[] PROGMEM = "dchgpause";
const char dbg_read[] PROGMEM = "read";	// ALL BMS
const char dbg_RWARN_period[] PROGMEM = "rwarn";
#else
#define DEBUG(s)
#define DEBUGN(s)
#define DEBUGIF(d,s)
#define DEBUGIFN(d,s)
#endif
const char sERR_BMS_NotAnswer[] PROGMEM	= "NOT ANSWER";
const char sERR_BMS_Read[] PROGMEM		= "READ ERROR";
const char sERR_BMS_Resistance[] PROGMEM= "BAD WIRES";
const char sERR_BMS_Config[] PROGMEM	= "CFG ERROR";

enum {
	round_true = 0,
	round_cut,
	round_up
};

enum { // options bits
	o_average = 0,
//	o_median = 1,
	o_equalize_cells_V	// корректировать напряжения на ячейках, чтобы сумма V ячеек соответствовала TotalV
};

enum I2C_MAP_MODE {
	M_OFF        = 0,//выключенно без сети
	M_OFFNET     = 1,//выключенно с сетью
	M_ON         = 2,//включенно без сети (работает МАП)
	M_ONNET      = 3,//включенно с сетью
	M_ONCHARGE   = 4,//включенно с зарядом
	M_NOT_READ   = 255
};

struct WORK {
	uint8_t  bms_num;
	uint8_t  bms_cells_qty;
	uint8_t  options;
	uint32_t BMS_read_period;		// ms, 1 - synch i2C
	uint32_t BMS_wait_answer_time;	// ms
	uint8_t  I2C_bms_rotate_period; // times, period (number of reading) before switch to next bms
	uint8_t  round;					// round_*
	int16_t  V_correct;				// mV
	int8_t   temp_correct;			// mV
	uint8_t  watchdog;				// reboot if no data over: 1 - I2C, 2 - BMS
	uint16_t cell_min_V;			// alarm, mV
	uint16_t cell_max_V;			// alarm, mV
	uint16_t cell_delta_max_V;		// alarm, mV
	int16_t  Vmaxhyst;				// *10mV
	uint16_t BalansDeltaPause;		// minutes wait for change to default delta
	uint16_t BalansDeltaDefault;	// Default balans delta voltage, mV
	uint16_t BalansDelta;			// mV
	uint8_t  BalansDeltaI;			// Current threshold
	uint8_t  RWARN_Send_Period;		// sec
} work;

struct _EEPROM {
	WORK    work;
} __attribute__ ((packed));

struct _EEPROM EEMEM EEPROM = {
	.work = {
		.bms_num = 2,
		.bms_cells_qty = 16,
		.options = (1<<o_equalize_cells_V),
		.BMS_read_period = 5000,
		.BMS_wait_answer_time = BMS_MIN_PAUSE_BETWEEN_READS,
		.I2C_bms_rotate_period = 5,
		.round = round_true,
		.V_correct = 0,
		.temp_correct = 0,
		.watchdog = 0,
		.cell_min_V = 2900,
		.cell_max_V = 3500,
		.cell_delta_max_V = 100,
		.Vmaxhyst = 1,
		.BalansDeltaPause = 17*60,
		.BalansDeltaDefault = 0, //5,
		.BalansDelta = 20,
		.BalansDeltaI = 2,
		.RWARN_Send_Period = 20
	}
};

enum { // last_error[n] =
	ERR_BMS_Ok = 0,
	ERR_BMS_NotAnswer = 1,
	ERR_BMS_Read = 2,
	ERR_BMS_Config = 3,
	ERR_BMS_Resistance = 4,
	ERR_BMS_Cell_Low = 5,
	ERR_BMS_Cell_High = 6,
	ERR_BMS_Cell_DeltaMax = 7
};
enum { // alarm =
	ALARM_BMS_Cell_Low = 1,
	ALARM_BMS_Cell_High = 2,
	ALARM_BMS_Cell_DeltaMax = 3
};
enum {
	f_BMS_Ready = 0,
	f_BMS_Need_Read = 1,
	f_BMS_Wait_Answer = 2,
	f_BMS_ReadOk = 3
};

enum {
//	RWARN_NONE = 0,
	RWARN_OK = 1,
	RWARN_BMS_NotAnswer = 2,
	RWARN_BMS_Error = 3,
	RWARN_Cell_Low = 4,
	RWARN_Cell_High = 5,
	RWARN_Cell_DeltaMax = 6
};

struct RWARN_BMS {
	uint8_t  last_status;	// bms_flags (b7=on/off, b6=balancing) + last_error
	uint8_t  bms_min_string;
	uint8_t  bms_max_string;
	uint16_t bms_min_cell_mV;
	uint16_t bms_max_cell_mV;
	int32_t  bms_total_mV;
} __attribute__ ((packed));
uint8_t rwarn_buf[1 + BMS_NUM_MAX * sizeof(RWARN_BMS) + 2]; // bms_num + RWARN_BMS + crc16
uint8_t  RWARN_period;	// sec
uint8_t  RWARN_idx = 0;
uint8_t  RWARN_send_len;
uint8_t  RWARN_bit = 0;
uint8_t  RWARN_byte;
uint32_t RWARN_quantum = 0;

uint8_t  flags = (1<<f_BMS_Need_Read);	// f_*
int8_t   debug = 0;					// 0 - off, 1 - on, 2 - detailed dump, 3 - full dump, 4 - BMS full
uint8_t  debugmode = 0;				// 0 - off, 1 - debug to TX
uint16_t bms[BMS_NUM_MAX][BMS_CELLS_QTY_MAX];	// *10mV
uint16_t bms_avg[BMS_NUM_MAX][BMS_CELLS_QTY_MAX];
uint8_t  bms_select = 0;
uint8_t  bms_Q[BMS_CELLS_QTY_MAX];// %
uint8_t  bms_idx = 0;
uint8_t  bms_idx_prev = 0;
uint32_t bms_loop_time;
uint8_t  bms_flags[BMS_NUM_MAX];	// b0 - on/off, b1 - balance current > 0
uint16_t bms_min_cell_mV[BMS_NUM_MAX];	// *1mV
uint8_t  bms_min_string[BMS_NUM_MAX];
uint16_t bms_max_cell_mV[BMS_NUM_MAX];	// *1mV
uint8_t  bms_max_string[BMS_NUM_MAX];
int32_t  bms_total_mV[BMS_NUM_MAX];		// *1mV
uint32_t sec_timer = 0;
uint8_t  watchdog_I2C = 0;
uint8_t  watchdog_BMS = 0;
uint8_t  I2C_bms_send_switch_timer = 0;
int16_t  map_cell_min = 0;			// *10mV (1 cell)
int16_t  map_cell_full = 0;			// *10mV (1 cell)
int16_t  bms_cell_max[BMS_NUM_MAX];		// *10mV
int16_t  bms_cell_min[BMS_NUM_MAX];		// *10mV
uint8_t  selected_bms = 0;
uint8_t  sending_bms = 0;
uint8_t  map_mode = M_NOT_READ;
uint8_t  temp = BMS_NO_TEMP;		// C, +50
uint8_t  crc;
uint8_t  last_error[BMS_NUM_MAX];
uint8_t  alarm[BMS_NUM_MAX];
uint8_t  error_alarm_time = 0;
uint8_t  error_new = 0;
uint8_t  error_send = 0;
uint8_t  read_buffer[74];
int8_t   read_idx = 0;
uint8_t  i2c_receive[32];
uint8_t  i2c_receive_idx = 0;
uint32_t bms_last_read_time = 0;
uint16_t delta_active = 0;			// mV
uint16_t delta_new = 0;				// mV
uint16_t delta_next = 0;
uint16_t delta_change_pause = 0;  	// sec
uint8_t  delta_change_equalizer = 0; // attempts
uint8_t  delta_new_cnt = 0;			// update cnt < max
uint8_t  debug_info	= 0b0011;		// b0 - I2C_W1, b1 - I2C_W2
uint8_t  read_bms_num = 0;			// last bms # read
uint8_t  beep_num = 0;
uint8_t  beep_cnt = 0;
uint8_t  beep_time = 0;
uint8_t  key1_status = 0;
uint8_t  key1_delay = 0;		// 1-2ms
volatile uint8_t *key1_PIN;
uint8_t  key1_MASK;
#ifdef LCD_ENABLED
uint8_t  LCD_refresh_sec = 3;	// счетчик обновления LCD
uint8_t  LCD_page = 0;
#endif

ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect));
ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect));
ISR(PCINT3_vect, ISR_ALIASOF(PCINT0_vect));
ISR(PCINT0_vect) {
	if(!key1_delay) {
		if(!(*key1_PIN & key1_MASK)) key1_status = 1; // pressed
	}
	key1_delay = 150;
}

// Called in delay()
void yield(void)
{
//	sleep_cpu();
}

void Delay100ms(uint8_t ms) {
	while(ms-- > 0) {
		_delay_ms(100); wdt_reset();
	}
}

void FlashLED(uint8_t num, uint8_t ton, uint8_t toff) {
	while (num-- > 0) {
		digitalWrite(LED_PD, HIGH);
		Delay100ms(ton);
		digitalWrite(LED_PD, LOW);
		Delay100ms(toff);
	}
}

void Set_New_Error(uint8_t _err)
{
	if(_err) {
		last_error[read_bms_num] = _err;
		BLINK_ALARM;
	}
}

void i2c_set_slave_addr(uint8_t addr)
{
	TWAR = (addr << 1) | 1; // +broardcast addr(0)
}

void i2c_write(uint8_t d)
{
	crc += d;
	Wire.write(d);
}

void I2C_Response() {
	crc = 0;
	i2c_write(7);									// size
	i2c_write(5);									// op_code
	if(bms_idx == 0) sending_bms = selected_bms;
	i2c_write(bms[sending_bms][bms_idx] & 0xFF);	// Ucell(low), V, hundreds
	i2c_write(bms[sending_bms][bms_idx] >> 8);		// Ucell(high), V, hundreds
	i2c_write(bms_idx == sending_bms ? temp : BMS_NO_TEMP);	// temp + 50, 255 - none
	i2c_write(bms_Q[bms_idx]);						// Q_Cell, %, I=(Q_Cell/100)*(Ucell/R), R=1
	i2c_write(bms_idx == sending_bms ? last_error[bms_idx] : 0);		// prev err
	crc = 0 - crc;
	i2c_write(crc);
	if(++bms_idx == work.bms_cells_qty) bms_idx = 0;
	i2c_set_slave_addr(bms_idx + 1);
	i2c_receive_idx = 0; // clear I2C_W garbage, if available
	watchdog_I2C = 0;
}

void I2C_Receive(int howMany) {
	(void)howMany;  // unused
	if(i2c_receive_idx >= sizeof(i2c_receive)) return;
	while(Wire.available()) {
		i2c_receive[i2c_receive_idx++] = Wire.read();
		if(i2c_receive_idx >= sizeof(i2c_receive)) break;
	}
}

void RWARN_check_send(void)
{
	uint32_t m = micros();
	if(RWARN_period == 0 && m - RWARN_quantum >= RWARN_PULSE_QT) {
		RWARN_quantum = m;
		if(RWARN_bit == 0) { // start pulse
			digitalWrite(RWARN_PULSE_PIN, RWARN_PULSE_LEVEL);
			if(RWARN_idx == 0){ // begin
				rwarn_buf[0] = work.bms_num + 0x80;
				RWARN_BMS *ptr = (RWARN_BMS *) &rwarn_buf[1];
				for(uint8_t i = 0; i < work.bms_num; i++) {
					ptr->last_status = (bms_flags[i]<<6) | last_error[i];
					ptr->bms_min_string = bms_min_string[i];
					ptr->bms_max_string = bms_max_string[i];
					ptr->bms_min_cell_mV = bms_min_cell_mV[i];
					ptr->bms_max_cell_mV = bms_max_cell_mV[i];
					ptr->bms_total_mV = bms_total_mV[i];
					ptr++;
				}
				*(uint16_t *)ptr = calc_crc16(rwarn_buf, RWARN_send_len = (uint8_t *)ptr - rwarn_buf, 0xFFFF);
				RWARN_send_len += 2; // +CRC16
			}
			RWARN_byte = rwarn_buf[RWARN_idx];
			//DEBUG(F("RW: ")); DEBUGN(RWARN_byte);
			RWARN_bit = 1;
		} else if(RWARN_bit == 9) { // stop pulse
			digitalWrite(RWARN_PULSE_PIN, !RWARN_PULSE_LEVEL);
			if(++RWARN_idx >= RWARN_send_len){
				RWARN_period = work.RWARN_Send_Period;
				RWARN_idx = 0;
			}
			RWARN_bit = 0;
		} else {
			digitalWrite(RWARN_PULSE_PIN, RWARN_byte & 1 ? !RWARN_PULSE_LEVEL : RWARN_PULSE_LEVEL);
			RWARN_byte >>= 1;
			RWARN_bit++;
		}
	}
}

#ifdef LCD_ENABLED
#define LCD_SCR_last_page 	0
#define LCD_SCR_last_err1	1
#define LCD_SCR_last_err2	2
#define LCD_SCR_last_pls	7
uint8_t LCD_SCR_last = 127;
int32_t LCD_SCR_TotalV[BMS_NUM_MAX];
uint16_t LCD_SCR_MinCellV[BMS_NUM_MAX];
uint16_t LCD_SCR_MaxCellV[BMS_NUM_MAX];

// Outs error text and fills remaining space in string with spaces
void LCD_Display_Err(uint8_t _err, uint8_t space)
{
	const char *str;
	if(_err == ERR_BMS_NotAnswer) str = sERR_BMS_NotAnswer;
	else if(_err == ERR_BMS_Read) str = sERR_BMS_Read;
	else if(_err == ERR_BMS_Resistance) str = sERR_BMS_Resistance;
	else if(_err == ERR_BMS_Config) str = sERR_BMS_Config;
	lcd.print(str);
	int8_t n = space - strlen(str);
	while(n-- > 0) lcd.print(' ');
}

// n.nnn
void LCD_print_num_d3(int32_t num)
{
	lcd.print(num / 1000);
	num = abs(num) % 1000;
	lcd.print('.');
	if(num < 100) lcd.print('0');
	if(num < 10) lcd.print('0');
	lcd.print(num);
}

void LCD_Display(void)
{
	uint8_t refresh_all = 0;
	if(LCD_page == 0) {
//  01234567890123456789
//  B1: ON* 53.421V ·5
//   3.223(12) 3.234(16)
//  B2: OFF 53.342V ·12
//   3.223(12) 3.234(16)
		if(bitRead(LCD_SCR_last, LCD_SCR_last_page)) {
			lcd.clear();
			refresh_all = 1;
		}
		bitClear(LCD_SCR_last, LCD_SCR_last_page);
		for(uint8_t i = 0; i < 2; i++) {
			if(refresh_all) {
				lcd.setCursor(0, i*2);
				lcd.print('B');
			} else lcd.setCursor(1, i*2);
			lcd.print(bitRead(LCD_SCR_last, LCD_SCR_last_pls) ? ':' : '.');
			lcd.print(' ');
			uint16_t sub_min = LCD_SCR_MinCellV[i] - bms_min_cell_mV[i];
			uint16_t sub_max = LCD_SCR_MaxCellV[i] - bms_max_cell_mV[i];
			if(last_error[i]) {
				LCD_Display_Err(last_error[i], 16);
				if(debugmode && i == 0 && bitRead(LCD_SCR_last, LCD_SCR_last_pls)) {
					lcd.setCursor(19, 0);
					lcd.print('D');
				}
				bitSet(LCD_SCR_last, LCD_SCR_last_err1 + i);
			} else {
				if(bitRead(LCD_SCR_last, LCD_SCR_last_err1 + i)) refresh_all = 1;
				bitClear(LCD_SCR_last, LCD_SCR_last_err1 + i);
				if(bitRead(bms_flags[i], 0)) {
					lcd.print(F("ON"));
					lcd.print(bitRead(bms_flags[i], 0) ? '*' : ' ');
				} else lcd.print(F("OFF"));
				if(LCD_SCR_TotalV[i] != bms_total_mV[i] || refresh_all) {
					lcd.print(' ');
					LCD_print_num_d3(LCD_SCR_TotalV[i] = bms_total_mV[i]);
					lcd.print(F("V"));
				}
				if(sub_min != 0 || sub_max != 0 || refresh_all) {
					uint16_t n = bms_max_cell_mV[i] - bms_min_cell_mV[i];
					if(debugmode && i == 0) {
						lcd.setCursor(n > 99 ? 15 : 16, i*2 + 1);
						lcd.print(0xA5); // '·'
						lcd.print(n);
						if(n < 1000) {
							lcd.print(' ');
							if(n < 100) lcd.print(' ');
							if(n < 10) lcd.print(' ');
						}
						if(bitRead(LCD_SCR_last, LCD_SCR_last_pls)) lcd.print('D');
					} else {
						lcd.setCursor(n > 999 ? 15 : 16, i*2 + 1);
						lcd.print(0xA5); // '·'
						lcd.print(n);
						if(n < 100) {
							lcd.print(' ');
							if(n < 10) lcd.print(' ');
						}
					}
				} else if(debugmode && i == 0) {
					lcd.setCursor(19, 0);
					lcd.print(bitRead(LCD_SCR_last, LCD_SCR_last_pls) ? 'D' : ' ');
				}
			}
			if(sub_min != 0 || refresh_all) {
				lcd.setCursor(1, i*2 + 1);
				LCD_print_num_d3(LCD_SCR_MinCellV[i] = bms_min_cell_mV[i]);
				lcd.print('(');
				lcd.print(bms_min_string[i]);
				lcd.print(F(") "));
			}
			if(sub_max != 0 || refresh_all) {
				lcd.setCursor(11, i*2 + 1);
				LCD_print_num_d3(LCD_SCR_MaxCellV[i] = bms_max_cell_mV[i]);
				lcd.print('(');
				lcd.print(bms_max_string[i]);
				lcd.print(F(") "));
			}
		}
		bitToggle(LCD_SCR_last, LCD_SCR_last_pls);
	} else if(LCD_page == 1) {
		LCD_page = 0;
		if(!bitRead(LCD_SCR_last, LCD_SCR_last_page)) {
			lcd.clear();
			// to do...
		}
	}
}
#endif

#ifdef DEBUG_TO_SERIAL

void DebugSerial_read(void)
{
	while(DebugSerial.available()) {
		int r = DebugSerial.read();
		if(r == -1 || r == '\n') break;
		debug_read_buffer[debug_read_idx++] = r;
		if(r == '\r' || debug_read_idx == sizeof(debug_read_buffer)-1) {
			debug_read_buffer[debug_read_idx-1] = '\0';
			debug_read_idx = 0;
			char *p = strchr(debug_read_buffer, '=');
			if(p == NULL) break;
			*p = '\0';
			DEBUG(F("CFG: ")); DEBUG(debug_read_buffer); DEBUG('=');
			uint16_t d = strtol(p + 1, NULL, 0);
			if(strncmp_P(debug_read_buffer, dbg_temp_correct, sizeof(dbg_temp_correct)-1) == 0) {
				work.temp_correct = d;
				DEBUG(work.temp_correct);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_temp, sizeof(dbg_temp)-1) == 0) {
				temp = d + 50;
				DEBUG(d);
			} else if(strncmp_P(debug_read_buffer, dbg_bms, sizeof(dbg_bms)-1) == 0) {
				if(d > BMS_NUM_MAX) d = BMS_NUM_MAX;
				work.bms_num = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_cells, sizeof(dbg_cells)-1) == 0) {
				if(d < 2) d = 2;
				work.bms_cells_qty = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_period, sizeof(dbg_period)-1) == 0) {
				work.BMS_read_period = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_wait_answer, sizeof(dbg_wait_answer)-1) == 0) {
				work.BMS_wait_answer_time = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_rotate, sizeof(dbg_rotate)-1) == 0) {
				work.I2C_bms_rotate_period = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_round, sizeof(dbg_round)-1) == 0) {
				work.round = d;
				DEBUGN(work.round == round_true ? "5/4" : work.round == round_cut ? "cut" : work.round == round_up ? "up" : "?");
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_correct, sizeof(dbg_correct)-1) == 0) {
				work.V_correct = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_vmaxhyst, sizeof(dbg_vmaxhyst)-1) == 0) {
				work.Vmaxhyst = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_cell_min_V, sizeof(dbg_cell_min_V)-1) == 0) {
				work.cell_min_V = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_cell_max_V, sizeof(dbg_cell_max_V)-1) == 0) {
				work.cell_max_V = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_delta_default, sizeof(dbg_delta_default)-1) == 0) {
				work.BalansDeltaDefault = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_delta_pause, sizeof(dbg_delta_pause)-1) == 0) {
				work.BalansDeltaPause = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_watchdog, sizeof(dbg_watchdog)-1) == 0) {
				work.watchdog = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_options, sizeof(dbg_options)-1) == 0) {
				work.options = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_debug, sizeof(dbg_debug)-1) == 0) {
				debug = d;
				DEBUG(d);
			} else if(strncmp_P(debug_read_buffer, dbg_seterr, sizeof(dbg_seterr)-1) == 0) {
				Set_New_Error(d);
				DEBUG(d);
			} else if(strncmp_P(debug_read_buffer, dbg_delta_change_pause, sizeof(dbg_delta_change_pause)-1) == 0) {
				delta_change_pause = d;
				DEBUG(d);
			} else if(strncmp_P(debug_read_buffer, dbg_read, sizeof(dbg_read)-1) == 0) {
				bitSet(flags, f_BMS_Need_Read);
				DEBUG(d);
			} else if(strncmp_P(debug_read_buffer, dbg_RWARN_period, sizeof(dbg_RWARN_period)-1) == 0) {
				work.RWARN_Send_Period = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(debug_read_buffer[1] >= '0' && debug_read_buffer[1] <= '3' && (!debug_read_buffer[2] || !debug_read_buffer[3])) {
				if((debug_read_buffer[0] | 0x20) == 'v') { // Vn=x, n={1..bms_cells_qty}, n=0 - all
					if(d) {
						uint8_t i = strtol(debug_read_buffer + 1, NULL, 0);
						ATOMIC_BLOCK(ATOMIC_FORCEON) {
							if(i == 0) {
								for(; i < work.bms_cells_qty; i++) {
									bms[0][i] = d;
									//if(bms_full && d > bms_full+1) d = bms_full+1;
								}
							} else if(--i < work.bms_cells_qty) bms[0][i] = d;
						}
						if(!bitRead(flags, f_BMS_Ready)) {
							i2c_set_slave_addr(bms_idx + 1);
							bitSet(flags, f_BMS_Ready);
						}
						DEBUG(d);
					}
				} else if((debug_read_buffer[0] | 0x20) == 'q') { // Qn=x, n={1..bms_cells_qty}
					uint8_t i = strtol(debug_read_buffer + 1, NULL, 0);
					if(--i < work.bms_cells_qty) bms_Q[i] = d;
					DEBUG(d);
				} else if((debug_read_buffer[0] | 0x20) == 'd') { // D=x
					work.BalansDelta = d;
					DEBUG(d);
					eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
				} else if((debug_read_buffer[0] | 0x20) == 'i') { // I=x
					work.BalansDeltaI = d;
					DEBUG(d);
					eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
				} else DEBUG(F("ERR!"));
			} else DEBUG(F("ERR!"));
			DEBUG('\n');
			break;
		}
	}
}
#endif

// Return CRC
uint8_t BMS_send_pgm_cmd(const uint8_t *cmd, uint8_t len, uint8_t crc)
{
	uint8_t b;
	while(len--) {
		BMS_SERIAL.write(b = pgm_read_byte(cmd++));
		crc += b;
	}
	return crc;
}

#ifdef DEBUG_BMS_SEND
const uint8_t _send1[] PROGMEM = "\xEB\x90\x01\xFF\x1E\xD3\x0F\x69\x14\x13\x02\x00\x00\x00\x07\x00\x00\x00\x05\x03\xE8\x01\x14\x0F\x67\x0F\x6B\x0F\x66\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x67\x0F\x69\x0F\x69\x0F\x69\x0F\x64\x0F\x6B\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x00\x16\x67";
const uint8_t _send2[] PROGMEM = "\xEB\x90\x01\xFF\x1E\xE4\x0F\x69\x14\x13\x02\x00\x00\x00\x07\x00\x00\x00\x05\x03\xE8\x01\x14\x0F\x69\x0F\x6B\x0F\x66\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x67\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x0F\x69\x00\x16\x7D";
#endif

// JK-DZ11-B2A24S active balancer
void BMS_Serial_read(void)
{
	if(!bitRead(flags, f_BMS_Wait_Answer)) return;
#ifdef DEBUG_BMS_SEND
	for(uint8_t i = 0; i < sizeof(read_buffer); i++) read_buffer[i] = pgm_read_byte(read_bms_num ? &_send2[i] : &_send1[i]);
	while(1) {
		{
#else
	uint8_t _err = 255;
	while(BMS_SERIAL.available()) {
		int16_t r = BMS_SERIAL.read();
		bms_last_read_time = millis();
		if(r == -1) break;
		read_buffer[read_idx++] = r;
		if(read_idx == sizeof(read_buffer)) {
#endif
			_err = ERR_BMS_Ok;
			read_idx = 0;
			if(read_buffer[0] != 0xEB || read_buffer[1] != 0x90) {
				DEBUGIFN(1,F("BMS: Header mismatch!"));
				_err = ERR_BMS_Read;
				break;
			}
			if(debug == 4) { DEBUG(F("BMS")); DEBUG(read_bms_num + 1); DEBUG(F(" Answer: ")); }
			uint8_t crc = 0;
			for(uint8_t i = 0; i < sizeof(read_buffer) - 1; i++) {
				if(debug == 4) {
					DEBUG(i); DEBUG('='); DEBUGH(read_buffer[i]); DEBUG(' ');
				}
				crc += read_buffer[i];
			}
			if(debug == 4) DEBUG('\n');
			if(crc != read_buffer[sizeof(read_buffer) - 1]) {
				DEBUGIF(1,F("BMS"));
				DEBUGIF(1, read_bms_num + 1);
				DEBUGIFN(1,F(": CRC Error!"));
				_err = ERR_BMS_Read;
				break;
			}
			if(read_buffer[BMS_OFFSET_Cmd] == 0xF2) { // Change delta voltage
				//delta_active = read_buffer[4]*256 + read_buffer[5];
				if(debug) {
					DEBUG(F("New delta: ")); DEBUGN(delta_active);
				}
				watchdog_BMS = 0;
				bms_last_read_time = millis();
			} else if(read_buffer[BMS_OFFSET_Cmd] == 0xFF) { // Request answer
				if(read_buffer[BMS_OFFSET_Alarm] & 0x03) {
					DEBUGIF(1,F("BMS"));
					DEBUGIF(1, read_bms_num + 1);
					DEBUGIF(1,F(" Alarm: "));
					if(read_buffer[BMS_OFFSET_Alarm] & (1<<0)) { // cells num wrong
						DEBUGIF(1,F("Cells_Num "));
						_err = ERR_BMS_Config;
					}
					if(read_buffer[BMS_OFFSET_Alarm] & (1<<1)) { // wire resistance is too large
						DEBUGIF(1,F("Wire_Resistance "));
						_err = ERR_BMS_Resistance;
					}
	//				if(read_buffer[BMS_OFFSET_Alarm] & (1<<2)) { // battery overvoltage
	//					last_error = ERR_BMS_Resistance;
	//					DEBUG(F("Overvoltage"));
	//				}
					DEBUGIF(1,'\n');
				}
				int16_t totalV = read_buffer[BMS_OFFSET_TotalV]*256 + read_buffer[BMS_OFFSET_TotalV + 1];
				int16_t totalV_by_cells = 0;
				uint16_t balance_current = read_buffer[BMS_OFFSET_BalansI]*256 + read_buffer[BMS_OFFSET_BalansI + 1];
				bms_flags[read_bms_num] = ((balance_current > 0)<<1) | (read_buffer[BMS_OFFSET_BalansOn] == 1);
				bms_min_string[read_bms_num] = read_buffer[BMS_OFFSET_LowV_Cell];
				bms_min_cell_mV[read_bms_num] = read_buffer[BMS_OFFSET_CellsV_Array + read_buffer[BMS_OFFSET_LowV_Cell]*2]*256 + read_buffer[BMS_OFFSET_CellsV_Array+1 + read_buffer[BMS_OFFSET_LowV_Cell]*2];
				bms_max_string[read_bms_num] = read_buffer[BMS_OFFSET_HighV_Cell];
				bms_max_cell_mV[read_bms_num] = read_buffer[BMS_OFFSET_CellsV_Array + read_buffer[BMS_OFFSET_HighV_Cell]*2]*256 + read_buffer[BMS_OFFSET_CellsV_Array+1 + read_buffer[BMS_OFFSET_HighV_Cell]*2];
				if(bms_min_cell_mV[read_bms_num] <= work.cell_min_V) {
					_err = ERR_BMS_Cell_Low;
				} else if(bms_max_cell_mV[read_bms_num] <= work.cell_max_V) {
					_err = ERR_BMS_Cell_High;
					alarm[read_bms_num] = ALARM_BMS_Cell_High;
				} else if(bms_max_cell_mV[read_bms_num] - bms_min_cell_mV[read_bms_num] >= work.cell_delta_max_V) {
					_err = ERR_BMS_Cell_DeltaMax;
				}
	#ifdef DEBUG_TO_SERIAL
				if(debug == 3) {
					DEBUG(F("BMS ")); DEBUG(read_bms_num + 1);	DEBUG(' ');
					if(read_buffer[BMS_OFFSET_BalansOn]) DEBUG(F("ON")); else DEBUG(F("OFF"));
					DEBUG(F(",V:")); DEBUG(totalV / 100); DEBUG('.'); if(totalV % 100 < 10) DEBUG('0'); DEBUG(totalV % 100);
					DEBUG(F(",D(mV):")); DEBUG(bms_max_cell_mV[read_bms_num] - bms_min_cell_mV[read_bms_num]);
					DEBUG(F(",Trg(mV):")); DEBUG(read_buffer[BMS_OFFSET_TriggerV]*256 + read_buffer[BMS_OFFSET_TriggerV + 1]);
					DEBUG(F(",Bal(mA):")); DEBUG(balance_current);
					DEBUG(F(",T(C):")); DEBUG(read_buffer[BMS_OFFSET_Temperature + 1]);
					//DEBUG(F(",W:")); DEBUG(read_buffer[BMS_OFFSET_Cells]); DEBUG(','); DEBUG(read_buffer[BMS_OFFSET_HighV_Cell]); DEBUG(','); DEBUG(read_buffer[BMS_OFFSET_LowV_Cell]); DEBUG(','); DEBUG(read_buffer[BMS_OFFSET_BalansDir]);
					DEBUG(F("\n"));
				}
	#endif
				if(read_buffer[BMS_OFFSET_Cells] != work.bms_cells_qty) {
					DEBUGIF(1,F("BMS"));  DEBUGIF(1, read_bms_num + 1);
					DEBUGIFN(1,F(" Cells num not equal setup!"));
					_err = ERR_BMS_Config;
					if(read_buffer[BMS_OFFSET_Cells] > BMS_CELLS_QTY_MAX) break;
				}
				int16_t _max = 0;
				int16_t _min = 32767;
				bms_total_mV[read_bms_num] = 0;
				uint16_t *p = &bms[read_bms_num][0];
				for(uint8_t i = 0; i < read_buffer[8]; i++,p++) {
					if(i > work.bms_cells_qty - 1) {
						ATOMIC_BLOCK(ATOMIC_FORCEON) *p = 0;
						continue;
					}
					int16_t v = read_buffer[BMS_OFFSET_CellsV_Array + i*2]*256 + read_buffer[BMS_OFFSET_CellsV_Array+1 + i*2];
					bms_total_mV[read_bms_num] += v;
					if(work.V_correct) {
						v += work.V_correct;
						if(v < 0) v = 0;
					}
//					if(bitRead(work.options, o_median)) {
//						// Медианный фильтр
//						static int16_t median1[BMS_QTY_MAX], median2[BMS_QTY_MAX];
//						if(median1[i] == 0) median1[i] = median2[i] = v;
//						int16_t median3 = v;
//						if(median1[i] <= median2[i] && median1[i] <= median3) {
//							v = median2[i] <= median3 ? median2[i] : median3;
//						} else if(median2[i] <= median1[i] && median2[i] <= median3) {
//							v = median1[i] <= median3 ? median1[i] : median3;
//						} else {
//							v = median1[i] <= median2[i] ? median1[i] : median2[i];
//						}
//						median1[i] = median2[i];
//						median2[i] = median3[i];
//						//
//					}
					if(bitRead(work.options, o_average)) {
						if(bms_avg[read_bms_num][i]) v = (bms_avg[read_bms_num][i] + v) / 2;
						bms_avg[read_bms_num][i] = v;
					}
					if(work.round == round_true) v += 5;
					else if(work.round == round_up) v += 9;
					v /= 10; // 0.001 -> 0.01
					if(work.Vmaxhyst && map_cell_full) {
						if(v >= map_cell_full && v < map_cell_full + work.Vmaxhyst) v = map_cell_full - 1;
					}
					ATOMIC_BLOCK(ATOMIC_FORCEON) *p = v;
					totalV_by_cells += v;
					if(v > _max) _max = v;
					if(v < _min) _min = v;
				}
				bms_cell_max[read_bms_num] = _max;
				bms_cell_min[read_bms_num] = _min;
				totalV -= totalV_by_cells;
				if(bitRead(work.options, o_equalize_cells_V) && totalV != 0) {
					if(debug == 3) {
						DEBUG(F("BMS V="));
						for(uint8_t i = 0; i < work.bms_cells_qty; i++) { DEBUG(bms[read_bms_num][i]); DEBUG(F(" ")); }
						DEBUG(F("\nTotalV diff ")); DEBUG(totalV); DEBUG(": ");

					}
					bool allow = false;
					int8_t d = totalV > 0 ? 1 : -1;
					while(totalV != 0) {
						uint16_t *p = &bms[read_bms_num][0];
						bool edge_skip = true;
						for(uint8_t i = 0; i < read_buffer[8]; i++,p++) {
							int16_t tmp = *p;
							if(d == 1) {
								if(edge_skip && tmp == _min) {
									edge_skip = false;
									continue;
								}
								tmp++;
								if(allow || tmp <= _max) {
									ATOMIC_BLOCK(ATOMIC_FORCEON) *p = tmp;
									totalV -= d;
									if(debug == 3) { DEBUG(i); DEBUG(' '); }
								}
							} else { // d == -1
								if(edge_skip && tmp == _max) {
									edge_skip = false;
									continue;
								}
								tmp--;
								if(allow || tmp >= _min) {
									ATOMIC_BLOCK(ATOMIC_FORCEON) *p = tmp;
									totalV -= d;
									if(debug == 3) { DEBUG(i); DEBUG(' '); }
								}
							}
							if(totalV == 0) break;
						}
						if(debug == 3 && totalV) DEBUG(" * ");
						allow = true;
					}
					if(debug == 3) {
						DEBUG(F("\nBMS V="));
						for(uint8_t i = 0; i < work.bms_cells_qty; i++) { DEBUG(bms[read_bms_num][i]); DEBUG(F(" ")); }
						DEBUG('\n');
					}
				}
				uint8_t _sel = 0xFF;
				if(map_mode == M_ON) { // discharge - select battery with min cell
					_min = 32767;
					for(uint8_t i = 0; i < work.bms_num; i++) {
						if(bms_cell_min[i] && bms_cell_min[i] < _min) {
							_min = bms_cell_min[i];
							_sel = i;
						}
					}
				} else { // select battery with overcharge cell
					_max = 0;
					if(map_mode != M_NOT_READ) {
						for(uint8_t i = 0; i < work.bms_num; i++) {
							if(bms_cell_max[i] >= map_cell_full && bms_cell_max[i] > _max) {
								_max = bms_cell_max[i];
								_sel = i;
							}
						}
					}
					if(_max == 0) {
						if(I2C_bms_send_switch_timer) I2C_bms_send_switch_timer--;
						else {
							I2C_bms_send_switch_timer = work.I2C_bms_rotate_period;
							if(++selected_bms == work.bms_num) selected_bms = 0;
						}
					} else selected_bms = _sel;
				}
				selected_bms = _sel;

				temp = read_buffer[BMS_OFFSET_Temperature + 1] + BMS_Temperature_Correct + work.temp_correct;
				delta_active = read_buffer[BMS_OFFSET_TriggerV]*256 + read_buffer[BMS_OFFSET_TriggerV + 1];
				if(read_bms_num == 0) memset(bms_Q, 0, sizeof(bms_Q));
				uint8_t i = read_buffer[BMS_OFFSET_HighV_Cell];
				if(i < work.bms_cells_qty && read_buffer[BMS_OFFSET_BalansDir]) bms_Q[i] = 100UL * (read_buffer[BMS_OFFSET_BalansI]*256 + read_buffer[BMS_OFFSET_BalansI+1]) / (read_buffer[BMS_OFFSET_CellsV_Array + i*2]*256 + read_buffer[BMS_OFFSET_CellsV_Array+1 + i*2]); // Q_Cell=100*I/(Ucell/R), R=1
				if(!bitRead(flags, f_BMS_Ready)) {
					i2c_set_slave_addr(bms_idx + 1);
					bitSet(flags, f_BMS_Ready);
					bms_loop_time = millis();
				}
				if(_err == 0) _err = RWARN_OK;
				bitSet(flags, f_BMS_ReadOk);
				watchdog_BMS = 0;
			} else if(debug) {
				DEBUG(F("BMS"));
				DEBUG(read_bms_num + 1);
				DEBUG(F(": Wrong response code: ")); DEBUGN(read_buffer[3]);
			}
			if(debug == 3) {
				DEBUG(F("Sel: ")); DEBUGN(selected_bms);
			}
			break;
		}
	}
	if(_err != 255) Set_New_Error(_err);
}

void setup()
{
	wdt_enable(WDTO_2S); // Enable WDT
	//set_sleep_mode((0<<SM2)|(0<<SM1)|(0<<SM0));	// idle
	sleep_enable();
	PRR0 = (1<<PRSPI0) | (1<<PRADC); // Power off: SPIs, ADC, PTC
	PRR1 = (1<<PRPTC) | (1<<PRSPI1);
	BMS_SERIAL.begin(BMS_SERIAL_RATE);

	pinMode(KEY1, INPUT_PULLUP);
	*digitalPinToPCMSK(KEY1) |= _BV(digitalPinToPCMSKbit(KEY1));
	*digitalPinToPCICR(KEY1) |= _BV(digitalPinToPCICRbit(KEY1));
	key1_PIN = portInputRegister(digitalPinToPort(KEY1));
	key1_MASK = digitalPinToBitMask(KEY1);

	pinMode(BUZZER_PD1, OUTPUT); pinMode(BUZZER_PD2, OUTPUT); pinMode(BUZZER_PD3, OUTPUT);
	digitalWrite(OUT_CELL_UNDER_V, LOW);
	pinMode(OUT_CELL_UNDER_V, INPUT_PULLUP);
	digitalWrite(OUT_CELL_OVER_V, LOW);
	pinMode(OUT_CELL_OVER_V, INPUT_PULLUP);
	pinMode(RWARN_PULSE_PIN, OUTPUT);
	digitalWrite(RWARN_PULSE_PIN, !RWARN_PULSE_LEVEL);
	pinMode(LED_PD, OUTPUT);
	digitalWrite(LED_PD, LOW);
#ifdef LCD_Backlite_pin
	pinMode(LCD_Backlite_pin, OUTPUT);
	digitalWrite(LCD_Backlite_pin, 1);
#endif
#ifdef DEBUG_TO_SERIAL
	DebugSerial.begin(DEBUG_TO_SERIAL_RATE);
 #ifdef DEBUG_ACTIVE_PD
	pinMode(DEBUG_ACTIVE_PD, INPUT_PULLUP);
  #ifdef DEBUG_ALWAYS_ON
	debugmode = 1;
  #else
	wdt_reset();
	delay(50);
	debugmode = !(*portInputRegister(digitalPinToPort(DEBUG_ACTIVE_PD)) & digitalPinToBitMask(DEBUG_ACTIVE_PD));
  #endif
	wdt_reset();
 #else
	debugmode = 1;
 #endif
 #if defined(__AVR_ATmega328PB__)
	debugmode <<= 1;
 #endif
	if(debugmode) {
		DEBUG(F("\nBMS n->1 gate to Microart, v")); DEBUGN(VERSION);
		DEBUGN(F("Copyright by Vadim Kulakov (c) 2025, vad7@yahoo.com"));
	}
#endif
	if(digitalPinToPort(BUZZER_PD1) != digitalPinToPort(BUZZER_PD2) || digitalPinToPort(BUZZER_PD1) != digitalPinToPort(BUZZER_PD3)) {
		while(1) {
			wdt_reset();
#ifdef DEBUG_TO_SERIAL
			DEBUGN(F("ERROR #define BUZZER_PD* - PINS PORTS ARE NOT THE SAME!"));
#endif
			*portOutputRegister(digitalPinToPort(LED_PD)) ^= digitalPinToBitMask(LED_PD);
			delay(500);
		}
	}
	eeprom_read_block(&work, &EEPROM.work, sizeof(EEPROM.work));
#ifdef DEBUG_BMS_SEND
	work.bms_cells_qty = 20;
	work.BMS_read_period = 5000;
	debug = 3;
#endif
	memset(bms_Q, 0, sizeof(bms_Q));
	memset(bms_cell_min, 0, sizeof(bms_cell_min));
	memset(bms_cell_max, 0, sizeof(bms_cell_max));
	memset(bms_min_cell_mV, 0, sizeof(bms_min_cell_mV));
	memset(bms_min_string, 0, sizeof(bms_min_string));
	memset(bms_max_cell_mV, 0, sizeof(bms_max_cell_mV));
	memset(bms_max_string, 0, sizeof(bms_max_string));
	memset(bms_flags, 0, sizeof(bms_flags));
	memset(last_error, 0, sizeof(last_error));
	RWARN_period = work.RWARN_Send_Period;
	Wire.begin();
	// deactivate internal pullups for twi.
	digitalWrite(SDA, 0);
	digitalWrite(SCL, 0);
#if I2C_FREQ < 30500
	TWSR |= (1<<TWPS1) | (1<<TWPS0); // Prescaler = 64
	TWBR = ((F_CPU / I2C_FREQ) - 16) / (2 * 64);
#else
	Wire.setClock(I2C_FREQ);
#endif
	Wire.onRequest(I2C_Response); // register event
	Wire.onReceive(I2C_Receive); // register event
	i2c_set_slave_addr(0);
#ifdef DEBUG_TO_SERIAL
	if(debugmode) {
		DEBUG(F("BMS: ")); DEBUG(work.bms_num); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_bms); DEBUGN(F("=X)"));
		DEBUG(F("Cells: ")); DEBUG(work.bms_cells_qty); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_cells); DEBUGN(F("=X)"));
		DEBUG(F("Watchdog(")); DEBUG(WATCHDOG_NO_CONN); DEBUG(F("s): ")); if(!work.watchdog) DEBUG(F("NONE")); else { if(work.watchdog & 1) DEBUG(F("I2C ")); if(work.watchdog & 2) DEBUG(F("BMS")); }
		DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_watchdog); DEBUGN(F("=1-I2C,2-BMS,3-all)"));
		DEBUG(F("RS485: ")); DEBUG(BMS_SERIAL_RATE); DEBUGN(F(" 8N1"));
		DEBUG(F("BMS MODBUS IDs: ")); for(uint8_t i = 1; i <= BMS_NUM_MAX; i++) { DEBUG(i); DEBUG(' '); }
		DEBUG(F("\nBMS read period, ms: "));
		if(work.BMS_read_period > 1) DEBUG(work.BMS_read_period);
		else if(work.BMS_read_period == 1) DEBUG(F("Synch I2C"));
		else DEBUG(F("OFF"));
		DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_period); DEBUGN(F("=0-off,1-synch,X ms)"));
		DEBUG(F("Wait answer from BMS, ms: ")); DEBUG(work.BMS_wait_answer_time); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_wait_answer); DEBUGN(F("=X)"));
		DEBUG(F("BMS rotate period: ")); DEBUG(work.I2C_bms_rotate_period); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_rotate); DEBUGN(F("=X)"));
		DEBUG(F("Remote warning period [D")); DEBUG(RWARN_PULSE_PIN); DEBUG(F("]: ")); DEBUG(work.RWARN_Send_Period); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_RWARN_period); DEBUGN(F("=X)"));
		//DEBUG(F("Options(bits: median,average): ")); if(bitRead(work.options, o_average)) DEBUG(F("average ")); if(bitRead(work.options, o_median)) DEBUG(F("median ")); DEBUG("\n");
		DEBUG(F("BMS cell min, mV: ")); DEBUG(work.cell_min_V); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_cell_min_V); DEBUGN(F("=NNNN)"));
		DEBUG(F("BMS cell max, mV: ")); DEBUG(work.cell_max_V); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_cell_max_V); DEBUGN(F("=NNNN)"));
		DEBUG(F("BMS cell delta max, mV: ")); DEBUG(work.cell_delta_max_V); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_cell_delta_max_V); DEBUGN(F("=NNNN)"));
		DEBUG(F("Options: ")); if(bitRead(work.options, o_average)) DEBUG(F("average ")); if(bitRead(work.options, o_equalize_cells_V)) DEBUG(F("equalize ")); DEBUGN("(options=+1-average,+2-equalize)");
		DEBUG(F("BMS voltage round: ")); DEBUG(work.round == round_true ? F("5/4") : work.round == round_cut ? F("cut") : work.round == round_up ? F("up") : F("?"));
		DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_round); DEBUGN(F("=0-5/4,1-cut,2-up)"));
		DEBUG(F("BMS voltage correct, mV: ")); DEBUG(work.V_correct); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_correct); DEBUGN(F("=X)"));
		DEBUG(F("BMS cell max catch, 10mV: ")); if(work.Vmaxhyst) { DEBUG('+'); DEBUG(work.Vmaxhyst); } else DEBUG(F("OFF"));
		DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_vmaxhyst); DEBUGN(F("=X)"));
		DEBUG(F("BMS Temp correct, C: ")); DEBUG(work.temp_correct); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_temp_correct); DEBUGN(F("=X)"));
		DEBUG(F("BMS Balans delta, mV: "));
		if(work.BalansDeltaDefault) DEBUG(work.BalansDeltaDefault); else DEBUG(F("OFF"));
		DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_delta_default); DEBUGN(F("=X)"));
		DEBUGN(F("BMS Balans delta, [MPPT(I)>=A:mV]: "));
		DEBUG(F(" Current: ")); DEBUG(work.BalansDeltaI); DEBUG(F(", Delta: ")); DEBUGN(work.BalansDelta);
		DEBUG(F("BMS Balans delta decrease pause, minutes: ")); DEBUG(work.BalansDeltaPause);  DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_delta_pause); DEBUGN(F("=X)"));
		DEBUGN(F("\nCommands:"));
		DEBUG((const __FlashStringHelper*)dbg_debug); DEBUGN(F("=0,1,2,3"));
		DEBUG((const __FlashStringHelper*)dbg_temp); DEBUGN(F("=X"));
		DEBUG((const __FlashStringHelper*)dbg_delta_change_pause); DEBUGN(F("=X"));
		DEBUG((const __FlashStringHelper*)dbg_read); DEBUGN(F("=1"));
		DEBUGN(F("Out: Vn=X (All: n=0)\nQn=X"));
		DEBUG((const __FlashStringHelper*)dbg_seterr); DEBUGN(F("=X"));
	}
#endif
#ifdef LCD_ENABLED
	lcd.begin(20, 4); // Setup: cols, rows
	lcd.print(F("BMS2 v"));
	lcd.print(VERSION);
	lcd.setCursor(0, 1);
	lcd.print(F(", 12.2025"));
	lcd.setCursor(0, 2);
	lcd.print(F("(C) Vadim Kulakov"));
	lcd.setCursor(0, 3);
	lcd.print(F("vad7@yahoo.com"));
#endif
	FlashLED(3, 1, 1);
}

void loop()
{
	static uint32_t led_flashing, bms_reading, cnt100ms;
	wdt_reset();
	RWARN_check_send();
	uint8_t _err;
	for(uint8_t i = 0; i < BMS_NUM_MAX; i++) {
		if((_err = last_error[i])) break;
	}
	uint32_t m = millis();
#ifdef LCD_ENABLED
	if(_err) {
		if(m - led_flashing >= 300UL) {
			led_flashing = m;
			*portOutputRegister(digitalPinToPort(LED_PD)) ^= digitalPinToBitMask(LED_PD);
		}
	} else *portOutputRegister(digitalPinToPort(LED_PD)) &= ~digitalPinToBitMask(LED_PD);
#else
	if(m - led_flashing >= (error_alarm_time == 0 ? 1500UL : 200UL)) {
		led_flashing = m;
		if(error_alarm_time) error_alarm_time--;
		if(debugmode) *portOutputRegister(digitalPinToPort(LED_PD)) |= digitalPinToBitMask(LED_PD);
		else *portOutputRegister(digitalPinToPort(LED_PD)) ^= digitalPinToBitMask(LED_PD);
	}
#endif
	if(!(*key1_PIN & key1_MASK)) { // pressed
		key1_delay = 150;
	} else if(key1_delay) key1_delay--;
	if(m - cnt100ms >= 100UL) { // 0.1 sec
		cnt100ms = m;
		if(key1_status == 1) {
			// to do...
			if(debugmode) DEBUGN("Key pressed");
			//
			key1_status = 0;
		}
		uint8_t d  = !(*portInputRegister(digitalPinToPort(DEBUG_ACTIVE_PD)) & digitalPinToBitMask(DEBUG_ACTIVE_PD));
#if defined(__AVR_ATmega328PB__)
		d <<= 1;
#endif
		if(d != debugmode) {
			if(*portOutputRegister(digitalPinToPort(BUZZER_PD1)) & digitalPinToBitMask(BUZZER_PD1)) {
				*portOutputRegister(digitalPinToPort(BUZZER_PD1)) &= ~(digitalPinToBitMask(BUZZER_PD1)|digitalPinToBitMask(BUZZER_PD2)|digitalPinToBitMask(BUZZER_PD3));
				beep_time = BEEP_DURATION;
			} else beep_time = 0;
			beep_cnt = 0;
			beep_num = 255;	// short beep
#ifdef LCD_ENABLED
			LCD_SCR_last = 127; // refresh screen
#endif
		}
		debugmode = d;
		if((_err && !debugmode) || beep_cnt || beep_num) {
			if(beep_time) beep_time--;
			else {
				if(beep_cnt) {
					if(--beep_cnt) {
						*portOutputRegister(digitalPinToPort(BUZZER_PD1)) ^= digitalPinToBitMask(BUZZER_PD1)|digitalPinToBitMask(BUZZER_PD2)|digitalPinToBitMask(BUZZER_PD3);
						beep_time = BEEP_DURATION;
					} else {
						*portOutputRegister(digitalPinToPort(BUZZER_PD1)) &= ~(digitalPinToBitMask(BUZZER_PD1)|digitalPinToBitMask(BUZZER_PD2)|digitalPinToBitMask(BUZZER_PD3));
						beep_time = BEEP_PAUSE;
					}
				} else {
					if(_err && _err != beep_num) beep_num = _err;
					if(beep_num) {
						if(beep_num == 255) {
							*portOutputRegister(digitalPinToPort(BUZZER_PD1)) |= digitalPinToBitMask(BUZZER_PD1)|digitalPinToBitMask(BUZZER_PD2)|digitalPinToBitMask(BUZZER_PD3);
							beep_cnt = 1;
							beep_time = 0;
						} else {
							beep_cnt = beep_num * 2;
							beep_time = BEEP_DURATION;
						}
						beep_num = 0;
					}
				}
			}
		}
	}
	if(m - sec_timer >= 1000UL) { // every 1 sec
		sec_timer = m;
		if(work.watchdog & 1) watchdog_I2C++;
		if(work.watchdog & 2) watchdog_BMS++;
		if(watchdog_I2C > WATCHDOG_NO_CONN || watchdog_BMS > WATCHDOG_NO_CONN) {
			Wire.end();
			if(debugmode) {
				DEBUG(F("* WATCHDOG: ")); DEBUG(watchdog_I2C); DEBUG(','); DEBUGN(watchdog_BMS);
				watchdog_BMS = watchdog_I2C = 0;
			} else while(1) { // reboot
				*portOutputRegister(digitalPinToPort(LED_PD)) ^= digitalPinToBitMask(LED_PD);
				_delay_ms(100);
			}
		}
		if(delta_change_pause < 0xFFFF) delta_change_pause++;
		if(RWARN_period) RWARN_period--;
#ifdef DEBUG_TO_SERIAL
		DebugSerial_read();
#endif
#ifdef LCD_ENABLED
		if(LCD_refresh_sec) LCD_refresh_sec--;
		if(LCD_refresh_sec == 0 && RWARN_bit == 0 && RWARN_idx == 0) {


			uint32_t m = millis();

			LCD_Display();

			uint32_t m2 = millis();

			DEBUG(F("D#")); DEBUGN(m2 - m);


			LCD_refresh_sec = 1;
		}
#endif
	}

	if(bms_idx_prev != bms_idx) {
		if(bms_idx == 0) {
			flags = f_BMS_Need_Read;
#ifdef DEBUG_TO_SERIAL
			if(debugmode && debug >= 2) {
				DEBUG(F("I2C ms: "));
				DEBUGN(m - bms_loop_time);
				bms_loop_time = m;
			}
#endif
		}
#ifdef DEBUG_TO_SERIAL
		if(debugmode && debug >= 4) {
			DEBUG(F("I2C_R_"));
			DEBUG(bms_idx_prev + 1);
			DEBUG(F("->"));
			DEBUGN(bms[sending_bms][bms_idx_prev]);
		}
#endif
		bms_idx_prev = bms_idx;
	}

	// Read from UART
	m = millis();
	if(work.BMS_read_period > 1 && m - bms_reading > work.BMS_read_period) {
		bms_reading = m;
		bitSet(flags, f_BMS_Need_Read);
	}
	if(debugmode != 1) {
		BMS_Serial_read();
		if(m - bms_last_read_time > work.BMS_wait_answer_time) {
			if(bitRead(flags, f_BMS_Wait_Answer)) {
				if(!bitRead(flags, f_BMS_ReadOk)) {
					if(debugmode) {	DEBUG(F("BMS not answer: ")); DEBUGN(read_bms_num + 1); }
					Set_New_Error(ERR_BMS_NotAnswer);
				}
				if(++read_bms_num == work.bms_num) {
					read_bms_num = 0;
					bms_reading = m;
					bitClear(flags, f_BMS_Wait_Answer);
				}
			}
			if(bitRead(flags, f_BMS_Need_Read) || bitRead(flags, f_BMS_Wait_Answer)) {
				read_idx = 0;	// reset read index
				if(debug == 3) { DEBUG(F("Send to BMS ")); DEBUGN(read_bms_num + 1); }
				uint8_t crc = BMS_send_pgm_cmd(&BMS_Cmd_Head[0], sizeof(BMS_Cmd_Head), 0);
				BMS_SERIAL.write(read_bms_num + 1);
				crc += read_bms_num + 1;
				if(delta_new_cnt) {
					if(debugmode) {	DEBUGN(F(" Send Delta")); }
					uint8_t b;
					crc += BMS_send_pgm_cmd(&BMS_Cmd_ChangeDelta[0], sizeof(BMS_Cmd_ChangeDelta), 0);
					b = delta_new >> 8;
					BMS_SERIAL.write(b);
					crc += b;
					b = delta_new & 0xFF;
					BMS_SERIAL.write(b);
					crc += b;
					if(--delta_new_cnt == 0)	{
						delta_active = delta_new;
						delta_new = 0;
						delta_change_pause = 0;
					}
				} else {
					crc += BMS_send_pgm_cmd(&BMS_Cmd_Request[0], sizeof(BMS_Cmd_Request), crc);
				}
				BMS_SERIAL.write(crc);
				bms_last_read_time = m;
				if(read_bms_num == 0) bitClear(flags, f_BMS_Need_Read);
				bitClear(flags, f_BMS_ReadOk);
				bitSet(flags, f_BMS_Wait_Answer);
			}
		}
	}
	// I2C slave receive
	if(i2c_receive_idx && i2c_receive_idx > i2c_receive[0]) { // i2c write
		if(debugmode) DEBUGIF(4,F("I2C_W: "));
		if(i2c_receive[0] >= sizeof(i2c_receive)) {
			if(debugmode) DEBUGIFN(1,F("I2C_W: LEN!"));
			i2c_receive_idx = 0;
		} else {
			uint8_t crc = 0;
			for(uint8_t i = 0; i <= i2c_receive[0]; i++) { // +CRC
				crc += i2c_receive[i];
				if(debugmode) {
					DEBUGIF(4,i2c_receive[i]);
					DEBUGIF(4," ");
				}
			}
			if(crc != 0) {
				BLINK_ALARM;
				if(debugmode) DEBUGIFN(0,F("- CRC ERROR!"));
			} else if(i2c_receive[1] == 4) { // Broadcast I2CCom_JobWR
				map_cell_min = i2c_receive[2] + 200;
				map_cell_full = i2c_receive[3] + 200;
				map_mode = i2c_receive[4];
				if(debugmode && (debug >= 2 || bitRead(debug_info, 0))) {
					bitClear(debug_info, 0);
					DEBUG(F("I2C_W: Min=")); DEBUG(map_cell_min); DEBUG(F(",Max=")); DEBUG(map_cell_full); DEBUG(F(",Mode=")); DEBUGN(map_mode);
				}
				if(work.BMS_read_period == 1) bitSet(flags, f_BMS_Need_Read);
			} else if(i2c_receive[1] == 6) { // Broadcast I2CCom_JobWR_MPPT
//				uint8_t A = i2c_receive[8];
//				if(debug == 2 || bitRead(debug_info, 1)) {
//					bitClear(debug_info, 1);
//					DEBUG(F("I2C_W: I=")); DEBUGN(A);
//				}
				if(work.BalansDeltaDefault && !delta_new && delta_active) {
					if(map_mode == M_ON) { // on battery
						if(delta_change_pause > BMS_CHANGE_DELTA_DISCHARGE) {
							if(delta_active != work.BalansDelta) { // last (max delta)
								delta_new = work.BalansDelta;
								delta_change_pause = 0;
								delta_change_equalizer = 0;
							}
						}
					} else if(delta_change_pause > BMS_CHANGE_DELTA_PAUSE_MIN) {
						uint8_t A = i2c_receive[8];
						uint16_t d = A >= work.BalansDeltaI ? work.BalansDelta : work.BalansDeltaDefault;
						if(d == delta_active || (d < delta_active && delta_change_pause <= work.BalansDeltaPause)) d = 0;
						if(d && d == delta_next) {
							if(++delta_change_equalizer > BMS_CHANGE_DELTA_EQUALIZER) {
								delta_new = d;
								delta_change_equalizer = 0;
							}
						} else {
							delta_next = d;
							delta_change_equalizer = 0;
						}
						if(delta_new) {
							if(debugmode && debug >= 1) { DEBUG(F("D_NEW: ")); DEBUGN(delta_new); }
							bitSet(flags, f_BMS_Need_Read);
							delta_new_cnt = work.bms_num;
							delta_change_pause = 0;
						}
					}
				}
			}
			if(i2c_receive_idx > i2c_receive[0] + 1) {
				memcpy(i2c_receive, i2c_receive + i2c_receive[0] + 1, i2c_receive_idx -= i2c_receive[0] + 1);
			} else i2c_receive_idx = 0;
			watchdog_I2C = 0;
		}
		if(debugmode) DEBUGIF(4,F("\n"));
	}
	//delay(MAIN_LOOP_PERIOD); // not less than, actually +8..16us
}
