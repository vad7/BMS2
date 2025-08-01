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

Connections:
Two active balancers JK-DZ11B2A24S RS485 are supported.
UART0 (TX,RX) - Debug, setup port
UART1 (D11,D12) - BMS 1/2
pin D2 - Debug to UART0 (TX,RX) - connect pin D2 to GND before power on (115200 bps)
pin D22+D19 -> resistor 0..50 Om -> Buzzer [-] (30 mA MAX)
pin D3 - Buzzer mute key (to GND)

Microart connector RJ-11 (6P6C):
1 - BMS_DISCHARGE / I2C_SLC_buf_iso (brown-white)
2 - GND								(brown)
3 - TEPM							(blue-white)
4 - BMS_CHARGE / I2C_SDA_buf_iso	(blue)
5 - +5v_iso							(orange)
6 - not connected on back side, on front side: +12V if enabled (orange-white)


*/

#define VERSION F("2.0")

#include "Arduino.h"
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <Wire.h>
extern "C" {
	#include "utility/twi.h"
}

#define BUZZER_PD1					19
#define BUZZER_PD2					22
#define BUZZER_MUTE_PD				3
#define DEBUG_ACTIVE_PD				2		// activate - connect pin to GND before power on.
#define LED_PD						LED_BUILTIN

#define I2C_FREQ					2500
#define BMS_NUM_MAX					3		// max number of connected BMS
#define BMS_CELLS_QTY_MAX			24
#define MAIN_LOOP_PERIOD			1		// msec
#define BMS_NO_TEMP					255
#define WATCHDOG_NO_CONN			30UL	// sec
#define BMS_MIN_PAUSE_BETWEEN_READS	15UL	// msec
#define BMS_CHANGE_DELTA_PAUSE_MIN  1800	// sec
#define BMS_CHANGE_DELTA_EQUALIZER  60		// attempts (* ~1 sec)
#define BMS_CHANGE_DELTA_DISCHARGE  30		// sec, When I2C_MAP_MODE = M_ON balance delta => BalansDelta[max]
#define BEEP_DURATION				5		// *0.1 sec
const uint8_t BMS_Cmd_Head[] PROGMEM = { 0x55, 0xAA };
const uint8_t BMS_Cmd_Request[] PROGMEM = { 0xFF, 0x00, 0x00 };
const uint8_t BMS_Cmd_ChangeDelta[] PROGMEM = { 0xF2 };

#if defined(__AVR_ATmega328PB__)
#define BMS_SERIAL 					Serial1	// First BMS
#else
#define BMS_SERIAL 					Serial	// First BMS
#endif
#define BMS_SERIAL_RATE				9600
#define DEBUG_TO_SERIAL				// use serial for debug or setup
#define DEBUG_TO_SERIAL_RATE		115200
#define DEBUG_READ_PERIOD			1000UL	// ms
#ifdef DEBUG_TO_SERIAL
#define DebugSerial 				Serial
#endif

#ifdef DEBUG_TO_SERIAL
#define DEBUG(s) DebugSerial.print(s)
#define DEBUGH(s) DebugSerial.print(s, 16)
#define DEBUGN(s) DebugSerial.println(s)
#define DEBUGIF(d,s) { if(debug >= d) DebugSerial.print(s); }
#define DEBUGIFN(d,s) { if(debug >= d) DebugSerial.println(s); }
char	debug_read_buffer[64];
uint8_t debug_read_idx = 0;
const char dbg_debug[] PROGMEM = "dbg";
const char dbg_bms[] PROGMEM = "bms";
const char dbg_cells[] PROGMEM = "cells";
const char dbg_period[] PROGMEM = "period";
const char dbg_rotate[] PROGMEM = "rotate";
const char dbg_round[] PROGMEM = "Vround";
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
#else
#define DEBUG(s)
#define DEBUGN(s)
#define DEBUGIF(d,s)
#define DEBUGIFN(d,s)
#endif

enum {
	round_true = 0,
	round_cut,
	round_up
};

enum { // options bits
	o_average = 0,
	o_median
};

enum I2C_MAP_MODE {
	M_OFF        = 0,//выключенно без сети
	M_OFFNET     = 1,//выключенно с сетью
	M_ON         = 2,//включенно без сети (работает МАП)
	M_ONNET      = 3,//включенно с сетью
	M_ONCHARGE   = 4,//включенно с зарядом
};

struct WORK {
	uint8_t  bms_num;
	uint8_t  bms_cells_qty;
	uint8_t  options;
	uint32_t UART_read_period;		// ms, 1 - synch i2C
	uint8_t  I2C_bms_rotate_period; // times, period (number of reading) before switch to next bms
	uint8_t  round;					// round_*
	int16_t  V_correct;				// mV
	int8_t   temp_correct;			// mV
	uint8_t  watchdog;				// reboot if no data over: 1 - I2C, 2 - BMS
	int16_t  Vmaxhyst;				// *10mV
	uint16_t BalansDeltaPause;		// minutes wait for change to default delta
	uint16_t BalansDeltaDefault;	// Default balans delta voltage, mV
	uint16_t BalansDelta;			// mV
	uint8_t  BalansDeltaI;			// Current threshold
} work;

struct _EEPROM {
	WORK    work;
} __attribute__ ((packed));

struct _EEPROM EEMEM EEPROM;

enum { // last_error =
	ERR_BMS_NotAnswer = 1,
	ERR_BMS_Read = 2,
	ERR_BMS_Config = 3,
	ERR_BMS_Hardware = 4
};
enum {
	f_BMS_Ready = 0,
	f_BMS_ReadOk,
	f_MUTE
};
uint8_t  flags = 0;					// f_*
int8_t   debug = 0;					// 0 - off, 1 - on, 2 - detailed dump, 3 - full dump, 4 - BMS full
uint8_t  debugmode = 0;				// 0 - off, 1 - debug to the same port as BMS_SERIAL
uint16_t bms[BMS_NUM_MAX][BMS_CELLS_QTY_MAX];	// *10mV
uint16_t bms_avg[BMS_NUM_MAX][BMS_CELLS_QTY_MAX];
uint8_t  bms_select = 0;
uint8_t  bms_Q[BMS_CELLS_QTY_MAX];// %
uint8_t  bms_idx = 0;
uint8_t  bms_idx_prev = 0;
uint32_t bms_loop_time;
uint32_t watchdog_timer = 0;
uint8_t  watchdog_I2C = 0;
uint8_t  watchdog_BMS = 0;
bool     bms_need_read = true;
uint8_t  i2c_send_bms_num = 0;
uint8_t  I2C_bms_send_switch_timer = 0;
int16_t  map_cell_min = 0;			// *10mV (1 cell)
int16_t  map_cell_full = 0;			// *10mV (1 cell)
int16_t  bms_cell_max[BMS_NUM_MAX];		// *10mV
int16_t  bms_cell_min[BMS_NUM_MAX];		// *10mV
uint8_t  selected_bms = 0;
uint8_t  sending_bms = 0;
uint8_t  map_mode = 0;
uint8_t  temp = BMS_NO_TEMP;		// C, +50
uint8_t  crc;
uint8_t  last_error = 0;
uint8_t  error_alarm_time = 0;
uint8_t  read_buffer[74];
uint8_t  read_idx = 0;
uint8_t  i2c_receive[32];
uint8_t  i2c_receive_idx = 0;
uint32_t bms_last_read_time = 0;
uint16_t delta_active = 0;			// mV
uint16_t delta_new = 0;				// mV
uint16_t delta_next = 0;
uint16_t delta_change_pause = 0;  	// sec
uint8_t  delta_change_equalizer = 0; // attempts
uint8_t  debug_info	= 0b0011;		// b0 - I2C_W1, b1 - I2C_W2
uint8_t  read_bms_num = 0;		    // last bms # read
uint8_t  beep_num = 0;
uint8_t  beep_cnt = 0;
uint8_t  beep_time = 0;

// Called in delay()
void yield(void)
{
	sleep_cpu();
	wdt_reset();
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
	i2c_write(bms_idx == 0 ? temp : BMS_NO_TEMP);	// temp + 50, 255 - none
	i2c_write(bms_Q[bms_idx]);						// Q_Cell, %, I=(Q_Cell/100)*(Ucell/R), R=1
	i2c_write(bms_idx == 0 ? last_error : 0);		// prev err
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
				work.UART_read_period = d;
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
				last_error = d;
				DEBUG(d);
			} else if(strncmp_P(debug_read_buffer, dbg_delta_change_pause, sizeof(dbg_delta_change_pause)-1) == 0) {
				delta_change_pause = d;
				DEBUG(d);
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

// JK-DZ11-B2A24S active balancer
void BMS_Serial_read(void)
{
	while(BMS_SERIAL.available()) {
		int16_t r = BMS_SERIAL.read();
		bms_last_read_time = millis();
		if(r == -1) break;
		read_buffer[read_idx++] = r;
		if(read_idx == sizeof(read_buffer)) {
			last_error = 0;
			read_idx = 0;
			if(read_buffer[0] != 0xEB || read_buffer[1] != 0x90) {
				DEBUGIFN(1,F("BMS: Header mismatch!"));
				last_error = ERR_BMS_Read;
				error_alarm_time = 50;
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
				last_error = ERR_BMS_Read;
				error_alarm_time = 50;
				break;
			}
			if(read_buffer[3] == 0xF2) { // Change delta voltage
				delta_active = read_buffer[4]*256 + read_buffer[5];
				if(debug) {
					DEBUG(F("New delta: ")); DEBUGN(delta_active);
				}
				delta_change_pause = 0;
				watchdog_BMS = 0;
				bms_last_read_time = millis();
			} else if(read_buffer[3] == 0xFF) { // Request answer
				if(read_buffer[12] & 0x03) {
					DEBUGIF(1,F("BMS"));
					DEBUGIF(1, read_bms_num + 1);
					DEBUGIF(1,F(" Alarm: "));
					error_alarm_time = 50;
					if(read_buffer[12] & (1<<0)) { // cells num wrong
						last_error = ERR_BMS_Config;
						DEBUGIF(1,F("Cells_Num "));
					}
					if(read_buffer[12] & (1<<1)) { // wire resistance is too large
						last_error = ERR_BMS_Hardware;
						DEBUGIF(1,F("Wire_Resistance "));
					}
	//				if(read_buffer[12] & (1<<2)) { // battery overvoltage
	//					last_error = ERR_BMS_Hardware;
	//					DEBUG(F("Overvoltage"));
	//				}
					DEBUGIF(1,'\n');
				}
	#ifdef DEBUG_TO_SERIAL
				if(debug == 3) {
					DEBUG(F("BMS ")); DEBUG(read_bms_num + 1);	DEBUG(' ');
					if(read_buffer[21]) DEBUG(F("ON")); else DEBUG(F("OFF"));
					uint16_t n = read_buffer[4]*256 + read_buffer[5]; // Total V
					DEBUG(F(",V:")); DEBUG(n / 100); DEBUG('.'); n %= 100; if(n < 10) DEBUG('0'); DEBUG(n);
					DEBUG(F(",D(mV):")); DEBUG(read_buffer[13]*256 + read_buffer[14]);
					DEBUG(F(",Trg(mV):")); DEBUG(read_buffer[17]*256 + read_buffer[18]);
					DEBUG(F(",Bal(mA):")); DEBUG(read_buffer[15]*256 + read_buffer[16]);
					//DEBUG(F(",T(C):")); DEBUG(read_buffer[72]);
					//DEBUG(F(",W:")); DEBUG(read_buffer[8]); DEBUG(','); DEBUG(read_buffer[9]); DEBUG(','); DEBUG(read_buffer[10]); DEBUG(','); DEBUG(read_buffer[11]);
					DEBUG(F("\n"));
				}
	#endif
				if(read_buffer[8] != work.bms_cells_qty) {
					DEBUGIF(1,F("BMS"));  DEBUGIF(1, read_bms_num + 1);
					DEBUGIFN(1,F(" Cells num not equal setup!"));
					last_error = ERR_BMS_Config;
					error_alarm_time = 50;
					if(read_buffer[8] > BMS_CELLS_QTY_MAX) break;
				}
				int16_t _max = 0;
				int16_t _min = 32767;
				for(uint8_t i = 0; i < read_buffer[8]; i++) {
					if(i > work.bms_cells_qty - 1) {
						ATOMIC_BLOCK(ATOMIC_FORCEON) bms[read_bms_num][i] = 0;
						continue;
					}
					int16_t v = read_buffer[23 + i*2]*256 + read_buffer[24 + i*2];
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
					ATOMIC_BLOCK(ATOMIC_FORCEON) bms[read_bms_num][i] = v;
					if(v > _max) _max = v;
					if(v < _min) _min = v;
				}
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
					bms_cell_max[read_bms_num] = _max;
					bms_cell_min[read_bms_num] = _min;
				}
				uint8_t _sel = 0;
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
					for(uint8_t i = 0; i < work.bms_num; i++) {
						if(bms_cell_max[i] >= map_cell_full && bms_cell_max[i] > _max) {
							_max = bms_cell_max[i];
							_sel = i;
						}
					}
					if(_max == 0) {
						if(I2C_bms_send_switch_timer) I2C_bms_send_switch_timer--;
						else {
							I2C_bms_send_switch_timer = work.I2C_bms_rotate_period;
							if(++i2c_send_bms_num == work.bms_num) i2c_send_bms_num = 0;
						}
					}
				}
				selected_bms = _sel;
				temp = read_buffer[72] + 50 + work.temp_correct;
				delta_active = read_buffer[17]*256 + read_buffer[18];
				if(read_bms_num == 0) memset(bms_Q, 0, sizeof(bms_Q));
				uint8_t i = read_buffer[9];
				if(i < work.bms_cells_qty && read_buffer[11]) bms_Q[i] = 100UL * (read_buffer[15]*256 + read_buffer[16]) / (read_buffer[23 + i*2]*256 + read_buffer[24 + i*2]); // Q_Cell=100*I/(Ucell/R), R=1
				if(!bitRead(flags, f_BMS_Ready)) {
					i2c_set_slave_addr(bms_idx + 1);
					bitSet(flags, f_BMS_Ready);
					bms_loop_time = millis();
				}
				bitSet(flags, f_BMS_ReadOk);
				watchdog_BMS = 0;
			} else if(debug) {
				DEBUG(F("BMS"));
				DEBUG(read_bms_num + 1);
				DEBUG(F(": Wrong response code: ")); DEBUGN(read_buffer[3]);
			}
			break;
		}
	}
}

void setup()
{
	wdt_enable(WDTO_2S); // Enable WDT
	sleep_enable();
	PRR0 = (1<<PRSPI0) | (1<<PRADC); // Power off: SPIs, ADC, PTC
	PRR1 = (1<<PRPTC) | (1<<PRSPI1);
	BMS_SERIAL.begin(BMS_SERIAL_RATE);
	pinMode(BUZZER_PD1, OUTPUT);
	pinMode(BUZZER_PD2, OUTPUT);
	pinMode(BUZZER_MUTE_PD, INPUT_PULLUP);
#ifdef DEBUG_TO_SERIAL
	DebugSerial.begin(DEBUG_TO_SERIAL_RATE);
#ifdef DEBUG_ACTIVE_PD
	pinMode(DEBUG_ACTIVE_PD, INPUT_PULLUP);
	wdt_reset();
	delay(50);
	debugmode = !(*portInputRegister(digitalPinToPort(DEBUG_ACTIVE_PD)) & digitalPinToBitMask(DEBUG_ACTIVE_PD));
	wdt_reset();
#else
	debugmode = 2;
#endif
	if(debugmode) {
		DEBUG(F("\nBMS 2->1 gate to Microart, v")); DEBUGN(VERSION);
		DEBUGN(F("Copyright by Vadim Kulakov (c) 2025, vad7@yahoo.com"));
	}
#endif
	uint8_t b = eeprom_read_byte((uint8_t*)&EEPROM.work.bms_cells_qty);
	if(b == 0 || b > 32) { // init EEPROM
		memset(&work, 0, sizeof(work));
		work.bms_num = 2;
		work.bms_cells_qty = 16;
		work.options = 0; //(1<<o_average);
		work.UART_read_period = 1;
		work.I2C_bms_rotate_period = 5;
		work.round = round_true;
		work.Vmaxhyst = 1;
		work.temp_correct = 6;
		work.BalansDeltaDefault = 5;
		work.BalansDeltaPause = 17*60; // min.
		work.BalansDeltaI = 2;
		work.BalansDelta = 20;
		work.watchdog = 3;
		eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
	}
	eeprom_read_block(&work, &EEPROM.work, sizeof(EEPROM.work));
	memset(bms_Q, 0, sizeof(bms_Q));
	memset(bms_cell_min, 0, sizeof(bms_cell_min));
	memset(bms_cell_max, 0, sizeof(bms_cell_max));
	I2C_bms_send_switch_timer = work.I2C_bms_rotate_period;
#ifdef DEBUG_TO_SERIAL
	if(debugmode) {
		DEBUG(F("BMS: ")); DEBUG(work.bms_num); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_bms); DEBUGN(F("=X)"));
		DEBUG(F("Cells: ")); DEBUG(work.bms_cells_qty); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_cells); DEBUGN(F("=X)"));
		DEBUG(F("Watchdog(")); DEBUG(WATCHDOG_NO_CONN); DEBUG(F("s): ")); if(!work.watchdog) DEBUG(F("NONE")); else { if(work.watchdog & 1) DEBUG(F("I2C ")); if(work.watchdog & 2) DEBUG(F("BMS")); }
		DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_watchdog); DEBUGN(F("=1-I2C,2-BMS,3-all)"));
		DEBUG(F("RS485: ")); DEBUG(BMS_SERIAL_RATE); DEBUGN(F(" 8N1"));
		DEBUG(F("BMS IDs: ")); for(uint8_t i = 0; i < BMS_NUM_MAX; i++) { DEBUG(i); DEBUG(' '); }
		DEBUG(F("\nBMS read period, ms: "));
		if(work.UART_read_period > 1) DEBUG(work.UART_read_period);
		else if(work.UART_read_period == 1) DEBUG(F("Synch I2C"));
		else DEBUG(F("OFF"));
		DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_period); DEBUGN(F("=0-off,1-synch,X ms)"));
		DEBUG(F("\nBMS rotate period: ")); DEBUG(work.I2C_bms_rotate_period); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_rotate); DEBUGN(F("=X)"));
		//DEBUG(F("Options(bits: median,average): ")); if(bitRead(work.options, o_average)) DEBUG(F("average ")); if(bitRead(work.options, o_median)) DEBUG(F("median ")); DEBUG("\n");
		DEBUG(F("Options(bits: average): ")); if(bitRead(work.options, o_average)) DEBUG(F("average ")); if(bitRead(work.options, o_median)) DEBUG(F("median ")); DEBUG("\n");
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
		DEBUGN(F("Out: Vn=X (All: n=0)\nQn=X"));
		DEBUG((const __FlashStringHelper*)dbg_seterr); DEBUGN(F("=X"));
	}
#endif
	//
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
	pinMode(LED_PD, OUTPUT);
	FlashLED(4, 1, 1);
}

void loop()
{
	wdt_reset(); sleep_cpu();
	static uint32_t led_flashing, bms_reading, beeping;
	uint32_t m = millis();
	if(m - watchdog_timer >= 1000UL) { // every 1 sec
		watchdog_timer = m;
		if(work.watchdog & 1) watchdog_I2C++;
		if(work.watchdog & 2) watchdog_BMS++;
		if(watchdog_I2C > WATCHDOG_NO_CONN || watchdog_BMS > WATCHDOG_NO_CONN) {
			Wire.end();
			if(debugmode) {
				DEBUG(F("* WATCHDOG: ")); DEBUG(watchdog_I2C); DEBUG(','); DEBUGN(watchdog_BMS);
			}
			if(debugmode == 1) {
				watchdog_BMS = watchdog_I2C = 0;
			} else while(1) { // reboot
					sleep_cpu();
					*portOutputRegister(digitalPinToPort(LED_PD)) ^= digitalPinToBitMask(LED_PD);
					_delay_ms(100);
				}
		}
		if(delta_change_pause < 0xFFFF) delta_change_pause++;
#ifdef DEBUG_TO_SERIAL
		DebugSerial_read();
#endif
	}
	m = millis();
	if(m - led_flashing >= (error_alarm_time == 0 ? 1500UL : 200UL)) {
		led_flashing = m;
		if(error_alarm_time) error_alarm_time--;
		if(debugmode == 1) *portOutputRegister(digitalPinToPort(LED_PD)) |= digitalPinToBitMask(LED_PD);
		else *portOutputRegister(digitalPinToPort(LED_PD)) ^= digitalPinToBitMask(LED_PD);
	}
	if(m - beeping >= 100UL) {
		beeping = m;
		if(last_error || beep_cnt || beep_num) {
			if(beep_time) beep_time--;
			else {
				if(beep_cnt) {
					*portOutputRegister(digitalPinToPort(BUZZER_PD1)) ^= digitalPinToBitMask(BUZZER_PD1);
					*portOutputRegister(digitalPinToPort(BUZZER_PD2)) ^= digitalPinToBitMask(BUZZER_PD2);
					beep_cnt--;
					beep_time = BEEP_DURATION;
				} else if(!bitRead(flags, f_MUTE)) {
					if(last_error && last_error != beep_num) beep_num = last_error;
					if(beep_num) beep_cnt = beep_num * 2;
					beep_time = BEEP_DURATION;
				}
			}
		}
		if(bitRead(flags, f_MUTE) || beep_num) {
			uint8_t d = !(*portInputRegister(digitalPinToPort(BUZZER_MUTE_PD)) & digitalPinToBitMask(BUZZER_MUTE_PD));
			if(d) {
				if(bitRead(flags, f_MUTE)) {
					bitClear(flags, f_MUTE); // beeper mute off
					*portOutputRegister(digitalPinToPort(BUZZER_PD1)) |= digitalPinToBitMask(BUZZER_PD1);
					*portOutputRegister(digitalPinToPort(BUZZER_PD2)) |= digitalPinToBitMask(BUZZER_PD2);
					beep_cnt = 1;
				} else {
					bitSet(flags, f_MUTE);
					*portOutputRegister(digitalPinToPort(BUZZER_PD1)) &= ~digitalPinToBitMask(BUZZER_PD1);
					*portOutputRegister(digitalPinToPort(BUZZER_PD2)) &= ~digitalPinToBitMask(BUZZER_PD2);
					beep_cnt = 0;
				}
				beeping += 1000UL; // pause
				beep_num = 0;
				beep_time = 0;
			}
		}
	}

	if(bms_idx_prev != bms_idx) {
		if(bms_idx == 0) {
			bms_need_read = true;
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
	if(debugmode != 1) {
		BMS_Serial_read();
		m = millis();
		if(m - bms_last_read_time > BMS_MIN_PAUSE_BETWEEN_READS) {
			if((work.UART_read_period == 1 && bms_need_read) || (work.UART_read_period > 1 && m - bms_reading > work.UART_read_period)) {
				if(!bitRead(flags, f_BMS_ReadOk)) last_error = ERR_BMS_NotAnswer;
				read_idx = 0;	// reset read index
				uint8_t crc = BMS_send_pgm_cmd(&BMS_Cmd_Head[0], sizeof(BMS_Cmd_Head), 0);
				BMS_SERIAL.write(read_bms_num + 1);
				crc += read_bms_num + 1;
				crc = BMS_send_pgm_cmd(&BMS_Cmd_Request[0], sizeof(BMS_Cmd_Request), crc);
				BMS_SERIAL.write(crc);
				if(++read_bms_num == work.bms_num) {
					read_bms_num = 0;
					bms_reading = m;
					bms_need_read = false;
				}
				bms_last_read_time = m;
				bitClear(flags, f_BMS_ReadOk);
			} else if(delta_new) {
				uint8_t b;
				read_idx = 0;	// reset read index
				uint8_t crc = BMS_send_pgm_cmd(&BMS_Cmd_ChangeDelta[0], sizeof(BMS_Cmd_ChangeDelta), 0);
				b = delta_new >> 8;
				BMS_SERIAL.write(b);
				crc += b;
				b = delta_new & 0xFF;
				BMS_SERIAL.write(b);
				crc += b;
				BMS_SERIAL.write(crc);
				delta_new = 0;
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
				error_alarm_time = 50;
				if(debugmode) DEBUGIFN(0,F("- CRC ERROR!"));
			} else if(i2c_receive[1] == 4) { // Broadcast I2CCom_JobWR
				map_cell_min = i2c_receive[2] + 200;
				map_cell_full = i2c_receive[3] + 200;
				map_mode = i2c_receive[4];
				if(debugmode && (debug == 2 || bitRead(debug_info, 0))) {
					bitClear(debug_info, 0);
					DEBUG(F("I2C_W: Min=")); DEBUG(map_cell_min); DEBUG(F(",Max=")); DEBUG(map_cell_full); DEBUG(F(",Mode=")); DEBUGN(map_mode);
				}
				if(work.UART_read_period == 1 && bms[0] == 0) bms_need_read = true;
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
	delay(MAIN_LOOP_PERIOD);
}
