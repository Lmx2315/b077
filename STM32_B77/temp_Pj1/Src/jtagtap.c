/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the low-level JTAG TAP interface.  */

#include <stdio.h>
#include <stdint.h>
#include "jtagtap.h"
#include "main.h"

#define TRST_PORT 1
#define TRST_PIN  1

#define TMS_PORT  2
#define TMS_PIN   1

#define TDI_PORT  3
#define TDI_PIN   1

#define TDO_PORT  4
#define TDO_PIN   1

#define TCK_PORT  5
#define TCK_PIN   1

jtag_proc_t jtag_proc;//в этой структуре храним реализации команд TMS,TDO,TDI,TCK

static void jtagtap_reset(void);
static void jtagtap_tms_seq(uint32_t MS, int ticks);
static void jtagtap_tdi_tdo_seq(
	uint8_t *DO, const uint8_t final_tms, const uint8_t *DI, int ticks);
static void jtagtap_tdi_seq(
	const uint8_t final_tms, const uint8_t *DI, int ticks);
static uint8_t jtagtap_next(uint8_t dTMS, uint8_t dTDI);

//----------------------------------

#define gpio_set_val(port, pin, val) do {	\
	if(val)					\
		gpio_set((port), (pin));	\
	else					\
		gpio_clear((port), (pin));	\
} while(0)

void gpio_set(uint32_t gpioport, uint16_t gpios)
{
	if (gpioport==TRST_PORT) {};
	if (gpioport==TMS_PORT ) {TMS(1);};
	if (gpioport==TDI_PORT ) {TDI(1);};
	if (gpioport==TCK_PORT ) {TCK(1);};
}

void gpio_clear(uint32_t gpioport, uint16_t gpios)
{
	if (gpioport==TRST_PORT) {};
	if (gpioport==TMS_PORT ) {TMS(0);};
	if (gpioport==TDI_PORT ) {TDI(0);};
	if (gpioport==TCK_PORT ) {TCK(0);};
}

u8 gpio_get(uint32_t gpioport, uint16_t gpios)
{
//	Delay(1);
	return TDO();
}

//----------------------------------



int jtagtap_init()  //эта функция привязывает к структуре jtag_proc (отвечает за реализацию сигналов TMS,TDO,TDI,TCK) реальные ножки микроконтроллера !!!
{
//	TMS_SET_MODE(); не надо

	jtag_proc.jtagtap_reset = jtagtap_reset;
	jtag_proc.jtagtap_next =jtagtap_next;
	jtag_proc.jtagtap_tms_seq = jtagtap_tms_seq;
	jtag_proc.jtagtap_tdi_tdo_seq = jtagtap_tdi_tdo_seq;
	jtag_proc.jtagtap_tdi_seq = jtagtap_tdi_seq;

	/* Go to JTAG mode for SWJ-DP */
//	for(int i = 0; i <= 50; i++) jtagtap_next(1, 0); /* Reset SW-DP */
//	jtagtap_tms_seq(0xE73C, 16);		/* SWD to JTAG sequence */
	jtagtap_soft_reset();

	return 0;
}

static void jtagtap_reset(void)
{
#ifdef TRST_PORT

#endif
	jtagtap_soft_reset();
}

static uint8_t jtagtap_next(uint8_t dTMS, uint8_t dTDI)
{
	uint16_t ret;

	gpio_set_val(TMS_PORT, TMS_PIN, dTMS);
	gpio_set_val(TDI_PORT, TDI_PIN, dTDI);
	gpio_set    (TCK_PORT, TCK_PIN);
	ret = gpio_get(TDO_PORT, TDO_PIN);
	gpio_clear(TCK_PORT, TCK_PIN);

	//DEBUG("jtagtap_next(TMS = %d, TDI = %d) = %d\n", dTMS, dTDI, ret);

	return ret != 0;
}

static void jtagtap_tms_seq(uint32_t MS, int ticks)
{
	gpio_set_val(TDI_PORT, TDI_PIN, 1);
	int data = MS & 1;
	while(ticks) {
		gpio_set_val(TMS_PORT, TMS_PIN, data);
		gpio_set(TCK_PORT, TCK_PIN);
		MS >>= 1;
		data = MS & 1;
		ticks--;
		gpio_clear(TCK_PORT, TCK_PIN);
	}
}

static void jtagtap_tdi_tdo_seq(
	uint8_t *DO, const uint8_t final_tms, const uint8_t *DI, int ticks)
{
	uint8_t index = 1;
	gpio_set_val(TMS_PORT, TMS_PIN, 0);
	uint8_t res = 0;
	while(ticks > 1) {
		gpio_set_val(TDI_PORT, TDI_PIN, *DI & index);
		gpio_set(TCK_PORT, TCK_PIN);
		if (gpio_get(TDO_PORT, TDO_PIN)) {
			res |= index;
		}
		if(!(index <<= 1)) {
			*DO = res;
			res = 0;
			index = 1;
			DI++; DO++;
		}
		ticks--;
		gpio_clear(TCK_PORT, TCK_PIN);
	}
	gpio_set_val(TMS_PORT, TMS_PIN, final_tms);
	gpio_set_val(TDI_PORT, TDI_PIN, *DI & index);
	gpio_set(TCK_PORT, TCK_PIN);
	if (gpio_get(TDO_PORT, TDO_PIN)) {
		res |= index;
	}
	*DO = res;
	gpio_clear(TCK_PORT, TCK_PIN);
}

static void jtagtap_tdi_seq(const uint8_t final_tms, const uint8_t *DI, int ticks)
{
	uint8_t index = 1;
	while(ticks--) {
		gpio_set_val(TMS_PORT, TMS_PIN, ticks? 0 : final_tms);
		gpio_set_val(TDI_PORT, TDI_PIN, *DI & index);
		gpio_set(TCK_PORT, TCK_PIN);
		if(!(index <<= 1)) {
			index = 1;
			DI++;
		}
		gpio_clear(TCK_PORT, TCK_PIN);
	}
}
