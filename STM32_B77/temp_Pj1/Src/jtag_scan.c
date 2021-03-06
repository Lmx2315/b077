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

/* This file implements JTAG protocol support.  Provides functionality
 * to detect devices on the scan chain and read their IDCODEs.
 * It depends on the low-level function provided by the platform's jtagtap.c.
 */


#include "jtagtap.h"
#include "jtag_scan.h"
#include "jtag_devs.h"

jtag_dev_t jtag_devs[JTAG_MAX_DEVS+1];
int jtag_dev_count;

/* bucket of ones for don't care TDI */
static const uint8_t ones[] = "\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF";

/* Scan JTAG chain for devices, store IR length and IDCODE (if present).
 * Reset TAP state machine.
 * Select Shift-IR state.
 * Each device is assumed to shift out IR at 0x01. (this may not always be true)
 * Shift in ones until we read two consecutive ones, then we have shifted out the
 * 	IRs of all devices.
 *
 * After this process all the IRs are loaded with the BYPASS command.
 * Select Shift-DR state.
 * Shift in ones and count zeros shifted out. Should be one for each device.
 * Check this against device count obtained by IR scan above.
 *
 * Reset the TAP state machine again. This should load all IRs with IDCODE.
 * For each device, shift out one bit. If this is zero IDCODE isn't present,
 *	continue to next device. If this is one shift out the remaining 31 bits
 *	of the IDCODE register.
 */
int jtag_scan(const uint8_t *irlens,uint8_t col)
{
	int i;
	uint32_t j;
	uint16_t z=0;
	uint16_t q=0;

	jtag_dev_count = 0;
	memset(&jtag_devs, 0, sizeof(jtag_devs));

	/* Run throught the SWD to JTAG sequence for the case where an attached SWJ-DP is
	 * in SW-DP mode.
	 */
	Transf("Resetting TAP\r\n");

	jtagtap_init();//???????????????????????????? ???????????????????? JTAG ???????????????? ?? ?????????????????????? ???????????????? ???????????????? ???????????????????? TMS,TCK,TDI,TDO

	jtag_proc.jtagtap_reset();//???????????????????? ???????????????????? JTAG

	if (irlens) 
	{
		Transf("Given list of IR lengths, skipping probe\n\r");
		Transf("Change state to Shift-IR\n\r");
		jtagtap_shift_ir();
		j = 0;
		while((jtag_dev_count <= JTAG_MAX_DEVS) &&(jtag_devs[jtag_dev_count].ir_len <= JTAG_MAX_IR_LEN)) 
		{
			uint32_t irout;
	//		if(*irlens == 0) //???????? ?? ?????????????? ?????????? "0" ?? ?????????????????? ???????????? ???? ?????????????????????? ??????????????
			if (col==0)
				break;
			jtag_proc.jtagtap_tdi_tdo_seq((uint8_t*)&irout, 0, ones, *irlens);//???????? ?????????????? ?????????????? ?????? ?????????????? ???????????? IR ????????????????
			if (!(irout & 1)) {
				Transf("check failed: IR[0] != 1\n\r");
				return -1;
			}
			jtag_devs[jtag_dev_count].ir_len = *irlens;
			jtag_devs[jtag_dev_count].ir_prescan = j;
			jtag_devs[jtag_dev_count].dev = jtag_dev_count;
			j += *irlens;
			irlens++; //?????????????????????? ?????????????????? ???? ???????????? ?? ?????????????? ???????? ?????????????????? IR
			jtag_dev_count++;
			col--;
		}
	} 
	else //?????????? ?????????????????? ?? ?????????????????? jtagtap_reset ?? IR ?????????????? ???????????????????????? ?????????? IDCODE, ?????? ARM ?????? b1110 !
	{
		Transf("Change state to Shift-IR\n\r");
		jtagtap_shift_ir();

		Transf("Scanning out IRs\n\r");
		if(!jtag_proc.jtagtap_next(0, 1)) 
		{
			Transf("jtag_scan: Sanity check failed: IR[0] shifted out as 0\n\r");
			jtag_dev_count = -1;
			return -1; // must be 1 
		}
		jtag_devs[0].ir_len = 1; j = 1;
	
		while((jtag_dev_count <= JTAG_MAX_DEVS) &&(jtag_devs[jtag_dev_count].ir_len <= JTAG_MAX_IR_LEN)) 
			{
				q=q<<1;
				q=q+jtag_proc.jtagtap_next(0, 1);
//				x_out("q:",q);
				if (q==0xffff) break;
				if (q==0x155)
					{
					     jtag_devs[++jtag_dev_count].ir_len =  1;
					     jtag_devs[  jtag_dev_count].ir_prescan = j;
					     jtag_devs[  jtag_dev_count].dev = jtag_dev_count;
//						 u_out("ir_len:",jtag_devs[jtag_dev_count].ir_len);
						 q=0;
					} 
						else jtag_devs[jtag_dev_count].ir_len++;				
				j++;
		    }
		
		if(jtag_dev_count > JTAG_MAX_DEVS) 
		{
			Transf("jtag_scan: Maximum device count exceeded\n\r");
			jtag_dev_count = -1;
			return -1;
		}
		
		if(jtag_devs[jtag_dev_count].ir_len > JTAG_MAX_IR_LEN) 
		{
			Transf("jtag_scan: Maximum IR length exceeded\n\r");
			jtag_dev_count = -1;
			return -1;
		}
	}
    u_out("jtag_dev_count:",jtag_dev_count);
	Transf("Return to Run-Test/Idle\n\r");
	jtag_proc.jtagtap_next(1, 1);
	jtagtap_return_idle();

	/* All devices should be in BYPASS now */

	/* Count device on chain */
	Transf("Change state to Shift-DR\n\r");
	jtagtap_shift_dr();
	for(i = 0; (jtag_proc.jtagtap_next(0, 1) == 0) && (i <= jtag_dev_count); i++)
		jtag_devs[i].dr_postscan = jtag_dev_count - i - 1;

	if(i != jtag_dev_count) {
		Transf("jtag_scan: Sanity check failed:BYPASS dev count doesn't match IR scan\n\r");
		u_out("i:",i);
		u_out("jtag_dev_count:",jtag_dev_count);
		jtag_dev_count = -1;
		return -1;
	}

	Transf("Return to Run-Test/Idle\n\r");
	jtag_proc.jtagtap_next(1, 1);
	jtagtap_return_idle();
	if(!jtag_dev_count) {
		return 0;
	}

	/* Fill in the ir_postscan fields */
	for(i = jtag_dev_count - 1; i; i--)
		jtag_devs[i-1].ir_postscan = jtag_devs[i].ir_postscan +
					jtag_devs[i].ir_len;

	/* Reset jtagtap: should take all devs to IDCODE */
	jtag_proc.jtagtap_reset();
	jtagtap_shift_dr();
	for(i = 0; i < jtag_dev_count; i++) {
		if(!jtag_proc.jtagtap_next(0, 1)) continue;
		jtag_devs[i].idcode = 1;
		for(j = 2; j; j <<= 1)
			if(jtag_proc.jtagtap_next(0, 1)) jtag_devs[i].idcode |= j;

	}
	Transf("Return to Run-Test/Idle\n\r");
	jtag_proc.jtagtap_next(1, 1);
	jtagtap_return_idle();
	Transf("\r\n");
	/* Check for known devices and handle accordingly */
	for(i = 0; i < jtag_dev_count; i++)
	{
		for(j = 0; dev_descr[j].idcode; j++)
			if((jtag_devs[i].idcode & dev_descr[j].idmask) == dev_descr[j].idcode) 
			{
				jtag_devs[i].current_ir = -1;
				/* Save description in table */
				jtag_devs[i].descr = dev_descr[j].descr;
				
				/* Call handler to initialise/probe device further */
				if(dev_descr[j].handler)
					dev_descr[j].handler(&jtag_devs[i]);
				break;
			}
			Transf("--------------------------------------------------------\r\n");
			un_out("",i);Transf("] ");xn_out("",jtag_devs[i].idcode);Transf(" - ");Transf(jtag_devs[i].descr);Transf("\r\n");
			u_out("dr_prescan :",jtag_devs[i].dr_prescan);
			u_out("dr_postscan:",jtag_devs[i].dr_postscan);
			u_out("ir_len     :",jtag_devs[i].ir_len);
			u_out("ir_prescan :",jtag_devs[i].ir_prescan);
			u_out("ir_postscan:",jtag_devs[i].ir_postscan);
			x_out("idcode     :",jtag_devs[i].idcode);
//			u_out("current_ir :",jtag_devs[i].current_ir);
//			Transf("--------------------------------------------------------\r\n");
	}

	return jtag_dev_count;
}

void jtag_dev_write_ir(jtag_proc_t *jp, jtag_dev_t *d, uint32_t ir)
{
	if(ir == d->current_ir) return;
	for(int i = 0; i < jtag_dev_count; i++)
		jtag_devs[i].current_ir = -1;
	d->current_ir = ir;

	jtagtap_shift_ir();
	jp->jtagtap_tdi_seq(0, ones, d->ir_prescan);
	jp->jtagtap_tdi_seq(d->ir_postscan?0:1, (void*)&ir, d->ir_len);
	jp->jtagtap_tdi_seq(1, ones, d->ir_postscan);
	jtagtap_return_idle();
}

void jtag_dev_shift_dr(jtag_proc_t *jp, jtag_dev_t *d, uint8_t *dout, const uint8_t *din, int ticks)
{
	jtagtap_shift_dr();
	jp->jtagtap_tdi_seq(0, ones, d->dr_prescan);
	if(dout)
		jp->jtagtap_tdi_tdo_seq((void*)dout, d->dr_postscan?0:1, (void*)din, ticks);
	else
		jp->jtagtap_tdi_seq(d->dr_postscan?0:1, (void*)din, ticks);
	jp->jtagtap_tdi_seq(1, ones, d->dr_postscan);
	jtagtap_return_idle();
}

