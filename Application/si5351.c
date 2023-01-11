/*
 * si5351.c - Si5351 driver
 *
 * Copyright (C) 2015 Yury Kuchura (EU1KY) <kuchura@gmail.com>
 *
 * This code is inspired by NT7S Arduino port of Linux Si5351 driver code:
 *       Copyright (C) 2014 Jason Milldrum <milldrum@gmail.com>
 *
 * But the calculation in original code is optimized to my purposes: it uses
 * the same PLLA always running on maximum possible frequency, and only changes
 * multisynth dividers for two clock outputs.
 *
 * Orginal code is derived from clk-si5351.c in the Linux kernel.
 *    Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
 *    Rabeeh Khoury <rabeeh@solid-run.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

//#include <stdio.h>
//#include <stdint.h>
//#include <stdlib.h>
//#include <string.h>
//#include <math.h>

#include "si5351.h"
//#include "config.h"
//#include "rational.h"
//#include "font.h"
//#include "crash.h"
//*******************************************************************************************
//*******************************************************************************************
struct Si5351Status 	dev_status;
struct Si5351IntStatus 	dev_int_status;
static uint32_t 		si5351xtalFreq = SI5351_XTAL_FREQ_DEFAULT;

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
/*******************************/
/* Suggested private functions */
/*******************************/
static uint8_t si5351_read(uint8_t addr, uint8_t *data){

	I2C_Master_Read(SI5351_I2C, SI5351_I2C_ADDR, addr, data, 1);
    return 0;
}
//************************************************************
//static uint8_t si5351_read_device_reg(uint8_t reg){
//
//    uint8_t data = 0;
//    //-------------------
//    si5351_read(reg, &data);
//    return data;
//}
//************************************************************
static uint8_t si5351_write(uint8_t addr, uint8_t data){

	I2C_Master_Write(SI5351_I2C, SI5351_I2C_ADDR, addr, &data, 1);
    return 0;
}
//************************************************************
static uint8_t si5351_write_bulk(uint8_t addr, uint8_t size, uint8_t *data){

	I2C_Master_Write(SI5351_I2C, SI5351_I2C_ADDR, addr, data, size);
    return 0;
}
//************************************************************
static void si5351_set_clk_control(enum si5351_clock clk, enum si5351_pll pll, int isIntegerMode, enum si5351_drive drive){

    //Bit  D7       D6      D5      D4       D3 D2         D1 D0
    //Name CLK0_PDN MS0_INT MS0_SRC CLK0_INV CLK0_SRC[1:0] CLK0_IDRV[1:0]
    uint8_t reg_val = 0x0C; //Select this multisynth as the source for clk output,
    //-------------------
    reg_val |= ((uint8_t)drive) & 3;//Set drive strength

    //Select PLLB as the source for this multisynth, otherwise PLLA will be used
    if(pll == SI5351_PLLB) reg_val |= SI5351_CLK_PLL_SELECT;

    //Set integer mode for this multisynth
    if (isIntegerMode) reg_val |= SI5351_CLK_INTEGER_MODE;

    //clk value can be used as offset to address proper register
    si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}
//************************************************************
static void si5351_set_pll(uint32_t a, uint32_t b, uint32_t c, enum si5351_pll pll){

	//Set PLL parameters
    uint8_t params[8];
    uint8_t i = 0;
    uint8_t temp;
    uint32_t p1, p2, p3;
    //-------------------
    p3  = c;
    p2  = (128 * b) % c;
    p1  = 128 * a;
    p1 += (128 * b / c);
    p1 -= 512;

    /* Registers 26-27 */
    temp = ((p3 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t)(p3  & 0xFF);
    params[i++] = temp;

    /* Register 28 */
    temp = (uint8_t)((p1 >> 16) & 0x03);
    params[i++] = temp;

    /* Registers 29-30 */
    temp = (uint8_t)((p1 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t)(p1  & 0xFF);
    params[i++] = temp;

    /* Register 31 */
    temp = (uint8_t)((p3 >> 12) & 0xF0);
    temp += (uint8_t)((p2 >> 16) & 0x0F);
    params[i++] = temp;

    /* Registers 32-33 */
    temp = (uint8_t)((p2 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t)(p2  & 0xFF);
    params[i++] = temp;

    if (pll == SI5351_PLLA)
    {
        si5351_write_bulk(SI5351_PLLA_PARAMETERS, 8, params);
        //Any change to the feedback Multisynth require a PLL reset
        si5351_write(SI5351_PLL_RESET, SI5351_PLL_RESET_A);
    }
    else
    {
        si5351_write_bulk(SI5351_PLLB_PARAMETERS, 8, params);
        //Any change to the feedback Multisynth require a PLL reset
        si5351_write(SI5351_PLL_RESET, SI5351_PLL_RESET_B);
    }
}

//************************************************************
static void si5351_set_ms(uint32_t a, uint32_t b, uint32_t c, uint8_t rdiv, enum si5351_clock clk)
{
    uint8_t params[8];
    uint8_t i = 0;
    uint8_t temp;
    uint32_t p1, p2, p3;
    //-------------------
    if (a == 4)
    {
        p1 = 0;
        p2 = 0;
        p3 = 1;
    }
    else
    {
        p3  =  c;
        p2  = (128 * b) % c;
        p1  =  128 * a;
        p1 += (128 * b / c);
        p1 -=  512;
    }

    /* Registers 42-43 */
    temp = (uint8_t)((p3 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t)(p3  & 0xFF);
    params[i++] = temp;

    /* Register 44 (or 52)*/
    temp = (uint8_t)((p1 >> 16) & 0x03);
    if (a == 4) temp |= 0x0C; //Set div by 4 bits
    temp |= ((rdiv & 7) << 4);
    params[i++] = temp;

    /* Registers 45-46 */
    temp = (uint8_t)((p1 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t)(p1  & 0xFF);
    params[i++] = temp;

    /* Register 47 */
    temp  = (uint8_t)((p3 >> 12) & 0xF0);
    temp += (uint8_t)((p2 >> 16) & 0x0F);
    params[i++] = temp;

    /* Registers 48-49 */
    temp = (uint8_t)((p2 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t)(p2  & 0xFF);
    params[i++] = temp;

    /* Write the parameters */
    	 if(clk == SI5351_CLK0) si5351_write_bulk(SI5351_CLK0_PARAMETERS, 8, params);
    else if(clk == SI5351_CLK1) si5351_write_bulk(SI5351_CLK1_PARAMETERS, 8, params);
    else if(clk == SI5351_CLK2) si5351_write_bulk(SI5351_CLK2_PARAMETERS, 8, params);
}
//************************************************************
//Return 0 if si5351 is not found, or its address on the i2c bus
//static uint8_t si5351_detect_address(void)
//{
//    uint8_t addr = 2;
//    while (1)
//    {
//       //uint8_t data = CAMERA_IO_Read(addr, 0);
//    	uint8_t data = I2C_StartAndSendDeviceAddr(SI5351_I2C, addr);
//        if(data != 0xFF && data != 0)
//            break; //Should be 0x10 - LOS bit is set
//        addr += 2;
//        if (addr == 0)
//            break;
//    }
//    return addr;
//}
//************************************************************
/*
 * Calculate best rational approximation for a given fraction
 * taking into account restricted register size, e.g. to find
 * appropriate values for a pll with 5 bit denominator and
 * 8 bit numerator register fields, trying to set up with a
 * frequency ratio of 3.1415, one would say:
 *
 * rational_best_approximation(31415, 10000,
 *              (1 << 8) - 1, (1 << 5) - 1, &n, &d);
 *
 * you may look at given_numerator as a fixed point number,
 * with the fractional part size described in given_denominator.
 *
 * for theoretical background, see:
 * http://en.wikipedia.org/wiki/Continued_fraction
 */
void rational_best_approximation(uint64_t  given_numerator, uint64_t  given_denominator,
								 uint32_t  max_numerator,   uint32_t  max_denominator,
								 uint32_t *best_numerator,  uint32_t *best_denominator){

    uint64_t n, d, n0, d0, n1, d1;
    n = given_numerator;
    d = given_denominator;
    n0 = d1 = 0;
    n1 = d0 = 1;
    for (;;)
    {
        uint64_t t = 0, a = 0;
        if ((n1 > max_numerator) || (d1 > max_denominator))
        {
            n1 = n0;
            d1 = d0;
            break;
        }
        if (d == 0)
            break;
        t = d;
        a = n / d;
        d = n % d;
        n = t;
        t = n0 + a * n1;
        n0 = n1;
        n1 = t;
        t = d0 + a * d1;
        d0 = d1;
        d1 = t;
    }
    *best_numerator = (uint32_t)n1;
    *best_denominator = (uint32_t)d1;
}
//************************************************************
static void set_multisynth_alt(uint32_t freq, enum si5351_clock clk){

    uint32_t a  = 0,
    		 b  = 0,
			 c  = 0,
    		 ap = 0,
			 bp = 0,
			 cp = 0;
//    uint32_t ap, bp, cp;

    uint64_t nom = 0,
    		 den = 0;
    double divpll = 0.0,
    	   divms  = 0.0,
		   fpll   = 0.0,
		   k      = 0.0;
    uint8_t  rdiv = SI5351_OUTPUT_CLK_DIV_1;

//    uint32_t si5351_XTAL_FREQ = SI5351_XTAL_FREQ;
    uint32_t si5351_XTAL_FREQ = si5351xtalFreq;
//    uint32_t si5351_XTAL_FREQ = (uint32_t)((int)CFG_GetParam(CFG_PARAM_SI5351_XTAL_FREQ) + (int)CFG_GetParam(CFG_PARAM_SI5351_CORR));

    if(freq < SI5351_MULTISYNTH_MIN_FREQ / 128) freq = SI5351_MULTISYNTH_MIN_FREQ / 128;
//    else if (freq > CFG_GetParam(CFG_PARAM_SI5351_MAX_FREQ))
//        freq = CFG_GetParam(CFG_PARAM_SI5351_MAX_FREQ);

    if(freq >= 112500000ul)
    {//Use output multisynth constant divider = 4,
     //calculate PLL feedback multisynth to set desired frequency
        if (freq >= SI5351_MULTISYNTH_DIVBY4_FREQ) divms = 4.0;
        else						               divms = 6.0;

        divpll = (freq / ((double)si5351_XTAL_FREQ)) * divms;
        //Calculate ap, bp, cp (pll feedback multisynth parameters)
        ap = (uint32_t)floor(divpll);
        bp = 0;
        cp = 1;
        k = divpll - ap;
        nom = (uint64_t)(k * 0x6FFFFFFFFFFFFFFFull);
        den = 0x6FFFFFFFFFFFFFFFull;
        rational_best_approximation(nom, den, SI5351_PLL_B_MAX, SI5351_PLL_C_MAX, &bp, &cp);
        fpll = si5351_XTAL_FREQ * ((double)ap + (double)bp / (double)cp);
    }
    else
    {//Set PLL to maximum frequency, calculate output multisynth divider
        uint32_t freq_m = freq;

        //Handle frequencies below 1 MHz
        	 if(freq < (SI5351_MULTISYNTH_MIN_FREQ / 64)) rdiv = SI5351_OUTPUT_CLK_DIV_128;
        else if(freq < (SI5351_MULTISYNTH_MIN_FREQ / 32)) rdiv = SI5351_OUTPUT_CLK_DIV_64;
        else if(freq < (SI5351_MULTISYNTH_MIN_FREQ / 16)) rdiv = SI5351_OUTPUT_CLK_DIV_32;
        else if(freq < (SI5351_MULTISYNTH_MIN_FREQ / 8))  rdiv = SI5351_OUTPUT_CLK_DIV_16;
        else if(freq < (SI5351_MULTISYNTH_MIN_FREQ / 4))  rdiv = SI5351_OUTPUT_CLK_DIV_8;
        else if(freq < (SI5351_MULTISYNTH_MIN_FREQ / 2))  rdiv = SI5351_OUTPUT_CLK_DIV_4;
        else if(freq < (SI5351_MULTISYNTH_MIN_FREQ))	  rdiv = SI5351_OUTPUT_CLK_DIV_2;

        freq_m *= (1 << rdiv);
        divpll  = ((double)SI5351_PLL_VCO_MAX) / ((double)si5351_XTAL_FREQ);

        //Calculate ap, bp, cp (pll feedback multisynth parameters)
        ap = (uint32_t)floor(divpll);
        bp = 0;
        cp = 1;
        k = divpll - ap;
        nom = (uint64_t)(k * 0x6FFFFFFFFFFFFFFFull);
        den = 0x6FFFFFFFFFFFFFFFull;
        rational_best_approximation(nom, den, SI5351_PLL_B_MAX, SI5351_PLL_C_MAX, &bp, &cp);
        fpll = si5351_XTAL_FREQ * ((double)ap + (double)bp / (double)cp);
        divms = fpll / (double)freq_m;
    }

    //Calculate a, b, c (output multisynth parameters) from divms calculated abvge
    b = 0;
    c = 1;
    	 if(divms == 4.0) a = 4; //MS divider is integer+
    else if(divms == 6.0) a = 6; //MS divider is integer
    else
    {//MS divider is fractional
        a = (uint32_t)floor(divms);
        k = divms - a;
        nom = (uint64_t)(k * 0x6FFFFFFFFFFFFFFFull);
        den = 0x6FFFFFFFFFFFFFFFull;
        rational_best_approximation(nom, den, SI5351_PLL_B_MAX, SI5351_PLL_C_MAX, &b, &c);
    }

    //Write PLL parameters
    if (clk == SI5351_CLK0)
    {
        si5351_set_clk_control(clk, SI5351_PLLA, (a == 4) || (a == 6), SI5351_DRIVE_8MA);
        si5351_set_ms(a, b, c, rdiv, clk);
        si5351_set_pll(ap, bp, cp, SI5351_PLLA);
    }
    else if (clk == SI5351_CLK1)
    {
        si5351_set_clk_control(clk, SI5351_PLLB, (a == 4) || (a == 6), SI5351_DRIVE_8MA);
        si5351_set_ms(a, b, c, rdiv, clk);
        si5351_set_pll(ap, bp, cp, SI5351_PLLB);
    }
    else if (clk == SI5351_CLK2)
    {
        si5351_set_clk_control(clk, SI5351_PLLB, (a == 4) || (a == 6), SI5351_DRIVE_8MA);
        si5351_set_ms(a, b, c, rdiv, clk);
        si5351_set_pll(ap, bp, cp, SI5351_PLLB);
    }
//    #if 0
//    {
//        //Calculate and print actual frequency
//        double fpll_actual;
//        double f_actual;
//        double divms;
//        double diff;
//        fpll_actual = si5351_XTAL_FREQ * ((double)ap + (double)bp / (double)cp);
//        divms = ((double)a + (double)b / (double)c);
//        f_actual = (fpll_actual / divms) / (1 << rdiv);
//        diff = f_actual - freq;
//        if (abs(diff) > 0.5)
//            DBGPRINT("CLK%d actual freq: %.1f Hz, desired %u, diff %.1f\n", clk, f_actual, (unsigned int)freq, f_actual - freq);
//    }
//    #endif
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
/******************************/
/* Suggested public functions */
/******************************/
/*
 * si5351_Init(void)
 *
 * Call this to initialize I2C communications and get the
 * Si5351 ready for use.
 */
void Si5351_Init(void){

	if(!Si5351_Check()) return;

	//Disable all outputs
    si5351_write(SI5351_OUTPUT_ENABLE_CTRL, 0xFF);

    //Powerdown all output drivers
    uint8_t a;
    for(a = SI5351_CLK0_CTRL; a <= SI5351_CLK0_CTRL; a++)
    {
        si5351_write(a, SI5351_CLK_POWERDOWN);
    }

    // Set crystal load capacitance
    //si5351_write(SI5351_CRYSTAL_LOAD, ((CFG_GetParam(CFG_PARAM_SI5351_CAPS) & 3) << 6) | 0x12); //Bits 5:0 should be written as 0x12
    si5351_write(SI5351_CRYSTAL_LOAD, SI5351_CRYSTAL_LOAD_8PF);

    //Set input source
    si5351_write(SI5351_PLL_INPUT_SOURCE, 0); // Input source is XTAL for both PLLs, CLK not divided

    //Disable spread spectrum (value after reset is unknown), including the entire range of SS registers
    for(a = SI5351_SSC_PARAM0; a <= SI5351_SSC_PARAM12; a++)
    {
        si5351_write(a, 0);
    }

    //Disable fanout (initial value is unknown)
    si5351_write(SI5351_FANOUT_ENABLE, 0);
}
//************************************************************
uint32_t Si5351_Check(void){

	return I2C_Master_CheckSlave(SI5351_I2C, SI5351_I2C_ADDR);
}
//************************************************************
/*
 * si5351_SetFreq(uint32_t freq, enum si5351_clock output)
 *
 * Sets the clock frequency of the specified CLK output
 *
 * freq - Output frequency in Hz
 * clk - Clock output
 *   (use the si5351_clock enum)
 */
void Si5351_SetFreq(uint32_t freq, enum si5351_clock clk){

	set_multisynth_alt(freq, clk);
}
//************************************************************
/*
 * si5351_ClockEnable(enum si5351_clock clk, uint8_t enable)
 *
 * Enable or disable a chosen clock
 * clk - Clock output
 *   (use the si5351_clock enum)
 * enable - Set to 1 to enable, 0 to disable
 */
void Si5351_ClockEnable(enum si5351_clock clk, uint8_t enable){

	uint8_t reg_val = 0;
	//-------------------
    if(si5351_read(SI5351_OUTPUT_ENABLE_CTRL, &reg_val) != 0) return;

    if(enable == SI5351_CLOCK_ENABLE)reg_val &= ~(1<<(uint8_t)clk);
    else		   					 reg_val |=  (1<<(uint8_t)clk);

    si5351_write(SI5351_OUTPUT_ENABLE_CTRL, reg_val);
}
//************************************************************
void Si5351_SetXtalFreq(uint32_t freq){

	si5351xtalFreq = freq;
}
//************************************************************
uint32_t Si5351_GetXtalFreq(void){

	return si5351xtalFreq;
}
//************************************************************
void Si5351_SetF0(uint32_t fhz){

	Si5351_SetFreq(fhz, SI5351_CLK0);
	Si5351_ClockEnable(SI5351_CLK0, SI5351_CLOCK_ENABLE);
}
//************************************************************
void Si5351_SetLO(uint32_t fhz){

	Si5351_SetFreq(fhz, SI5351_CLK1);
	Si5351_ClockEnable(SI5351_CLK1, SI5351_CLOCK_ENABLE);
}
//************************************************************
void Si5351_SetF2(uint32_t fhz){

	Si5351_SetFreq(fhz, SI5351_CLK2);
	Si5351_ClockEnable(SI5351_CLK2, SI5351_CLOCK_ENABLE);
}
//************************************************************
void Si5351_AllOff(void){

	Si5351_ClockEnable(SI5351_CLK0, SI5351_CLOCK_DISABLE);
	Si5351_ClockEnable(SI5351_CLK1, SI5351_CLOCK_DISABLE);
	Si5351_ClockEnable(SI5351_CLK2, SI5351_CLOCK_DISABLE);
    si5351_write(SI5351_CLK0_CTRL, SI5351_CLK_POWERDOWN);
    si5351_write(SI5351_CLK1_CTRL, SI5351_CLK_POWERDOWN);
    si5351_write(SI5351_CLK2_CTRL, SI5351_CLK_POWERDOWN);
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************














