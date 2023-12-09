/*
 * init_PWM_motor_1_2_3.c
 *
 *  Created on: 18 may. 2020
 *      Author: FACU
 */
#include "F28x_Project.h"
#include "init_PWM_motor_1_2_3.h"

//salidas PWM
void InitEPwm2(Uint16 Period, Uint16 phaseOffset2, Uint16 Duty2)   //corresponde al motor 2
{
   // Setup TBCLK
   EPwm2Regs.TBCTL.bit.CTRMODE = 0;             // Count up
   EPwm2Regs.TBPRD = Period;               // Same period as PWM1
   EPwm2Regs.TBCTL.bit.PHSEN = 1;               // Enable phase loading
   EPwm2Regs.TBPHS.bit.TBPHS = phaseOffset2;    // Phase
   EPwm2Regs.TBCTR = 0x0000;                    // Clear counter
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = 1;           // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV = 0;

   // Setup shadow register load on ZERO
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = 0;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = 0;
   EPwm2Regs.CMPCTL.bit.LOADBMODE = 0;

   // Set Compare values
   EPwm2Regs.CMPA.bit.CMPA = Duty2;        // Set compare A value

   // Set actions
   EPwm2Regs.AQCTLA.bit.ZRO = 2;                // Set PWM1A on Zero
   EPwm2Regs.AQCTLA.bit.CAU = 1;                // Clear PWM1A on event A, up count
}
//
void InitEPwm3(Uint16 Period, Uint16 phaseOffset3, Uint16 Duty3)  //corresponde al motor 3
{
   // Setup TBCLK
   EPwm3Regs.TBCTL.bit.CTRMODE = 0;             // Count up
   EPwm3Regs.TBPRD = Period;               // Same period as PWM1
   EPwm3Regs.TBCTL.bit.PHSEN = 1;               // Enable phase loading
   EPwm3Regs.TBPHS.bit.TBPHS = phaseOffset3;    // Phase
   EPwm3Regs.TBCTR = 0x0000;                    // Clear counter
   EPwm3Regs.TBCTL.bit.HSPCLKDIV = 1;           // Clock ratio to SYSCLKOUT
   EPwm3Regs.TBCTL.bit.CLKDIV = 0;

   // Setup shadow register load on ZERO
   EPwm3Regs.CMPCTL.bit.SHDWAMODE = 0;
   EPwm3Regs.CMPCTL.bit.SHDWBMODE = 0;
   EPwm3Regs.CMPCTL.bit.LOADAMODE = 0;
   EPwm3Regs.CMPCTL.bit.LOADBMODE = 0;

   // Set Compare values
   EPwm3Regs.CMPA.bit.CMPA = Duty3;        // Set compare A value

   // Set actions
   EPwm3Regs.AQCTLA.bit.ZRO = 2;                // Set PWM1A on Zero
   EPwm3Regs.AQCTLA.bit.CAU = 1;                // Clear PWM1A on event A, up count
}
//
void InitEPwm4(Uint16 Period, Uint16 phaseOffset1, Uint16 Duty1)   //corresponde al motor 1
{
   // Setup TBCLK
   EPwm4Regs.TBCTL.bit.CTRMODE = 0;             // Count up
   EPwm4Regs.TBPRD = Period;               // Same period as PWM4
   EPwm4Regs.TBCTL.bit.PHSEN = 1;               // Enable phase loading
   EPwm4Regs.TBPHS.bit.TBPHS = phaseOffset1;    // Phase
   EPwm4Regs.TBCTR = 0x0000;                    // Clear counter
   EPwm4Regs.TBCTL.bit.HSPCLKDIV = 1;           // Clock ratio to SYSCLKOUT
   EPwm4Regs.TBCTL.bit.CLKDIV = 0;

   // Setup shadow register load on ZERO
   EPwm4Regs.CMPCTL.bit.SHDWAMODE = 0;
   EPwm4Regs.CMPCTL.bit.SHDWBMODE = 0;
   EPwm4Regs.CMPCTL.bit.LOADAMODE = 0;
   EPwm4Regs.CMPCTL.bit.LOADBMODE = 0;

   // Set Compare values
   EPwm4Regs.CMPA.bit.CMPA = Duty1;        // Set compare A value

   // Set actions
   EPwm4Regs.AQCTLA.bit.ZRO = 2;                // Set PWM1A on Zero
   EPwm4Regs.AQCTLA.bit.CAU = 1;                // Clear PWM1A on event A, up count
}
/////////////Funcion del PWM que dispara los ADCs////////////////////////////
void InitEPwm5(void)
{
    /*EALLOW;
    // Assumes ePWM clock is already enabled
    EPwm5Regs.TBCTL.bit.CTRMODE = 3;            // Freeze counter
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = 0;          // TBCLK pre-scaler = /1
    EPwm5Regs.TBPRD = 0x07D0;                   // Set period to 2000 counts (50kHz)
    EPwm5Regs.ETSEL.bit.SOCAEN  = 0;            // Disable SOC on A group
    EPwm5Regs.ETSEL.bit.SOCASEL = 2;            // Select SOCA on period match
    EPwm5Regs.ETSEL.bit.SOCAEN = 1;             // Enable SOCA
    EPwm5Regs.ETPS.bit.SOCAPRD = 1;             // Generate pulse on 1st event
    EDIS;*/

    //ANDA - SACADO DEL EJEMPLO
    //
    EALLOW;
    // Assumes ePWM clock is already enabled
    EPwm5Regs.ETSEL.bit.SOCAEN = 0;             // Enable SOCA
    EPwm5Regs.ETSEL.bit.SOCASEL = 4;            // Select SOCA on up-count
    EPwm5Regs.ETPS.bit.SOCAPRD = 1;             // Generate pulse on 1st event
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = 0;          // TBCLK pre-scaler = /1
    //EPwm5Regs.CMPA.bit.CMPA = 0x03E8;           // Set compare A value to 1000 counts
    EPwm5Regs.CMPA.bit.CMPA = 40;                // Set compare A value to 40 counts
    //EPwm5Regs.TBPRD = 2000*5;                   // Set period to 2000 counts (50kHz)
    EPwm5Regs.TBPRD = 380;                      // Set period to 380 counts (33kHz)
    EPwm5Regs.TBCTL.bit.CTRMODE = 3;            // Freeze counter
    EDIS;
}
////////////////////////////////////////////////////////////////////////////////



