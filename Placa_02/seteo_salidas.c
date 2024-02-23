/*
 * seteo_salidas.c
 *
 *  Created on: 18 may. 2020
 *      Author: FACU
 */
#include "F28x_Project.h"
#include "DCLF32_scorbot.h"
#include "DCL_scorbot.h"
#include "eQEP_motor_1_2_3.h"
#include "init_PWM_motor_1_2_3.h"
#include "adc_motor_1_2_3.h"
#include "protecciones.h"
#include "Example_EPwmSetup.h"
#include "seteo_salidas.h"
#include "control.h"
#include "muestreo.h"

/////////////////////Funcion para setear las salidas PWM///////////////////////
void Set_Salidas(){
    EPwm4Regs.CMPA.bit.CMPA = (Uint16) Duty1;
    EPwm2Regs.CMPA.bit.CMPA = (Uint16) Duty2;
    EPwm3Regs.CMPA.bit.CMPA = (Uint16) Duty3;
}


