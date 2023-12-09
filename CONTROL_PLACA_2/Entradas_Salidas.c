/*
 * Entradas_Salidas.c
 *
 *  Created on: 28 may. 2020
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
#include "Entradas_Salidas.h"

void Entradas_Salidas_init(void){

    ///////////////////////////////////INPUTS///////////////////////////////////////
    // init para los HOME_SWITCHS
    //
    EALLOW;
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;        // GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;         // GPIO32 como entrada --> HOME_MSW_1
    //GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;       // normalmente bajo
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;       // Asynch input GPIO32

    GpioCtrlRegs.GPDMUX1.bit.GPIO111 = 0;        // GPIO
    GpioCtrlRegs.GPDDIR.bit.GPIO111 = 0;         // GPIO19 como entrada --> HOME_MSW_2
    //GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;       // normalmente bajo
    GpioCtrlRegs.GPDQSEL1.bit.GPIO111 = 3;       // Asynch input GPIO111

    GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;        // GPIO
    GpioCtrlRegs.GPCDIR.bit.GPIO67 = 0;         // GPIO18 como entrada --> HOME_MSW_3
    //GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;       // normalmente bajo
    GpioCtrlRegs.GPCQSEL1.bit.GPIO67 = 3;       // Asynch input GPIO67
    EDIS;
    //
    // init para detectar las T_FLAG_x
    //
    EALLOW;
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;        // GPIO
    GpioCtrlRegs.GPCDIR.bit.GPIO66 = 0;         // GPIO66 como entrada --> T_FLAG_1
    //GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;     // normalmente bajo
    GpioCtrlRegs.GPCPUD.bit.GPIO66 = 0;         // R PULL UP
    GpioCtrlRegs.GPCQSEL1.bit.GPIO66 = 3;       // Asynch input GPIO66

    GpioCtrlRegs.GPEMUX1.bit.GPIO131 = 0;        // GPIO
    GpioCtrlRegs.GPEDIR.bit.GPIO131 = 0;         // GPIO131 como entrada --> T_FLAG_2
    //GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;       // normalmente bajo
    GpioCtrlRegs.GPEPUD.bit.GPIO131 = 0;
    GpioCtrlRegs.GPEQSEL1.bit.GPIO131 = 3;       // Asynch input GPIO131

    GpioCtrlRegs.GPEMUX1.bit.GPIO130 = 0;        // GPIO
    GpioCtrlRegs.GPEDIR.bit.GPIO130 = 0;         // GPIO130 como entrada --> T_FLAG_3
    //GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;       // normalmente bajo
    GpioCtrlRegs.GPEPUD.bit.GPIO130 = 0;
    GpioCtrlRegs.GPEQSEL1.bit.GPIO130 = 3;       // Asynch input GPIO130
    EDIS;
    //
    // init para las XINT
    //
    EALLOW;
    GpioCtrlRegs.GPCMUX2.bit.GPIO95 = 0;         // GPIO
    GpioCtrlRegs.GPCDIR.bit.GPIO95 = 0;          // GPIO95 como entrada
    GpioCtrlRegs.GPCPUD.bit.GPIO95 = 0;          // Enable pull-up on GPIO95 .... OCP
    GpioCtrlRegs.GPCQSEL2.bit.GPIO95 = 3;        // Asynch input GPIO95

    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;         // GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO52 = 0;          // GPIO52 como entrada
    GpioCtrlRegs.GPBPUD.bit.GPIO52 = 0;          // Enable pull-UP on GPIO52 .... OTP -- puede ser que no ande sin esto
    GpioCtrlRegs.GPBQSEL2.bit.GPIO52 = 3;        // Asynch input GPIO52

    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;         // GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO59 = 0;          // GPIO59 como entrada
    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;          // Enable pull-up on GPIO59 .... STOP EMERGENCY
    GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3;        // Asynch input GPIO59
    EDIS;
    //
    // init GPIO para los pulsadores del reconocimiento de falla por temperatura, corriente
    //
    EALLOW;
    GpioCtrlRegs.GPDMUX1.bit.GPIO97 = 0;         // GPIO
    GpioCtrlRegs.GPDDIR.bit.GPIO97 = 0;          // como entrada CLR_FAULT_OCP
    GpioCtrlRegs.GPDPUD.bit.GPIO97 = 0;         // R PULL UP
    GpioCtrlRegs.GPDQSEL1.bit.GPIO97 = 3;        // entrada asincronica
    //GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;        // inicio en bajo

    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;         // GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO58 = 0;          // como entrada ENABLE_OTP
    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3;        // entrada asincronica
    //GpioDataRegs.GPBCLEAR.bit.GPIO58 = 1;        // inicio en bajo
    EDIS;
    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////OUTPUTS/////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////CLR_FAULT_OTP_1/////////////////////////////////////
    // Cuando hay una sobre temperatura se debe interrumpir el mcu, reconocerla con el pulsador y dar la orden desde el mcu que bajo
    EALLOW;
    GpioCtrlRegs.GPCDIR.bit.GPIO94 = 1;          // GPIO94 = output
    GpioCtrlRegs.GPCPUD.bit.GPIO94 = 0;          // Enable pullup on GPIO94
    GpioCtrlRegs.GPCQSEL2.bit.GPIO94 = 3;        // Salida asincrónica
    GpioCtrlRegs.GPCMUX2.bit.GPIO94 = 0;         // GPIO = GPIO94
    GpioDataRegs.GPCSET.bit.GPIO94 = 0;          // inicio en bajo - estaba en alto
    EDIS;
    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////EN_DIS_M1///////////////////////////////////////////
    EALLOW;
    GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;          // GPIO61 = output
    GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;          // Enable pullup on GPIO61
    GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3;        // Salida asincrónica
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;         // GPIO = GPIO61
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;          // inicio en alto (ver bien esto, porque si es pull-up y esta en 1 al incio capaz que hay un cero a la salida)
    EDIS;
    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////EN_DIS_M2///////////////////////////////////////////
    EALLOW;
    GpioCtrlRegs.GPDDIR.bit.GPIO123 = 1;          // GPIO123 = output
    GpioCtrlRegs.GPDPUD.bit.GPIO123 = 0;          // Enable pullup on GPIO123
    GpioCtrlRegs.GPDQSEL2.bit.GPIO123 = 3;        // Salida asincrónica
    GpioCtrlRegs.GPDMUX2.bit.GPIO123 = 0;         // GPIO = GPIO123
    GpioDataRegs.GPDSET.bit.GPIO123 = 1;          // inicio en alto (ver bien esto, porque si es pull-up y esta en 1 al incio capaz que hay un cero a la salida)
    EDIS;
    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////EN_DIS_M3///////////////////////////////////////////
    EALLOW;
    GpioCtrlRegs.GPDDIR.bit.GPIO122 = 1;          // GPIO122 = output
    GpioCtrlRegs.GPDPUD.bit.GPIO122 = 0;          // Enable pullup on GPIO122
    GpioCtrlRegs.GPDQSEL2.bit.GPIO122 = 3;        // Salida asincrónica
    GpioCtrlRegs.GPDMUX2.bit.GPIO122 = 0;         // GPIO = GPIO122
    GpioDataRegs.GPDSET.bit.GPIO122 = 1;          // inicio en alto (ver bien esto, porque si es pull-up y esta en 1 al incio capaz que hay un cero a la salida)
    EDIS;
}






