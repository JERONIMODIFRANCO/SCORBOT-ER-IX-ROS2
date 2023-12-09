/*
 * control.c
 *
 *  Created on: 19 may. 2020
 *      Author: FACUNDO
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
//
#define PWM_PERIOD 760                              // PWM1 frequency = 33kHz
#define PWM_CMPR50 380                              // PWM1 initial duty cycle = 50% --> motores en reposo
//
int16 rk1 = 0.0f;                                   // Referencia en radianes
int16 rk2 = 0.0f;
int16 rk3 = 0.0f;
int16 yk1 = 0, yk2 = 0, yk3 = 0, e1 = 0, e2 = 0, e3 = 0;
float uk1, uk2, uk3, uz1, uz2, uz3;
volatile Naxis4 = 180, Naxis5 = 100, Naxis6 = 1;
volatile float Period = PWM_PERIOD;                 // Periodo PWM salidas
volatile float Duty1 = PWM_CMPR50;                  // Ciclo de trabajo PWM motor 1
volatile float Duty2 = PWM_CMPR50;                  // Ciclo de trabajo PWM motor 2
volatile float Duty3 = PWM_CMPR50;                  // Ciclo de trabajo PWM motor 3
volatile Uint16 phaseOffset1 = 0;                   // PWM1 phase offset = 0
volatile Uint16 phaseOffset2 = 0;                   // PWM2 phase offset = 0
volatile Uint16 phaseOffset3 = 0;                   // PWM3 phase offset = 0
volatile int16 angulo_1=0;
volatile int16 angulo_2=0;
volatile int16 angulo_3=0;
volatile DCL_PID pid1 = PID_1_DEFAULTS; //POSICION LOS PRIMEROS 3 
volatile DCL_PID pid2 = PID_2_DEFAULTS;
volatile DCL_PID pid3 = PID_3_DEFAULTS; // GRIPPER
volatile DCL_PID pid4 = PID_4_DEFAULTS;// CORRIENTE LOS 3 RESTANTES
volatile DCL_PID pid5 = PID_5_DEFAULTS;
volatile Uint16 Abrir = 0;
volatile Uint16 Cerrar = 0;
volatile float corriente_real_con_signo_1 = 0, corriente_real_con_signo_2 = 0, corriente_real_con_signo_3 = 0;
volatile Uint16 rutina = 0;
//
/////////////Funcion para ejecutar el control////////////
void Control(void){
        //
        // Lectura de posicion actual -- en Grados
        //
        yk1 = Angulo_grados_eje_1;
        yk2 = Angulo_grados_eje_2;
        yk3 = Angulo_grados_eje_3;
        //
        // Lectura de posicion de referencia que ingresa por los ADCs -- en Grados
        //
        //////////////////Entrada por TECLADO - ESCALON////////////////////////
        //
        //RUTINA DE PRUEBA
        //
//        if (rutina != 0){
//            rk1 = -20;
//            rk2 = 0;
//        }
//        else
//        {
//            rk1 = 0;
//            rk2 = 0;
//
//        }

        rk1 = angulo_1*Naxis4;   // Descomentar para usarlo normal
        rk2 = angulo_2*Naxis5;
        rk3 = angulo_3*Naxis6;
        //
        //////////////////Entrada por POTENCIOMENTRO//////////////////
        //
        //rk1 = SP_REF_1;
        //rk2 = SP_REF_2;
        //
        // Error (Solo Lectura - Variable informativa)
        //
        e1 = rk1 - yk1;
        e2 = rk2 - yk2;  
        e3 = rk3 - yk3;
        //
        // Corrientes reales con signo por los drivers [A]
        //
        if (e1 >= 0) corriente_real_con_signo_1 = corriente_real_1;
        else corriente_real_con_signo_1 = -corriente_real_1;

        if (e2 >= 0) corriente_real_con_signo_2 = corriente_real_2;
        else corriente_real_con_signo_2 = -corriente_real_2;

        if (e3 >= 0) corriente_real_con_signo_3 = corriente_real_3;
        else corriente_real_con_signo_3 = -corriente_real_3;
        //
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////CONTROL DE POSICION//////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        // Ejecuta control PID paralelo POSICION
        //
        uk1 = DCL_runPID_C3(&pid1, rk1, yk1, 0.0f)+380;
        uk2 = DCL_runPID_C3(&pid2, rk2, yk2, 0.0f)+380;
        //GRIPPER 
        uk3 = DCL_runPID_C3(&pid3, rk3, yk3, 0.0f)+380;
        //
        ///////////////////SEGUNDA PLACA/////////////////////////
        //
        // Lineas para que no haya oscilacion en los ejes
        //
        if(e1 == 0) Duty1 = PWM_CMPR50;    // Eje 4
        else Duty1 = uk1;

        if(e2 == 0) Duty2 = PWM_CMPR50;    // Eje 5
        else Duty2 = uk2;

        if(e3 == 0) Duty3 = PWM_CMPR50;    // GRIPPER
        else Duty3 = uk3;
        //
        //////////////////GRIPPER - Por TECLADO - SEGUNDA PLACA//////////////
        // ESTO SE REEMPLAZA CON EL PID
        // if (Cerrar != 0){
        // Duty3 = 285;    // -12V aplicados al Gripper
        // EPwm3Regs.CMPA.bit.CMPA = (Uint16) Duty3;
        // DELAY_US(1000000/15);
        // Duty3 = PWM_CMPR50;
        // Cerrar = 0;
        // Abrir = 0;
        // }
        // if (Abrir != 0){
        // Duty3 = 475;    // +12V aplicados al Gripper
        // EPwm3Regs.CMPA.bit.CMPA = (Uint16) Duty3;
        // DELAY_US(1000000/15);
        // Duty3 = PWM_CMPR50;
        // Abrir = 0;
        // Cerrar = 0;
        // }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////CONTROL DE POSICION Y CORRIENTE///////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        // Ejecuta control PID paralelo POSICION y PID paralelo CORRIENTE
        //
        ////////////////////////////SEGUNDA PLACA////////////////////////////
        //
//        uk1 = DCL_runPID_C3(&pid1, rk1, yk1, 0.0f);
//        uz1 = DCL_runPID_C3(&pid4, uk1, corriente_real_con_signo_1, 0.0f)+380;
//        //
//        uk2 = DCL_runPID_C3(&pid2, rk2, yk2, 0.0f);
//        uz2 = DCL_runPID_C3(&pid5, uk2, corriente_real_con_signo_2, 0.0f)+380;
//        //
        // Valor del Duty Cycle adaptado para cada eje para una frecuencia de PWM de 33KHz - POSICION Y CORRIENTE
        //
        //if(e1 == 0) Duty1 = PWM_CMPR50;
        //else Duty1 = uz1;           // Eje 4
        //
        //if(e2 == 0) Duty2 = PWM_CMPR50;
        //else Duty2 = uz2;           // Eje 5
}
