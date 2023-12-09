/*
 * muestreo.c
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

volatile POSSPEED qep_posspeed_1 = POSSPEED_1_DEFAULTS; // SE USA EN eQEP
volatile POSSPEED qep_posspeed_2 = POSSPEED_2_DEFAULTS;
volatile POSSPEED qep_posspeed_3 = POSSPEED_3_DEFAULTS;
volatile Uint16 corriente_medida_1;
volatile Uint16 corriente_medida_2;
volatile Uint16 corriente_medida_3;
volatile float corriente_real_1, corriente_real_2,corriente_real_3;
volatile Uint16 limite_temperatura_1;
volatile Uint16 limite_temperatura_2;
volatile Uint16 limite_temperatura_3;
volatile int32 Angulo_grados_eje_1 = 0, Angulo_grados_eje_2 = 0, Angulo_grados_eje_3 = 0;
volatile float aux_POS_1 = 0, aux_POS_2 = 0, aux_POS_3 = 0;
volatile Uint16 ref_entrada_1 = 0, ref_entrada_2 = 0, ref_entrada_3 = 0;
volatile int16 SP_REF_1 = 0, SP_REF_2 = 0, SP_REF_3 = 0;
//
/////////////Funcion para hacer el muestreo/////////////////////////////////////
//
void Muestreo(void){

    // Lee las corrientes con ADC channel A, B and C
    //
    corriente_medida_1 = AdcaResultRegs.ADCRESULT0;                             // Las corrientes sirven solamente para activar las protecciones. No forman parte del PI de posicion
    corriente_medida_2 = AdcbResultRegs.ADCRESULT0;
    corriente_medida_3 = AdccResultRegs.ADCRESULT0;
    //
    // Corrientes reales por los drivers [A]
    //
    corriente_real_1 = corriente_medida_1*0.001465f;                            // Relacion lineal --> 6/4095 = 0.001465
    corriente_real_2 = corriente_medida_2*0.001465f;
    corriente_real_3 = corriente_medida_3*0.001465f;
    //
    // Detecta el exceso de temperatura
    //
    limite_temperatura_1 = GpioDataRegs.GPCDAT.bit.GPIO66;                      // En 1 -> funcionamiento normal. En 0 -> sobretemperatura
    limite_temperatura_2 = GpioDataRegs.GPEDAT.bit.GPIO131;
    limite_temperatura_3 = GpioDataRegs.GPEDAT.bit.GPIO130;
    //
    // Referencias de posicion que entran por los ADCs 0-4095
    //
    ref_entrada_1 = AdcaResultRegs.ADCRESULT1;
    ref_entrada_2 = AdcbResultRegs.ADCRESULT1;
    ref_entrada_3 = AdccResultRegs.ADCRESULT1;
    //
    // Referencias de posicion que entran por los ADCs en GRADOS - Conversion de sin signo (flotante) a con signo (entero)
    //
    //////////////////////////////////////////////////////////////////////
    ////////////////////////REFERENCIAS POR POTENCIOMETRO/////////////////
    //////////////////////////////////////////////////////////////////////
    ////////////////////////Primera Placa/////////////////////////////////
    //////////////////////////////////////////////////////////////////////
    //
//    // Espacio de trabajo eje 1 - 270�
//    //
//    aux_POS_1 = 0.065918f*ref_entrada_1-135;      //Flotante --> sin signo
//
//    SP_REF_1 = aux_POS_1;                         //Entero --> con signo
//    //
    // Espacio de trabajo eje 2 - 145�
    //
//    aux_POS_2 = 0.0354f*ref_entrada_2-72.5f;
//
//    SP_REF_2 = aux_POS_2;
////    //
    // Espacio de trabajo eje 3 - 210�
    //
//    aux_POS_3 = 0.05127f*ref_entrada_3-105;
//
//    SP_REF_3 = aux_POS_3;
//
///////////////////////////////////////////////////////////
    // Lee las variables que me dan los eQEP
    //
    qep_posspeed_1.calc(&qep_posspeed_1);                                       // Calcula todas las variables de la estructura posspeed de QEP
    qep_posspeed_2.calc(&qep_posspeed_2);
    qep_posspeed_3.calc(&qep_posspeed_3);
    //
    // Posiciones de los ejes 1, 2 y 3 en este momento en GRADOS
    //
    Angulo_grados_eje_1 = qep_posspeed_1.angle_robot;                          // Las posiciones son las variables a controlar con el PID de POSICION
    Angulo_grados_eje_2 = qep_posspeed_2.angle_robot;
    Angulo_grados_eje_3 = qep_posspeed_3.angle_robot;
}
