/*
 * SetParametrosPID.c
 *
 *  Created on: 28 mar. 2021
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
#include "SetParametrosPID.h"

 void Set_Parametros_PID(){
     //
     ///////////////////////////////PARAMETROS PID - POSICION/////////////////////////////////////////
     //
     ///////////////////////////////PRIMERA PLACA/////////////////////////////////////////////////////
     //
     //PID1
     //
     pid1.Kp = 10;

     pid1.Ki = 0.0002;

     pid1.Kd = 0;

     pid1.c1 = 1;

     pid1.c2 = 1;

     pid1.Umax = 100;     // Saturacion positiva del PID

     pid1.Umin = -100;    // Saturacion negativa del PID

     //PID2
     //
     pid2.Kp = 10;

     pid2.Ki = 0.0002;

     pid2.Kd = 0;

     pid2.c1 = 1;

     pid2.c2 = 1;

     pid2.Umax = 380;     // Saturacion positiva del PID

     pid2.Umin = -380;    // Saturacion negativa del PID

     //PID3
     //
     pid3.Kp = 12;

     pid3.Ki = 0.0005;

     pid3.Kd = 0;

     pid3.c1 = 1;

     pid3.c2 = 1;

     pid3.Umax = 380;     // Saturacion positiva del PID

     pid3.Umin = -380;    // Saturacion negativa del PID
       //

//    ///////////////////////////////PARAMETROS PID - CORRIENTE/////////////////////////////////////////
      //
//    ///////////////////////////////PRIMERA PLACA//////////////////////////////////////////////////////
//      //
//      // PID 4
//      //
//      pid4.Kp = 1;
//
//      pid4.Ki = 1.6;
//
//      pid4.Kd = 0;
//
//      pid4.c1 = 1;
//
//      pid4.c2 = 1;
//
//      pid4.Umax = 380;     // Saturacion positiva del PID
//
//      pid4.Umin = -380;    // Saturacion negativa del PID
//
//      // PID 5
//      //
//      pid5.Kp = 100;
//
//      pid5.Ki = 1;
//
//      pid5.Kd = 0;
//
//      pid5.c1 = 1;
//
//      pid5.c2 = 1;
//
//      pid5.Umax = 380;     // Saturacion positiva del PID
//
//      pid5.Umin = -380;    // Saturacion negativa del PID
//
//      // PID 6
//      //
//      pid6.Kp = 100;
//
//      pid6.Ki = 1;
//
//      pid6.Kd = 0;
//
//      pid6.c1 = 1;
//
//      pid6.c2 = 1;
//
//      pid6.Umax = 380;     // Saturacion positiva del PID
//
//      pid6.Umin = -380;    // Saturacion negativa del PID
}



