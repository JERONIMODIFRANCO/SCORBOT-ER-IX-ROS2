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
     /////////////////////////////SEGUNDA PLACA/////////////////////////////////////////////////////
     //
     //PID1
     //
     pid1.Kp = 15;

     pid1.Ki = 1;

     pid1.Kd = 0;

     pid1.c1 = 1;

     pid1.c2 = 1;

     pid1.Umax = 380;     // Saturacion positiva del PID

     pid1.Umin = -380;    // Saturacion negativa del PID

     //PID2
     //
     pid2.Kp = 15;

     pid2.Ki = 1;

     pid2.Kd = 0;

     pid2.c1 = 1;

     pid2.c2 = 1;

     pid2.Umax = 380;     // Saturacion positiva del PID

     pid2.Umin = -380;    // Saturacion negativa del PID

//    ///////////////////////////////PARAMETROS PID - CORRIENTE/////////////////////////////////////////
      //
//      /////////////////////////////////SEGUNDA PLACA//////////////////////////////////////////////////////
//      //
//      // PID 4
//      //
//      pid4.Kp = 100;
//
//      pid4.Ki = 1;
//
//      pid4.Kd = 0;
//
//      pid4.c1 = 1;
//
//      pid4.c2 = 1;
//
//      pid4.Umax = 60;     // Saturacion positiva del PID
//
//      pid4.Umin = -60;    // Saturacion negativa del PID
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
//      pid5.Umax = 60;     // Saturacion positiva del PID
//
//      pid5.Umin = -60;    // Saturacion negativa del PID
}



