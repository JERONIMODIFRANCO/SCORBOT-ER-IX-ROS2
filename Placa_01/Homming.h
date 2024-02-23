/*
 * Homming.h
 *
 *  Created on: 20 dic. 2023
 *      Author: LAC-TELECOM-OLI-02
 */

#ifndef HOMMING_H_
#define HOMMING_H_

#include "F28x_Project.h"
#include "DCLF32_scorbot.h"
#include "DCL_scorbot.h"
#include "eQEP_motor_1_2_3.h"
#include "init_PWM_motor_1_2_3.h"
#include "adc_motor_1_2_3.h"
#include "protecciones.h"
//#include "Example_EPwmSetup.h"
#include "seteo_salidas.h"
#include "control.h"
#include "muestreo.h"
#include <stdlib.h>
#include <math.h>

#define igual(x,y,tolerancia) ((fabs((x) - (y)) <= (tolerancia)) ? 1 : 0)




typedef enum {
    aprox_rapida,
    salida_home,
    aprox_lenta,
    centrar,
} enum_estados_homming;

int homming_junta(volatile float Angulo_grados_eje_x, int HOME_SWITCH, volatile float *angulo_x, int delta_angulo, int ang_fin);
int homming3_junta(volatile float Angulo_grados_eje_x, int HOME_SWITCH, volatile float *angulo_x, int delta_angulo, int ang_fin);
int homming4_junta(volatile float Angulo_grados_eje_x, int HOME_SWITCH, volatile float *angulo_x, int delta_angulo, int ang_fin);
int homming(void);





#endif /* HOMMING_H_ */
