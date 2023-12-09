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
#include "F2837xD_input_xbar.h"

volatile limites limitesProteccion = {IMAX_1,IMAX_2,IMAX_3};
volatile flags flagsProtecciones = {0,0,0,0,0,0};

int ComprobarLimites(void){
	if(corriente_real_1 > limitesProteccion.I1Max) {flagsProtecciones.I1 = 1; return 1;}
	else flagsProtecciones.I1 = 0;

	if(corriente_real_2 > limitesProteccion.I2Max) {flagsProtecciones.I2 = 1; return 1;}
	else flagsProtecciones.I2 = 0;

	if(corriente_real_3 > limitesProteccion.I3Max) {flagsProtecciones.I3 = 1; return 1;}
	else flagsProtecciones.I3 = 0;

	if(limite_temperatura_1 != 1) {flagsProtecciones.T1 = 1; return 1;}
	else flagsProtecciones.T1 = 0;

    if(limite_temperatura_2 != 1) {flagsProtecciones.T2 = 1; return 1;}
    else flagsProtecciones.T1 = 0;

    if(limite_temperatura_3 != 1) {flagsProtecciones.T3 = 1; return 1;}
    else flagsProtecciones.T1 = 0;

	return 0;
}

void Reset_PWM(void){
    Duty1 = PWM_CMPR50;
    Duty2 = PWM_CMPR50;
    Duty3 = PWM_CMPR50;

    Set_Salidas();
}
