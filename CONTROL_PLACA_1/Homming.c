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

typedef mef{
    int inicializacion = 0;
    int en_home; 
    int aprox_der;
};


void homming_junta(int *qep_posspeed_x, int HOME_SWITCH, int angle_pos, int angle_neg, int *angulo_x) {
    int estado_mef = inicializacion;
    int delta_angulo = 5;
    if(HOME_SWITCH == 1){
                    estado_mef = en_home;
                }
    switch (estado_mef) {
        case inicializacion:
            if (qep_posspeed_x->angle_robot == *angulo_x) {
                *angulo_x = ((abs(qep_posspeed_x->angle_robot) + delta_angulo)*(-sgn(qep_posspeed_x->angle_robot)));
                if(HOME_SWITCH == 1){
                    estado_mef = en_home;
                    break;
                }
            }
        case en_home: 
            if (sgn(qep_posspeed_x->angle_robot))
            
            
    }
}

    
        

    