#include "Homming.h"


extern volatile float Angulo_grados_eje_1, Angulo_grados_eje_2, Angulo_grados_eje_3;
extern volatile int HOME_SW_1, HOME_SW_2, HOME_SW_3;
extern volatile float angulo_1, angulo_2, angulo_3;



#define tolerancia  0.5

int signo( float num){
    if(num<0){
        return -1;
    }
    else {
        return 1;
    }
}

int homming_junta(volatile float Angulo_grados_eje_x, int HOME_SWITCH, volatile float *angulo_x, int delta_angulo, int ang_fin) {
    static int fin_homming = 0;
    static enum_estados_homming estado_mef = aprox_rapida;

        switch (estado_mef) {

            case aprox_rapida:

                fin_homming = 0;
                if (igual(Angulo_grados_eje_x,*angulo_x,tolerancia)) {
                    if(HOME_SWITCH == 1){
                        estado_mef = salida_home;
                        break;
                    }
                    *angulo_x = ((abs(Angulo_grados_eje_x) + delta_angulo)*(-signo(Angulo_grados_eje_x)));
                 }
                break;

            case salida_home:
                if (igual(Angulo_grados_eje_x,*angulo_x,tolerancia)) {
                    if(HOME_SWITCH == 0){
                        estado_mef = aprox_lenta;
                        break;
                    }
                    *angulo_x = Angulo_grados_eje_x + delta_angulo;
                }
                break;

            case aprox_lenta:
                if (igual(Angulo_grados_eje_x,*angulo_x,tolerancia)) {
                    if(HOME_SWITCH == 1){
                        estado_mef = centrar;
                        *angulo_x = Angulo_grados_eje_x + ang_fin;
                        break;
                    }
                    *angulo_x = Angulo_grados_eje_x - 1; //Chequear si se puede mover menos de 2 grados
                }
                break;

            case centrar:
                if (igual(Angulo_grados_eje_x,*angulo_x,tolerancia)) {
                    fin_homming = 1;
                    estado_mef = aprox_rapida;
                    break;
                }
                break;

        }

    return fin_homming;
}




int homming3_junta(volatile float Angulo_grados_eje_x, int HOME_SWITCH, volatile float *angulo_x, int delta_angulo, int ang_fin) {
    static enum_estados_homming estado_mef = aprox_rapida;
    int angulo_salida;
    static int fin_homming = 0; //cambiar

    switch (estado_mef) {
        case aprox_rapida:
            fin_homming = 0;
            if (igual(Angulo_grados_eje_x,*angulo_x,tolerancia)) {
                if(HOME_SWITCH == 1){
                    estado_mef = salida_home;
                    break;
                }
                *angulo_x = Angulo_grados_eje_x + delta_angulo;
             }
            break;

        case salida_home:
            if (igual(Angulo_grados_eje_x,*angulo_x,tolerancia)) {
                if(HOME_SWITCH == 0){
                    estado_mef = aprox_lenta;
                    break;
                }
                *angulo_x = Angulo_grados_eje_x - delta_angulo;
            }
            break;

        case aprox_lenta:
            if (igual(Angulo_grados_eje_x,*angulo_x,tolerancia)) {
                if(HOME_SWITCH == 1){
                    estado_mef = centrar;
                    angulo_salida = Angulo_grados_eje_x;
                    *angulo_x = angulo_salida - ang_fin;
                    break;
                }
                *angulo_x = Angulo_grados_eje_x + 1 * signo(delta_angulo); //Chequear si se puede mover menos de 2 grados
            }
            break;

        case centrar:
            if (igual(Angulo_grados_eje_x,*angulo_x,tolerancia)) {
                fin_homming = 1;
                estado_mef = aprox_rapida;
            }
            break;

    }
    return fin_homming;
}






int homming_placa2(void){
    static int fin_homming1 = 0, fin_homming2 = 0;
    static int fin_homming = 0;
    static int primera_init = 1;

if(primera_init){
    if (!fin_homming2) {fin_homming2 = homming3_junta( Angulo_grados_eje_2,  HOME_SW_2,  &angulo_2, -5, 0);}
    if (!fin_homming1 & fin_homming2 == 1) {fin_homming1 = homming3_junta( Angulo_grados_eje_1,  HOME_SW_1,  &angulo_1, 5, 98);}

    fin_homming = fin_homming1 * fin_homming2;
    if(fin_homming1 & fin_homming2){
//        fin_homming = 1;
        fin_homming1 = 0;
        fin_homming2 = 0;

        Angle_Reset(1, &angulo_1, 0);
        Angle_Reset(2, &angulo_2, 0);
        Angle_Reset(3, &angulo_3, 0);
        primera_init = 0;
        return fin_homming;
    }
}else{
    angulo_1 = 0;
    angulo_2 = 0;
    angulo_3 = 0;
    return 1;
}

}


