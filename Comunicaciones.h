/*
 * MEFs.h
 *
 *  Created on: 17 dic. 2023
 *      Author: gasti
 */

#ifndef MEFS_H_
#define MEFS_H_

#include "F28x_Project.h"




//---------------- MEF Acción ----------------
typedef enum {
    Reset = 0,
    Reposo,
    Homming,
    Trans_HS,
    Trans_Temp,
    Trans_Corr,
    Trans_Pos,
    Almacenar_ref,
}Enum_est_Accion;

//---------------- MEF Recepción ----------------
typedef enum
{
    reposo = 0,
    comando,
    J1B1,
    J1B2,
    J2B1,
    J2B2,
    J3B1,
    J3B2,
}Enum_est_Recep;

typedef enum
{
    Nada = 0,
    Inicializar,
    Home_switch,
    Temperaturas,
    Corrientes,
    Posiciones,
    Referencias,
}Comando_recibido_enum;

//Los siguientes bits los voy a leer únicamente si llega un R
#define b0(x)         (x == 'I' || x == 'H' || x == 'T' || x == 'C' || x == 'P' || x == 'R') //Comandos

//extern int instruccion;        // Frame check flag.
/*==================[MEFs]==========================*/

//---------------- MEF recepción ----------------
void MEF_Recepcion(void);

//---------------- MEF accion -------------------
void MEF_Transmision(void);


//---------------- Init comunicaciones y GPIO-------------------
void InitComunicacionesGPIO(void);
void Init_Comunicaciones(void);

//---------------- Función global -------------------
void Comunicaciones(void);

void scib_xmit(int a);



#endif /* MEFS_H_ */
