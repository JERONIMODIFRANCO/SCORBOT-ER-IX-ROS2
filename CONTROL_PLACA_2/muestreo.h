/*
 * muestreo.h
 *
 *  Created on: 19 may. 2020
 *      Author: FACUNDO
 */

#ifndef MUESTREO_H_
#define MUESTREO_H_

extern volatile POSSPEED qep_posspeed_1;
extern volatile POSSPEED qep_posspeed_2;
extern volatile Uint16 corriente_medida_1;
extern volatile Uint16 corriente_medida_2;
extern volatile Uint16 corriente_medida_3;
extern volatile Uint16 limite_temperatura_1;
extern volatile Uint16 limite_temperatura_2;
extern volatile Uint16 limite_temperatura_3;
extern volatile int32 Angulo_grados_eje_1, Angulo_grados_eje_2, Angulo_grados_eje_3;
extern volatile Uint16 ref_entrada_1;
extern volatile Uint16 ref_entrada_2;
extern volatile float aux_POS_1, aux_POS_2, aux_POS_3;
extern volatile int16 SP_REF_1, SP_REF_2, SP_REF_3;
extern volatile float corriente_real_1, corriente_real_2,corriente_real_3;

void Muestreo(void);


#endif /* MUESTREO_H_ */
