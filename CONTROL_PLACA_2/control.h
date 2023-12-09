/*
 * control.h
 *
 *  Created on: 19 may. 2020
 *      Author: FACUNDO
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#define PWM_PERIOD 760                             // PWM1 frequency = 33kHz
#define PWM_CMPR50 380                             // PWM1 initial duty cycle = 50% --> motores en reposo

extern int16 rk1, rk2, rk3;
extern int16 yk1, yk2, yk3, e1, e2, e3;
extern float uk1, uk2, uk3, uz1, uz2, uz3;
extern volatile float Period;                               // Periodo PWM salidas
extern volatile float Duty1;                                // Ciclo de trabajo PWM motor 1
extern volatile float Duty2;                                // Ciclo de trabajo PWM motor 2
extern volatile float Duty3;                                // Ciclo de trabajo PWM motor 3
extern volatile Uint16 phaseOffset1;                        // PWM1 phase offset = 0
extern volatile Uint16 phaseOffset2;                        // PWM2 phase offset = 0
extern volatile Uint16 phaseOffset3;                        // PWM3 phase offset = 0
extern volatile DCL_PID pid1, pid2, pid3, pid4, pid5;
extern volatile int16 angulo_1, angulo_2, angulo_3;
extern volatile Uint16 Abrir;
extern volatile Uint16 Cerrar;
extern volatile float corriente_real_con_signo_1, corriente_real_con_signo_2, corriente_real_con_signo_3;
extern volatile Uint16 rutina;

void Control(void);


#endif /* CONTROL_H_ */
