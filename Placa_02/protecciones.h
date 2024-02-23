#ifndef INC_PROTECCIONES_H_
#define INC_PROTECCIONES_H_

#define IMAX_1 2.5
#define IMAX_2 2.5
#define IMAX_3 2.5

typedef struct{
	float I1Max;
	float I2Max;
	float I3Max;
}limites;

typedef struct{
    int I1;
    int I2;
    int I3;
    int T1;
    int T2;
    int T3;
}flags;

#define NORMAL 0
#define PROTECCION_ON 1
#define PROTEGIDO 1

extern volatile limites limitesProteccion;

int ComprobarLimites(void);
void Reset_PWM(void);

#endif /* INC_PROTECCIONES_H_ */
