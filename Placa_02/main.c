//###########################################################################
//
// FILE:   adc_soc_epwm_cpu01.c
//
// TITLE:   Disparo de ADC via epwm para F2837xD modificado para ejecutar 3 PIDs independientes

//###########################################################################
//
// Archivos de inclusión
//
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
#include "Entradas_Salidas.h"
#include "F2837xD_input_xbar.h"
#include "F2837xD_SWPrioritizedIsrLevels.h"
#include "SetParametrosPID.h"
#include "Comunicaciones.h"
#include "Homming.h"

#define SE_SIGN  (GpioDataRegs.GPBDAT.bit.GPIO59)       // Valor en tiempo real de la XINT de la Parada de Emergencia
#define OCP_SIGN  (GpioDataRegs.GPCDAT.bit.GPIO95)      // Valor en tiempo real de la XINT de la Sobre Corriente
#define OTP_SIGN  (GpioDataRegs.GPBDAT.bit.GPIO52)      // Valor en tiempo real de la XINT de la Sobre Temperatura
#define ENABLE_OTP (GpioDataRegs.GPBDAT.bit.GPIO58)     // Valor en tiempo real del pulsador reconocimiento OTP
#define ACK_OCP (GpioDataRegs.GPDDAT.bit.GPIO97)        // Valor en tiempo real del pulsador de reconocimiento de OCP
//
// Funciones prototipo
//
interrupt void adca1_isr(void);
interrupt void xint_SE_isr(void);
interrupt void xint_OCP_isr(void);
interrupt void xint_OTP_isr(void);
interrupt void timer_gripper_isr(void); // Interrupción para la apertura/cierre del gripper
//
// Variables globales
//
volatile int32 doSample = 0;
volatile int32 doLoop = 0;
volatile int HOME_SW_1 = 0, HOME_SW_2 = 0, HOME_SW_3 = 0, SE_IN_ISR=0, OCP_IN_ISR=0, OTP_IN_ISR=0, TEMP_1=0, TEMP_2=0, TEMP_3=0, CLEAR_OTP=0;
volatile int se_ack, ocp_ack, otp_ack;
volatile int32 Modo = NORMAL;
volatile int init = 1;
volatile int init2 = 0;
volatile int init3 = 0;

extern volatile int mov_gripper;
const int segundos_gripper = 4;

// Frecuencia periféricos - No def - Funciones Ram
void main(void)
{
    //
    // Inicializo el sistema de control
    //
    InitSysCtrl();
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;
    EDIS;
    //
    // Inicializo las GPIOs
    //
    InitGpio();
    InitEQep1Gpio();
    InitEQep2Gpio();
    InitEQep3Gpio();
    InitEPwm1Gpio();
    InitEPwm2Gpio();                                            // Configura el GPIO como EPWM2 --> pwm del motor 2
    InitEPwm3Gpio();                                            // Configura el GPIO como EPWM3 --> pwm del motor 3
    InitEPwm4Gpio();                                            // Configura el GPIO como EPWM4 --> pwm del motor 1
    InitComunicacionesGPIO();                                   // Configura el GPIO como RX y TX del SCIB
    Entradas_Salidas_init();
    //
    // Borra todas las interrupciones e inicializa la PIE vector table
    //
    DINT;                                                       // Deshabilita las interrupciones
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
    //
    // Mapea las funciones ISR -- esto trae problemas si no lo comento y quiero probar por partes
    //
    EALLOW;                                                     // Pide permiso para modificar registro
    PieVectTable.ADCA1_INT = &adca1_isr;                        // INTERRUPCION 1 DE GRUPO 1 - PRIORIDAD 5 EN CORE - PRIORIDAD 1 EN GRUPO 1 - MAS PRIORIDAD
    PieVectTable.XINT1_INT = &xint_SE_isr;                      // INTERRUPCION 4 DE GRUPO 1 - PRIORIDAD 5 EN CORE - PRIORIDAD 4 EN GRUPO 1
    PieVectTable.XINT2_INT = &xint_OCP_isr;                     // INTERRUPCION 5 DE GRUPO 1 - PRIORIDAD 5 EN CORE - PRIORIDAD 5 EN GRUPO 1
    PieVectTable.XINT3_INT = &xint_OTP_isr;                     // INTERRUPCION 1 DE GRUPO 12 - PRIORIDAD 5 EN CORE - PRIORIDAD 1 EN GRUPO 12 - MENOS PRIORIDAD
    PieVectTable.TIMER0_INT = &timer_gripper_isr;               // INTERRUPCION 7 DE GRUPO 1 - PRIORIDAD 5 EN CORE - PRIORIDAD 7 EN GRUPO 1
    EDIS;                                                       // Cierra el permiso
    //
    // Configura el ADC y lo enciende
    //
    ConfigureADC();     // Coincide
    //
    // Configura el ePWM
    //
    initEpwm();                                                 // Esta funcion está en Example_EPwmSetup.c
    InitEPwm2(Period, phaseOffset2, Duty2);
    InitEPwm3(Period, phaseOffset3, Duty3);
    InitEPwm4(Period, phaseOffset1, Duty1);
    InitEPwm5();        // ePWM5 para disparar ADCA1 -- Coincide
    //
    // Configura el ADC para que sea disparado por ePWM
    //
    SetupADCEpwm();
    //
    // Inicializo y configuro el timer que cuenta el tiempo de apertura del gripper
    //
    InitCpuTimers();
    #ifdef _LAUNCHXL_F28379D
        ConfigCpuTimer(&CpuTimer0, 200, segundos_gripper*1000000); //200 MHz, segundos * 1.000.000 us
    #else
        ConfigCpuTimer(&CpuTimer0, 100, segundos_gripper*1000000); //200 MHz, segundos * 1.000.000 us
    #endif
    CpuTimer0Regs.TCR.all = 0x4001;
    //
    // Enable TINT0 in the PIE: Group 1 __interrupt 7
    //
       PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    IER |= M_INT1;                                              // Habilita CPU INT1
    IER |= M_INT12;                                             // Habilita CPU INT12
    EINT;
    ERTM;
    //
    // Inicializo los 3 modulo eQEP
    //
    qep_posspeed_1.init(&qep_posspeed_1);
    qep_posspeed_2.init(&qep_posspeed_2);
    qep_posspeed_3.init(&qep_posspeed_3);
    //                                                       // Habilita interrupcion global INTM
    //ERTM;                                                  // Habilita interrupcion en tiempo real global DBGM
    //
    // Habilita INTn de ePWM en el PIE y la ISR del ADC1: Grupo 1
    // Habilita las interrupciones externas XINT1 (Grupo 12 interrupcion 1), XINT2 y XINT3 (Grupo 1 interrupcion 2 y 3) en el PIE
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;                        // Habilita ADCA1_INT - INTERRUPCION 1 DE GRUPO 1
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;                          // Habilita XINT1 - INTERRUPCION 4 DE GRUPO 1 - SE
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;                          // Habilita XINT2 - INTERRUPCION 5 DE GRUPO 1 - OCP
    PieCtrlRegs.PIEIER12.bit.INTx1 = 1;                         // Habilita XINT3 - INTERRUPCION 1 DE GRUPO 12 - OTP
    //
    // GPIO59 es XINT1, GPIO95 es XINT2 y GPIO52 es XINT3
    //
    GPIO_SetupXINT1Gpio(59);                                    // SE
    GPIO_SetupXINT2Gpio(95);                                    // OCP
    GPIO_SetupXINT3Gpio(52);                                    // OTP
    //
    // CONFIGURA POLARIDADES
    XintRegs.XINT1CR.bit.POLARITY = 1;  //FLANCO ASCENDENTE DE ACTIVACION - SE
    XintRegs.XINT2CR.bit.POLARITY = 1;  //FLANCO DESCENDENTE DE ACTIVACION - OCP
    XintRegs.XINT3CR.bit.POLARITY = 1;  //FLANCO DESCENDENTE DE ACTIVACION - OTP
    //
    // Habilita XINT1, XINT2 y XINT3 para las protecciones
    //
    XintRegs.XINT1CR.bit.ENABLE = 1;                            // Habilita XINT1
    XintRegs.XINT2CR.bit.ENABLE = 1;                            // Habilita XINT2
    XintRegs.XINT3CR.bit.ENABLE = 1;                            // Habilita XINT3
    //
    // Sincroniza el ePWM
    //
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
    //
    // Parametros del PID
    //
    Set_Parametros_PID();
    //
    // Comunicaciones via UART/SCI
    Init_Comunicaciones();
    //
    for(;;){

        // Empieza ePWM para disparar ADC A, B y C
        //
        EPwm5Regs.ETSEL.bit.SOCAEN = 1;                             // Habilita SOCA -- ESTE DEBE ACTIVA TODOS
        EPwm5Regs.TBCTL.bit.CTRMODE = 0;                            // Descongela e ingresa al modo de cuenta ascendente
        //
        //GpioDataRegs.GPBDAT.bit.GPIO59 = 0;                     // Baja el pin GPIO59 --> flag para controlar el tiempo de ejecucion

        //////////////////////////////////////////////////VARIABLES UTILES/////////////////////////////////////////////////////////////////
        //
        HOME_SW_1 = GpioDataRegs.GPBDAT.bit.GPIO32;
        HOME_SW_2 = GpioDataRegs.GPDDAT.bit.GPIO111;
        HOME_SW_3 = GpioDataRegs.GPCDAT.bit.GPIO67;
        //
        TEMP_1 = GpioDataRegs.GPCDAT.bit.GPIO66;
        TEMP_2 = GpioDataRegs.GPEDAT.bit.GPIO131;
        TEMP_3 = GpioDataRegs.GPEDAT.bit.GPIO130;
        //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////En esta parte gestiono el muestreo y el control que va a depender de la interrupcion del ADCA1/////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        Comunicaciones();

        if (doSample){
            Muestreo();
            doSample = 0;
            //if(ComprobarLimites() || SE_SIGN || OCP_SIGN || OTP_SIGN)         // Todas las protecciones activadas - si no hay control de corriente va a saltar siempre
            if (ComprobarLimites() || SE_SIGN )
            {
                    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;           // Baja el pin PWM GPIO61 -- Frenan todos los motores -- La señal ePWM entra al pin DIR de cada driver
                    GpioDataRegs.GPDCLEAR.bit.GPIO123 = 1;          // Baja el pin PWM GPIO123
                    GpioDataRegs.GPDCLEAR.bit.GPIO122 = 1;          // Baja el pin PWM GPIO122
                    Reset_PWM();                                    // Resetea las salidas PWM al 50% --> 0V de tension media aplicada a bornes de cada motor
                    Modo = PROTECCION_ON;
            }
            else
            {
                    GpioDataRegs.GPBSET.bit.GPIO61 = 1;             // Sube el pin PWM GPIO61
                    GpioDataRegs.GPDSET.bit.GPIO123 = 1;            // Sube el pin PWM GPIO123
                    GpioDataRegs.GPDSET.bit.GPIO122 = 1;            // Sube el pin PWM GPIO122
                    Modo = NORMAL;
            }
        }

        if (doLoop){
                switch(Modo){
                case NORMAL:
                    Control();
                    Set_Salidas();
                    if(!init){ // Si esta activada la inicialización
                        init = homming_placa2(); // homming() devuelve 1 si se terminó de inicializar
                    }

                    break;
                case PROTECCION_ON:
                    break;
                }

                doLoop = 0;
        }
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////Restablecimiento de fallas////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        if (ACK_OCP == 0) OCP_IN_ISR = 0;    // Si se presiona pulsador de reconocimiento de OCP pone bandera OCP_IN_ISR en cero

        if (ENABLE_OTP == 0) {               // Pulsador de reconocimiento de falla por OTP
            //
            OTP_IN_ISR = 0;                 // Borro bandera de ingresa a OTP_isr
            //
            // Generacion de pulso de 0 - 1 para resetear la proteccion de OTP
            //
            GpioDataRegs.GPCSET.bit.GPIO94 = 1;           // Pone un 1 en la salida CLR_FAULT_OTP
            DELAY_US(1000000/15);                         // Espera para luego bajar la salida
            GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;         // Pone un 0 en la salida CLR_FAULT_OTP y lo deja
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////INTERRPCIONES///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////adca1_isr//////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
interrupt void adca1_isr(void){
    //
    // Cada vez que interrumpe el ADCA1 a la frecuencia que le impone el EWPM5 entra aqui
    // Aqui adentro le damos la orden de muestrear (doSample) y hacer el control (doLoop)
    // Esta ISR es la mas prioritaria por defecto y no se puede cambiar
    //
    doSample=1;                                     // Da la orden para muestrear
    doLoop=1;                                       // Da la orden para hacer el lazo de control

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;          // Borra la INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;         // Reconoce las interrupciones del GRUPO 1
}
/////////////////////////////////////////////////////////////////////
///////////////////////////////SE_isr////////////////////////////////
/////////////////////////////////////////////////////////////////////
void xint_SE_isr(void){
    //
    // Cada vez que se presiona la parada de emergencia entra aqui y reinicia los PWMs al 50%
    // Una vez que se restablece la parada de emergencia el funcionamiento continua normalmente
    //
    SE_IN_ISR = 3;                                  // Bandera de ingreso a la ISR
    //
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;           // Baja el pin PWM GPIO61
    GpioDataRegs.GPDCLEAR.bit.GPIO123 = 1;          // Baja el pin PWM GPIO123
    GpioDataRegs.GPDCLEAR.bit.GPIO122 = 1;          // Baja el pin PWM GPIO122
    Reset_PWM();                                    // Pone los tres DUTY en el 50%
    //
    SE_IN_ISR = 4;                                  // Se reconoce ISR
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    XintRegs.XINT1CR.bit.ENABLE = 1;                 //Habilita XINT1
}
/////////////////////////////////////////////////////////////////////
////////////////////////////////OCP_isr//////////////////////////////
/////////////////////////////////////////////////////////////////////
void xint_OCP_isr(void){
    //
    // Cada vez que se detecta sobrecorriente en alguno o varios drives entra aqui y reinicia los PWMs al 50%
    // Una vez que se restablece la falla con el pulsador el funcionamiento continua normalmente
    //
    OCP_IN_ISR = 1;                                 // Bandera de ingreso a la ISR
    //
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;           // Baja el pin PWM GPIO61
    GpioDataRegs.GPDCLEAR.bit.GPIO123 = 1;          // Baja el pin PWM GPIO123
    GpioDataRegs.GPDCLEAR.bit.GPIO122 = 1;          // Baja el pin PWM GPIO122
    Reset_PWM();                                    // Pone los tres DUTY en el 50%
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    XintRegs.XINT2CR.bit.ENABLE = 1;                //Habilita XINT2
}
/////////////////////////////////////////////////////////////////////
/////////////////////////////////OTP_isr/////////////////////////////
/////////////////////////////////////////////////////////////////////
void xint_OTP_isr(void){
    //
    // Cada vez que se detecta sobretemperatura en alguno o varios drives entra aqui y reinicia los PWMs al 50%
    // Una vez que baja la temperatura y se restablece la falla con el pulsador el funcionamiento continua normalmente
    // Para que se pueda usar el pulsador la temperatura tiene que haber bajado
    //
    OTP_IN_ISR = 1;                                 // Bandera de ingreso a la ISR
    //
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;           // Baja el pin PWM GPIO61
    GpioDataRegs.GPDCLEAR.bit.GPIO123 = 1;          // Baja el pin PWM GPIO123
    GpioDataRegs.GPDCLEAR.bit.GPIO122 = 1;          // Baja el pin PWM GPIO122
    Reset_PWM();                                    // Pone los tres DUTY en el 50%
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    XintRegs.XINT3CR.bit.ENABLE = 1;                //Habilita XINT3
}
/////////////////////////////////////////////////////////////////////
/////////////////////////////////Timer_isr/////////////////////////////
/////////////////////////////////////////////////////////////////////
__interrupt void timer_gripper_isr(void)
{
   CpuTimer0.InterruptCount++;
   mov_gripper = 0;
   CpuTimer0.RegsAddr->TCR.bit.TSS = 1;     // 1 = Stop timer, 0 = Start/Restart Timer
   CpuTimer0.RegsAddr->TCR.bit.TRB = 1;     // 1 = reload timer

   //
   // Acknowledge this __interrupt to receive more __interrupts from group 1
   //
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
/////////////////////////////////////////////////////////////////////
// End of file
//

