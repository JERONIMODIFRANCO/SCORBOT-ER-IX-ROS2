/*
 * adc_motor_1_2_3.c
 *
 *  Created on: 18 may. 2020
 *      Author: FACU
 */
#include "F28x_Project.h"
#include "adc_motor_1_2_3.h"

void ConfigureADC(void)
{
    EALLOW;
    //
    // ADC-A
    //
    #ifdef _LAUNCHXL_F28379D
        AdcaRegs.ADCCTL2.bit.PRESCALE = 6;      // Set ADCCLK divider to /4
    #else
        AdcaRegs.ADCCTL2.bit.PRESCALE = 2;      // Set ADCCLK divider to /2
    #endif
    AdcaRegs.ADCCTL2.bit.RESOLUTION =  0;       // 12-bit resolution
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0;        // Single-ended channel conversions (12-bit mode only)
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // Power up the ADC
    //
    DELAY_US(1000);                             // Delay for 1ms to allow ADC time to power up
    //
    // ADC-B
    //

    #ifdef _LAUNCHXL_F28379D
        AdcbRegs.ADCCTL2.bit.PRESCALE = 6;      // Set ADCCLK divider to /4
    #else
        AdcbRegs.ADCCTL2.bit.PRESCALE = 2;      // Set ADCCLK divider to /2
    #endif
    AdcbRegs.ADCCTL2.bit.RESOLUTION =  0;       // 12-bit resolution RESOLUTION_12BIT;
    AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0;        // Single-ended channel conversions (12-bit mode only)
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Set pulse positions to late
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // Power up the ADC
    //
    DELAY_US(1000);                             // Delay for 1ms to allow ADC time to power up
    //
    // ADC-C
    //

    #ifdef _LAUNCHXL_F28379D
        AdccRegs.ADCCTL2.bit.PRESCALE = 6;      // Set ADCCLK divider to /4
    #else
        AdccRegs.ADCCTL2.bit.PRESCALE = 2;      // Set ADCCLK divider to /2
    #endif
    AdccRegs.ADCCTL2.bit.RESOLUTION =  0;       // 12-bit resolution RESOLUTION_12BIT;
    AdccRegs.ADCCTL2.bit.SIGNALMODE = 0;        // Single-ended channel conversions (12-bit mode only)
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Set pulse positions to late
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // Power up the ADC
    EDIS;
    //
    DELAY_US(1000);                             // Delay for 1ms to allow ADC time to power up
}
//
void SetupADCEpwm(void)
{
    EALLOW;
    //
    // Configuracion ADC-A0
    //
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;          // SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;         // Sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;       // Trigger on ePWM5, ADCSOCA/C
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;      // End of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;        // Enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Make sure INT1 flag is cleared
    //
    // Configuracion ADC-B2
    //
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;          // SOC0 will convert pin B2
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14;         // Sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 13;       // Trigger on ePWM5, ADCSOCA/C
    //
    // Configuracion ADC-C2
    //
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;          // SOC0 will convert pin C2
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 14;         // Sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 13;       // Trigger on ePWM5, ADCSOCA/C
    //
    // Configuracion ADC-A3
    //
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;          // SOC1 will convert pin A3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14;         // Sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 13;       // Trigger on ePWM5, ADCSOCA/C
    //
    // Configuracion ADC-B3
    //
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3;          // SOC1 will convert pin B3
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 14;         // Sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 13;       // Trigger on ePWM5, ADCSOCA/C
    //
    // Configuracion ADC-C3
    //
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3;          // SOC1 will convert pin C3
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 14;         // Sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 13;       // Trigger on ePWM5, ADCSOCA/C
    //
    EDIS;

}

