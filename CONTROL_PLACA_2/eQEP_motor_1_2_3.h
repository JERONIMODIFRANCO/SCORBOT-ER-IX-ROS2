/*
 * eQEP_motor_1_2_3.h
 *
 *  Created on: 18 may. 2020
 *      Author: FACU
 */

#ifndef EQEP_MOTOR_1_2_3_H_
#define EQEP_MOTOR_1_2_3_H_

// Included Files
//
#include "IQmathLib.h"

//
// Defines
//
#define POSSPEED_1_DEFAULTS {0x0,0x0, 0x0,0x0,0x0,0x0,8589.312,0x0,1,0,0x0,\
                          370,0,1000,0,\
                           0,0,0,\
                           (void (*)(long))POSSPEED_1_Init,\
                          (void (*)(long))POSSPEED_1_Calc }

#define POSSPEED_2_DEFAULTS {0x0,0x0, 0x0,0x0,0x0,0x0,8589.312,0x0,1,0,0x0,\
                          370,0,1000,0,\
                           0,0,0,\
                           (void (*)(long))POSSPEED_2_Init,\
                          (void (*)(long))POSSPEED_2_Calc }

extern volatile Naxis4 = 180, Naxis5 = 100;

//
// Globals
//
typedef struct {//int theta_elec;         // Output: Motor Electrical angle (Q15)
                int32 theta_elec;
                float angle_robot;
                int theta_mech;         // Output: Motor Mechanical Angle (Q15)
                int DirectionQep;       // Output: Motor rotation direction (Q0)
                int QEP_cnt_idx;        // Variable: Encoder counter index (Q0)
                //int theta_raw;          // Variable: Raw angle from Timer 2 (Q0)
                Uint32 theta_raw;

                int mech_scaler;        // Parameter: 0.9999/total count, total
                                        // count = 4000 (Q26)
                int theta_raw1;
                int pole_pairs;         // Parameter: Number of pole pairs (Q0)
                Uint32 cal_angle;
                //int cal_angle;          // Parameter: Raw angular offset
                                        // between encoder and phase a (Q0)
                int index_sync_flag;    // Output: Index sync status (Q0)

                Uint32 SpeedScaler;     // Parameter :  Scaler converting 1/N
                                        // cycles to a GLOBAL_Q speed (Q0) -
                                        // independently with global Q
                _iq Speed_pr;           // Output :  speed in per-unit
                Uint32 BaseRpm;         // Parameter : Scaler converting
                                        // GLOBAL_Q speed to rpm (Q0) speed -
                                        // independently with global Q
                int32 SpeedRpm_pr;      // Output : speed in r.p.m. (Q0) -
                                        // independently with global Q
                _iq  oldpos;            // Input: Electrical angle (pu)
                _iq Speed_fr;           // Output :  speed in per-unit
                int32 SpeedRpm_fr;      // Output : Speed in rpm  (Q0) -
                                        // independently with global Q
                void (*init)();         // Pointer to the init funcion
                void (*calc)();         // Pointer to the calc funtion
                }  POSSPEED;

typedef POSSPEED *POSSPEED_1_handle;
typedef POSSPEED *POSSPEED_2_handle;
//
// Function Prototypes
//
void POSSPEED_1_Init(void);
void POSSPEED_1_Calc(POSSPEED_1_handle);

void POSSPEED_2_Init(void);
void POSSPEED_2_Calc(POSSPEED_2_handle);

//
//
#endif /* EQEP_MOTOR_1_2_3_H_ */
