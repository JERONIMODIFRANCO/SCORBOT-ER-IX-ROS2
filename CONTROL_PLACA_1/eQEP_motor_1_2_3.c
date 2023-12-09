// Included Files
//
#include "F28x_Project.h"
#include "eQEP_motor_1_2_3.h"


//init posspeed module
//
//
void  POSSPEED_1_Init(void)
{

    EQep1Regs.QUPRD = 2000000;            // Unit Timer for 100Hz at 200 MHz
                                          // SYSCLKOUT
    EQep1Regs.QDECCTL.bit.QSRC = 00;      // QEP quadrature count mode
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;
    EQep1Regs.QEPCTL.bit.PCRM = 00;       // PCRM=00 mode - QPOSCNT reset on
                                          // index event
    //EQep1Regs.QEPCTL.bit.PCRM = 01;
    EQep1Regs.QEPCTL.bit.UTE = 1;         // Unit Timeout Enable
    EQep1Regs.QEPCTL.bit.QCLM = 1;        // Latch on unit time out
    EQep1Regs.QPOSMAX = 0xffffffff;
    EQep1Regs.QEPCTL.bit.QPEN = 1;        // QEP enable
    EQep1Regs.QCAPCTL.bit.UPPS = 5;       // 1/32 for unit position
    EQep1Regs.QCAPCTL.bit.CCPS = 6;       // 1/64 for CAP clock
    EQep1Regs.QCAPCTL.bit.CEN = 1;        // QEP Capture Enable
}

void  POSSPEED_2_Init(void)
{

    EQep2Regs.QUPRD = 2000000;            // Unit Timer for 100Hz at 200 MHz
                                          // SYSCLKOUT
    EQep2Regs.QDECCTL.bit.QSRC = 00;      // QEP quadrature count mode
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2;
    EQep2Regs.QEPCTL.bit.PCRM = 00;       // PCRM=00 mode - QPOSCNT reset on
                                          // index event
    //EQep2Regs.QEPCTL.bit.PCRM = 01;
    EQep2Regs.QEPCTL.bit.UTE = 1;         // Unit Timeout Enable
    EQep2Regs.QEPCTL.bit.QCLM = 1;        // Latch on unit time out
    EQep2Regs.QPOSMAX = 0xffffffff;
    EQep2Regs.QEPCTL.bit.QPEN = 1;        // QEP enable
    EQep2Regs.QCAPCTL.bit.UPPS = 5;       // 1/32 for unit position
    EQep2Regs.QCAPCTL.bit.CCPS = 6;       // 1/64 for CAP clock
    EQep2Regs.QCAPCTL.bit.CEN = 1;        // QEP Capture Enable
}

void  POSSPEED_3_Init(void)
{

    EQep3Regs.QUPRD = 2000000;            // Unit Timer for 100Hz at 200 MHz
                                          // SYSCLKOUT
    EQep3Regs.QDECCTL.bit.QSRC = 00;      // QEP quadrature count mode
    EQep3Regs.QEPCTL.bit.FREE_SOFT = 2;
    EQep3Regs.QEPCTL.bit.PCRM = 00;       // PCRM=00 mode - QPOSCNT reset on
                                          // index event
    //EQep3Regs.QEPCTL.bit.PCRM = 01;
    EQep3Regs.QEPCTL.bit.UTE = 1;         // Unit Timeout Enable
    EQep3Regs.QEPCTL.bit.QCLM = 1;        // Latch on unit time out
    EQep3Regs.QPOSMAX = 0xffffffff;
    EQep3Regs.QEPCTL.bit.QPEN = 1;        // QEP enable
    EQep3Regs.QCAPCTL.bit.UPPS = 5;       // 1/32 for unit position
    EQep3Regs.QCAPCTL.bit.CCPS = 6;       // 1/64 for CAP clock
    EQep3Regs.QCAPCTL.bit.CEN = 1;        // QEP Capture Enable
}

// POSSPEED_Calc - Perform the position calculations
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////Primer Placa///////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void POSSPEED_1_Calc(POSSPEED *p)
{
    long tmp;
    unsigned int pos16bval,temp1;
    Uint32 pos32bval;
    _iq Tmp1,newp,oldp; //fixed-point arithmetic

    //
    // Position calculation - mechanical and electrical motor angle
    //
    p->DirectionQep = EQep1Regs.QEPSTS.bit.QDF;  // Motor direction:
                                                 // 0=CCW/reverse, 1=CW/forward
    //ESTO ES LO QUE ESTABA ANTES
    pos16bval = (unsigned int)EQep1Regs.QPOSCNT; // capture position once
                                                 // per QA/QB period
    p->theta_raw1 = pos16bval+ p->cal_angle;      // raw theta = current pos. + 
                                                 // ang. offset from QA 
    //FIN

    //LO QUE ESTA AHORA : LO MISMO PERO PARA 32
    pos32bval = (Uint32)EQep1Regs.QPOSCNT;  //cuentas que hace el enconder
    p->theta_raw = pos32bval+ p->cal_angle;




    // funciones que estan en el informe para calculo del angulo en funcion del numero de cuentas pag 120
    if((p->theta_raw < 4294967296)&&(p->theta_raw > 4294791959/2)){ //el numero grande es la cantidad de cuentas maximas del encoder 2^32
       p->angle_robot = -7.716e-4*(p->theta_raw)+3314003.765; //ANGLE ROBOT SE COPIA EN MUESTREO Y SE USA EN CONTROL VISTO DESDE LA JUNTA, PASARLO A MOTOR
       p->angle_robot = Naxis1*p->angle_robot;
    } 
    // numero asociado a tita +135 : 152251*2
    if((p->theta_raw < 152251*2)&&(p->theta_raw > 0)){ //RANGO FUNCION 1 
       p->angle_robot = -8.9085e-4*(p->theta_raw);
       p->angle_robot = Naxis1*p->angle_robot;
    }
    if(p->theta_raw == 0){
        p->angle_robot=0;
    }


    //
    // The following lines calculate
    // p->theta_mech ~= QPOSCNT/mech_scaler [current cnt/(total cnt in 1 rev.)]
    // where mech_scaler = 4000 cnts/revolution
    //
    // OBTIENE EL VALOR DE ANGULO MECANICO THETA_RAW1 EN VEZ DE RAW
    //tmp = (long)((long)p->theta_raw*(long)p->mech_scaler);   // Q0*Q26 = Q26
    tmp = (long)((long)p->theta_raw1*(long)p->mech_scaler);   // Q0*Q26 = Q26
    tmp &= 0x03FFF000;
    p->theta_mech = (int)(tmp>>11);                          // Q26 -> Q15
    p->theta_mech &= 0x7FFF;   //32767         0111 1111 1111 1111     PREGUNTAR A JOAQUIN PARA PASARLO A Q31!


    //
    // The following lines calculate p->elec_mech
    // ANGULO ELECTRICO 
    p->theta_elec = p->pole_pairs*p->theta_mech;             // Q0*Q15 = Q15
    //p->theta_elec &= 0x7FFF;

    //
    // Check an index occurrence
    //
    if(EQep1Regs.QFLG.bit.IEL == 1)
    {
       p->index_sync_flag = 0x00F0;
       EQep1Regs.QCLR.bit.IEL = 1;    // Clear __interrupt flag
    }

    //
    // High Speed Calculation using QEP Position counter
    //
    // Check unit Time out-event for speed calculation:
    // Unit Timer is configured for 100Hz in INIT function
    // HIGH SPEED, DIFERENCIACION
    if(EQep1Regs.QFLG.bit.UTO == 1)    // If unit timeout (one 100Hz period)
    {
        //
        // Differentiator
        //
        // The following lines calculate
        // position = (x2-x1)/4000 (position in 1 revolution)
        //
        pos16bval = (unsigned int)EQep1Regs.QPOSLAT;   // Latched POSCNT value

        // SE HACEN OPERACIONES PARA OBTENER UN FIXED-ARICMETIC POINT EN Q15: 15 BIT FIXED POINT 
        tmp = (long)((long)pos16bval*(long)p->mech_scaler); // Q0*Q26 = Q26 CAST A LONG 
        tmp &= 0x03FFF000;
        tmp = (int)(tmp>>11);                               // Q26 -> Q15
        tmp &= 0x7FFF;

        newp = _IQ15toIQ(tmp); // CONVERSION PARA FORMATO Q ADECUADO 
        oldp = p->oldpos;


        //OBTIENE LA DIRECCION DE GIRO Y DIFERENCIA DE ANGULO PARA POSTERIOR CALCULO DE VELOCIDAD
        if(p->DirectionQep==0)                     // POSCNT is counting down
        {
            //_IQ(1) ESTE ES EL VALOR 1 EN Q FORMAT PARA ASEGURANOS QUE LAS CUENTAS DEN NEGATIVAS O POSITIVAS SEGUN CORRESPONDA
            if(newp>oldp)
            {
                Tmp1 = - (_IQ(1) - newp + oldp);    // x2-x1 should be negative
            }
            else
            {
                Tmp1 = newp -oldp;
            }
        }
        else if(p->DirectionQep == 1)              // POSCNT is counting up
        {
            if(newp<oldp)
            {
                Tmp1 = _IQ(1) + newp - oldp;
            }
            else
            {
                Tmp1 = newp - oldp;                 // x2-x1 should be positive
            }
        }

        if(Tmp1>_IQ(1))
        {
            p->Speed_fr = _IQ(1);
        }
        else if(Tmp1<_IQ(-1))
        {
            p->Speed_fr = _IQ(-1);
        }
        else
        {
            p->Speed_fr = Tmp1; //VALE ENTRE 1 Y -1
        }

        //
        // Update the electrical angle
        //
        p->oldpos = newp;

        //
        // Change motor speed from pu value to rpm value (Q15 -> Q0)
        // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
        //
        p->SpeedRpm_fr = _IQmpy(p->BaseRpm,p->Speed_fr);

        EQep1Regs.QCLR.bit.UTO=1;                   // Clear __interrupt flag
    }

    //LOW SPEED 
    // Low-speed computation using QEP capture counter
    //
    if(EQep1Regs.QEPSTS.bit.UPEVNT == 1)               // Unit position event
    {
        if(EQep1Regs.QEPSTS.bit.COEF == 0)             // No Capture overflow
        {
            temp1 = (unsigned long)EQep1Regs.QCPRDLAT; // temp1 = t2-t1
        }
        else   // Capture overflow, saturate the result
        {
            temp1 = 0xFFFF;
        }

        //
        // p->Speed_pr = p->SpeedScaler/temp1
        //
        p->Speed_pr = _IQdiv(p->SpeedScaler,temp1);
        Tmp1 = p->Speed_pr;

        if (Tmp1>_IQ(1))
        {
            p->Speed_pr = _IQ(1);
        }
        else
        {
            p->Speed_pr = Tmp1;
        }

        // RPM 
        // Convert p->Speed_pr to RPM
        //
        if(p->DirectionQep == 0)  // Reverse direction = negative
        {
            //
            // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
            //
            p->SpeedRpm_pr = _IQmpy(p->BaseRpm,p->Speed_pr); //MAS IQ
        }
        else                     // Forward direction = positive
        {
            //
            // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
            //
            p->SpeedRpm_pr = -_IQmpy(p->BaseRpm,p->Speed_pr);//MENOS EN IQ 
        }

        EQep1Regs.QEPSTS.all = 0x88; // Clear Unit position event flag
                                     // Clear overflow error flag
    }
}

void POSSPEED_2_Calc(POSSPEED *p)
{
    long tmp;
    unsigned int pos16bval,temp1;
    Uint32 pos32bval;
    _iq Tmp1,newp,oldp;

    //
    // Position calculation - mechanical and electrical motor angle
    //
    p->DirectionQep = EQep2Regs.QEPSTS.bit.QDF;  // Motor direction:
                                                 // 0=CCW/reverse, 1=CW/forward
    //ESTO ES LO QUE ESTABA ANTES
    pos16bval = (unsigned int)EQep2Regs.QPOSCNT; // capture position once
                                                 // per QA/QB period
    p->theta_raw1 = pos16bval+ p->cal_angle;      // raw theta = current pos. +
                                                 // ang. offset from QA
    //FIN

    pos32bval = (Uint32)EQep2Regs.QPOSCNT;  //cuentas que hace el enconder
    p->theta_raw = pos32bval+ p->cal_angle;

    if((p->theta_raw < 4294967295)&&(p->theta_raw > 4294866943/2)){
       p->angle_robot = -7.225e-4*(p->theta_raw)+3103113.871;
       p->angle_robot = Naxis2*p->angle_robot;
    }
    if((p->theta_raw < 2*100352)&&(p->theta_raw > 0)){
       p->angle_robot = -7.225e-4*(p->theta_raw);
       p->angle_robot = Naxis2*p->angle_robot;
    }
    if(p->theta_raw == 0){
        p->angle_robot=0;
    }


    //
    // The following lines calculate
    // p->theta_mech ~= QPOSCNT/mech_scaler [current cnt/(total cnt in 1 rev.)]
    // where mech_scaler = 4000 cnts/revolution
    //
    //tmp = (long)((long)p->theta_raw*(long)p->mech_scaler);   // Q0*Q26 = Q26
    tmp = (long)((long)p->theta_raw1*(long)p->mech_scaler);   // Q0*Q26 = Q26
    tmp &= 0x03FFF000;
    p->theta_mech = (int)(tmp>>11);                          // Q26 -> Q15
    p->theta_mech &= 0x7FFF;                                                                //32767         0111 1111 1111 1111     PREGUNTAR A JOAQUIN PARA PASARLO A Q31!


    //
    // The following lines calculate p->elec_mech
    //
    p->theta_elec = p->pole_pairs*p->theta_mech;             // Q0*Q15 = Q15
    //p->theta_elec &= 0x7FFF;

    //
    // Check an index occurrence
    //
    if(EQep2Regs.QFLG.bit.IEL == 1)
    {
       p->index_sync_flag = 0x00F0;
       EQep2Regs.QCLR.bit.IEL = 1;    // Clear __interrupt flag
    }

    //
    // High Speed Calculation using QEP Position counter
    //
    // Check unit Time out-event for speed calculation:
    // Unit Timer is configured for 100Hz in INIT function
    //
    if(EQep2Regs.QFLG.bit.UTO == 1)    // If unit timeout (one 100Hz period)
    {
        //
        // Differentiator
        //
        // The following lines calculate
        // position = (x2-x1)/4000 (position in 1 revolution)
        //
        pos16bval = (unsigned int)EQep2Regs.QPOSLAT;   // Latched POSCNT value

        tmp = (long)((long)pos16bval*(long)p->mech_scaler); // Q0*Q26 = Q26
        tmp &= 0x03FFF000;
        tmp = (int)(tmp>>11);                               // Q26 -> Q15
        tmp &= 0x7FFF;
        newp = _IQ15toIQ(tmp);
        oldp = p->oldpos;

        if(p->DirectionQep==0)                     // POSCNT is counting down
        {
            if(newp>oldp)
            {
                Tmp1 = - (_IQ(1) - newp + oldp);    // x2-x1 should be negative
            }
            else
            {
                Tmp1 = newp -oldp;
            }
        }
        else if(p->DirectionQep == 1)              // POSCNT is counting up
        {
            if(newp<oldp)
            {
                Tmp1 = _IQ(1) + newp - oldp;
            }
            else
            {
                Tmp1 = newp - oldp;                 // x2-x1 should be positive
            }
        }

        if(Tmp1>_IQ(1))
        {
            p->Speed_fr = _IQ(1);
        }
        else if(Tmp1<_IQ(-1))
        {
            p->Speed_fr = _IQ(-1);
        }
        else
        {
            p->Speed_fr = Tmp1;
        }

        //
        // Update the electrical angle
        //
        p->oldpos = newp;

        //
        // Change motor speed from pu value to rpm value (Q15 -> Q0)
        // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
        //
        p->SpeedRpm_fr = _IQmpy(p->BaseRpm,p->Speed_fr);

        EQep2Regs.QCLR.bit.UTO=1;                   // Clear __interrupt flag
    }

    //
    // Low-speed computation using QEP capture counter
    //
    if(EQep2Regs.QEPSTS.bit.UPEVNT == 1)               // Unit position event
    {
        if(EQep2Regs.QEPSTS.bit.COEF == 0)             // No Capture overflow
        {
            temp1 = (unsigned long)EQep2Regs.QCPRDLAT; // temp1 = t2-t1
        }
        else   // Capture overflow, saturate the result
        {
            temp1 = 0xFFFF;
        }

        //
        // p->Speed_pr = p->SpeedScaler/temp1
        //
        p->Speed_pr = _IQdiv(p->SpeedScaler,temp1);
        Tmp1 = p->Speed_pr;

        if (Tmp1>_IQ(1))
        {
            p->Speed_pr = _IQ(1);
        }
        else
        {
            p->Speed_pr = Tmp1;
        }

        //
        // Convert p->Speed_pr to RPM
        //
        if(p->DirectionQep == 0)  // Reverse direction = negative
        {
            //
            // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
            //
            p->SpeedRpm_pr = _IQmpy(p->BaseRpm,p->Speed_pr); //habia un menos
        }
        else                     // Forward direction = positive
        {
            //
            // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
            //
            p->SpeedRpm_pr = -_IQmpy(p->BaseRpm,p->Speed_pr);
        }

        EQep2Regs.QEPSTS.all = 0x88; // Clear Unit position event flag
                                     // Clear overflow error flag
    }
}

void POSSPEED_3_Calc(POSSPEED *p)
{
    long tmp;
    unsigned int pos16bval,temp1;
    Uint32 pos32bval;
    _iq Tmp1,newp,oldp;

    //
    // Position calculation - mechanical and electrical motor angle
    //
    p->DirectionQep = EQep3Regs.QEPSTS.bit.QDF;  // Motor direction:
                                                 // 0=CCW/reverse, 1=CW/forward
    //ESTO ES LO QUE ESTABA ANTES
    pos16bval = (unsigned int)EQep3Regs.QPOSCNT; // capture position once
                                                 // per QA/QB period
    p->theta_raw1 = pos16bval+ p->cal_angle;      // raw theta = current pos. +
                                                 // ang. offset from QA
    //FIN

    pos32bval = (Uint32)EQep3Regs.QPOSCNT;  //cuentas que hace el enconder
    p->theta_raw = pos32bval+ p->cal_angle;

    if((p->theta_raw < 4294967295)&&(p->theta_raw > 4294839866/2)){
       p->angle_robot = 8.24e-4*(p->theta_raw)-3539053.05;
       p->angle_robot=Naxis3*p->angle_robot; //RELACION NAXIS, TRANSFORMACION DE COORDENADAS DE JUNTA A COORDENADAS DE MOTOR
    }
    if((p->theta_raw < 127429.12*2)&&(p->theta_raw > 0)){
       p->angle_robot = 8.24e-4*(p->theta_raw);
       p->angle_robot=Naxis3*p->angle_robot; //RELACION NAXIS, TRANSFORMACION DE COORDENADAS DE JUNTA A COORDENADAS DE MOTOR
    }
    if(p->theta_raw == 0){
        p->angle_robot=0;
        
    }


    //
    // The following lines calculate
    // p->theta_mech ~= QPOSCNT/mech_scaler [current cnt/(total cnt in 1 rev.)]
    // where mech_scaler = 4000 cnts/revolution
    //
    //tmp = (long)((long)p->theta_raw*(long)p->mech_scaler);   // Q0*Q26 = Q26
    tmp = (long)((long)p->theta_raw1*(long)p->mech_scaler);   // Q0*Q26 = Q26
    tmp &= 0x03FFF000;
    p->theta_mech = (int)(tmp>>11);                          // Q26 -> Q15
    p->theta_mech &= 0x7FFF;                                                                //32767         0111 1111 1111 1111     PREGUNTAR A JOAQUIN PARA PASARLO A Q31!


    //
    // The following lines calculate p->elec_mech
    //
    p->theta_elec = p->pole_pairs*p->theta_mech;             // Q0*Q15 = Q15
    //p->theta_elec &= 0x7FFF;

    //
    // Check an index occurrence
    //
    if(EQep3Regs.QFLG.bit.IEL == 1)
    {
       p->index_sync_flag = 0x00F0;
       EQep3Regs.QCLR.bit.IEL = 1;    // Clear __interrupt flag
    }

    //
    // High Speed Calculation using QEP Position counter
    //
    // Check unit Time out-event for speed calculation:
    // Unit Timer is configured for 100Hz in INIT function
    //
    if(EQep3Regs.QFLG.bit.UTO == 1)    // If unit timeout (one 100Hz period)
    {
        //
        // Differentiator
        //
        // The following lines calculate
        // position = (x2-x1)/4000 (position in 1 revolution)
        //
        pos16bval = (unsigned int)EQep3Regs.QPOSLAT;   // Latched POSCNT value

        tmp = (long)((long)pos16bval*(long)p->mech_scaler); // Q0*Q26 = Q26
        tmp &= 0x03FFF000;
        tmp = (int)(tmp>>11);                               // Q26 -> Q15
        tmp &= 0x7FFF;
        newp = _IQ15toIQ(tmp);
        oldp = p->oldpos;

        if(p->DirectionQep==0)                     // POSCNT is counting down
        {
            if(newp>oldp)
            {
                Tmp1 = - (_IQ(1) - newp + oldp);    // x2-x1 should be negative
            }
            else
            {
                Tmp1 = newp -oldp;
            }
        }
        else if(p->DirectionQep == 1)              // POSCNT is counting up
        {
            if(newp<oldp)
            {
                Tmp1 = _IQ(1) + newp - oldp;
            }
            else
            {
                Tmp1 = newp - oldp;                 // x2-x1 should be positive
            }
        }

        if(Tmp1>_IQ(1))
        {
            p->Speed_fr = _IQ(1);
        }
        else if(Tmp1<_IQ(-1))
        {
            p->Speed_fr = _IQ(-1);
        }
        else
        {
            p->Speed_fr = Tmp1;
        }

        //
        // Update the electrical angle
        //
        p->oldpos = newp;

        //
        // Change motor speed from pu value to rpm value (Q15 -> Q0)
        // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
        //
        p->SpeedRpm_fr = _IQmpy(p->BaseRpm,p->Speed_fr);

        EQep3Regs.QCLR.bit.UTO=1;                   // Clear __interrupt flag
    }

    //
    // Low-speed computation using QEP capture counter
    //
    if(EQep3Regs.QEPSTS.bit.UPEVNT == 1)               // Unit position event
    {
        if(EQep3Regs.QEPSTS.bit.COEF == 0)             // No Capture overflow
        {
            temp1 = (unsigned long)EQep3Regs.QCPRDLAT; // temp1 = t2-t1
        }
        else   // Capture overflow, saturate the result
        {
            temp1 = 0xFFFF;
        }

        //
        // p->Speed_pr = p->SpeedScaler/temp1
        //
        p->Speed_pr = _IQdiv(p->SpeedScaler,temp1);
        Tmp1 = p->Speed_pr;

        if (Tmp1>_IQ(1))
        {
            p->Speed_pr = _IQ(1);
        }
        else
        {
            p->Speed_pr = Tmp1;
        }

        //
        // Convert p->Speed_pr to RPM
        //
        if(p->DirectionQep == 0)  // Reverse direction = negative
        {
            //
            // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
            //
            p->SpeedRpm_pr = _IQmpy(p->BaseRpm,p->Speed_pr); //habia un menos
        }
        else                     // Forward direction = positive
        {
            //
            // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
            //
            p->SpeedRpm_pr = -_IQmpy(p->BaseRpm,p->Speed_pr);
        }

        EQep3Regs.QEPSTS.all = 0x88; // Clear Unit position event flag
                                     // Clear overflow error flag
    }
}
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////Segunda Placa////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//void POSSPEED_1_Calc(POSSPEED *p)
//{
//        long tmp;
//        unsigned int pos16bval,temp1;
//        Uint32 pos32bval;
//        _iq Tmp1,newp,oldp;
//
//        //
//        // Position calculation - mechanical and electrical motor angle
//        //
//        p->DirectionQep = EQep1Regs.QEPSTS.bit.QDF;  // Motor direction:
//                                                     // 0=CCW/reverse, 1=CW/forward
//    //ESTO ES LO QUE ESTABA ANTES
//        pos16bval = (unsigned int)EQep1Regs.QPOSCNT; // capture position once
//                                                     // per QA/QB period
//        p->theta_raw1 = pos16bval+ p->cal_angle;      // raw theta = current pos. +
//                                                     // ang. offset from QA
//    //FIN
//
//        pos32bval = (Uint32)EQep1Regs.QPOSCNT;  //cuentas que hace el enconder
//        p->theta_raw = pos32bval+ p->cal_angle;
//
//        if((p->theta_raw < 4294967296)&&(p->theta_raw > 4.29486694e9/2)){
//           p->angle_robot = -976.5625e-6*(p->theta_raw)+4194304;
//        }
//        if((p->theta_raw < 100352*2)&&(p->theta_raw > 0)){
//           p->angle_robot = -976.5625e-6*(p->theta_raw);
//        }
//        if(p->theta_raw == 0){
//            p->angle_robot=0;
//        }
//
//
//        //
//        // The following lines calculate
//        // p->theta_mech ~= QPOSCNT/mech_scaler [current cnt/(total cnt in 1 rev.)]
//        // where mech_scaler = 4000 cnts/revolution
//        //
//        //tmp = (long)((long)p->theta_raw*(long)p->mech_scaler);   // Q0*Q26 = Q26
//        tmp = (long)((long)p->theta_raw1*(long)p->mech_scaler);   // Q0*Q26 = Q26
//        tmp &= 0x03FFF000;
//        p->theta_mech = (int)(tmp>>11);                          // Q26 -> Q15
//        p->theta_mech &= 0x7FFF;                                                                //32767         0111 1111 1111 1111     PREGUNTAR A JOAQUIN PARA PASARLO A Q31!
//
//
//        //
//        // The following lines calculate p->elec_mech
//        //
//        p->theta_elec = p->pole_pairs*p->theta_mech;             // Q0*Q15 = Q15
//        //p->theta_elec &= 0x7FFF;
//
//        //
//        // Check an index occurrence
//        //
//        if(EQep1Regs.QFLG.bit.IEL == 1)
//        {
//           p->index_sync_flag = 0x00F0;
//           EQep1Regs.QCLR.bit.IEL = 1;    // Clear __interrupt flag
//        }
//
//        //
//        // High Speed Calculation using QEP Position counter
//        //
//        // Check unit Time out-event for speed calculation:
//        // Unit Timer is configured for 100Hz in INIT function
//        //
//        if(EQep1Regs.QFLG.bit.UTO == 1)    // If unit timeout (one 100Hz period)
//        {
//            //
//            // Differentiator
//            //
//            // The following lines calculate
//            // position = (x2-x1)/4000 (position in 1 revolution)
//            //
//            pos16bval = (unsigned int)EQep1Regs.QPOSLAT;   // Latched POSCNT value
//
//            tmp = (long)((long)pos16bval*(long)p->mech_scaler); // Q0*Q26 = Q26
//            tmp &= 0x03FFF000;
//            tmp = (int)(tmp>>11);                               // Q26 -> Q15
//            tmp &= 0x7FFF;
//            newp = _IQ15toIQ(tmp);
//            oldp = p->oldpos;
//
//            if(p->DirectionQep==0)                     // POSCNT is counting down
//            {
//                if(newp>oldp)
//                {
//                    Tmp1 = - (_IQ(1) - newp + oldp);    // x2-x1 should be negative
//                }
//                else
//                {
//                    Tmp1 = newp -oldp;
//                }
//            }
//            else if(p->DirectionQep == 1)              // POSCNT is counting up
//            {
//                if(newp<oldp)
//                {
//                    Tmp1 = _IQ(1) + newp - oldp;
//                }
//                else
//                {
//                    Tmp1 = newp - oldp;                 // x2-x1 should be positive
//                }
//            }
//
//            if(Tmp1>_IQ(1))
//            {
//                p->Speed_fr = _IQ(1);
//            }
//            else if(Tmp1<_IQ(-1))
//            {
//                p->Speed_fr = _IQ(-1);
//            }
//            else
//            {
//                p->Speed_fr = Tmp1;
//            }
//
//            //
//            // Update the electrical angle
//            //
//            p->oldpos = newp;
//
//            //
//            // Change motor speed from pu value to rpm value (Q15 -> Q0)
//            // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
//            //
//            p->SpeedRpm_fr = _IQmpy(p->BaseRpm,p->Speed_fr);
//
//            EQep1Regs.QCLR.bit.UTO=1;                   // Clear __interrupt flag
//        }
//
//        //
//        // Low-speed computation using QEP capture counter
//        //
//        if(EQep1Regs.QEPSTS.bit.UPEVNT == 1)               // Unit position event
//        {
//            if(EQep1Regs.QEPSTS.bit.COEF == 0)             // No Capture overflow
//            {
//                temp1 = (unsigned long)EQep1Regs.QCPRDLAT; // temp1 = t2-t1
//            }
//            else   // Capture overflow, saturate the result
//            {
//                temp1 = 0xFFFF;
//            }
//
//            //
//            // p->Speed_pr = p->SpeedScaler/temp1
//            //
//            p->Speed_pr = _IQdiv(p->SpeedScaler,temp1);
//            Tmp1 = p->Speed_pr;
//
//            if (Tmp1>_IQ(1))
//            {
//                p->Speed_pr = _IQ(1);
//            }
//            else
//            {
//                p->Speed_pr = Tmp1;
//            }
//
//            //
//            // Convert p->Speed_pr to RPM
//            //
//            if(p->DirectionQep == 0)  // Reverse direction = negative
//            {
//                //
//                // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
//                //
//                p->SpeedRpm_pr = _IQmpy(p->BaseRpm,p->Speed_pr); //habia un menos
//            }
//            else                     // Forward direction = positive
//            {
//                //
//                // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
//                //
//                p->SpeedRpm_pr = -_IQmpy(p->BaseRpm,p->Speed_pr);
//            }
//
//            EQep1Regs.QEPSTS.all = 0x88; // Clear Unit position event flag
//                                         // Clear overflow error flag
//        }
//}
//
//void POSSPEED_2_Calc(POSSPEED *p)
//{
//    long tmp;
//    unsigned int pos16bval,temp1;
//    Uint32 pos32bval;
//    _iq Tmp1,newp,oldp;
//
//    //
//    // Position calculation - mechanical and electrical motor angle
//    //
//    p->DirectionQep = EQep2Regs.QEPSTS.bit.QDF;  // Motor direction:
//                                                 // 0=CCW/reverse, 1=CW/forward
////ESTO ES LO QUE ESTABA ANTES
//    pos16bval = (unsigned int)EQep2Regs.QPOSCNT; // capture position once
//                                                 // per QA/QB period
//    p->theta_raw1 = pos16bval+ p->cal_angle;      // raw theta = current pos. +
//                                                 // ang. offset from QA
////FIN
//
//    pos32bval = (Uint32)EQep2Regs.QPOSCNT;  //cuentas que hace el enconder
//    p->theta_raw = pos32bval+ p->cal_angle;
//
//    if((p->theta_raw < 4294967295)&&(p->theta_raw > 4294865105/2)){
//       p->angle_robot = -1.75541252e-3*(p->theta_raw)+7539439.36;
//    }
//    if((p->theta_raw < 2*102540)&&(p->theta_raw > 0)){
//       p->angle_robot = -1.75541252e-3*(p->theta_raw);
//    }
//    if(p->theta_raw == 0){
//        p->angle_robot=0;
//    }
//
//
//    //
//    // The following lines calculate
//    // p->theta_mech ~= QPOSCNT/mech_scaler [current cnt/(total cnt in 1 rev.)]
//    // where mech_scaler = 4000 cnts/revolution
//    //
//    //tmp = (long)((long)p->theta_raw*(long)p->mech_scaler);   // Q0*Q26 = Q26
//    tmp = (long)((long)p->theta_raw1*(long)p->mech_scaler);   // Q0*Q26 = Q26
//    tmp &= 0x03FFF000;
//    p->theta_mech = (int)(tmp>>11);                          // Q26 -> Q15
//    p->theta_mech &= 0x7FFF;                                                                //32767         0111 1111 1111 1111     PREGUNTAR A JOAQUIN PARA PASARLO A Q31!
//
//
//    //
//    // The following lines calculate p->elec_mech
//    //
//    p->theta_elec = p->pole_pairs*p->theta_mech;             // Q0*Q15 = Q15
//    //p->theta_elec &= 0x7FFF;
//
//    //
//    // Check an index occurrence
//    //
//    if(EQep2Regs.QFLG.bit.IEL == 1)
//    {
//       p->index_sync_flag = 0x00F0;
//       EQep2Regs.QCLR.bit.IEL = 1;    // Clear __interrupt flag
//    }
//
//    //
//    // High Speed Calculation using QEP Position counter
//    //
//    // Check unit Time out-event for speed calculation:
//    // Unit Timer is configured for 100Hz in INIT function
//    //
//    if(EQep2Regs.QFLG.bit.UTO == 1)    // If unit timeout (one 100Hz period)
//    {
//        //
//        // Differentiator
//        //
//        // The following lines calculate
//        // position = (x2-x1)/4000 (position in 1 revolution)
//        //
//        pos16bval = (unsigned int)EQep2Regs.QPOSLAT;   // Latched POSCNT value
//
//        tmp = (long)((long)pos16bval*(long)p->mech_scaler); // Q0*Q26 = Q26
//        tmp &= 0x03FFF000;
//        tmp = (int)(tmp>>11);                               // Q26 -> Q15
//        tmp &= 0x7FFF;
//        newp = _IQ15toIQ(tmp);
//        oldp = p->oldpos;
//
//        if(p->DirectionQep==0)                     // POSCNT is counting down
//        {
//            if(newp>oldp)
//            {
//                Tmp1 = - (_IQ(1) - newp + oldp);    // x2-x1 should be negative
//            }
//            else
//            {
//                Tmp1 = newp -oldp;
//            }
//        }
//        else if(p->DirectionQep == 1)              // POSCNT is counting up
//        {
//            if(newp<oldp)
//            {
//                Tmp1 = _IQ(1) + newp - oldp;
//            }
//            else
//            {
//                Tmp1 = newp - oldp;                 // x2-x1 should be positive
//            }
//        }
//
//        if(Tmp1>_IQ(1))
//        {
//            p->Speed_fr = _IQ(1);
//        }
//        else if(Tmp1<_IQ(-1))
//        {
//            p->Speed_fr = _IQ(-1);
//        }
//        else
//        {
//            p->Speed_fr = Tmp1;
//        }
//
//        //
//        // Update the electrical angle
//        //
//        p->oldpos = newp;
//
//        //
//        // Change motor speed from pu value to rpm value (Q15 -> Q0)
//        // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
//        //
//        p->SpeedRpm_fr = _IQmpy(p->BaseRpm,p->Speed_fr);
//
//        EQep2Regs.QCLR.bit.UTO=1;                   // Clear __interrupt flag
//    }
//
//    //
//    // Low-speed computation using QEP capture counter
//    //
//    if(EQep2Regs.QEPSTS.bit.UPEVNT == 1)               // Unit position event
//    {
//        if(EQep2Regs.QEPSTS.bit.COEF == 0)             // No Capture overflow
//        {
//            temp1 = (unsigned long)EQep2Regs.QCPRDLAT; // temp1 = t2-t1
//        }
//        else   // Capture overflow, saturate the result
//        {
//            temp1 = 0xFFFF;
//        }
//
//        //
//        // p->Speed_pr = p->SpeedScaler/temp1
//        //
//        p->Speed_pr = _IQdiv(p->SpeedScaler,temp1);
//        Tmp1 = p->Speed_pr;
//
//        if (Tmp1>_IQ(1))
//        {
//            p->Speed_pr = _IQ(1);
//        }
//        else
//        {
//            p->Speed_pr = Tmp1;
//        }
//
//        //
//        // Convert p->Speed_pr to RPM
//        //
//        if(p->DirectionQep == 0)  // Reverse direction = negative
//        {
//            //
//            // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
//            //
//            p->SpeedRpm_pr = _IQmpy(p->BaseRpm,p->Speed_pr); //habia un menos
//        }
//        else                     // Forward direction = positive
//        {
//            //
//            // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
//            //
//            p->SpeedRpm_pr = -_IQmpy(p->BaseRpm,p->Speed_pr);
//        }
//
//        EQep2Regs.QEPSTS.all = 0x88; // Clear Unit position event flag
//                                     // Clear overflow error flag
//    }
//}
