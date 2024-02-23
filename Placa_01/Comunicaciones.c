/*
 * MEFs.c
 *
 *  Created on: 17 dic. 2023
 *      Author: gasti
 */

#include "Comunicaciones.h"

union float_union {
    float float_value;
    uint16_t byte_array[4];
} FloatToByte;

union int32_union {
    float float_value;
    uint32_t int_value;
} FloatToInt;

void Int32ToByte(int32_t int_value, uint16_t *byte_array){
    uint32_t temp;
    memcpy(&temp, &int_value, sizeof(int_value));
    byte_array[3] = (temp >> 24) & 0xFF;
    byte_array[2] = (temp >> 16) & 0xFF;
    byte_array[1] = (temp >> 8) & 0xFF;
    byte_array[0] = temp & 0xFF;
}

//void ByteToInt16(volatile int16_t* int_value, uint16_t byte1, uint16_t byte2){
//    *int_value = (int16_t)byte2 + ((int16_t)byte1 << 8);
//}

void ByteToInt16(volatile int16_t* int_value, uint16_t byte1, uint16_t byte2){
    if(byte1 == 0){
        *int_value = (int16_t)byte2;
    }else if(byte1 == 1){
        *int_value = -(int16_t)byte2;
    }

}

void Float2Byte(float float_value, uint16_t *byte_array){
    uint32_t temp;
    memcpy(&temp, &float_value, sizeof(float_value));
    byte_array[0] = (temp >> 24) & 0xFF;
    byte_array[1] = (temp >> 16) & 0xFF;
    byte_array[2] = (temp >> 8) & 0xFF;
    byte_array[3] = temp & 0xFF;
}

float Byte2Float(uint16_t *byte_array){
    uint16_t alto = (byte_array[0] & 0xFF) << 8 | (byte_array[1] & 0xFF);
    uint16_t bajo = (byte_array[2] & 0xFF) << 8 | (byte_array[3] & 0xFF);
    uint32_t todo = alto;
    todo = todo << 16;
    todo |= bajo;
    FloatToInt.int_value = todo;
//    FloatToInt.int_value = (((byte_array[0] & 0xFF) << 24) | ((byte_array[1] & 0xFF) << 16) | ((byte_array[2] & 0xFF) << 8) | ((byte_array[3] & 0xFF)));

    return FloatToInt.float_value;
}


//union int32_union {
//    int32_t int_value;
//    uint8_t byte_array[4];
//} Int32ToByte;

union uchar_union {
    int int_value;
    unsigned char char_value[2];
} IntToChar, CharToInt;

union uint16_union {
    int int_value;
    uint16_t uint_value;
} IntToUint;


/*==================[Variables mefs]==========================*/
static int32_t ret;             // Number of bytes received.
//static uint8_t buffer[20];      // Ring Buffer.
static Uint16 transmision[7];      // Frame.
static Uint16 transmision1[4];      // Frame.
static Uint16 transmision2[4];      // Frame.
static Uint16 transmision3[4];      // Frame.
int instruccion = 0;        // Frame check flag.
static Comando_recibido_enum comando_recibido = Nada;

extern volatile int HOME_SW_1, HOME_SW_2, HOME_SW_3; //varialbes home switch
extern volatile int TEMP_1, TEMP_2, TEMP_3;
extern volatile float corriente_real_1, corriente_real_2, corriente_real_3;
extern volatile float Angulo_grados_eje_1, Angulo_grados_eje_2, Angulo_grados_eje_3;
extern volatile float angulo_1, angulo_2, angulo_3;
extern volatile int init;


/*==================[MEFs]==========================*/

//---------------- MEF recepción ----------------
void MEF_Recepcion(void){
    static Enum_est_Recep est_mef_recep = reposo;
    static Uint16 receivedChar;
//    static uint16_t LF = 10;

    switch (est_mef_recep){
    case reposo:

        if(!instruccion){
            est_mef_recep = comando;
        }
        break;

    case comando:

        ret = ScibRegs.SCIFFRX.bit.RXFFST;

        if (ret){
            receivedChar = ScibRegs.SCIRXBUF.all;

//            if (b0(receivedChar)){
                transmision[0] = receivedChar;
                receivedChar = ' ';
//                scib_xmit(receivedChar);

                switch (transmision[0]){

                case 'I':
                    comando_recibido = Inicializar;
                    instruccion = 1;
                    est_mef_recep = reposo;
                    break;

                case 'H':
                    comando_recibido = Home_switch;
                    instruccion = 1;
                    est_mef_recep = reposo;
                    break;

                case 'T':
                    comando_recibido = Temperaturas;
                    instruccion = 1;
                    est_mef_recep = reposo;
                    break;

                case 'C':
                    comando_recibido = Corrientes;
                    instruccion = 1;
                    est_mef_recep = reposo;
                    break;

                case 'P':
                    comando_recibido = Posiciones;
                    instruccion = 1;
                    est_mef_recep = reposo;
                    break;

                case 'R':
                    comando_recibido = Referencias;
                    est_mef_recep = J1B1;
                    break;

                default:
                    break;

                }
//            }
        }

    break;

    case J1B1:

        ret = ScibRegs.SCIFFRX.bit.RXFFST;

        if (ret){
            transmision1[0] = ScibRegs.SCIRXBUF.all;
//            scib_xmit(transmision[1]);
            est_mef_recep = J1B2;
        }

        break;

    case J1B2:

        ret = ScibRegs.SCIFFRX.bit.RXFFST;

        if (ret){
            transmision1[1] = ScibRegs.SCIRXBUF.all;
//            scib_xmit((transmision[2]));
            est_mef_recep = J1B3;
        }

        break;

    case J1B3:

        ret = ScibRegs.SCIFFRX.bit.RXFFST;

        if (ret){
            transmision1[2] = ScibRegs.SCIRXBUF.all;
//            scib_xmit((transmision[2]));
            est_mef_recep = J1B4;
        }

        break;

    case J1B4:

        ret = ScibRegs.SCIFFRX.bit.RXFFST;

        if (ret){
            transmision1[3] = ScibRegs.SCIRXBUF.all;
//            scib_xmit((transmision[2]));
            est_mef_recep = J2B1;
        }

        break;

    case J2B1:

        ret = ScibRegs.SCIFFRX.bit.RXFFST;

        if (ret){
            transmision2[0] = ScibRegs.SCIRXBUF.all;
//            scib_xmit(transmision[3]);
            est_mef_recep = J2B2;
        }

        break;

    case J2B2:

        ret = ScibRegs.SCIFFRX.bit.RXFFST;

        if (ret){
            transmision2[1] = ScibRegs.SCIRXBUF.all;
//            scib_xmit(transmision[4]);
            est_mef_recep = J2B3;
        }

        break;

    case J2B3:

        ret = ScibRegs.SCIFFRX.bit.RXFFST;

        if (ret){
            transmision2[2] = ScibRegs.SCIRXBUF.all;
//            scib_xmit(transmision[3]);
            est_mef_recep = J2B4;
        }

        break;

    case J2B4:

        ret = ScibRegs.SCIFFRX.bit.RXFFST;

        if (ret){
            transmision2[3] = ScibRegs.SCIRXBUF.all;
//            scib_xmit(transmision[3]);
            est_mef_recep = J3B1;
        }

        break;

    case J3B1:

        ret = ScibRegs.SCIFFRX.bit.RXFFST;

        if (ret){
            transmision3[0] = ScibRegs.SCIRXBUF.all;
//            scib_xmit(transmision[5]);
            est_mef_recep = J3B2;
        }

        break;

    case J3B2:

        ret = ScibRegs.SCIFFRX.bit.RXFFST;

        if (ret){
            transmision3[1] = ScibRegs.SCIRXBUF.all;
//            scib_xmit(transmision[6]);
            est_mef_recep = J3B3;
        }

        break;

    case J3B3:

        ret = ScibRegs.SCIFFRX.bit.RXFFST;

        if (ret){
            transmision3[2] = ScibRegs.SCIRXBUF.all;
//            scib_xmit(transmision[5]);
            est_mef_recep = J3B4;
        }

        break;

    case J3B4:

        ret = ScibRegs.SCIFFRX.bit.RXFFST;

        if (ret){
            transmision3[3] = ScibRegs.SCIRXBUF.all;
//            scib_xmit(transmision[5]);
            est_mef_recep = reposo;
            instruccion = 1;
        }

        break;
    }
}


//---------------- MEF accion -------------------
void MEF_Transmision(void) {
    static Enum_est_Accion est_mef_Acc = Reposo;
    static int acc_terminada = 0;
    static uint16_t LF = 10;
    static uint16_t espacio = 32;
    static uint16_t barra = 47;
    static uint16_t bytes_transm[4];

    switch (est_mef_Acc) {

//        case Reset:
//            est_mef_Acc = Reposo;
//            break;

        case Reposo:
            if(instruccion == 1){
                switch (comando_recibido){
                case Inicializar:
                    est_mef_Acc = Homming;
                    init = 0; // Indicamos que realice la inicialización
                    break;

                case Home_switch:
                    est_mef_Acc = Trans_HS;
                    break;

                case Temperaturas:
                    est_mef_Acc = Trans_Temp;
                    break;

                case Corrientes:
                    est_mef_Acc = Trans_Corr;
                    break;

                case Posiciones:
                    est_mef_Acc = Trans_Pos;
                    break;

                case Referencias:
                    est_mef_Acc = Almacenar_ref;
                    break;
                }

//                SCI_writeCharArray(SCIB_BASE, (uint16_t*)&inst, 7);

            }

            break;

        case Homming:
//            scib_xmit('I');
//            scib_xmit(58); // Código ascii para los ":"
//            scib_xmit(espacio); // Código ascii para el espacio
//            scib_xmit(barra); // Código ascii para el salto de línea
//            scib_xmit(barra);
//            scib_xmit(LF);
//            if(init == 1){
//                scib_xmit(1);
//                acc_terminada = 1;
//            }
            acc_terminada = 1;
            if(acc_terminada){
                est_mef_Acc = Reposo;
                instruccion = false;
            }
            break;

        case Trans_HS:
//            scib_xmit('H');
//            scib_xmit(58);
//            scib_xmit(espacio);

            IntToUint.int_value = HOME_SW_1;
            scib_xmit(IntToUint.uint_value);
            scib_xmit(barra);
            IntToUint.int_value = HOME_SW_2;
            scib_xmit(IntToUint.uint_value);
            scib_xmit(barra);
            IntToUint.int_value = HOME_SW_3;
            scib_xmit(IntToUint.uint_value);
            scib_xmit(LF);

            acc_terminada = 1;
            if(acc_terminada){
                est_mef_Acc = Reposo;
                instruccion = false;
            }
            break;

        case Trans_Temp:
//            scib_xmit('T');
//            scib_xmit(58);
//            scib_xmit(espacio);
            IntToUint.int_value = TEMP_1;
            scib_xmit(IntToUint.uint_value);
            scib_xmit(barra);
            IntToUint.int_value = TEMP_2;
            scib_xmit(IntToUint.uint_value);
            scib_xmit(barra);
            IntToUint.int_value = TEMP_3;
            scib_xmit(IntToUint.uint_value);
            scib_xmit(LF);

            acc_terminada = 1;
            if(acc_terminada){
                est_mef_Acc = Reposo;
                instruccion = false;
            }
            break;

        case Trans_Corr:
//            scib_xmit('C');
//            scib_xmit(58);
//            scib_xmit(espacio);
//            corriente_real_1 = 15.0;
//            corriente_real_2 = -2.15;
//            corriente_real_3 = 84.3;
            Float2Byte(corriente_real_1,bytes_transm);
//            if(bytes_transm[3] != 0) {scib_xmit(bytes_transm[3]);}
//            if(bytes_transm[2] != 0) {scib_xmit(bytes_transm[2]);}
//            if(bytes_transm[1] != 0) {scib_xmit(bytes_transm[1]);}
            scib_xmit(bytes_transm[3]);
            scib_xmit(bytes_transm[2]);
            scib_xmit(bytes_transm[1]);
            scib_xmit(bytes_transm[0]);
//            scib_xmit(barra);
            Float2Byte(corriente_real_2,bytes_transm);
//            if(bytes_transm[3] != 0) {scib_xmit(bytes_transm[3]);}
//            if(bytes_transm[2] != 0) {scib_xmit(bytes_transm[2]);}
//            if(bytes_transm[1] != 0) {scib_xmit(bytes_transm[1]);}
            scib_xmit(bytes_transm[3]);
            scib_xmit(bytes_transm[2]);
            scib_xmit(bytes_transm[1]);
            scib_xmit(bytes_transm[0]);
//            scib_xmit(barra);
            Float2Byte(corriente_real_3,bytes_transm);
//            if(bytes_transm[3] != 0) {scib_xmit(bytes_transm[3]);}
//            if(bytes_transm[2] != 0) {scib_xmit(bytes_transm[2]);}
//            if(bytes_transm[1] != 0) {scib_xmit(bytes_transm[1]);}
            scib_xmit(bytes_transm[3]);
            scib_xmit(bytes_transm[2]);
            scib_xmit(bytes_transm[1]);
            scib_xmit(bytes_transm[0]);
//            scib_xmit(LF);

            acc_terminada = 1;
            if(acc_terminada){
                est_mef_Acc = Reposo;
                instruccion = false;
            }
            break;
        case Trans_Pos:
//            scib_xmit('P');
//            scib_xmit(58);
//            scib_xmit(espacio);
//            Angulo_grados_eje_1 = 0;
//            Angulo_grados_eje_2 = 0;
//            Angulo_grados_eje_3 = 0;
            Float2Byte(Angulo_grados_eje_1,bytes_transm);
//            if(bytes_transm[3] != 0) {scib_xmit(bytes_transm[3]);}
//            if(bytes_transm[2] != 0) {scib_xmit(bytes_transm[2]);}
//            if(bytes_transm[1] != 0) {scib_xmit(bytes_transm[1]);}
            scib_xmit(bytes_transm[3]);
            scib_xmit(bytes_transm[2]);
            scib_xmit(bytes_transm[1]);
            scib_xmit(bytes_transm[0]);

//            scib_xmit(barra);
            Float2Byte(Angulo_grados_eje_2,bytes_transm);
//            if(bytes_transm[3] != 0) {scib_xmit(bytes_transm[3]);}
//            if(bytes_transm[2] != 0) {scib_xmit(bytes_transm[2]);}
//            if(bytes_transm[1] != 0) {scib_xmit(bytes_transm[1]);}
            scib_xmit(bytes_transm[3]);
            scib_xmit(bytes_transm[2]);
            scib_xmit(bytes_transm[1]);
            scib_xmit(bytes_transm[0]);
//            scib_xmit(barra);
            Float2Byte(Angulo_grados_eje_3,bytes_transm);
//            if(bytes_transm[3] != 0) {scib_xmit(bytes_transm[3]);}
//            if(bytes_transm[2] != 0) {scib_xmit(bytes_transm[2]);}
//            if(bytes_transm[1] != 0) {scib_xmit(bytes_transm[1]);}
            scib_xmit(bytes_transm[3]);
            scib_xmit(bytes_transm[2]);
            scib_xmit(bytes_transm[1]);
            scib_xmit(bytes_transm[0]);
//            scib_xmit(LF);

            acc_terminada = 1;
            if(acc_terminada){
                est_mef_Acc = Reposo;
                instruccion = false;
            }
            break;

        case Almacenar_ref:
//            scib_xmit('R');
//            scib_xmit(58);
//            scib_xmit(espacio);
//            ByteToInt16(&angulo_1, transmision[1], transmision[2]);
            if(init == 1){ //Verifico que el sistema ya se encuentra inicializado para poder comandarlo
            angulo_1 = Byte2Float(transmision1);
//            scib_xmit(transmision[1]);
//            scib_xmit(transmision[2]);
//            scib_xmit(espacio);
//            ByteToInt16(&angulo_2, transmision[3], transmision[4]);
            angulo_2 = Byte2Float(transmision2);
//            scib_xmit(transmision[3]);
//            scib_xmit(transmision[4]);
//            scib_xmit(espacio);
//            ByteToInt16(&angulo_3, transmision[5], transmision[6]);
            angulo_3 = Byte2Float(transmision3);
//            scib_xmit(transmision[5]);
//            scib_xmit(transmision[6]);
//            scib_xmit(LF);
            }
            acc_terminada = 1;
            if(acc_terminada){
                est_mef_Acc = Reposo;
                instruccion = false;
            }
            break;
    }
}

/*==================[Inits]==========================*/
void InitComunicacionesGPIO(void){
    //Multiplexado de los GPIO 19 (RX) y 18 (TX)
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 2);
    GPIO_SetupPinOptions(19, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(18, GPIO_MUX_CPU1, 2);
    GPIO_SetupPinOptions(18, GPIO_OUTPUT, GPIO_ASYNC);
}

void Init_Comunicaciones(void){

    //--------------- Init SCIB Peripheral
    //
    // Note: Clocks were turned on to the SCIB peripheral
    // in the InitSysCtrl() function
    //

    ScibRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
    ScibRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
    ScibRegs.SCICTL2.all = 0x0003;
    ScibRegs.SCICTL2.bit.TXINTENA = 1;
    ScibRegs.SCICTL2.bit.RXBKINTENA = 1;

    //

    // SCIB at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 30 MHz (120 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x86.
    //
//        ScibRegs.SCIHBAUD.all = 0x0002;
//        ScibRegs.SCILBAUD.all = 0x008B;
        ScibRegs.SCIHBAUD.all = 0x0000;
        ScibRegs.SCILBAUD.all = 0x00A2; // baudrate = 57600 o 38K...

    ScibRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset

    // Init SCIB FIFO
    ScibRegs.SCIFFTX.all = 0xE040;
    ScibRegs.SCIFFRX.all = 0x2044;
    ScibRegs.SCIFFCT.all = 0x0;

}

/*==================[Funtions]==========================*/
void scib_xmit(int a)
{
    while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {}
    ScibRegs.SCITXBUF.all =a;
}

void Comunicaciones(void){

    MEF_Recepcion();
    if(instruccion){
        MEF_Transmision();
    }
}
