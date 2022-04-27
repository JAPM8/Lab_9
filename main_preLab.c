/*
 * File:   main_preLab.c
 * Author: Javier Alejandro Pérez Marín
 * Potenciómetro en RA0/AN0 que funciona como entrada analógica y servomotor en RC2.
 *
 * Created on 26 de abril de 2022, 08:45 AM
 */

// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*
 * CONSTANTES
 */
#define _XTAL_FREQ 500000
#define IN_MIN 0 // Valor mínimo de entrada del potenciometro
#define IN_MAX 255 // Valor máximo de entrada del potenciometro
#define OUT_MIN 30  //Valor mínimo de ancho de pulso de señal PWM
#define OUT_MAX 63 // Valor máximo de ancho de pulso de señal PWM

/*
 * VARIABLES
 */
unsigned short CCPR = 0; //Variable para almacenar ancho de pulso en interpolación lineal

/*
 * PROTOTIPO DE FUNCIÓN
 */
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);


void __interrupt() isr(void){
    //Se revisa interrupción ADC
    if (PIR1bits.ADIF){
        if (ADCON0bits.CHS == 0){ //Se verifica canal AN0        
            CCPR = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso variable
            CCPR1L = (uint8_t)(CCPR>>2);    // Se guardan los 8 bits más significativos en CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11; // Se guardan los 2 bits menos significativos en DC1B
        }
        PIR1bits.ADIF = 0; // Limpiamos bandera ADC
    }
    return;
}

void main(void) {  
   
    setup(); // Se pasa a configurar PIC
        
    while(1)
    {
       if(ADCON0bits.GO == 0){ // Si no hay proceso de conversión
            ADCON0bits.GO = 1; // Se inicia proceso de conversión
        } 
    }
}

void setup(void){
    ANSEL = 0x01; //Se configura PORTA0/AN0 como entrada analógica
    ANSELH = 0;   //I/O DIGITALES
    
    OSCCONbits.IRCF = 0b011;   //Oscilador interno de 500 KHz
    OSCCONbits.SCS = 1;         //Oscilador interno
    
    TRISA = 0x01; //PORTA0/AN0 como INPUT    
    PORTA = 0;    //CLEAR DE PUERTO A  
    TRISC = 0;    //PORTC como OUTPUT
    PORTC = 0;    //CLEAR DE PUERTO C
      
    //Config ADC
    ADCON0bits.ADCS = 0b11; // FRC
    ADCON1bits.VCFG0 = 0;  // Referencia VDD
    ADCON1bits.VCFG1 = 0;  // Referencia VSS
    ADCON0bits.CHS = 0; // Se selecciona PORTA0/AN0
    ADCON1bits.ADFM = 0; // Se indica que se tendrá un justificado a la izquierda
    ADCON0bits.ADON = 1; // Se habilita el modulo ADC
    __delay_us(40);     // Delay para sample time
    
    //Config PWM
    CCP1CON = 0; // Se apaga CCP1
    TRISCbits.TRISC2 = 1; // RC2/CCP1 como salida deshabilitado
    PR2 = 155; // Período de 20 ms 
    
    // Config CCP
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // Modo PWM
    
    CCPR1L = 30>>2; //Ciclo de trabajo base pues se va a variar
    CCP1CONbits.DC1B = 30 & 0b11; // Base de 1 ms ancho de pulso
    
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Se enciende TMR2
    while(!PIR1bits.TMR2IF);    // Se espera un ciclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Se habilita salida de PWM
    
    //Config interrupciones
    INTCONbits.GIE = 1; //Se habilitan interrupciones globales
    PIE1bits.ADIE = 1;  //Se habilita interrupcion del ADC
    PIR1bits.ADIF = 0; // Limpieza de bandera del ADC
    INTCONbits.PEIE = 1; // Se habilitan interrupciones de periféricos
 }

/* Función para hacer la interpolación lineal del valor de la entrada analógica 
*  usando solo el registro ADRESH (8 bits) al ancho de pulso del PWM (10 bits), 
* usando la ecuación:
*  y = y0 + [(y1 - y0)/(x1-x0)]*(x-x0)
*  -------------------------------------------------------------------
*  | x0 -> valor mínimo de ADC | y0 -> valor mínimo de ancho de pulso|
*  | x  -> valor actual de ADC | y  -> resultado de la interpolación | 
*  | x1 -> valor máximo de ADC | y1 -> valor máximo de ancho de puslo|
*  ------------------------------------------------------------------- 
*/
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}