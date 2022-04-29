/* 
 * File:   main.c
 * Author: josej
 *
 * Created on April 25, 2022, 3:30 PM
 */

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

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 500000
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 255              // Valor máximo de entrada del potenciometro
#define OUT_MIN 16               // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 75             // Valor máximo de ancho de pulso de señal PWM



/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
unsigned short CCPR = 0;        // Variable para almacenar ancho de pulso al hacer la interpolación lineal
uint8_t contador;               // Variable de contador para PWM manual 
uint8_t valorADC;               // Variable para PWM manual

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);
void R_TMR0(void);
/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PIR1bits.ADIF){                      // Fue interrupción del ADC?
        if(ADCON0bits.CHS == 0){            // Verificamos sea AN0 el canal seleccionado
            CCPR = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
            CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
        }
        else if (ADCON0bits.CHS == 1){      // Verificamos AN1
            CCPR = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
            CCPR2L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
            CCP2CONbits.DC2B0 = CCPR & 0b1; // Guardamos los 2 bits menos significativos en DC1B
            CCP2CONbits.DC2B1 = CCPR>>1 & 0b1;
        }
        if (ADCON0bits.CHS == 2){      // Verificamos AN2
            valorADC = map(ADRESH, IN_MIN, IN_MAX, 0, 10);
        }
        PIR1bits.ADIF = 0;          // Limpiamos bandera de interrupción
    }
    
    if(INTCONbits.T0IF){
        contador++;               // Aumentar el contador 
        R_TMR0();                 // Reset de TMR0
        if(contador==10){
            contador = 0;
        }
    }

    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        if(ADCON0bits.GO == 0){             // No hay proceso de conversion

            if(ADCON0bits.CHS == 0b0000)    
                ADCON0bits.CHS = 0b0001;    // Cambio de canal
            else if(ADCON0bits.CHS == 0b0001)
                ADCON0bits.CHS = 0b0010;    // Cambio de canal
            else if(ADCON0bits.CHS == 0b0010)
                ADCON0bits.CHS = 0b0000;    // Cambio de canal
            __delay_us(40);                 // Tiempo de adquisición
            ADCON0bits.GO = 1;              // Iniciamos proceso de conversión
        } 
        
        if(contador<=valorADC){
            PORTDbits.RD0 = 1;
        }
        else{
            PORTDbits.RD0 = 0;
        }
        
        PORTB = contador;
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSELH = 0;         // I/O digitales)
    ANSEL = 0b00000111; // AN0, AN1 y AN2 como entrada analógica
    
    
    TRISA = 0b00000111; // AN0, AN1 y AN2 como entrada
    PORTA = 0; 
    
    TRISD = 0b00000000; // PORTC como salida
    PORTD = 0b00000000; // PORTC apagado
    
    TRISB = 0b00000000;
    PORTB = 0b00000000;
    
    // Configuración reloj interno
    OSCCONbits.IRCF = 0b011;    // 500kHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuración ADC
    ADCON0bits.ADCS = 0b11;     // Personalizada
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);             // Sample time
    
    // Configuración PWM
    TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
    PR2 = 155;                  // periodo de 20ms
    
    // Configuración CCP
    CCP1CON = 0;                // Apagamos CCP1
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // PWM
    
    CCPR1L = 125>>2;
    CCP1CONbits.DC1B = 125 & 0b11;    // 0.5ms ancho de pulso / 25% ciclo de trabajo
    
    // Configuración CCP2
    TRISCbits.TRISC1 = 1;       // Deshabilitar salida de CCP1
    CCP2CON = 0;
    CCP2CONbits.CCP2M = 0b1100; // PWM
    CCPR2L = 16>>2;
    CCP2CONbits.DC2B0 = 16 & 0b1;   
    CCP2CONbits.DC2B1 = 16>>1 & 0b1;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC1 = 0;
    
    // Timer2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM
    
    // Timer0
    OPTION_REGbits.T0CS = 0;    // TIMR0 como temporizador
    OPTION_REGbits.T0SE = 0;  
    OPTION_REGbits.PSA = 0;     // Prescaler asignado a TMR0
    OPTION_REGbits.PS2 = 0;     // Prescaler = 1:16
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS0 = 1;
    TMR0 = 240;                 // Delay de 0.5 mS

    // Configuracion interrupciones
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    INTCONbits.T0IE = 1;        // Habilitamos interrupciones del TMR0
    INTCONbits.T0IF = 0;        // Limpiamos bandera de TMR0
    
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

void R_TMR0(void){
     TMR0 = 240;                // Delay de 0.5 mS
     INTCONbits.T0IF = 0;
     return;
 }
