/*
 * File:   newmain.c
 * Author: Ajmal
 *
 * Created on November 2, 2015, 11:58 AM
 */
#include <xc.h>

#define _XTAL_FREQ 20000000

void InitTimer1(void);
void interrupt ISR(void);
// BEGIN CONFIG
#pragma config FOSC = HS // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF // Flash Program Memory Code Protection bit (Code protection off)
//END CONFIG

int main()
{ 
    InitTimer1();
    TRISD = 0x00; //PortD as Output PIN
    TRISA = 0x00; //PortA as Output PIN
    PORTA=0x01;
    PORTD=0xFF;
    INTCONbits.GIE=1;
    INTCONbits.PEIE=1;

    while(1);
  return 0;
}

void InitTimer1(void)
{
    T1CONbits.T1CKPS0=1;
    T1CONbits.T1CKPS1=0;
    T1CONbits.T1OSCEN=1;
    PIE1bits.TMR1IE=1;
    T1CONbits.TMR1ON=1;
}

void interrupt ISR(void)
{
    if (PIR1bits.TMR1IF==1)
    {
        //PORTD=0x00;
        PORTD ^= PORTD;
        PORTDbits.RD0 ^= PORTDbits.RD0;
     
        TMR1=0xffff;
        PIR1bits.TMR1IF=0;
        T1CONbits.TMR1ON=1;
    }
/*    if(T0IF)
    {
           PORTD= ~PORTD;
           T0IF=0;
    }
*/
}