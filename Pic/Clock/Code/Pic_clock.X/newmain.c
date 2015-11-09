/*
 * File:   newmain.c
 * Author: Ajmal
 *
 * Created on November 2, 2015, 11:58 AM
 */
#include <xc.h>

#define _XTAL_FREQ 20000000
#define SEGMENT PORTA;


void InitTimer1(void);
char element(char num);
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
unsigned char tim_sec=0,tim_min=0,tim_hr=0;

int main()
{ 
    InitTimer1();
    TRISD = 0x00; //PortD as Output PIN
    TRISA = 0b00110000; //PortA as Output PIN
    TRISB = 0b00000010;
    PORTA=0xff;
    PORTD=0x7f;
    PORTB=0xff;
    INTCONbits.GIE=1;
    INTCONbits.PEIE=1;

    while(1)
    {
        PORTA=0x00;
        PORTD=element(tim_min%10);
        PORTA=0x01;
        __delay_ms(10);
        PORTA=0x00;
        PORTD=element(tim_min/10);
        PORTA=0x02;
        __delay_ms(10);
        PORTA=0x00;
        PORTD=(element(tim_hr%10)&0x7f);
        PORTA=0x04;
        __delay_ms(10);
        PORTA=0x00;
        PORTD=element(tim_hr/10);
        PORTA=0x08;
        __delay_ms(10);
        
        if(PORTBbits.RB1==0)
        {
            tim_min++;
        }
        
        if(PORTBbits.RB2==0)
        {
            tim_hr++;
        }

    }
  return 0;
}

void InitTimer1(void)
{
    T1CONbits.T1CKPS0=1;
    T1CONbits.T1CKPS1=1;
    T1CONbits.T1OSCEN=1;
    PIE1bits.TMR1IE=1;
    T1CONbits.TMR1ON=1;
}

void interrupt ISR(void)
{
    static char tim_cnt=0;
    if (PIR1bits.TMR1IF==1)
    {
        tim_cnt++;
        if(tim_cnt==0x0a)
        {
            tim_sec++;
            PORTD = ~PORTD;
            tim_cnt=0;
        }
        if(tim_sec>=60)
        {
            tim_min++;
            tim_sec=0;
        }
        if(tim_min>=60)
        {
            tim_hr++;
            tim_min=0;
            tim_sec=0;
        }
        if(tim_hr>=60)
        {
            tim_sec=0;
            tim_min=0;
            tim_hr=0;
        }
        TMR1=0x0bdb;
        PIR1bits.TMR1IF=0;
        T1CONbits.TMR1ON=1;
    }
}

char element(char num)
{
 switch (num)
 {
     case 0:
         return 0xc0;

     case 1:
         return 0xf9;

     case 2:
         return 0xa4;
             
     case 3:
         return 0xb0;
         
     case 4:
         return 0x99;
         
     case 5:
         return 0x92;
         
     case 6:
         return 0x82;

     case 7:
         return 0xf8;

     case 8:
         return 0x80;
         
     case 9:
         return 0x98;

     default:
         return 0xbf;
         
 }
}