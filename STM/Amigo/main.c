/*******************************Description*************************************/

/* Program checked on stm32f103c8
 * Port A and B toggled successfully with a delay
 */
 
//include the file that defines all kind of GPIO stuff
#include "stm32f10x.h"

//User Function Declaration
void delay(void);

//============================================================
//      MAIN
//============================================================
int main(void){

        //------------------------------
        //      declare variables
        //------------------------------

        //------------------------------
        //      init the micro controller
        //------------------------------
        //enable the GPIO clock for port GPIOB
        RCC->APB2ENR|=0x001C;

        //set all PB08-PB15 ports as output
        GPIOB->CRH=0x33333333;
				GPIOB->CRL=0x33333333;
				GPIOA->CRH=0X33333333;
				GPIOA->CRL=0X33333333;


        //------------------------------
        //      main loop
        //------------------------------

        //forever do...
        for(;;){
           //switch all Leds on
                GPIOB->ODR=0x0000ffff;
								GPIOA->ODR=0x0000ffff;

					//roughly wait
								delay();
					
				  //switch all Leds off
                GPIOB->ODR=0;
								GPIOA->ODR=0;

					//roughly wait
								delay();
								
        }//for

}//main


void delay(void)
{
	int x=0,i=0;
                for(x=0;x<0xff;x++) {
									for(i=0;i<0xffff;i++){
										}//for
								}	
}