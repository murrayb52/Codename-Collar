//********************************************************************
//*                    EEE3017W C                          			 *
//*                    General Purpose Timers                        *
//*==================================================================*
//* WRITTEN BY: Willem van Zyl and Murray Buchanan                   *
//* DATE CREATED: 1 September 2016                                   *
//* MODIFIED: 1 September 2016                                       *
//*==================================================================*
//* PROGRAMMED IN: Eclipse Luna Service Release 1 (4.4.1)            *
//* DEV. BOARD:    UCT STM32 Development Board                       *
//*==================================================================*
//* DESCRIPTION:                                                     *
//* General Purpose Timers                                       	 *
//********************************************************************
// INCLUDE FILES
//====================================================================
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
//====================================================================
// GLOBAL CONSTANTS
//====================================================================
int zero = 0b00110000;
//====================================================================
// GLOBAL VARIABLES
//====================================================================
int timarry[15];
int hund=0;
int sec=0;
int min=0;
int TIM14Flag=0;
int LapFlag1=0;
int StopFlag2=0;
int ResFlag3=0;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void main(void);
void init_LCD(void);
void init_ports(void);
void init_timer14(void);
void init_EXTI(void);
void init_NVIC(void);
void lcd_goto(int, int);
void run_ADC(int);
void s_delay (int,char);
void hang_delay(int);
int button(int);
void ODRer(int,int);
void BCD_conv(int,int,int); //(Hundredths,soconds,minutes)
//====================================================================
// MAIN FUNCTION
//====================================================================
void main(void) {
//__________Initialisations___________________________________________
	init_LCD();
	init_ports();
//__________Main Code_________________________________________________
	lcd_goto(0,0);					//Write initial text to LCD
	lcd_putstring("Stop Watch");
	lcd_goto(1,0);
	lcd_putstring("Press SW0...");

	while (button(0)==0);			//wait for SW0 press
	ODRer(0b1111,0);
	ODRer(0b1,1);
	lcd_command(CLEAR);
	lcd_goto(0,0);
	lcd_putstring("Time");
	timarry[2]=0b00111010;			//Place ":" in array
	timarry[5]=0b00101110;			//Place "." in array
	
	init_timer14();					//initialise timer
	init_EXTI();					//initialise External Interrupts
	init_NVIC();					//initialise NVIC
	
	for (;;){						//start writing values
		if (hund==100){				//incriment values if within boundary
			hund=0;
			sec++;
		}
		if (sec==60){
			sec=0;
			min++;
		}
		if (min==100){
			min=0;
		}
		//function that converts numbers into BCD and places it in array:
		BCD_conv(hund,sec,min);	
		lcd_goto(1,0);// return back to first position on LCD screen
		if (((LapFlag1==0)&(StopFlag2==0))|((ResFlag3!=0)&(LapFlag1==0))){
			for (int i=0; i<8; i++){	//write array of characters to lcd screen
				lcd_putchar(timarry[i]);
			}
			ResFlag3++;			//Flag that indicates that reset has been pressed
			if (ResFlag3>2){
				ResFlag3=0;
			}
		}
		while (TIM14Flag=0);	//wait for timer interrupt to occur
		TIM14Flag=0;			//clear timer interrupt to continue increment
	}
}
//====================================================================
// INTERRUPT HANDLERS
//====================================================================
	void EXTI0_1_IRQHandler(void)	{
		if( (EXTI->IMR & EXTI_IMR_MR0) && (EXTI->PR & EXTI_PR_PR0)){	//SW0 interrupt
			EXTI->PR |= EXTI_PR_PR0;
			LapFlag1=0;		//Clear lap flag
			StopFlag2=0;	//clear stop flag
			ODRer(0b1111,0);//clear first 4 leds
			ODRer(0b1,1);	//write 1 to first led
		}
		if( (EXTI->IMR & EXTI_IMR_MR1) && (EXTI->PR & EXTI_PR_PR1)){	// Interrupt on line 1
			EXTI->PR |= EXTI_PR_PR1;									// Clears interrupt flag
			LapFlag1=1;		//set lap flag
			ODRer(0b1111,0);	
			ODRer(0b10,1);
		}
	}

	void EXTI2_3_IRQHandler(void)	{
		if( (EXTI->IMR & EXTI_IMR_MR2) && (EXTI->PR & EXTI_PR_PR2)){	// Interrupt on line 2
			EXTI->PR |= EXTI_PR_PR2;									// Clears interrupt flag
			StopFlag2=1;	//set stop flag
			ODRer(0b1111,0);
			ODRer(0b100,1);
		
		}
		if( (EXTI->IMR & EXTI_IMR_MR3) && (EXTI->PR & EXTI_PR_PR3)){	// Interrupt on line 3
			EXTI->PR |= EXTI_PR_PR3;									// Clears interrupt flag
			ResFlag3=1;	//set reset flag
			hund=0;		//reset counter values
			sec=0;
			min=0;
			ODRer(0b1111,0);
			ODRer(0b1000,1);
		}	
	}


	void TIM14_IRQHandler(void) {
	TIM14->SR &= ~TIM_SR_UIF;	// Clears interrupt flag (acknowledge interrupt)
		if (StopFlag2==0) hund++;
		TIM14Flag=1;		//set timer interrupt flag
	}
//====================================================================
// FUNCTIONS
//====================================================================
void init_ports(void) {
	RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;    	//enable clock for GPIOB
    //Set Moders -> 0-Output, 1-Input,
	GPIOB->MODER |= GPIO_MODER_MODER0_0;        //set PB0 to output -> LED0
    GPIOB->MODER |= GPIO_MODER_MODER1_0;        //set PB1 to output -> LED1
    GPIOB->MODER |= GPIO_MODER_MODER2_0;        //set PB2 to output -> LED2
    GPIOB->MODER |= GPIO_MODER_MODER3_0;        //set PB3 to output -> LED3
    GPIOB->MODER |= GPIO_MODER_MODER4_0;        //set PB4 to output -> LED4
	GPIOB->MODER |= GPIO_MODER_MODER5_0;        //set PB5 to output -> LED5
    GPIOB->MODER |= GPIO_MODER_MODER6_0;        //set PB6 to output -> LED6
	GPIOB->MODER |= GPIO_MODER_MODER7_0;        //set PB7 to output -> LED7
    GPIOB->MODER |= GPIO_MODER_MODER10_0;		//RG LED to output
    GPIOB->MODER |= GPIO_MODER_MODER11_0;		//RG LED to Output

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 		//enable clock for push-buttons
	GPIOA->MODER &= ~GPIO_MODER_MODER0; 	//set PA0 to input
	GPIOA->MODER &= ~GPIO_MODER_MODER1; 	//set PA2 to input
	GPIOA->MODER &= ~GPIO_MODER_MODER2; 	//set PA3 to input
	GPIOA->MODER &= ~GPIO_MODER_MODER3; 	//set PA3 to input 
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0; 	//enable pull-up for PA0
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_0; 	//enable pull-up for PA1
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0; 	//enable pull-up for PA2
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR3_0; 	//enable pull-up for PA3
    GPIOA->MODER |= GPIO_MODER_MODER5;		// set PA5 to analog -> Pot @ 5
    GPIOA->MODER |= GPIO_MODER_MODER6;		// set PA6 to analog -> Pot @ 6
}


void init_ADC(void)	{						// res == 0 for 12-bit
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN; 		// enable clock for ADC
	ADC1->CFGR1 &= ~(0xFFFFFFFF);			// Resets all data including alignment, single-shot mode
	ADC1->CFGR1 |= 0b10<<3;
}


void init_NVIC(void)	{
	NVIC_SetPriority(EXTI0_1_IRQn,3);
	NVIC_SetPriority(EXTI2_3_IRQn,3);
	NVIC_SetPriority(TIM14_IRQn,1);
	NVIC_EnableIRQ(EXTI0_1_IRQn);					// Enables EXTI Line 0 and 1 Interrupts
	NVIC_EnableIRQ(EXTI2_3_IRQn);					// Enables EXTI Line 2 and 3 Interrupts
	NVIC_EnableIRQ(TIM14_IRQn);						// setting the NVIC for TIM14
}


void init_EXTI(void)	{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;		// Enable CLK for SYSCFG and COMP
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;	// Map PA0->3 in External interrupt registers
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PA;
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI0_PA;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI0_PA;
	EXTI->IMR |= EXTI_IMR_MR0;	// Set all SW interrupt to falling edge (since SW's are pull-up)
	EXTI->IMR |= EXTI_IMR_MR1;
	EXTI->IMR |= EXTI_IMR_MR2;
	EXTI->IMR |= EXTI_IMR_MR3;
	EXTI->FTSR |= EXTI_FTSR_TR0; 					// Interrupt requests for SW's not masked
	EXTI->FTSR |= EXTI_FTSR_TR1;
	EXTI->FTSR |= EXTI_FTSR_TR2; 
	EXTI->FTSR |= EXTI_FTSR_TR3; 
}


void init_timer14(void)	{
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN; 			//enable clock for TIM14
	TIM14->PSC = 661;
	TIM14->ARR = 724; 								// period should be around 100 ms
	TIM14->DIER |= TIM_DIER_UIE; 					// enable the update event interrupt
	TIM14->CR1 |= TIM_CR1_CEN; 						// enable the counter
}

//LCD
	//////////////////////////////////////////////
	//IN LIBRARY:								//
	//	lcd_putchar(unsigned char character);	//
	//	lcd_putstring(char *instring);			//
	//	pulse_strobe();							//
	//	init_LCD();								//
	//	lcd_command(unsigned char command)		//
	//////////////////////////////////////////////
	void lcd_goto (int line ,int chara){	//function that takes curser to point on lcd
			if (line==0) lcd_command(0x80+chara);
			if (line==1) lcd_command(0xc0+chara);
	}

//Delays
	void s_delay (int time,char unit){
		int uni;
		if (unit=='s') uni = 2000000;
		if (unit=='m') uni = 2000;
		if (unit=='u') uni = 2;
		for(int c=0; c <(time*uni); c++);
	}
	void hang_delay(int us){
		/* Hangs for specified number of microseconds. */
		volatile uint32_t counter = 0;
		us *= 3;
		for(; counter<us; counter++) {
			__asm("nop");
			__asm("nop");
		}
	}
	
	void BCD_conv(int h, int s, int m){		//Number to BCD converter
		int m_2=0;							//initialise & save multiples of 10 for each input
		int s_2=0;
		int h_2=0;
		m_2=m/10;							
		timarry[0]=m_2+0b00110000;			//place bcd value offset by "0" for mult of 10
		timarry[1]=(m-(m_2*10)+0b00110000);	//"  							for units
		s_2=s/10;
		timarry[3]=s_2+0b00110000;
		timarry[4]=(s-(s_2*10)+0b00110000);
		h_2=h/10;
		timarry[6]=h_2+0b00110000;
		timarry[7]=(h-(h_2*10)+0b00110000);		
	}
	
//LEDs
	void ODRer(int pins, int CW){	//pins in binary
            if (CW==1){
				GPIOB->ODR |= pins;
			}
			if (CW==0){
				GPIOB->ODR &= ~(pins);
			}
		}

//Button Press
	int button(int num){					//function to indicate button press
		int buttons = GPIOA->IDR & 0b1111;
		if (buttons==(0b1111&~(1<<num))) return 1;
		else return 0;
	}
//********************************************************************
// END OF PROGRAM
//********************************************************************
