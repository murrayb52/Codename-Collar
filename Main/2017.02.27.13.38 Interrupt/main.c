//********************************************************************
//*                    Project: Codename Collar	                     *
//*                    Developed for: Gert Coetzee                   *
//*==================================================================*
//* WRITTEN BY: Murray Buchanan        		                         *
//* DATE CREATED: 14/02/2017                                         *
//* MODIFIED:                                                        *
//*==================================================================*
//* PROGRAMMED IN: Eclipse Luna Service Release 1 (4.4.1)            *
//* DEV. BOARD:    Custom Development board using STM32F01           *
//*==================================================================*
//* DESCRIPTION:                                                     *
//* The following document contains the code that runs the hardware  *
//* for project 'Codename Collar'. This hardware board attaches onto *
//* a modified Petainer collar (Model no. IS-PET998DR-1) to enable   *
//* the user to easily use one Petainer remote to control two		 *
//* collars. This is done by rewiring the remote to have 2 extra     *
//* buttons that will automatically select their respective channel  *
//* and activate the collar with one press - while still keeping all *
//* original functionality of the remote and collar.				 *
//*																	 *
//********************************************************************
// INCLUDE FILES
//====================================================================
#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
//====================================================================
// GLOBAL CONSTANTS
//====================================================================
	
	// Port A bit number constants
#define	   	transmit_1	0				// user button 1
#define	   	transmit_2	1				// user button 2
#define    	channel_interrupt	2		// intercept signal
#define    	mode_interrupt		3		// intercept signal

	// Port B bit number constants
#define    	led_init		0
#define	   	transmit_pin	1
#define	   	chselr		2
#define	   	mode_selr	3
#define    	level_up		4
#define    	led_transmit	5
#define	   	chan		6
#define	   	mode		7

	// Define time values [no. of cycles]
#define 	half	4000000
#define		two		16000000
#define		five	40000000
#define		ten		80000000

	// Define level values
#define		level1	3
#define		level2	4

// this is the reset state for GPIOB
#define		reset	0b00011110
// this is the set state for GPIOB
#define		set		0b11100001

//====================================================================
// GLOBAL VARIABLES
//====================================================================
int channel_count = 0;
int mode_count;
int channel_parity;
int j;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void main (void);
void init_ports (void);
void init_interrupts (void);
void delay (int loop);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)	{
	init_ports();									// enable clock for PA & PB; outputs = PB0-7; inputs, pull-up = PA0-2
	init_interrupts();								// initialise interrupts for inputs PA0-3 [NOT ENABLED IN NVIC YET!]
	
	GPIOB->ODR |= 0b1<<chselr;						// 'release' chselr button 		(relinquish control of button to user)
/*	[BUTTON ALREADY RELEASED]
	GPIOB->ODR |= 0b1<<mode_selr;					// 'release' mode_selr button 	(relinquish control of button to user)
*/

// reset GPIOB but exclude:
//		- PB7: Mode Notification
//		- PB6: Channel Notification
//		- PB0: init_led
//	NOTE: full reset = 0b00011110	
//	GPIOB->ODR &= 0b11011111;						// this releases 'chselr' and 'mode_selr' [among other things]
	
	NVIC_EnableIRQ(EXTI0_1_IRQn);					// enable EXTI0_1 interrupt in the NVIC (chselr and mode_selr respectively)
	NVIC_EnableIRQ(EXTI2_3_IRQn);					// enable EXTI2_3 interrupt in the NVIC (transmit 1 and transmit 2 respectively)

//	GPIOB->ODR &= ~(0b1<<led_init);					// turn off 'initialise LED'
	GPIOB->ODR &= ~0b11111111;						// reset all except led_init

	
	
	for(;;)	{										// Loop forever
/*	if (chan_press == 1)	{
		channel_count += 1;											// if press (whether user or micro-controller), increase channel count by 1							
		// Update current channel [channel_count]
		channel_count = (channel_count%2);							// display current channel on pin defined as 'chan' [0 = channel 1]
		GPIOB->ODR &= ~(0b1<<chan);
		GPIOB->ODR |= (channel_count<<chan);
		chan_press = 0;
		}
	if (mode_press == 1)	{
		mode_count += 1;
		// Update current channel [channel_count]
		mode_count = (mode_count%2);							// display current mode on pin defined as 'mode' [0 = channel 1/channel 3]
		GPIOB->ODR &= ~(0b1<<mode);
		GPIOB->ODR |= (mode_count<<mode);
		mode_press = 0;
		}
*/	}
}													// End of main


//********************************************************************
// END OF MAIN FUNCTION
//********************************************************************


//====================================================================
// SECONDARY FUNCTIONS
//====================================================================
void init_ports (void)	{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;    			// enable clock for GPIOA
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;    			// enable clock for GPIOB
 
	GPIOA->MODER &= ~GPIO_MODER_MODER0; 			// set PA0 to input
	GPIOA->MODER &= ~GPIO_MODER_MODER1; 			// set PA1 to input
	GPIOA->MODER &= ~GPIO_MODER_MODER2; 			// set PA2 to input
    GPIOB->MODER |= GPIO_MODER_MODER0_0;        	// set PB0 to output
/* NOTE: Setting PB1 to output automatically outputs the reset value [zero] which will send a 'transmit' signal. However, since this is one of the first steps
		completed after resetting power to the remote, the 'transmit' action will transmit a 'light' mode signal. This will also only happen for a minuscule
		period of time (a period whereby the action is practically negated).
		*/
    GPIOB->MODER |= GPIO_MODER_MODER1_0;        	// set PB1 to output
    GPIOB->MODER |= GPIO_MODER_MODER2_0;        	// set PB2 to output
    GPIOB->MODER |= GPIO_MODER_MODER3_0;        	// set PB3 to output
    GPIOB->MODER |= GPIO_MODER_MODER4_0;        	// set PB4 to output
    GPIOB->MODER |= GPIO_MODER_MODER5_0;        	// set PB5 to output
    GPIOB->MODER |= GPIO_MODER_MODER6_0;        	// set PB6 to output
    GPIOB->MODER |= GPIO_MODER_MODER7_0;       		// set PB7 to output
	
	GPIOB->ODR |= 0b1<<led_init;					// turn on 'initialise LED' (and reset all other outputs)
	
// Update current channel [channel_count]
	channel_count += 1;
	channel_count = (channel_count%2);
	GPIOB->ODR &= ~(0b1<<chan);
	GPIOB->ODR |= (channel_count<<chan);
		
	GPIOB->ODR &= 0b11011111;						// reset all outputs [maintain notification outputs PB6 & PB7 and LED_init]
	GPIOB->ODR |= 0b011110;							// reset transmit, chselr & mode_selr pins

	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0; 			// set pull-up mode for PA0
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_0; 			// set pull-up mode for PA1
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0; 			// set pull-up mode for PA2
	
	/*	_____________________________________________ CHECKED _____________________________________________	*/
}



void init_interrupts (void)	{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;		// enable CLK for SYSCFG  controller
// Map PA0->2 in External interrupt registers
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;	// map PA0 to EXTI0
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PA;	// map PA1 to EXTI1
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI0_PA;	// map PA2 to EXTI2
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI0_PA;	// map PA3 to EXTI3
// Unmask external line EXTI0
	EXTI->IMR |= EXTI_IMR_MR0;						// unmask external interrupt 0
	EXTI->IMR |= EXTI_IMR_MR1;						// unmask external interrupt 1
	EXTI->IMR |= EXTI_IMR_MR2;						// unmask external interrupt 2
	EXTI->IMR |= EXTI_IMR_MR3;						// unmask external interrupt 3
// Set all SW interrupts to falling edge (since SW's are pull-up)
	EXTI->FTSR |= EXTI_FTSR_TR0;
	EXTI->FTSR |= EXTI_FTSR_TR1;
	EXTI->FTSR |= EXTI_FTSR_TR2;
	EXTI->FTSR |= EXTI_FTSR_TR3;

	//NVIC_SetPriority(EXTI0_1_IRQn,2);
	//NVIC_SetPriority(EXTI2_3_IRQn,2);
	
	//NVIC_EnableIRQ(EXTI0_1_IRQn);					// enable EXTI0_1 interrupt in the NVIC (lines 0 and 1)81
	//NVIC_EnableIRQ(EXTI2_3_IRQn);					// enable EXTI2_3 interrupt in the NVIC (lines 2 and 3)
	
	/*	/////////////////////////////////////// CAN'T CHECK ///////////////////////////////////////	*/
}

void delay (int loop)	{
	int i;
	for (i = 0; i < loop; i++)	{
		for (j = 0; j < 1000; j++);
	}
}


void EXTI0_1_IRQHandler (void)	{
	if( (EXTI->IMR & EXTI_IMR_MR0) && (EXTI->PR & EXTI_PR_PR0))	{		// interrupt on line 0 ('Transmit 1' press)
		// Update current channel [channel_count]
		channel_count += 1;
		channel_count = (channel_count%2);								// display current channel on pin defined as 'chan' [0 = channel 1]
		GPIOB->ODR &= ~(0b1);
/*		if ((channel_count != 1) && (channel_count !=0))	{
			GPIOB->ODR |= (0b1<<7);
		}
*/		if (channel_count == 1)	{
			GPIOB->ODR |= 0b1;
		}
		EXTI->PR |= EXTI_PR_PR0;										// clears interrupt flag
		}

	if( (EXTI->IMR & EXTI_IMR_MR1) && (EXTI->PR & EXTI_PR_PR1))	{		// interrupt on line 1 ('Transmit 2' press)
		// Update current channel [channel_count]
		channel_count += 1;
		channel_count = (channel_count%2);								// display current channel on pin defined as 'chan' [0 = channel 1]
		GPIOB->ODR &= ~(0b1<<5);
		GPIOB->ODR |= (0b11111000);
		EXTI->PR |= EXTI_PR_PR1;										// clears interrupt flag (after button released - in order to prevent second interrupt)
		}
}

void EXTI2_3_IRQHandler (void)	{
	if( (EXTI->IMR & EXTI_IMR_MR2) && (EXTI->PR & EXTI_PR_PR2))	{		// interrupt on line 2 ('Channel_selr' press)
		// Update current channel [channel_count]
		channel_count += 1;
		channel_count = (channel_count%2);								// display current channel on pin defined as 'chan' [0 = channel 1]
		GPIOB->ODR &=(0b0);
		GPIOB->ODR |= (channel_count<<7);
		
//		delay();
//		chan_press = 1;
		EXTI->PR |= EXTI_PR_PR2;			// clears interrupt flag
	}
		if( (EXTI->IMR & EXTI_IMR_MR3) && (EXTI->PR & EXTI_PR_PR3))	{	// interrupt on line 3 ('Mode_selr' press)
		// Update current channel [channel_count]
		channel_count += 1;
		channel_count = (channel_count%2);								// display current channel on pin defined as 'chan' [0 = channel 1]
		GPIOB->ODR &= ~(0b1<<5);
		GPIOB->ODR |= (0b11111);
		
//		delay();
//		mode_press = 1;													// if press (whether user or micro-controller), increase mode count by 1
		EXTI->PR |= EXTI_PR_PR3;										// clears interrupt flag
		}	
	}

//********************************************************************
// END OF PROGRAM
//********************************************************************