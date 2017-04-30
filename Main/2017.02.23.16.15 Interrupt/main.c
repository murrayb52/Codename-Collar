//*==================================================================*
//* WRITTEN BY: Murray Buchanan        		                         *
//* DATE CREATED: 14/02/2017                                         *
//* MODIFIED:                                                        *
//********************************************************************
// INCLUDE FILES
//====================================================================
#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
//====================================================================
// GLOBAL CONSTANTS
//====================================================================

//====================================================================
// GLOBAL VARIABLES
//====================================================================

//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void main (void);
void init_ports (void);
void init_interrupts (void);

//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)	{
	
	init_ports();									// enable clock for PA & PB; outputs = PB0-7; inputs, pull-up = PA0-2		
	init_interrupts();								// initialise interrupts for inputs PA0-3 [NOT ENABLED IN NVIC YET!]
	
	GPIOB->ODR |= 0b1<<0;							// turn on 'initialise LED' (and reset all other outputs)
	
	NVIC_EnableIRQ(EXTI0_1_IRQn);					// enable EXTI0_1 interrupt in the NVIC (chselr and mode_selr respectively)
	NVIC_EnableIRQ(EXTI2_3_IRQn);					// enable EXTI2_3 interrupt in the NVIC (transmit 1 and transmit 2 respectively)

	for(;;);										// Loop forever
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
		
	GPIOB->ODR &= 0b00000000;						// reset all outputs [maintain notification outputs PB6 & PB7]

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

	//NVIC_EnableIRQ(EXTI0_1_IRQn);					// enable EXTI0_1 interrupt in the NVIC (lines 0 and 1)81
	//NVIC_EnableIRQ(EXTI2_3_IRQn);					// enable EXTI2_3 interrupt in the NVIC (lines 2 and 3)
	
	/*	/////////////////////////////////////// CAN'T CHECK ///////////////////////////////////////	*/
}



void EXTI0_1_IRQHandler (void)	{
	if( (EXTI->IMR & EXTI_IMR_MR0) && (EXTI->PR & EXTI_PR_PR0))	{		// interrupt on line 0 ('Transmit 1' press)
		GPIOB->ODR |= 0b1<<4;													// change remote to channel 1 and then transmit
		EXTI->PR |= EXTI_PR_PR0;										// clears interrupt flag
		}

	if( (EXTI->IMR & EXTI_IMR_MR1) && (EXTI->PR & EXTI_PR_PR1))	{		// interrupt on line 1 ('Transmit 2' press)
		GPIOB->ODR |= 0b1<<5;													// change remote to channel 2 and then transmit
		EXTI->PR |= EXTI_PR_PR1;										// clears interrupt flag (after button released - in order to prevent second interrupt)
		}
}

void EXTI2_3_IRQHandler (void)	{
	if( (EXTI->IMR & EXTI_IMR_MR2) && (EXTI->PR & EXTI_PR_PR2))	{	// interrupt on line 2 ('Channel_selr' press)
		GPIOB->ODR |= 0b1<<6;
		EXTI->PR |= EXTI_PR_PR2;			// clears interrupt flag
	}
		
	if( (EXTI->IMR & EXTI_IMR_MR3) && (EXTI->PR & EXTI_PR_PR3))	{	// interrupt on line 3 ('Mode_selr' press)
		GPIOB->ODR |= 0b1<<7;
		EXTI->PR |= EXTI_PR_PR3;									// clears interrupt flag
	}	
}

//********************************************************************
// END OF PROGRAM
//********************************************************************