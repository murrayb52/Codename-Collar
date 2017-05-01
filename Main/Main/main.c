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
	
	// ---------- PORT A BIT NUMBER CONSTANTS ----------	
	// Remote I/O (grouped in I/O pairs if applicable)
#define    	channel_interrupt	0		// INPUT to micro: 'CHANNEL' press
#define	   	chselr				1		// OUTPUT to remote: 'CHANNEL' force press

//#define--------mode_interrupt-------2------// INPUT to micro: 'MODE' press----------- **NOT USED**-----
#define	   	mode_selr			3		// OUTPUT to remote: 'MODE' force press

#define 	level_up			4		// OUTPUT: 'LEVEL_UP' press

#define 	transmit_pin		5		// OUTPUT: 'TRANSMIT' press

#define 	chan 				6 		// --- FOR TESTING: OUTPUT: 'chan' indicator LED start address

	// User I/O pairs
#define	   	transmit_1			8		// INPUT: 'RED BUTTON' press
#define	   	led_transmit_1		10		// OUTPUT: led_transmit_1

#define	   	transmit_2			9		// INPUT: 'BLUE BUTTON' press
#define	   	led_transmit_2		11		// OUTPUT: led_transmit_2

	// ---------- PORT B BIT NUMBER CONSTANTS ----------
#define 	led_wait			0		// OUTPUT: led_wait


	// Define time values [no. of cycles]
#define 	half				4000000
#define		two					16000000
#define		five				40000000
#define		ten					80000000

	// Define level values
#define		level1				50
#define		level2				50
	
	// Resets/Presets
// this is the reset state for GPIOA where all outputs will send deactivating signals to all their respective destinations.
// "GPIOA->ODR |= reset" sends a logic high to all outputs connected to the remote force inputs
// (which are interpreted by the remote pull-up switches as a 'release' signal)
#define		reset				0b00111010
// this is the set state for GPIOA where all outputs will send activating signals to all their respective destinations.
// "GPIOA->ODR &= ~reset" sends a logic low to all general outputs e.g. LED indicators 
#define		set					0b11000101

// The mode_preset value corresponds to the preset use/mode of the collar [1: light, 2: sound, 3: vibrate, 4: shock]
#define		mode_preset 		4		
//====================================================================
// GLOBAL VARIABLES
//====================================================================
int channel_count = 0;
int mode_count = 1;
int moder;
int channel_parity;
int j;
int chan_press = 0;
int mode_press = 0;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void main (void);
void init_ports (void);
void config_selr_buttons (void);
void init_interrupts (void);
void transmit (int channel_select);
void select_channel (int channel_select0);
void select_mode (void);
void preset (void);
void delay (int wait);
void update_channel (void);
void toggle (int selr);
void level (int lvl);
void update_mode (void);
void display_mode (void);

//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)	{
	init_ports();									// enable clock for PA & PB; set inputs and outputs
	config_selr_buttons();							// change channel to 1 and freeze button control; change mode to shock and freeze button control
	init_interrupts();								// initialise interrupts for inputs PA0-3 [NOT ENABLED IN NVIC YET!]
	preset();										// change channel 1 shock to 50
	
	NVIC_EnableIRQ(EXTI0_1_IRQn);					// enable EXTI0_1 interrupt in the NVIC (chselr and mode_selr respectively)
	NVIC_EnableIRQ(EXTI2_3_IRQn);					// enable EXTI2_3 interrupt in the NVIC (transmit 1 and transmit 2 respectively)

	//update_channel();								// force manual channel update (due to NVIC bug)
	GPIOB->ODR &= ~(0b1<<led_wait);					// turn off 'initialise LED'
	
	for(;;)	{										// Loop forever
	}
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
 
	GPIOA->MODER &= ~GPIO_MODER_MODER0; 			// set PA0 to input 				channel_interrput
	GPIOA->MODER |= GPIO_MODER_MODER1_0;        		// set PA1 to output 			chselr
//	GPIOA->MODER &= ~GPIO_MODER_MODER2; 			// set PA2 to input 				mode_interrupt
	GPIOA->MODER |= GPIO_MODER_MODER3_0;        		// set PA3 to output 			mode_selr
	GPIOA->MODER |= GPIO_MODER_MODER4_0;        		// set PA4 to output 			level_up
    GPIOA->MODER |= GPIO_MODER_MODER5_0;       			// set PA5 to output 			transmit
    GPIOA->MODER |= GPIO_MODER_MODER6_0;        		// set PA6 to output 			--- FOR TESTING: channel 1 indicator ---
    GPIOA->MODER |= GPIO_MODER_MODER7_0;       			// set PA7 to output 			--- FOR TESTING: channel 2 indicator ---
	GPIOA->MODER &= ~GPIO_MODER_MODER8; 			// set PA8 to input 				transmit_1
	GPIOA->MODER |= GPIO_MODER_MODER10_0;       		// set PA10 to output 			led_transmit_1
	GPIOA->MODER &= ~GPIO_MODER_MODER9; 			// set PA9 to input 				transmit_2
    GPIOA->MODER |= GPIO_MODER_MODER11_0;       		// set PA11 to output 			led_transmit_2
    
    GPIOB->MODER |= GPIO_MODER_MODER0_0;        		// set PB0 to output 			led_wait

/* NOTE: Setting PA5 to output automatically outputs the reset value [zero] which will send a 'transmit' signal and activate the collar.
		However, since this is one of the first steps completed after resetting power to the remote, the 'transmit' action will transmit
		a 'light' mode signal. This will also only happen for a negligible amount of time so there is no chance of accidentally
		activating the shock mode of the collar and causing harm to the animal.
		*/

	GPIOA->ODR |= 0b111010;							// send 'release' signal to remote buttons (transmit, level_up, mode_selr & chselr pull-up pins)
	delay(3);
	GPIOB->ODR |= (0b1<<led_wait);					// turn on 'initialise LED'

	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0; 			// set pull-up mode for PA0
//--GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0;------------// set pull-up mode for PA2	---------- **NOT USED** -----
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR8_0; 			// set pull-up mode for PA8
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_0; 			// set pull-up mode for PA9
}


void config_selr_buttons (void)	{
/*
	The current channel and mode upon entering this function are as follows:
		channel = 1
		mode = light
*/
// Mode selector button: (set default start = shock; current = sound) 	[cycle = 1: light, 2: sound, 3: vibrate, 4: shock]
	if (mode_preset >= 2)	{
		toggle(3);									// toggle mode_selr button 		(enter sound mode)
		}
	if (mode_preset >= 3)	{
		toggle(3);									// toggle mode_selr button 		(enter vibrate mode)
		}
	if (mode_preset == 4) 	{
		toggle(3);									// toggle mode_selr button 		(enter shock mode)
		}
	mode_count = mode_preset;						// set mode_count to mode_preset value (4 = end of sequence)
}



void init_interrupts (void)	{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;		// enable CLK for SYSCFG  controller
// Map PA0->2 in External interrupt registers
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;	// map PA0 to EXTI0
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PA;	// map PA1 to EXTI1
	SYSCFG->EXTICR[8] |= SYSCFG_EXTICR1_EXTI0_PA;	// map PA8 to EXTI2
	SYSCFG->EXTICR[9] |= SYSCFG_EXTICR1_EXTI0_PA;	// map PA9 to EXTI3
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
}


void preset (void)	{
	/*
	NOTE: On entering this loop, channel 1 and mode 'mode_preset' are selected as default start positions.
		  Additionally, the level_up button cannot be 'pressed' while mode_selr is 'held' so mode_selr is 'released' to change this and then
		  pressed again after the correct level has been set. This effect does not hold for the chselr button.
	*/
	delay(1);
	level(level1);										// change channel 1 to level X
	delay(2);
	toggle(2);											// change to channel 2	
	delay(2);
	level(level2);										// change channel 2 to level X
	delay(2);
	toggle(2);											// change back to channel 1
}



//********************************************************************
// NVIC Handlers
//********************************************************************

void EXTI0_1_IRQHandler (void)	{
	if( (EXTI->IMR & EXTI_IMR_MR0) && (EXTI->PR & EXTI_PR_PR0))	{		// interrupt on line 0 ('Transmit 1' press)
		//delay(1);
		transmit(1);													// change remote to channel 1 and then transmit
		EXTI->PR |= EXTI_PR_PR0;										// clears interrupt flag
		}

	if( (EXTI->IMR & EXTI_IMR_MR1) && (EXTI->PR & EXTI_PR_PR1))	{		// interrupt on line 1 ('Transmit 2' press)
		//delay(1);
		transmit(2);													// change remote to channel 2 and then transmit
		EXTI->PR |= EXTI_PR_PR1;										// clears interrupt flag (after button released - in order to prevent second interrupt)
		}
}

void EXTI2_3_IRQHandler (void)	{
	if((EXTI->IMR & EXTI_IMR_MR2) && (EXTI->PR & EXTI_PR_PR2) && !(EXTI->PR & EXTI_PR_PR3))	{		// interrupt on line 2 ('Channel_selr' press)
//	if ((GPIOA_ODR & 'channel press' signal) == 0): this is due to micro-controller output and interrupt is ignored.
//	if ((GPIOA_ODR & 'channel press' signal) != 0): this is due to user input and should be accounted for																	//  
		for (j = 0; j < 155325; j++);									// ensure that button press on remote actually activates chselr before updating
		if (((GPIOA->IDR) & (0b1<<chselr)) == 0)	{					// if, after one remote clock cycle, button still pushed: remote channel changes, therefore update channel 
			update_channel();											// update channel 
		}
		EXTI->PR |= EXTI_PR_PR2;										// clears interrupt flag
	}
	
/*	if( (EXTI->IMR & EXTI_IMR_MR3) && (EXTI->PR & EXTI_PR_PR3) && !(EXTI->PR & EXTI_PR_PR2))	{		// interrupt on line 3 ('Mode_selr' press)
		EXTI->PR |= EXTI_PR_PR3;										// clears interrupt flag (ignore mode interrupt
	}
*/	
}


//********************************************************************
// GENERIC FUNCTIONS
//********************************************************************

void toggle (int selr)	{
	if (selr == 2)	{
		GPIOA->ODR &= ~(0b1<<chselr);				// 'press' chselr button
		delay(2);
		GPIOA->ODR |= 0b1<<chselr;					// 'release' chselr button 		(relinquish control of button)
		delay(2);
		update_channel();
	}
	if (selr == 3)	{
		GPIOA->ODR &= ~(0b1<<mode_selr);			// 'press' mode_selr button 
		delay(2);
		GPIOA->ODR |= (0b1<<mode_selr);				// 'release' mode_selr button 	(relinquish control of button)
		delay(2);
		//update_mode();
		//display_mode();
	}
}


void update_channel(void)	{
/* This function primarily updates the 'channel_count' value when user presses 'channel_interrupt' button on remote. Secondary function
	is for troubleshooting when using testing dev board (UCT dev board) - the function will indicate (with LEDs) which channel the remote is currently
	set to. PA6 = channel 1;  PA7 = channel 2
*/
	channel_count += 1;								// increase 'channel_count' value (0 = channel 1; 1 = channel 2)
	channel_count = (channel_count%2);				// update 'channel_count' value to correspond to PA6 or PA7 (TESTING indicator LEDs)
	GPIOA->ODR &= ~(0b11<<chan);					// reset 'chan' indicator LED 	--- FOR TESTING: to reset channel indicators ----------
	GPIOA->ODR |= (channel_count<<chan);			// set 'chan' indicator LED 	--- FOR TESTING: indicate channel 1 (PA6) or 2 (PA7)---
}



void transmit (int channel_select)	{
	select_channel (channel_select);							// change to desired channel (if necessary)
	GPIOA->ODR &= ~(0b11<<chan);								// ----- FOR TESTING: clear channel transmit indicators -----
	GPIOA->ODR |= ((0b01 + channel_count)<<chan);				// ----- FOR TESTING: PB6 = channel 1; PB7 = channel 2 ------
	delay(2);
	while ((GPIOA->IDR & (0b1<<(transmit_1 + channel_select - 1))) == 0)	{	// while activate button held
		GPIOA->ODR &= ~(0b1<<transmit_pin);						// transmit signal [CONTINUOUSLY]
		GPIOA->ODR |= (0b1<<(led_transmit_1 + channel_select - 1);				// activate transmission LED
	}
	GPIOA->ODR |= 0b1<<transmit_pin;							// stop transmission after activate button released by user
	GPIOA->ODR |= 0b1<<chselr;									// 'release' channel select button after transmit button released by user
	GPIOA->ODR &= ~(0b11<<chan);								// ----- FOR TESTING: clear channel transmit indicators ------
	GPIOA->ODR &= ~(0b11<<led_transmit_1);						// deactivate transmission LEDs
}


void select_channel (int channel_select0)	{
/* This function detects which channel the remote is currently on by checking the value of 'channel_count' and adjusting to the correct channel accordingly.
In order to prevent a the 'channel_selr' button being pressed on the remote, the 'chselr' button is held down to freeze control. Therefore the channel is
changed twice at the end of the function to make sure 'chselr' is frozen even if the remote was on the correct function to begin.
*/
	//delay(1);
	channel_parity = channel_count%2;							// channel parity even = 0 = channel 1
	if ((channel_parity) != (channel_select0 - 1))	{			// if channel_select != current channel, change channel
// NOTE: To change channel, micro-controller 'presses AND HOLDS' the channel select button to keep it on the rising edge. This process
// ensures that an accidental user button press will not change the channel and activate other collar since the pull up resistor must
// first fall before it can trigger again. [Button press is cleared after loop]
		toggle(2);												// change channel
	} 
}


void level(int lvl)	{
	int i;
	for (i = 0; i < lvl; i++)	{					// set channel X to lvl
		GPIOA->ODR &= ~(0b1<<level_up);				// 'press' level_up
		for (j = 0; j < 165000; j++);
		GPIOA->ODR |= 0b1<<level_up;				// 'release' level_up
		for (j = 0; j < 1000; j++);
	}
}


void delay (int wait)	{
	int k = 120000*wait;
	for (j = 0; j < k; j++);
}


//********************************************************************
// END OF PROGRAM
//********************************************************************