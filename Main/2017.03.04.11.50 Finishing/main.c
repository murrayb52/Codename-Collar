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
#define		level1	5
#define		level2	7
	
	// Resets/Presets
// this is the reset state for GPIOB
#define		reset	0b00011110
// this is the set state for GPIOB
#define		set		0b11100001
#define		mode_preset 4

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
	init_ports();									// enable clock for PA & PB; outputs = PB0-7; inputs, pull-up = PA0-2
	config_selr_buttons();							// change channel to 1 and freeze button control; change mode to shock and freeze button control
	init_interrupts();								// initialise interrupts for inputs PA0-3 [NOT ENABLED IN NVIC YET!]
	preset();										// change channel 1 shock to 50
	
	NVIC_EnableIRQ(EXTI0_1_IRQn);					// enable EXTI0_1 interrupt in the NVIC (chselr and mode_selr respectively)
	NVIC_EnableIRQ(EXTI2_3_IRQn);					// enable EXTI2_3 interrupt in the NVIC (transmit 1 and transmit 2 respectively)

	update_channel();								// force manual channel update (due to NVIC bug)
	GPIOB->ODR &= ~(0b1<<led_init);					// turn off 'initialise LED'
	
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
	//GPIOB->ODR |= (0b1<<mode);
    GPIOB->MODER |= GPIO_MODER_MODER4_0;        	// set PB4 to output
    GPIOB->MODER |= GPIO_MODER_MODER5_0;        	// set PB5 to output
    GPIOB->MODER |= GPIO_MODER_MODER6_0;        	// set PB6 to output
    GPIOB->MODER |= GPIO_MODER_MODER7_0;       		// set PB7 to output

	GPIOB->ODR |= 0b11110;							// reset outputs to switches (level_up, mode_selr, chselr & transmit pull-up pins)
	delay(3);
	GPIOB->ODR |= (0b1<<led_init);					// turn on 'initialise LED' (and reset all other outputs)

	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0; 			// set pull-up mode for PA0
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_0; 			// set pull-up mode for PA1
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0; 			// set pull-up mode for PA2
	
	/*	__________________
___________________________ CHECKED _____________________________________________	*/
}


void config_selr_buttons (void)	{
/*
This function disables the 'channel selector' & 'mode selector' remote buttons from being pressed during initialisation steps of start-up.
This is done by cycling through each selector by continuously 'pressing' and then 'releasing' the respective selector button until reaching the
default start option. After the last option of each cycle, the button is 'pressed' once more but not 'released'. This ensures that, until all
initialisations are complete and the buttons have been 'released' by the micro-controller, these buttons cannot be pressed and will remain in their
default start positions. This ensures that the micro-controller accurately keeps track of which channel/mode the remote is in.

NOTE: Upon configuration of chselr and mode_selr as outputs, each has already been pressed and released once.
	The current channel and mode are as follows:
		channel = 1
		mode = sound
*/

// Mode selector button: (set default start = shock; current = sound) 	[cycle = light, sound, vibrate, shock]
	toggle(3);									// toggle mode_selr button 		(enter sound mode)
	toggle(3);									// toggle mode_selr button 		(enter vibrate mode)
	//toggle(3);								// toggle mode_selr button 		(enter shock mode)
	mode_count = mode_preset;					// set mode_count to mode_preset of 4 (end of sequence)
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
	
	/*	/////////////////////////////////////// CAN'T CHECK ///////////////////////////////////////	*/
}


void preset (void)	{
	/*
	NOTE: On entering this loop, channel 1 and mode 'shock' are selected as default start positions.
		  Additionally, the level_up button cannot be 'pressed' while mode_selr is 'held' so mode_selr is 'released' to change this and then
		  pressed again after the correct level has been set. This effect does not hold for the chselr button.
	*/
	delay(1);
	level(5);										// change channel 1 to level X
	delay(2);
	toggle(2);										// change to  channel 2	
	delay(2);
	level(7);										// change channel 2 to level X

	toggle(2);										// change back to channel 1
	
	/*	_____________________________________________ CHECKED _____________________________________________	*/
}


void select_channel (int channel_select0)	{
/* This function detects which channel the remote is currently on by checking the value of 'channel_count' and adjusting to the correct channel accordingly.
In order to prevent a the 'channel_selr' button being pressed on the remote, the 'chselr' button is held down to freeze control. Therefore the channel is
changed twice at the end of the function to make sure 'chselr' is frozen even if the remote was on the correct function to begin.
*/
	delay(1);
	channel_parity = channel_count%2;							// channel parity even = 0 = channel 1
	if ((channel_parity) != (channel_select0 - 1))	{			// if channel_select != current channel, change channel
// NOTE: To change channel, micro-controller 'presses AND HOLDS' the channel select button to keep it on the rising edge. This process
// ensures that an accidental user button press will not change the channel and activate other collar since the pull up resistor must
// first fall before it can trigger again. [Button press is cleared after loop]
		toggle(2);												// change channel
	}
	/*	_____________________________________________ CHECKED _____________________________________________	*/
}




//********************************************************************
// NVIC Handlers
//********************************************************************

void EXTI0_1_IRQHandler (void)	{
	if( (EXTI->IMR & EXTI_IMR_MR0) && (EXTI->PR & EXTI_PR_PR0))	{		// interrupt on line 0 ('Transmit 1' press)
		delay(1);
		transmit(1);													// change remote to channel 1 and then transmit
		EXTI->PR |= EXTI_PR_PR0;										// clears interrupt flag
		}

	if( (EXTI->IMR & EXTI_IMR_MR1) && (EXTI->PR & EXTI_PR_PR1))	{		// interrupt on line 1 ('Transmit 2' press)
		delay(1);
		transmit(2);													// change remote to channel 2 and then transmit
		EXTI->PR |= EXTI_PR_PR1;										// clears interrupt flag (after button released - in order to prevent second interrupt)
		}
}

void EXTI2_3_IRQHandler (void)	{
	if( (EXTI->IMR & EXTI_IMR_MR2) && (EXTI->PR & EXTI_PR_PR2))	{		// interrupt on line 2 ('Channel_selr' press)
//	if ((GPIOB_ODR & 'channel press' signal) == 0): this is due to micro-controller output and interrupt is ignored.
//	if ((GPIOB_ODR & 'channel press' signal) != 0): this is due to user input and should be accounted for																	//  
		delay(1);
		update_channel();												// update channel
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
		GPIOB->ODR &= ~(0b1<<chselr);				// 'press' chselr button
		delay(2);
		GPIOB->ODR |= 0b1<<chselr;					// 'release' chselr button 		(relinquish control of button)
		delay(2);
		update_channel();
	}
	if (selr == 3)	{
		GPIOB->ODR &= ~(0b1<<mode_selr);			// 'press' mode_selr button 
		delay(2);
		GPIOB->ODR |= (0b1<<mode_selr);				// 'release' mode_selr button 		(relinquish control of button)
		delay(2);
		//update_mode();
		//display_mode();
	}
}

/*
void update_mode (void)	{
	mode_count += 1;
	if (mode_count == 5)	{
		mode_count = 1;
		GPIOB->ODR &= ~(0b1<<mode);
	}
	if (mode_count == 1)	{
		//GPIOB->ODR |= 0b1<<mode;
	}
	if (mode_count == 2)	{
		//GPIOB->ODR &= ~(0b1<<mode);
	}
	if (mode_count == 3)	{
		//GPIOB->ODR |= 0b1<<mode;
	}
	if (mode_count == 4)	{
		GPIOB->ODR |= 0b1<<mode;
	}
	while (mode_count > 5)	{
		GPIOB->ODR |= 0b1<<mode;
		delay(5);
		GPIOB->ODR &= ~(0b1<<mode);
	}
}
*/
void update_channel(void)	{
	channel_count += 1;
	channel_count = (channel_count%2);
	GPIOB->ODR &= ~(0b1<<chan);
	GPIOB->ODR |= (channel_count<<chan);
}
/*
void select_mode (void)	{
	moder = 4 - mode_count; 									// determine number of times to change mode to preset value of mode_count == 4
	while (moder != 0)	{										// change mode until number presses until mode 4 reached == 0 (i.e. until mode 4 reached)
		toggle(3);												// change mode (toggle pin 3) 
	}
	mode_count = mode_preset;									// reset mode_count back to preset value
}
*/
void transmit (int channel_select)	{
	select_channel (channel_select);							// change to desired channel (if necessary)
	delay(2);
	//select_mode();												// change to desired mode (if necessary)
	while ((GPIOA->IDR & (0b1<<(channel_select - 1))) == 0)	{	// while activate button held
		GPIOB->ODR &= ~(0b1<<transmit_pin);						// transmit signal [CONTINUOUSLY]
		GPIOB->ODR |= (0b1<<led_transmit);						// activate transmission LED
	}
	GPIOB->ODR |= 0b1<<transmit_pin;							// stop transmission after activate button released by user
	GPIOB->ODR |= 0b1<<chselr;									// 'release' channel select button after transmit button released by user
	GPIOB->ODR &= ~(0b1<<led_transmit);							// deactivate transmission LED
}



void display_mode(void)	{
	GPIOB->ODR &= ~(0b1<<mode);
	GPIOB->ODR |= ((mode_count%4)<<mode);					// display current channel on pin defined as 'chan' [0 = channel 1/3]
}



void level(int lvl)	{
	int i;
	for (i = 0; i < lvl; i++)	{					// set channel X to lvl
		GPIOB->ODR &= ~(0b1<<level_up);				// 'press' level_up
		delay(2);
		GPIOB->ODR |= 0b1<<level_up;				// 'release' level_up
		delay(2);
	}
}


void delay (int wait)	{
	int k = 120000*wait;
	for (j = 0; j < k; j++);
}


//********************************************************************
// END OF PROGRAM
//********************************************************************