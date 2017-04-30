#define STM32F051

#include <stdint.h>
#include "stm32f0xx.h"

//const
#define    LIGHT_TRIGGER   1423
#define    Counter_offset  80 //useconds
#define    PROX_CLOSE_DIFF 200
#define    TIME_GRIP_CLOSE 40//time for gripper to close in /10 seconds
#define    TIME_ARM_UP     40//time for gripper to close in /10 seconds
#define    LEFT_STALL      600000 //motor stall /1 000 000
#define    RIGHT_STALL     600000
#define    CHANGE_FACTOR   100 //10 slow 50 vfast
#define    PWM_MULT		   80//

#define    LEGS_EN         0x01
#define    ARM_EN          0x02
#define    GRIPPER_EN      0x04
#define    LEGS_DIS        0x08
#define    ARM_DIS         0x10
#define    GRIPPER_DIS     0x20//

#define    L         	   0x01
#define    R         	   0x02
#define    A         	   0x04
#define    G         	   0x08//

#define    F         	   0x01
#define    B         	   0x02//

#define    SENSOR0         0x0
#define    SENSOR1         0x1
#define    SENSOR2         0x2
#define    LINE3           0x3
#define    LINE2           0x4
#define    LIGHT		   0x5
#define    LINE0		   0x6
#define    PROX 		   0x7
#define    LINE1		   0x8
#define    SENSOR9		   0x9//

#define    LR_on           0x01
#define    LR_off          0x02
#define    RR_on           0x04
#define    RR_off          0x08
#define    G_on            0x10
#define    G_off		   0x20

# define GPIO_AFRL_PIN3_AF2  0b00000000000000000010000000000000;
# define GPIO_AFRL_PIN4_AF1  0b00000000000000010000000000000000;
# define GPIO_AFRL_PIN5_AF1  0b00000000000100000000000000000000;
# define GPIO_AFRL_PIN1_AF1  0b00000000000000000000000000010000;

//var
int speedr; //speed converted to direction
int readers[10];
int cals[10];
int speeds[4];
int counters[4];
int counter=0;
int temp=0;
int temp2=0;
int phase=0;


//DECLAR
void init_timers(void);
void init_GPIO_ADC(void);
void set(unsigned char);
int ADC_read(int);
void drive(unsigned char, int, unsigned char);
void call_speed(void);
void led(unsigned char);
void read_sens(void);
int	ADC_read_chan(int);
void calibrator(void);
void s_delay (int,char);



void main(void) {
	init_GPIO_ADC();
	init_timers();
	
	//CALLIBRATION
	led(G_on); //-0-	wait for start
	while ((GPIOC->IDR&0b1<<13)!=0);
	led(LR_on);
	led(RR_on);
	led(G_off); //0-0	busy with cal
	calibrator();
	led(LR_off);
	led(RR_off);
	led(G_on); //-0-	waiting for ready
	while ((GPIOC->IDR&0b1<<13)!=0);
	
	//Phase 1 - light
	led(LR_on);
	led(G_off);	//o-O   waiting for green light
	phase=1;
	int counter=0;
	while (counter==0){
		read_sens();
		if ((readers[LIGHT])>(cals[LIGHT])+400)counter=1;
	}
	
	//Phase 2 - walk
	counter=0;
	phase=2;
	//TIM14->ARR=700; //f#$@ fast 0.25s
	led(LR_off);
	led(RR_off);
	led(G_on);	//-0-    GO!!!!!!!
	set(LEGS_EN);
	speeds[0]=900000;
	speeds[1]=900000;
	drive(L,speeds[0],F);
	drive(R,speeds[1],F);
	//code in handler
	
	//Phase 3 - grip
	while (readers[PROX]>1400);
	phase=0;
	led(LR_on);
	led(RR_off);
	led(G_off);	//0--    Gripping
	drive(L,0,F);
	drive(R,0,F);
	set(LEGS_DIS); //stop legs
	set(GRIPPER_EN);//start gripper
	drive(G,800000,B);
	s_delay(1   bgvyt,'s');nn
	set(GRIPPER_DIS);
	
	//Phase 4 - arm_up
	led(LR_on);
	led(RR_on);
	led(G_off);	//0-0   Arming
	set(ARM_EN);
	drive(A,800000,F);
	s_delay(3,'s');
	set(ARM_DIS);
	
	//Phase 5 - Walk back
	led(LR_off);
	led(RR_off);
	led(G_on);	//---    GO BACK!!!!!!!	
	phase=5;
	//TIM14->ARR=700; //f#$@ fast 0.25s
	counters[0]=0;
	counters[1]=0;
	set(LEGS_EN);
	speeds[0]=700000;
	speeds[1]=700000;
	drive(L,speeds[0],B);
	drive(R,speeds[1],B);
	//code in handler
	
	//Phase 5.5 - stop
	while(temp<3){ //counting crosses
		if ((readers[LINE2]>=cals[LINE2]+600)&&(readers[LINE3]>=cals[LINE3]+600)){
			while ((readers[LINE2]>=cals[LINE2]+600)&&(readers[LINE3]>=cals[LINE3]+600));
			temp++;
		}
	}
	//last crossing
	while ((readers[LINE2]<=cals[LINE2]+600)&&(readers[LINE3]<=cals[LINE3]+600));
	phase=0;
	set(LEGS_DIS);
	set(ARM_EN);
	drive(A,800000,F);
	led(LR_on);
	led(RR_off);
	led(G_off);	//0--    Arming
	s_delay(3000,'m');
	set(ARM_DIS);
	set(GRIPPER_EN);
	drive(G,800000,B);
	led(LR_on);
	led(RR_on);
	led(G_on);	//000    Letting Go!!!
	s_delay(3000,'m');
	set(GRIPPER_DIS);
	drive(G,0,B);
	led(LR_off);
	led(RR_off);
	
	
	
  for(;;);
}

//interrupts
void TIM14_IRQHandler(void){
	TIM14->SR &= ~(TIM_SR_UIF);
	if (phase==1){
		if ((GPIOB->ODR & 0b1<<6)==0){
			led(LR_off);
			led(RR_on);
		}
		else{
			led(LR_on);
			led(RR_off);
		}
	}
	if (phase==2){
		counters[0]++;
		counters[1]++;
		read_sens();
		//indicators
		if (readers[LINE0]>cals[LINE0]+600) led(LR_on);
		else led(LR_off);
		if (readers[LINE1]>cals[LINE1]+600) led(RR_on);
		else led(RR_off);
		if ((readers[LINE0]<=cals[LINE0]+600)&&(readers[LINE1]<=cals[LINE1]+600)) {
		led(G_on);
		counters[0]=0;	//if middle is detected reset change multipliers
		counters[1]=0;
		}
		else led(G_off);
		//speed decrease
		if (readers[LINE0]>cals[LINE0]+600){
			if (speeds[0]>=LEFT_STALL) speeds[0]=speeds[0]-((counters[0]+Counter_offset)*CHANGE_FACTOR);	
		}
		if (readers[LINE1]>cals[LINE1]+600){
			if (speeds[1]>=RIGHT_STALL) speeds[1]=speeds[1]-((counters[1]+Counter_offset)*CHANGE_FACTOR);
		}
		//Corrections
		while ((speeds[0]<999000)&&(speeds[1]<999000)){
			speeds[0]+=1000;
			speeds[1]+=1000;
		}
		drive(L,speeds[0],F);
		drive(R,speeds[1],F);
	}
	if (phase==3){
		counter++;
		if (counter==TIME_GRIP_CLOSE) {
		set(GRIPPER_DIS);//stop gripper
		temp=1;
		}
	}
	if (phase==4){
		if (counter==TIME_ARM_UP) {
			set(ARM_DIS);
			temp=1;
		}
	}
	if (phase==5){
		counters[0]++;
		counters[1]++;
		read_sens();
		//indicators
		if (readers[LINE2]>cals[LINE2]+600) led(LR_on);
		else led(LR_off);
		if (readers[LINE3]>cals[LINE3]+600) led(RR_on);
		else led(RR_off);
		if ((readers[LINE2]<=cals[LINE2]+600)&&(readers[LINE3]<=cals[LINE3]+600)) {
		led(G_on);
		counters[0]=0;	//if middle is detected reset change multipliers
		counters[1]=0;
		}
		else led(G_off);
		//speed decrease
		if (readers[LINE2]>cals[LINE2]+600){
			if (speeds[0]>=LEFT_STALL) speeds[0]=speeds[0]-((counters[0]+Counter_offset)*CHANGE_FACTOR);	
		}
		if (readers[LINE3]>cals[LINE3]+600){
			if (speeds[1]>=RIGHT_STALL) speeds[1]=speeds[1]-((counters[1]+Counter_offset)*CHANGE_FACTOR);
		}
		//Corrections
		while ((speeds[0]<999000)&&(speeds[1]<999000)){
			speeds[0]+=1000;
			speeds[1]+=1000;
		}
		drive(L,speeds[0],B);
		drive(R,speeds[1],B);
	}
}




//functions


void init_GPIO_ADC(void){

	RCC->APB2ENR |= RCC_APB2ENR_ADCEN; //enable clock for ADC
	ADC1->CR |= ADC_CR_ADCAL;
	while( (ADC1->CR & ADC_CR_ADCAL) != 0);
	ADC1->CR |= ADC_CR_ADEN; // set ADEN=1 in the ADC_CR register
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0); //wait until ADRDY==1 in ADC_ISR
  
	RCC->AHBENR |= 0b100111<<17; //GPIOA,B,C,F
	GPIOA->MODER |= 0b01<<30;// PA15 OUTPUT
	GPIOA->MODER |= 0b1111111111111111; //PA0-7 Analog
	GPIOB->MODER |= 0b1111;// PB0 Analog
	GPIOB->MODER |= 0b010101010101010101;// PB0-8 to OUTPUT
	GPIOB->MODER |= 0b01010101<<24;// PB12-15 OUTPUT
	GPIOC->MODER |= 0b00<<26;// PC13 input
	GPIOC->PUPDR |= 0b01<<26;//Pull up for PC13
	GPIOF->MODER |= 0b0101<<12;//PF 6-7 OUTPUT
}

void init_timers(void) {
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	TIM14->ARR = 2000;
	TIM14->PSC = 2400;
	TIM14->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM14_IRQn);
	TIM14->CR1 |= TIM_CR1_CEN;
	TIM14->SR &= ~(TIM_SR_UIF);
	
	
	//PWM -> TIM1
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	GPIOA->MODER |= 0b10101010<<16;//PA8-11 to altF
	GPIOA->AFR[1] |= 0b0010001000100010; // PA8-11 to AF2
	  
	TIM1->ARR = 8000;  // f = 1 KHz
	// specify PWM mode: OCxM bits in CCMRx. We want mode 1
	TIM1->CCMR1 |= 0b110<<4; //output and PWM mode 1
	TIM1->CCMR1 |= 0b110<<12; //output and PWM mode 1
	TIM1->CCMR2 |= 0b110<<4; //output and PWM mode 1
	TIM1->CCMR2 |= 0b110<<12; //output and PWM mode 1

	// enable the OC channels
	TIM1->CCER |= 0b1000100010001;//
	TIM1->BDTR |= 0b1<<15;
	TIM1->CR1 |= TIM_CR1_CEN; // counter enable
}

void read_sens(){
	for(int i=0;i<10;i++){
		readers[i]=0;
	}
	for(int i=0;i<10;i++){
		for(int j=0;j<1000;j++){
			readers[i]=readers[i]+ADC_read_chan(0b1<<i);
		}
		readers[i]=readers[i]/1000;
	}
}

void calibrator(void){
	for(int i=0;i<10;i++){
		cals[i]=0;
	}
	for(int i=0;i<10;i++){
		for(int j=0;j<50000;j++){
			cals[i]=cals[i]+ADC_read_chan(0b1<<i);
		}
		cals[i]=cals[i]/50000;
	}
}

int ADC_read(int sensor) {
	ADC1->CHSELR = 0b1<<sensor;
	ADC1->CR |= ADC_CR_ADSTART; // start a conversion
	while((ADC1->ISR & ADC_ISR_EOC) == 0); // wait till conversion complete
	return (((ADC1->DR)*100)/123);//returns voltage*1000
}
int	ADC_read_chan(int channel) {
	ADC1->CHSELR = channel;
	ADC1->CR |= ADC_CR_ADSTART; // start a conversion
	while((ADC1->ISR & ADC_ISR_EOC) == 0); // wait till conversion complete
	return (((ADC1->DR)*100)/123);//returns voltage*1000
}

void drive(unsigned char motor, int speedy, unsigned char dir){
	//set speed veriable
	int speed=speedy/10000;
	if ((dir & 0x01) != 0){ //F
		speedr=100-speed;
	}
	if ((dir & 0x02) != 0){ //B
		speedr=speed;
	}
	//L
	if ((motor & 0x01) != 0)		
    {	
		if ((dir & 0x01) != 0){ //F
			GPIOB->ODR |= 0b1<<13;
		}
		if ((dir & 0x02) != 0){ //B
			GPIOB->ODR &= ~(0b1<<13);
		}
        TIM1->CCR2 = speedr * PWM_MULT; // Left Motor
    }
    //R
    if ((motor & 0x02) != 0)
    {
		if ((dir & 0x01) != 0){ //F
			GPIOB->ODR |= 0b1<<12;
		}
		if ((dir & 0x02) != 0){ //B
			GPIOB->ODR &= ~(0b1<<12);
		}
    	TIM1->CCR1 = speedr * PWM_MULT; // Right Motor     
    }
    //A
    if ((motor & 0x04) != 0)	
    {
		if ((dir & 0x01) != 0){ //F
			GPIOB->ODR |= 0b1<<14;
		}
		if ((dir & 0x02) != 0){ //B
			GPIOB->ODR &= ~(0b1<<14);
		}
    	TIM1->CCR3 = speedr * PWM_MULT; // Arm Motor
    }
	//G
    if ((motor & 0x08) != 0)		
    {
		if ((dir & 0x01) != 0){ //F
			GPIOB->ODR |= 0b1<<15;
		}
		if ((dir & 0x02) != 0){ //B
			GPIOB->ODR &= ~(0b1<<15);
		}
    	TIM1->CCR4 = speedr * PWM_MULT; // Gripper Motor
    }
}

void s_delay (int time,char unit){
	int uni;
	if (unit=='s') uni = 2000000;
	if (unit=='m') uni = 2000;
	if (unit=='u') uni = 2;
	for(int c=0; c <(time*uni); c++);
}

void set(unsigned char command){
	// LEGS_EN
    if ((command & 0x01) != 0)		
    {
        GPIOF->ODR |= 0b1<<6;
    }
    // ARM_EN
    if ((command & 0x02) != 0)
    {
    	GPIOA->ODR |= 0b1<<15;     
    }
    // GRIPPER_EN
    if ((command & 0x04) != 0)	
    {
    	GPIOF->ODR |= 0b1<<7; 
    }
	// LEGS_DIS
    if ((command & 0x08) != 0)		
    {
    	GPIOF->ODR &= ~(0b1<<6);
    }
	// ARM_DIS
    if ((command & 0x10) != 0)	
    {
    	GPIOA->ODR &= ~(0b1<<15); 
    }
	// GRIPPER_DIS
    if ((command & 0x20) != 0)		
    {
    	GPIOF->ODR &= ~(0b1<<7);
    }
}

void led(unsigned char command){
	//LR_on
	 if ((command & 0x01) != 0)		
    {
    	GPIOB->BSRR |= 0b1<<8;
    }
	//LR_off
	 if ((command & 0x02) != 0)		
    {
    	GPIOB->BSRR |= 0b1<<(8+16);
    }
	//RR_on
	 if ((command & 0x04) != 0)		
    {
    	GPIOB->BSRR |= 0b1<<6;
    }
	//RR_off
	 if ((command & 0x08) != 0)		
    {
    	GPIOB->BSRR |= 0b1<<(6+16);
    }
	//G_on
	 if ((command & 0x10) != 0)		
    {
    	GPIOB->BSRR |= 0b1<<7;
    }
	//G_off
	 if ((command & 0x20) != 0)		
    {
    	GPIOB->BSRR |= 0b1<<(7+16);
    }
}
