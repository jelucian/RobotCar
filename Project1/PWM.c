// PWM.c
// Runs on TM4C123


/* 
		PD0 - M0PWM6 - val 4 for PCTL - PWM motor A
		PD1 - M1PWM1 - val 5 for PCTL - PWM motor B
		
		PF0 - sw2 - direction
		PF1 - red
		PF2 - blue
		PF3 - green
		PF4 - sw1 - speed
		
		PD2 for dir A - GPIO
		PD3 for dir B - GPIO
 */
 
#include <stdint.h>
//#include "inc/tm4c123gh6pm.h"
#include "tm4c123gh6pm.h"
#define PWM_0_GENA_ACTCMPAD_ONE 0x000000C0  // Set the output signal to 1
#define PWM_0_GENA_ACTLOAD_ZERO 0x00000008  // Set the output signal to 0
#define PWM_0_GENB_ACTCMPBD_ONE 0x00000C00  // Set the output signal to 1
#define PWM_0_GENB_ACTLOAD_ZERO 0x00000008  // Set the output signal to 0

#define SYSCTL_RCC_USEPWMDIV    0x00100000  // Enable PWM Clock Divisor
#define SYSCTL_RCC_PWMDIV_M     0x000E0000  // PWM Unit Clock Divisor
#define SYSCTL_RCC_PWMDIV_2     0x00000000  // /2

int16_t P = 0;//period
int16_t D = 0;//duty
// period is 16-bit number of PWM clock cycles in one period (3<=period)
// period for PB6 and PB7 must be the same
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
// PWM clock rate = processor clock rate/SYSCTL_RCC_PWMDIV
//                = BusClock/2 
//                = 80 MHz/2 = 40 MHz (in this example)
// Output on PB6/M0PWM0

// change duty cycle of PF1
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
void PWM1F_Duty(uint16_t duty){
  PWM1_2_CMPB_R = duty - 1;             // 6) count value when output rises
}

//enable sw1 & sw2 and use those to toggle PWM output on red onboard LED
void PWM1F_Init(uint16_t period, uint16_t duty){ 
	//set global variables for interrupt purposes
	P = period -1 ;
	D = duty -1;
	
	unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // activate clock for port F
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock GPIO Port F
  
	GPIO_PORTF_CR_R = 0x13;           // allow changes to PF4,1,0
	GPIO_PORTF_DIR_R &= ~0x13;        // make PF4,0 in (built-in button)
	GPIO_PORTF_DIR_R |= 0x02;         // make PF1 output
	
	GPIO_PORTF_AFSEL_R &= ~0x11;   		// disable alt funct on PF4,0
  GPIO_PORTF_AFSEL_R |= 0x02;       // enable alt function on PF1
	
	GPIO_PORTF_DEN_R |= 0x13;         //  enable digital I/O on PF4,1,0
  
	GPIO_PORTF_PCTL_R &= ~0x000F00FF; // configure PF4,0 as GPIO
	GPIO_PORTF_PCTL_R |=  0x00000050; // configure PF1 as M1_PWM5
  
	GPIO_PORTF_AMSEL_R &= ~0x13;  //     disable analog functionality on PF4,1,0
  
	GPIO_PORTF_PUR_R |= 0x13;     //     enable weak pull-up on PF4,1,0
  
	//interrupt control
	GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4,PF0 is edge-sensitive
	GPIO_PORTF_IBE_R &= ~0x11;    //     PF4,PF0 is not both edges
	GPIO_PORTF_IEV_R &= ~0x11;    //     PF4,PF0 falling edge event
	GPIO_PORTF_ICR_R = 0x11;      // (e) clear flags 4,0
	GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4,PF0
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; // (g) priority 2
	NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
	
	//PWM control - for M1PWM5 on pin PF5
	SYSCTL_RCGCPWM_R |= 0x02; 		// enable PWM M1
	SYSCTL_RCGCGPIO_R |= 0x20;    // activate Port F
																// AFSEL already taken car of
	SYSCTL_RCC_R |=  0x00100000;  // enable PWM divider	 - bit 20		
	SYSCTL_RCC_R &= ~0x000E0000;  // clear divider bits 19-17
	SYSCTL_RCC_R |=  0x00000000;  // set bits 19-17 to 0, for divider of 2
	
	//using M1 and generator 3 for output 5
	//PWM1_2_LOAD_R = period -1;	// set period duration with period variable
	PWM1_2_LOAD_R = period -1;			// set period to variable
  
	//using M1, generator 3, output 2, so CMPB is used instead of CMPA
	//PWM1_2_CMPB_R = duty cycle -1;
	PWM1_2_CMPB_R = duty -1;     // set duty cycle to variable, for initial half brightness
	
	PWM1_2_CTL_R |= 0x00000001;   // enable PWM signal, count down mode
	PWM1_2_GENB_R |= 0x0000080C; 				// low on LOAD, high on CMPB down
	
	PWM1_ENABLE_R |= 0x20; 				// enable M1 PWM5 output
}


//interrupt handler
void GPIOPortF_Handler(void){ // called on touch of either SW1 or SW2
  if(GPIO_PORTF_RIS_R&0x01){  // SW2 touch controls direction
		GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
		GPIO_PORTD_DATA_R = GPIO_PORTD_DATA_R ^ 0x0C;
  }
  if(GPIO_PORTF_RIS_R&0x10){  // SW1 touch controls speed
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
		if(speed == 100)
			speed = 0;
		else if (speed == 0)
			speed = 25;
		else
			speed = speed * 2;
  }
	
	//Regardless of what is clicked, it is best to assume LED changed
	//Check if speed = 0, if so set light to red
	//else, set light according to the direction 1 meaning green 0 meaning blue
	if(speed == 0)
		GPIO_PORTF_DATA_R = 0x02;
	else
		GPIO_PORTF_DATA_R = GPIO_PORTD_DATA_R & 0x0C;

	//Edit for future usage: Turn direction into a two bit GPIO_PORTA_DATA_R for bits 3 and 2 so it can easily connect to LED
}

void Switch_Init(void){  
	unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;         // allow changes to PF4,0
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4,0 in (built-in button)
	GPIO_PORTF_DIR_R |=  0x0E;
  GPIO_PORTF_AFSEL_R &= ~0x1F;  //     disable alt funct on PF4,0
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4,0
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; //  configure PF4,0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x1F;  //     disable analog functionality on PF4,0
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4,PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; // (g) priority 2
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
	
	SYSCTL_RCGC2_R 			|= 0x00000002;     	// 1) B clock
  delay 							 = SYSCTL_RCGC2_R;  // delay   
  GPIO_PORTD_CR_R 		|= 0x3F;          	// allow changes to PB5-0       
  GPIO_PORTD_AMSEL_R 	 = 0x00;        		// 3) disable analog function
  GPIO_PORTD_PCTL_R 	 = 0x00000000;   		// 4) GPIO clear bit PCTL  
  GPIO_PORTD_DIR_R 		|= 0x3F;         		// 5) Output PB0-5
  GPIO_PORTD_AFSEL_R 	 = 0x00;        		// 6) no alternate function
  GPIO_PORTD_DEN_R 		|= 0x3F;         		// 7) enable digital pins PB2-PB0
	delay = delay;													// 8) Dummy Command
}
