// PWM.c
// Runs on TM4C123


/* 
		PD0 - M0PWM6 - val 4 for PCTL - PWM motor A - generator 3
		PD1 - M1PWM1 - val 5 for PCTL - PWM motor B - generator 0
		
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

int speed; 

void PortD_Init(void){
	unsigned long volatile delay;
	SYSCTL_RCGC2_R |= 0x00000008; //enable Port D clock
	delay = SYSCTL_RCGC2_R;
	
	GPIO_PORTD_AMSEL_R &= ~0x0F; 	//disable analog for PD3-0

	GPIO_PORTD_PCTL_R &= ~0x0000FF00;	//make PD3, 2 GPIO
	GPIO_PORTD_PCTL_R &= ~0x000000FF;	//clear PD 1, 0
	GPIO_PORTD_PCTL_R |= 0x00000054;	//set PD1 as M1PWM1 and PD0 as M0PWM6

	GPIO_PORTD_DIR_R |= 0x0F;	//make PD3-0 outputs
	
	GPIO_PORTD_AFSEL_R &= ~0x0C; //disable alternate functions PD3, PD2
	GPIO_PORTD_AFSEL_R |=  0x03; //enable alternate functions PD1, PD0
	
	GPIO_PORTD_PUR_R |= 0x0F; //enable pull up resistors PD3-0
	
	GPIO_PORTD_DEN_R |= 0x0F; //enable digital I/O on PD3-0
	
	//PWM control
	SYSCTL_RCGCPWM_R |= 0x00000003; //enable PWM M0 and M1
	SYSCTL_RCGCGPIO_R |= 0x08;//enable Port D clock 
	
	SYSCTL_RCC_R |=  0x00100000; //enable divider
	SYSCTL_RCC_R &= ~0x000E0000; //clear bits 19-17 for divider value
	SYSCTL_RCC_R |=  0x00000000; //set bits 19-17 to 0, for divider of 2
	
	//M1
	PWM
	
	//M0
	

	
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

void PortF_Init(void){  
	unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;         // allow changes to PF4,0
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4,0 in (built-in button)
	GPIO_PORTF_DIR_R |=  0x0E;		// PF1,2,3 output
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
	
}
