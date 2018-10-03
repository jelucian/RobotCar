// PWM.c
// Runs on TM4C123


/* 
		PD0 - M0PWM6 - val 4 for PCTL - PWM motor A - generator 3 output a
		PD1 - M1PWM1 - val 5 for PCTL - PWM motor B - generator 0 output b
		
		PF0 - sw2 - direction
		PF1 - red
		PF2 - blue
		PF3 - green
		PF4 - sw1 - speed
		
		PD2 for dir A - GPIO
		PD3 for dir B - GPIO
 */
#include "PLL.h"
#include <stdint.h>
#include "tm4c123gh6pm.h"
#define PWM_0_GENA_ACTCMPAD_ONE 0x000000C0  // Set the output signal to 1
#define PWM_0_GENA_ACTLOAD_ZERO 0x00000008  // Set the output signal to 0
#define PWM_0_GENB_ACTCMPBD_ONE 0x00000C00  // Set the output signal to 1
#define PWM_0_GENB_ACTLOAD_ZERO 0x00000008  // Set the output signal to 0

int speed; 
int dir = 1;
int val = 998;

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
	PWM1_0_LOAD_R = 1000 - 1;	//M1 generator 1 for output 1 output b
	PWM1_0_CMPB_R = 998;	//set 0% duty cycle for generator 3 output b
	
	PWM1_0_CTL_R &= ~0x00000010; //set mode to countdown
	PWM1_0_CTL_R |=  0x00000001; //enable generator

	PWM1_0_GENB_R |= 0x0000080C; //immediate updates to parameters
	
	PWM1_ENABLE_R |= 0x02; // enable output 1 of module 1
	
	//M0
	PWM0_3_LOAD_R = 1000 - 1;//M0 generator 3 for output 6 output a
	PWM0_3_CMPA_R = 998;//0% duty cycle
	
	PWM0_3_CTL_R &= ~0x00000010; //set mode to countdown
	PWM0_3_CTL_R |=  0x00000001; //enable generator
	
	PWM0_3_GENA_R |= 0x0000008C; //immediate updates for parameters
	
	PWM0_ENABLE_R |= 0x40; //enable output 6
}


//interrupt handler
void GPIOPortF_Handler(void){ // called on touch of either SW1 or SW2

	if(GPIO_PORTF_RIS_R&0x01){  // SW2 touch controls direction
			GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
			
			dir ^= 1;
			GPIO_PORTC_DATA_R ^= 0xFF;
			if(speed != 0)
				GPIO_PORTF_DATA_R ^= 0x0C;// toggle blue and green
			else
				GPIO_PORTF_DATA_R = 0x02; //red
		
			val = 1000 - val;

  }
  
	if(GPIO_PORTF_RIS_R&0x10){  // SW1 touch controls speed
			GPIO_PORTF_ICR_R = 0x11;  // acknowledge flag4
		
		if(speed == 100){
			GPIO_PORTF_DATA_R = 0x02; //red
			speed = 0;			
			if(dir == 1){
				val = 998;
			}
			else{
				val = 2;
			}
			
		}
		else if (speed == 0){
			if(dir == 1)
				GPIO_PORTF_DATA_R = 0x04; ////blue
			else
				GPIO_PORTF_DATA_R = 0x08; //green
			
			speed = 25;
			if(dir == 1){
					val = 300;
			}
			else{
					val = 700;
			}


		}
		else if(speed == 25){
			//GPIO_PORTF_DATA_R = 0x08; //green
			
			speed = 50;			
			if(dir == 1){	
					val = 200;
			}
			else{	
					val = 800;
			}

		}
		else{
			//GPIO_PORTF_DATA_R = 0x0A; //yellow
		
			speed = 100;			
			if(dir == 1){
					val = 2;
			}
			else{
					val = 998;
			}
		}
  }
	

	PWM0_3_CMPA_R = val;
	PWM1_0_CMPB_R = val;
	// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06
//			PWM0_3_CMPA_R = 
//			PWM1_0_CMPB_R = 
	//Regardless of what is clicked, it is best to assume LED changed
	//Check if speed = 0, if so set light to red
	//else, set light according to the direction 1 meaning green 0 meaning blue
	/*
	if(speed == 0)
		GPIO_PORTF_DATA_R = 0x02;
	else
		GPIO_PORTF_DATA_R = 0x04;
	//GPIO_PORTD_DATA_R & 0x0C;
*/
	//Edit for future usage: Turn direction into a two bit GPIO_PORTA_DATA_R for bits 3 and 2 so it can easily connect to LED
}

void PortF_Init(void){  
	unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x00000020; 			// enable Port F clock
  delay = SYSCTL_RCGC2_R;
	

	//Configuration of GPIO PORT F switches and LED
	GPIO_PORTF_LOCK_R 	=  0x4C4F434B; 	// unlock GPIO Port F
  GPIO_PORTF_PCTL_R  &= ~0x000FFFFF; 	// configure Port F as GPIO
	GPIO_PORTF_AMSEL_R  =  0x00;  			// disable analog functionality for Port F
	
  GPIO_PORTF_CR_R     =  0x1F;      	// allow changes to Port F (5 bits)
	
  GPIO_PORTF_DIR_R   &= ~0x11;    		// make PF4 and PF0 inputs (onboard switch buttons)
	GPIO_PORTF_PUR_R   |=  0x11;    		// enable weak pull-up on PF4,0
	
	GPIO_PORTF_DIR_R   |=  0x0E;				// make PF1, PF2, and PF3 outputs (LED display)
	
	GPIO_PORTF_AFSEL_R &= ~0x1F;  			// disable alternate functions for Port F
	GPIO_PORTF_DEN_R   |=  0x1F;     		// enable digital I/O for Port F

	
	//Interrupt Logic
  GPIO_PORTF_IS_R 	 &= ~0x11;     		// PF4 & PF0 is edge-sensitive
  GPIO_PORTF_IBE_R   &= ~0x11;    		// PF4 & PF0 is not both edges
  GPIO_PORTF_IEV_R   &= ~0x11;   			// PF4 & PF0 falling edge event
	
  GPIO_PORTF_ICR_R    =  0x11;     		// clear flags for PF4 and PF0
	
  GPIO_PORTF_IM_R    |=  0x11;      	// arm interrupt on PF4 & PF0
	
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; 	// priority 2 interrupt for switches				 
  NVIC_EN0_R  = 0x40000000;      			// enable interrupt 30 in NVIC
}

//PC4 & 5
void PortC_Init(void){
	unsigned long volatile delay;
	SYSCTL_RCGC2_R |= 0x00000004; //enable Port C clock
	delay = SYSCTL_RCGC2_R;
	
	GPIO_PORTC_AMSEL_R &= ~0x30; 	//disable analog for PC4,5

	GPIO_PORTC_PCTL_R &= ~0x00FF0000;	//make PC4,5 GPIO
	GPIO_PORTC_PCTL_R &= ~0x00FF0000;	//clear PC4,5

	GPIO_PORTC_DIR_R |= 0x30;	//make PC4,5 outputs
	
	GPIO_PORTC_AFSEL_R &= ~0x30; //disable alternate functions PC4,5
	
	GPIO_PORTC_PUR_R |= 0x30; //enable pull up resistors PC4,5
	
	GPIO_PORTC_DEN_R |= 0x30; //enable digital I/O on PC4,5
	
}
