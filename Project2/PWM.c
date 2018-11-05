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
		
		PC4 for dir A - GPIO
		PC5 for dir B - GPIO
 */
#include "PLL.h"
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "Nokia5110.h"
#include "ADCSWTrigger.h"
#define PWM_0_GENA_ACTCMPAD_ONE 0x000000C0  // Set the output signal to 1
#define PWM_0_GENA_ACTLOAD_ZERO 0x00000008  // Set the output signal to 0
#define PWM_0_GENB_ACTCMPBD_ONE 0x00000C00  // Set the output signal to 1
#define PWM_0_GENB_ACTLOAD_ZERO 0x00000008  // Set the output signal to 0


int speed, prevSpeed; 
int dir = 1;
int val = 998;
unsigned char prev_s, current_s;	//Debounce logic

unsigned long ADCvalue1, ADCvalue2, ADCvalue3, ADCvalue4;
volatile float distance;
float constA = 32914.3622;
float constB = 0.43923022;
unsigned int distance1, distance2, distance3;

int i = 0;
int tableADCValue[13] = {4095, 3775,2400,1770,1330,1064,915,805,740,700, 630, 590, 560};
int tableDistance[13] = {7, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65};
int indexCheck1, indexCheck2, indexCheck3 = -1;
float distanceFromTable = 0;


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

	//unsigned long InA, InB, In;  // input from PF
	
	
	GPIO_PORTF_ICR_R = 0x11;  // acknowledge flags
	/*
	In = GPIO_PORTF_DATA_R & 0x11;//Data for debounce
	InA = GPIO_PORTF_DATA_R& 0x10;//Data for flag SW1
	InB = GPIO_PORTF_DATA_R& 0x01;//Data for flag SW2
	
	if((In == 0x11)&& (current_s==0)){ // zero means SW1 or SW2 is pressed
		if(prev_s == current_s) //If the state is the same, allow change, if not, still debouncing
			current_s = 1;
	}	
	Nokia5110_SetCursor(10, 5);
	if((InB==0x00)&&(current_s == 1)&&(prev_s == current_s)){  // SW2 touch controls direction
			dir ^= 1;
			GPIO_PORTC_DATA_R ^= 0xFF;
			if(speed != 0)//toggles blue and green only if car is moving
				GPIO_PORTF_DATA_R ^= 0x0C;// toggle blue and green
			else//sets red LED if car is off
				GPIO_PORTF_DATA_R |= 0x02; //red
		
			val = 1000 - val;//invert duty cycle when direction is switched

			Nokia5110_SetCursor(10, 5);
			Nokia5110_SetCursor(10, 5);
			Nokia5110_SetCursor(10, 5);
			if(dir == 1)//forward
				Nokia5110_OutChar('F');
			else//backward
				Nokia5110_OutChar('B');
  }
  Nokia5110_SetCursor(4, 5);
	if((InA==0x00)&&(current_s == 1)&&(prev_s == current_s)){  // SW1 touch controls speed
		Nokia5110_SetCursor(4, 5);
		Nokia5110_SetCursor(4, 5);
		Nokia5110_SetCursor(4, 5);
		if(speed == 100){//max speed changes to 0 speed
			GPIO_PORTF_DATA_R = 0x02; //red
			speed = 0;			
			if(dir == 1){
				val = 998;
			}
			else{
				val = 2;
			}
			
			Nokia5110_OutString("  0");
		}
		else if (speed == 0){//car not moving
			if(dir == 1)//forward
				GPIO_PORTF_DATA_R = 0x04; ////blue
			else//backward
				GPIO_PORTF_DATA_R = 0x08; //green
			
			speed = 25;//25% speed
			if(dir == 1){
					val = 300;
			}
			else{
					val = 700;
			}
			
			Nokia5110_OutString(" 25");
		}
		else if(speed == 25){
			speed = 50;//change to 50% speed
			if(dir == 1){	
					val = 200;
			}
			else{	
					val = 800;
			}
			
			Nokia5110_OutString(" 50");
		}
		else{		
			speed = 100;	//change to maxspeed		
			if(dir == 1){
					val = 2;
			}
			else{
					val = 998;
			}
			
			Nokia5110_OutString("100");
		}
  }
	//set duty cycle every time a button is pressed
	//and set the button flag clear if logic is set
	if((current_s == 1)&&(prev_s == current_s)&&((InB==0x00)||(InA==0x00)))
	{
		PWM0_3_CMPA_R = val;
		PWM1_0_CMPB_R = val;
		current_s = 0;
	}
	*/
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
  GPIO_PORTF_IBE_R   |=  0x11;    		// PF4 & PF0 is both edges
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

void WaitForInterrupt(void);  // low power mode

int main(void){
  PLL_Init();                      // bus clock at 80 MHz
	PortD_Init();
	PortF_Init();
	PortC_Init();
	ADC_Init298();
	Nokia5110_Init();
  Nokia5110_Clear();
									 //("EQD:        * LCD Test *************Dist:   ADC:------- ----");
	Nokia5110_OutString("L1:         R2:         F3:         SP:                     ");
	GPIO_PORTF_DATA_R = 0x02;
	
	current_s=0;
  prev_s = 0;	
	//Default values
  while(1){
							//PE5, PE4, PE2, PE1 (Sensor Left/SensorRight/SensorFront/Potentiomenter)
    ADC_In298(&ADCvalue4, &ADCvalue2, &ADCvalue3, &ADCvalue1);
		for(i = 0; i < 12; i++){
			if(ADCvalue1 <= tableADCValue[i] && ADCvalue1 > tableADCValue[i+1])
				indexCheck1 = i;
			
			
			if(ADCvalue2 <= tableADCValue[i] && ADCvalue2 > tableADCValue[i+1])
				indexCheck2 = i;
			
			
			if(ADCvalue3 <= tableADCValue[i] && ADCvalue3 > tableADCValue[i+1])
				indexCheck3 = i;
		}
		
		
		
		
		
		
		Nokia5110_SetCursor(5,0);
		if(indexCheck1 == -1)
			Nokia5110_OutString("<10");
		else{
			distance1				= tableDistance[indexCheck1] + (ADCvalue1 - tableADCValue[indexCheck1]) 
											* (tableDistance[indexCheck1] - tableDistance[indexCheck1+1]) 
											/ (tableADCValue[indexCheck1] - tableADCValue[indexCheck1+1]);
			if(distance1 >= 51)
				Nokia5110_OutString("OOR   ");
			else
				Nokia5110_OutUDec(distance1);
		}
		
		
		
		Nokia5110_SetCursor(5,1);
		if(indexCheck2 == -1)
			Nokia5110_OutString("<10");
		else{
			distance2				= tableDistance[indexCheck2] + (ADCvalue2 - tableADCValue[indexCheck2]) 
											* (tableDistance[indexCheck2] - tableDistance[indexCheck2+1]) 
											/ (tableADCValue[indexCheck2] - tableADCValue[indexCheck2+1]);
			if(distance2 >= 51)
				Nokia5110_OutString("OOR   ");
			else
				Nokia5110_OutUDec(distance2);
		}
		
		
		
		Nokia5110_SetCursor(5,2);
		if(indexCheck3 == -1)
			Nokia5110_OutString("<10");
		else{
			distance3				= tableDistance[indexCheck3] + (ADCvalue3 - tableADCValue[indexCheck3]) 
											* (tableDistance[indexCheck3] - tableDistance[indexCheck3+1]) 
											/ (tableADCValue[indexCheck3] - tableADCValue[indexCheck3+1]);
			if(distance3 >= 51)
				Nokia5110_OutString("OOR   ");
			else
				Nokia5110_OutUDec(distance3);
		}
		
		
		
		Nokia5110_SetCursor(5,3);
		
		speed				= 0 + (2900 - ADCvalue4) //ADC @ 0
								* (100) 
								/ (2900 - 15); //ADC @ 0 then ADC @ Max
		if(speed <= prevSpeed + 1 && speed >= prevSpeed - 1){
			speed = prevSpeed;
		}
		
		Nokia5110_OutUDec(speed);
		
		
		
		//PWM Logic
		
		//Speed
		//3cm give or take
		//2 = 0; 700 = 1% to 998 = 100%
		//1000 is value to subtract from to make it negative
		int value1, value2;
		if(ADCvalue4 >= 2700) //ADC @0
		{
			value1 = 998;
			value2 = 998;
			Nokia5110_SetCursor(5,3);
			speed = 0;
			Nokia5110_OutUDec(speed);
		}
		else{
			value1 = ((100 - 100 * (ADCvalue4 / 2900)) * 3 + 700);
			if(value1 < 700)
				value1 = 700;
		
			value2 = value1;	
		}
		prevSpeed = speed;
		PWM0_3_CMPA_R = value1;
		PWM1_0_CMPB_R = value2;
		//Sensor Logics
		/*
		if(distance3 <= 10){ //Front Sensor Logic if distance is less than 10
			if(distance1 <= distance2 + 3 && distance1 <= distance2 - 3){//Turn right
				value2 = 1000 - value2;
				GPIO_PORTC_DATA_R = 0x10;
			}
			else if (distance2 <= distance1 + 3 && distance2 <= distance1 - 3){ //Turn left
				value1 = 1000 - value1;
				GPIO_PORTC_DATA_R = 0x20;
			}
			else{//Stop everything as we are at equilibrium
				value1 = 2;
				value2 = 2;
				GPIO_PORTC_DATA_R = 0x00;
			}
		}
		else if (distance2 <= 10){//Lean left
			value1 = value1-100;
			GPIO_PORTC_DATA_R = 0x00;
		}
		else if (distance1 <= 10){//Lean right
			value2 = value2-100;
			GPIO_PORTC_DATA_R = 0x00;
		}
		else{GPIO_PORTC_DATA_R = 0x00;}
		PWM0_3_CMPA_R = value1;
		PWM1_0_CMPB_R = value2;
		*/
    for(int delay=0; delay<100000; delay++){};
		
		
		
		
		
		
		/*
    if (prev_s != current_s) {//Debounce logic check
			unsigned int time = 727240*20/91;  // 0.01sec
			while(time){
				time--;
			}
			prev_s = current_s;
		}*/
  }
}



