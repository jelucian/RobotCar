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

unsigned long ADCvalue1, ADCvalue2, ADCvalue3, ADCvalue4;
volatile float distance;
float constA = 32914.3622;
float constB = 0.43923022;
unsigned int distance1, distance2, distance3;

int i = 0;
int tableADCValue[13] = {4095, 3775,2400,1770,1330,1064,915,805,740,700, 630, 590, 560};
int tableDistance[13] = {0, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65};
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
void updateADC(void);
void updateSpeed(void);
unsigned int updateADCDistance(unsigned int, unsigned int, unsigned long);

int main(void){
  PLL_Init();                      // bus clock at 80 MHz
	PortD_Init();
	PortC_Init();
	ADC_Init298();
	Nokia5110_Init();
  Nokia5110_Clear();
	while(1){
		updateADC();
		updateSpeed();
  }
}

void updateADC(){
						//PE5, PE4, PE2, PE1 (Sensor Left/SensorRight/SensorFront/Potentiomenter)
    ADC_In298(&ADCvalue4, &ADCvalue2, &ADCvalue3, &ADCvalue1);
		for(i = 0; i < 12; i++){
			if(ADCvalue1 <= tableADCValue[i] && ADCvalue1 > tableADCValue[i+1])
				indexCheck1 = i;
			
			
			if(ADCvalue2 <= tableADCValue[i] && ADCvalue2 > tableADCValue[i+1])
				indexCheck2 = i;
		}
		
		
		distance1 = updateADCDistance(indexCheck1, 0, ADCvalue1);
		distance2 = updateADCDistance(indexCheck2, 1, ADCvalue2);

}

void updateSpeed(){
	Nokia5110_SetCursor(5,3);
		
		speed				= 0 + (2900 - ADCvalue4) //ADC @ 0
								* (100) 
								/ (2900 - 15); //ADC @ 0 then ADC @ Max
		if(speed <= prevSpeed + 1 && speed >= prevSpeed - 1){
			speed = prevSpeed;
		}
		
		Nokia5110_OutUDec(speed);
		
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
			value1 = 1000 - (speed * 3 + 698);
			value2 = value1;	
		}
		prevSpeed = speed;
		PWM0_3_CMPA_R = value1;
		PWM1_0_CMPB_R = value2;
		
    for(int delay=0; delay<100000; delay++){};
}


unsigned int updateADCDistance(unsigned int index, unsigned int place, unsigned long ADCValue){
		Nokia5110_SetCursor(5,place);
		unsigned int distanceCalc;
		distanceCalc		= tableDistance[index] + (ADCValue - tableADCValue[index]) 
										* (tableDistance[index] - tableDistance[index+1]) 
										/ (tableADCValue[index] - tableADCValue[index+1]);
	
	
		if(distanceCalc <10)
			distance = 10;
		
		
		if(distanceCalc >= 51)
			Nokia5110_OutString("OOR   ");
		else
			Nokia5110_OutUDec(distanceCalc);
	
	return distanceCalc;
}
