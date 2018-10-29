// ADCTestMain.c
// Runs on LM4F120/TM4C123
// This program periodically samples ADC channel 1 and stores the
// result to a global variable that can be accessed with the JTAG
// debugger and viewed with the variable watch feature.
// Daniel Valvano
// October 20, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// input signal connected to PE2/AIN1

#include "ADCSWTrigger.h"
#include "tm4c123gh6pm.h"
#include "PLL.h"

#include "Nokia5110.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

volatile unsigned long ADCvalue;
volatile float distance;
float constA = 32914.3622;
float constB = 0.43923022;
unsigned int distance2;

int i = 0;
int tableADCValue[13] = {4095, 3775,2400,1770,1330,1064,915,805,740,700, 630, 590, 560};
int tableDistance[13] = {7, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65};
int indexCheck = -1;
float distanceFromTable = 0;
// The digital number ADCvalue is a representation of the voltage on PE4 
// voltage  ADCvalue
// 0.00V     0
// 0.75V    1024
// 1.50V    2048
// 2.25V    3072
// 3.00V    4095
int main(void){unsigned long volatile delay;
  PLL_Init();                           // 80 MHz
  ADC0_InitSWTriggerSeq3_Ch1();         // ADC initialization PE2/AIN1
	Nokia5110_Init();
  Nokia5110_Clear();
	Nokia5110_OutString("EQD:       ** LCD Test *************Dist:   ADC:------- ---- ");
	
	
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port F
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_DIR_R |= 0x04;             // make PF2 out (built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x04;          // disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x04;             // enable digital I/O on PF2
                                        // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;               // disable analog functionality on PF
  while(1){
    GPIO_PORTF_DATA_R |= 0x04;          // profile
    ADCvalue = ADC0_InSeq3();
		for(i = 0; i < 12; i++){
			if(ADCvalue <= tableADCValue[i] && ADCvalue > tableADCValue[i+1])
				indexCheck = i;
		}
		Nokia5110_SetCursor(5,0);
		if(indexCheck == -1)
			Nokia5110_OutString("10");
		else{
			distanceFromTable = tableDistance[indexCheck] + (ADCvalue - tableADCValue[indexCheck]) 
											* (tableDistance[indexCheck] - tableDistance[indexCheck+1]) 
											/ (tableADCValue[indexCheck] - tableADCValue[indexCheck+1]);
			
			Nokia5110_OutUDec(distanceFromTable/1);
		}
		distance = (constA / ADCvalue) + constB;
		if(ADCvalue > 4000)
			distance = 0;
		if(distance < 10)
			distance = 10;
		distance2 = distance/1;
		Nokia5110_SetCursor(0, 5);
		Nokia5110_OutUDec(distance2);
		Nokia5110_SetCursor(7, 5);
		Nokia5110_OutUDec(ADCvalue/1);
    GPIO_PORTF_DATA_R &= ~0x04;
    for(delay=0; delay<100000; delay++){};
  }
}
