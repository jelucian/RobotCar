// PWMtest.c
// Runs on TM4C123
// Jesus Luciano and Aaron Lee
// October 8th, 2018
// Lab 3
 
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
#include <stdint.h>
#include "PLL.h"
#include "PWM.h"
#include "tm4c123gh6pm.h"


void WaitForInterrupt(void);  // low power mode

int main(void){
  PLL_Init();                      // bus clock at 80 MHz
	PortD_Init();
	PortF_Init();
	PortC_Init();

  while(1){
    WaitForInterrupt();
  }
}
