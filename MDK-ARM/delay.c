/**
  ******************************************************************************
  * @file      delay.c
  * @author    Arif Mandal
  * @date      12/05/2020
  ******************************************************************************
  */
   
#include "delay.h"

#ifdef __cplusplus
extern "C" {
#endif
	
static __IO uint32_t counter;

void SysTick_Handler()  
{
   if(counter > 0)
      counter--;   
}

void delay_ms(uint32_t time)
{
   counter = time;

   while(counter);
}

void DelayInit()
{
   SystemCoreClockUpdate();
   SysTick_Config(SystemCoreClock / 1000);
}

#ifdef __cplusplus
}
#endif

/********************************* END OF FILE ********************************/
