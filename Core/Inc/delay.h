/**
  ******************************************************************************
  * @file      delay.h
  * @author   Arif Mandal
  * @date      12/05/2020
  ******************************************************************************
  */

#include "stm32f1xx.h"

#ifndef __DELAY_H
#define __DELAY_H

#ifdef __cplusplus
 extern "C" {
#endif


void DelayInit(void);
void delay_ms(uint32_t time);

#ifdef __cplusplus
}
#endif

#endif

/********************************* END OF FILE ********************************/
