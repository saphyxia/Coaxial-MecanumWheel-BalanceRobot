//
// Created by YanYuanbin on 22-10-2.
//

#ifndef GIMBAL_HALTICK_H
#define GIMBAL_HALTICK_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#define ABS(x) ((x)>0 ?  (x):(-(x)))

/* Exported functions --------------------------------------------------------*/
extern uint32_t micros(void);
extern void delay_us(uint32_t us);
extern void delay_ms(uint32_t ms);

#endif //GIMBAL_HALTICK_H
