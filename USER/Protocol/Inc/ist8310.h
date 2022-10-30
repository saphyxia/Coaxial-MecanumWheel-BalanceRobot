//
// Created by YanYuanbin on 22-10-3.
//

#ifndef GIMBAL_IST8310_H
#define GIMBAL_IST8310_H

#include "stm32f4xx.h"

#define IST8310_DATA_READY_BIT 2
#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40

typedef struct ist8310_real_data_t
{
    uint8_t status;
    float mag[3];
} ist8310_real_data_t;

extern void ist8310_RST_H(void);
extern void ist8310_RST_L(void);
extern uint8_t ist8310_init(void);
extern void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data);
extern void ist8310_read_mag(float mag[3]);
extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
#endif //GIMBAL_IST8310_H
