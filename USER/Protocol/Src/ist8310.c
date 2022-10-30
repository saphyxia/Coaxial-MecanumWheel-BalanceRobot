//
// Created by YanYuanbin on 22-10-3.
//
#include "ist8310.h"
#include "myiic.h"
#include "haltick.h"

#define MAG_SEN 0.3f
#define IST8310_IIC_ADDRESS 0x0E
#define IST8310_WHO_AM_I 0x00
#define IST8310_WHO_AM_I_VALUE 0x10
#define IST8310_WRITE_REG_NUM 4

uint8_t res = 0;
static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] ={
        {0x0B, 0x08, 0x01},
        {0x41, 0x09, 0x02},
        {0x42, 0xC0, 0x03},
        {0x0A, 0x0B, 0x04}
};
uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res;
    IIC_Start();
    IIC_Send_Byte(0x1C);
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(0x1D);
    IIC_Wait_Ack();
    res=IIC_Read_Byte(0);
    IIC_Stop();
    return res;
}

//��ist8310����ֽ�
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    IIC_Start();
    IIC_Send_Byte(0x1C);
    if(IIC_Wait_Ack())
    {
        IIC_Stop();
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(0x1D);
    IIC_Wait_Ack();
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);
        else *buf=IIC_Read_Byte(1);
        len--;
        buf++;
    }
    IIC_Stop();
}

//дist8310�����ֽ�
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    IIC_Start();
    IIC_Send_Byte(0x1C);
    if(IIC_Wait_Ack())
    {
        IIC_Stop();
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Send_Byte(data);
    if(IIC_Wait_Ack())
    {
        IIC_Stop();
    }
    IIC_Stop();

}
void ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
}

void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
}

uint8_t ist8310_init(void)
{
    static const uint8_t wait_time = 150;
    static const uint8_t sleepTime = 50;
    uint8_t writeNum = 0;
    ist8310_RST_L();
    delay_ms(sleepTime);
    ist8310_RST_H();
    delay_ms(sleepTime);
    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
    if (res != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_NO_SENSOR;
    }

    //set mpu6500 sonsor config and check
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++)
    {
        ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0], ist8310_write_reg_data_error[writeNum][1]);
        delay_us(wait_time);
        res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
        delay_us(wait_time);
        if (res != ist8310_write_reg_data_error[writeNum][1])
        {
            return ist8310_write_reg_data_error[writeNum][2];
        }
    }
    return IST8310_NO_ERROR;
}

void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data)
{

    if (status_buf[0] & 0x01)
    {
        int16_t temp_ist8310_data = 0;
        ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;

        temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
        ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
        ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
        ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
    }
    else
    {
        ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}

//������ȡ������DMA
void ist8310_read_mag(float mag[3])
{
    uint8_t buf[6];
    int16_t temp_ist8310_data = 0;
    //read the "DATAXL" register (0x03)
    ist8310_IIC_read_muli_reg(0x03, buf, 6);

    temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
    mag[0] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
    mag[1] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
    mag[2] = MAG_SEN * temp_ist8310_data;
}
