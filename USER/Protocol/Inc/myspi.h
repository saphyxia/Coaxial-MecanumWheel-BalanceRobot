//
// Created by YanYuanbin on 22-10-3.
//

#ifndef GIMBAL_MYSPI_H
#define GIMBAL_MYSPI_H
#include "spi.h"

/* Exported functions --------------------------------------------------------*/
extern void SPI_SetSpeed(SPI_HandleTypeDef *spi_Handler,uint8_t SPI_BaudRatePrescaler);
extern uint8_t SPI_ReadWriteByte(SPI_HandleTypeDef *spi_Handler,uint8_t TxData);
extern void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);
extern void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
#endif //GIMBAL_MYSPI_H
