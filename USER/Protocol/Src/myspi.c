//
// Created by YanYuanbin on 22-10-3.
//
#include "myspi.h"

/*
 * SPI_SetSpeed
 * SPI 速度=fAPB1/分频系数
 * */
void SPI_SetSpeed(SPI_HandleTypeDef *spi_Handler,uint8_t SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));

    __HAL_SPI_DISABLE(spi_Handler); //关闭 SPI
    spi_Handler->Instance->CR1 &= 0XFFC7; //位 3-5 清零，用来设置波特率
    spi_Handler->Instance->CR1 |= SPI_BaudRatePrescaler;//设置 SPI 速度
    __HAL_SPI_ENABLE(spi_Handler); //使能 SPI
}

/*
 * SPI 读写一个字节
 * TxData:要写入的字节
 * 返回值:读取到的字节
 * */
uint8_t SPI_ReadWriteByte(SPI_HandleTypeDef *spi_Handler,uint8_t TxData)
{
    uint8_t Rxdata = 0;

    HAL_SPI_TransmitReceive(spi_Handler,&TxData,&Rxdata,1, 1000);

    return Rxdata; //返回收到的数据
}

void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num)
{
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);

    __HAL_SPI_ENABLE(&hspi1);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_spi1_rx);
    
    while(hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, DMA_LISR_TCIF2);

    hdma_spi1_rx.Instance->PAR = (uint32_t) & (SPI1->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_spi1_rx.Instance->M0AR = (uint32_t)(rx_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, num);

    __HAL_DMA_ENABLE_IT(&hdma_spi1_rx, DMA_IT_TC);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_spi1_tx);
    
    while(hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_tx);
    }


    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_tx, DMA_LISR_TCIF3);

    hdma_spi1_tx.Instance->PAR = (uint32_t) & (SPI1->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_spi1_tx.Instance->M0AR = (uint32_t)(tx_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, num);


}

void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_spi1_rx);
    __HAL_DMA_DISABLE(&hdma_spi1_tx);
    while(hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_rx);
    }
    while(hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_tx);
    }
    //clear flag
    //清除标志位
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmarx));

    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmatx));
    //set memory address
    //设置数据地址
    hdma_spi1_rx.Instance->M0AR = rx_buf;
    hdma_spi1_tx.Instance->M0AR = tx_buf;
    //set data length
    //设置数据长度
    __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, ndtr);
    __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, ndtr);
    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_spi1_rx);
    __HAL_DMA_ENABLE(&hdma_spi1_tx);
}
