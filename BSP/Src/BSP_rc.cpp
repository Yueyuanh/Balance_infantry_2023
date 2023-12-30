#include "Bsp_rc.h"
#include "main.h"
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void Transfer_Image_USART6_Init(void);

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

extern uint8_t transfer_image_rx_buf[TRANSFER_IMAGE_BUFFER_LENGTH];

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

}

void Transfer_Image_USART6_Init(void)
{
  //使能 DMA 串口接收
  SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
  SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

  //使能空闲中断
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

  // enable DMA
  //失效DMA
  __HAL_DMA_DISABLE(&hdma_usart6_rx);

  while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
  }

  __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5);

  hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
  hdma_usart6_rx.Instance->M0AR = (uint32_t)(transfer_image_rx_buf);

  __HAL_DMA_SET_COUNTER(huart6.hdmarx, TRANSFER_IMAGE_BUFFER_LENGTH);
  __HAL_DMA_ENABLE(&hdma_usart6_rx);
}

void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart3);
}

void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);

}
