/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң��������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"
#include "usart.h"
#include "detect_task.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "BSP_rc.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

/**
  * @brief      ͼ��ң����·Э�����
  * @param[in]  transfer_image_buf��ԭ������ָ��
  * @param[out] rc_ctrl��ң��������ָ��
  * @retval     none
  * @attention  ԭ�����ݰ����˲���ϵͳ���ݵ�֡ͷ��������id��֡β����
  */
static void transfer_image_to_rc(volatile const uint8_t *transfer_image_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//ң�������Ʊ���
RC_ctrl_t rc_ctrl;

//receive data, 18 bytes one frame, but set 36 bytes 
//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

//ͼ����·���յ�ԭʼ����
uint8_t transfer_image_rx_buf[TRANSFER_IMAGE_BUFFER_LENGTH];

/**
  * @brief          remote control init
  * @param[in]      rc_ctrl_init:remote control data point
  * @retval         none
  */
/**
  * @brief          ң������ʼ��
  * @param[in]      rc_ctrl_init��ң��������ָ��
  * @retval         none
  */
void remote_control_init(RC_ctrl_t *rc_ctrl_init)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
    Transfer_Image_USART6_Init();
    rc_ctrl_init->ifuse_transfer_image_control = 0;
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          ��ȡң��������ָ��
  * @param[in]      none
  * @retval         ң��������ָ��
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}


//�����ж�
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            { 
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
						    detect_hook(DBUS_TOE);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            { 
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
							  detect_hook(DBUS_TOE);                
            }
        }
    }
}

/**
  * @brief      ͼ����·�����ж�
  * @attention  ÿ����cubemx�������ɹ��̺�USART6_IRQHandler��������stm32f4xx_it.c��.h���ض��壬Ŀǰû�����������������Ҫ�ֶ�ɾ��
  */
void USART6_IRQHandler(void)
{
    if (USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    __HAL_DMA_DISABLE(huart6.hdmarx);

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5);

    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, TRANSFER_IMAGE_BUFFER_LENGTH);

    __HAL_DMA_ENABLE(huart6.hdmarx);
    HAL_UART_Receive_DMA(&huart6, transfer_image_rx_buf, TRANSFER_IMAGE_BUFFER_LENGTH);
    transfer_image_to_rc(transfer_image_rx_buf, &rc_ctrl);
}

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right

    if(rc_ctrl->ifuse_transfer_image_control == 0)
    {
      rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
      rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
      rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
      rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
      rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
      rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    }
   
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

/**
  * @brief      ͼ��ң����·Э�����
  * @param[in]  transfer_image_buf��ԭ������ָ��
  * @param[out] rc_ctrl��ң��������ָ��
  * @retval     none
  * @attention  ԭ�����ݰ����˲���ϵͳ���ݵ�֡ͷ��������id��֡β����
  */
static void transfer_image_to_rc(volatile const uint8_t *transfer_image_buf, RC_ctrl_t *rc_ctrl)
{
    if(transfer_image_buf == NULL || rc_ctrl == NULL || rc_ctrl->ifuse_transfer_image_control == 0)
    {
        return;
    }

    rc_ctrl->mouse.x = transfer_image_buf[7] | (transfer_image_buf[8] << 8);
    rc_ctrl->mouse.y = transfer_image_buf[9] | (transfer_image_buf[10] << 8);
    rc_ctrl->mouse.z = transfer_image_buf[11] | (transfer_image_buf[12] << 8);
    rc_ctrl->mouse.press_l = transfer_image_buf[13];
    rc_ctrl->mouse.press_r = transfer_image_buf[14];
    rc_ctrl->key.v = transfer_image_buf[15] | (transfer_image_buf[16] << 8);
}

/**
  * @brief  ͼ��ң����·�����͹رյ��л�
  */
void Transfer_Image_Mode_Set(void)
{
  rc_ctrl.ifuse_transfer_image_control = !(rc_ctrl.ifuse_transfer_image_control);
}
