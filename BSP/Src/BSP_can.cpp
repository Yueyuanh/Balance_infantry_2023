#include "BSP_can.h"
#include "struct_typedef.h"
#include "main.h"
#include "cmsis_os.h"
extern "C"{
#include "pm01.h"
}


#define get_chassis_measure(ptr, data)                                  \
    {                                                                   \
																																				\
        (ptr)->pitch_angle_set = (int16_t)((data)[0] << 8 | (data)[1]);            \
		}																																		\




extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static CAN_TxHeaderTypeDef  CAN1_200_tx_message;
static uint8_t              CAN1_200_send_data[8];
static CAN_TxHeaderTypeDef  CAN1_1FF_tx_message;
static uint8_t              CAN1_1FF_send_data[8];
static CAN_TxHeaderTypeDef  CAN2_200_tx_message;
static uint8_t              CAN2_200_send_data[8];
static CAN_TxHeaderTypeDef  CAN2_1FF_tx_message;
static uint8_t              CAN2_1FF_send_data[8];
static CAN_TxHeaderTypeDef  Reset_tx_message;
static uint8_t              Reset_tx_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];


Chassis_date_t    chassis_measure;
Chassis_Power_t		cap_measure;
			#define	get_cap_measure(ptr, data)                \
{    uint16_t *buf =    (uint16_t*) data;               \
		(ptr)->Cap_input_vol = ((fp32)buf[0]/100.f);        \
		(ptr)->Cap_voltage = ((fp32)buf[1]/100.f);     			\
		(ptr)->Cap_current = ((fp32)buf[2]/100.f); 					\
		(ptr)->Cap_power = ((fp32)buf[3]/100.f); 						\
}

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);



}

/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	 detect_hook(CAN_TOE);
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data1[8];
	uint8_t rx_data2[8];
	
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data1);
		
		switch (rx_header.StdId)
		{

      case CAN1_3508_LF_ID:
			{
				get_motor_measure(LF,rx_data1);
				break;
			}
			case CAN1_3508_RF_ID:
			{
				get_motor_measure(RF,rx_data1);
				break;
			}
			case CAN1_3508_LB_ID:
			{
				get_motor_measure(LB,rx_data1);
				break;
			}
			case CAN1_3508_RB_ID:
			{
				get_motor_measure(RB,rx_data1);
				break;
			}
			case CAN1_YAW_MOTOR_ID:
			{
				get_motor_measure(YAW,rx_data1);
				break;
			}
       case 0x211:
			{
			get_cap_measure(&cap_measure , rx_data1);
					break;
			}
			case 0x600:
			case 0x601:
			case 0x602:
			case 0x603:
			case 0x610:
			case 0x611:
			case 0x612:
			case 0x613:
			{
			pm01_response_handle(&rx_header, rx_data1);
			break;
			}
			default:
			{
				break;
			}
	  }
		
	}
	
	if (hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data2);
		
		switch(rx_header.StdId)
		{
			case CAN2_FireL_Motor_ID:
			{
				get_motor_measure(FireL,rx_data2);
				break;
			}
			case CAN2_FireR_Motor_ID:
			{
				get_motor_measure(FireR,rx_data2);
				break;
			}
			case CAN2_TRIGGER_MOTOR_ID:
			{
				get_motor_measure(TRIGGER,rx_data2);
				break;
		  }
			case CAN2_PITCH_Motor_ID:
			{
				get_motor_measure(PITCH,rx_data2);
				break;
			}



			case CAN2_CHASSIS_DATE_ID:
			{
				get_chassis_measure(&chassis_measure,rx_data2);
				break;
			}
		}
		
	}	
}

//����CAN1ID:1-4
/**
  * @apply          can1_motor1:LF  can1_motor2:RF can1_motor3:LB can1_motor4:RB
  * @brief          ���͵�����Ƶ���������CAN1 ID:1-4��
  * @param[in]      can1_motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      can1_motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      can1_motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      can1_motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN1_200_cmd_motor(int16_t can1_motor1, int16_t can1_motor2, int16_t can1_motor3, int16_t can1_motor4)
{
    uint32_t send_mail_box;
    CAN1_200_tx_message.StdId = 0x200;
    CAN1_200_tx_message.IDE = CAN_ID_STD;
    CAN1_200_tx_message.RTR = CAN_RTR_DATA;
    CAN1_200_tx_message.DLC = 0x08;
    CAN1_200_send_data[0] = (can1_motor1 >> 8);
    CAN1_200_send_data[1] = can1_motor1;
    CAN1_200_send_data[2] = (can1_motor2 >> 8);
    CAN1_200_send_data[3] = can1_motor2;
    CAN1_200_send_data[4] = (can1_motor3 >> 8);
    CAN1_200_send_data[5] = can1_motor3;
    CAN1_200_send_data[6] = (can1_motor4 >> 8);
    CAN1_200_send_data[7] = can1_motor4;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &CAN1_200_tx_message, CAN1_200_send_data, &send_mail_box);
}

//����CAN1ID:5-8
/**
  * @apply          can1_motor5:YAW  can1_motor6:PITCH
  * @brief          ���͵�����Ƶ���������CAN1 ID:5-8��
  * @param[in]      can1_motor5: (0x205) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      can1_motor6: (0x206) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      can1_motor7: (0x207) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      can1_motor8: (0x208) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN1_1FF_cmd_motor(int16_t can1_motor5, int16_t can1_motor6, int16_t can1_motor7, int16_t can1_motor8)
{
    uint32_t send_mail_box;
    CAN1_1FF_tx_message.StdId = 0x1FF;
    CAN1_1FF_tx_message.IDE = CAN_ID_STD;
    CAN1_1FF_tx_message.RTR = CAN_RTR_DATA;
    CAN1_1FF_tx_message.DLC = 0x08;
    CAN1_1FF_send_data[0] = (can1_motor5 >> 8);
    CAN1_1FF_send_data[1] = can1_motor5;
    CAN1_1FF_send_data[2] = (can1_motor6 >> 8);
    CAN1_1FF_send_data[3] = can1_motor6;
    CAN1_1FF_send_data[4] = (can1_motor7 >> 8);
    CAN1_1FF_send_data[5] = can1_motor7;
    CAN1_1FF_send_data[6] = (can1_motor8 >> 8);
    CAN1_1FF_send_data[7] = can1_motor8;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &CAN1_1FF_tx_message, CAN1_1FF_send_data, &send_mail_box);
}

//����CAN2ID:1-4
/**
  * @apply          can2_motor1:FireL  can2_motor2:FireR
  * @brief          ���͵�����Ƶ���������CAN2 ID:1-4��
  * @param[in]      can2_motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      can2_motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      can2_motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      can2_motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN2_200_cmd_motor(int16_t can2_motor1, int16_t can2_motor2, int16_t can2_motor3, int16_t can2_motor4)
{
    uint32_t send_mail_box;
    CAN2_200_tx_message.StdId = 0x200;
    CAN2_200_tx_message.IDE = CAN_ID_STD;
    CAN2_200_tx_message.RTR = CAN_RTR_DATA;
    CAN2_200_tx_message.DLC = 0x08;
    CAN2_200_send_data[0] = (can2_motor1 >> 8);
    CAN2_200_send_data[1] = can2_motor1;
    CAN2_200_send_data[2] = (can2_motor2 >> 8);
    CAN2_200_send_data[3] = can2_motor2;
    CAN2_200_send_data[4] = (can2_motor3 >> 8);
    CAN2_200_send_data[5] = can2_motor3;
    CAN2_200_send_data[6] = (can2_motor4 >> 8);
    CAN2_200_send_data[7] = can2_motor4;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &CAN2_200_tx_message, CAN2_200_send_data, &send_mail_box);
}

//c
/**
  * @apply          ��δʹ��
  * @brief          ���͵�����Ƶ���������CAN2 ID:5-8��
  * @param[in]      can2_motor5: (0x205) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      can2_motor6: (0x206) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      can2_motor7: (0x207) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      can2_motor8: (0x208) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN2_1FF_cmd_motor(int16_t can2_motor5, int16_t can2_motor6, int16_t can2_motor7, int16_t can2_motor8)
{
    uint32_t send_mail_box;
    CAN2_1FF_tx_message.StdId = 0x1FF;
    CAN2_1FF_tx_message.IDE = CAN_ID_STD;
    CAN2_1FF_tx_message.RTR = CAN_RTR_DATA;
    CAN2_1FF_tx_message.DLC = 0x08;
    CAN2_1FF_send_data[0] = (can2_motor5 >> 8);
    CAN2_1FF_send_data[1] = can2_motor5;
    CAN2_1FF_send_data[2] = (can2_motor6 >> 8);
    CAN2_1FF_send_data[3] = can2_motor6;
    CAN2_1FF_send_data[4] = (can2_motor7 >> 8);
    CAN2_1FF_send_data[5] = can2_motor7;
    CAN2_1FF_send_data[6] = (can2_motor8 >> 8);
    CAN2_1FF_send_data[7] = can2_motor8;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &CAN2_1FF_tx_message, CAN2_1FF_send_data, &send_mail_box);
}

void CAN_CMD_CAP(uint16_t motor1)
{
	  uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x210;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    
		HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
} 
//���ض�Ӧ�����ַ
const motor_t *get_motor_measure_class(uint16_t type)
{
	return &motor[type];
}

const Chassis_Power_t *get_cap_measure_point(void)
{
    return &cap_measure;
}
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    Reset_tx_message.StdId = 0x700;
    Reset_tx_message.IDE = CAN_ID_STD;
    Reset_tx_message.RTR = CAN_RTR_DATA;
    Reset_tx_message.DLC = 0x08;
    Reset_tx_send_data[0] = 0;
    Reset_tx_send_data[1] = 0;
    Reset_tx_send_data[2] = 0;
    Reset_tx_send_data[3] = 0;
    Reset_tx_send_data[4] = 0;
    Reset_tx_send_data[5] = 0;
    Reset_tx_send_data[6] = 0;
    Reset_tx_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &Reset_tx_message, Reset_tx_send_data, &send_mail_box);
}
