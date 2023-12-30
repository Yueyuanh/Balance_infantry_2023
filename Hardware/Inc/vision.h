#ifndef __VISION_H
#define __VISION_H

#include "struct_typedef.h"
#include "crc_check.h"
#include "INS_task.h"
#include "arm_math.h"
#include "gimbal_task.h"

/*--------------------------------�ݶ�Э��-------------------------------------*/

#define    VISION_LENGTH        22     		 //�ݶ�22�ֽ�,ͷ3�ֽ�,����17�ֽ�,β2�ֽ�

//��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define    VISION_SOF         (0xA5)
#define    VISION_SENTRY1_SOF         (0xB5)
#define    VISION_SENTRY2_SOF         (0xC5)

//���ȸ���Э�鶨��,���ݶγ���Ϊn��Ҫ����֡ͷ�ڶ��ֽ�����ȡ
#define    VISION_LEN_HEADER    3         //֡ͷ��
#define    VISION_LEN_DATA      59       //���ݶγ���
#define    VISION_LEN_TAIL      2	      //֡βCRC16
#define    VISION_LEN_PACKED    64        //���ݰ�����

#define    VISION_OFF         		(0x00)
#define    VISION_RED           	(0x01)
#define    VISION_BLUE          	(0x02)
#define    VISION_RBUFF_ANTI   	 	(0x03)//������
#define    VISION_BBUFF_ANTI   		(0x04)//������
#define    VISION_RBUFF_CLOCKWISE   (0x05)//��˳���
#define    VISION_BBUFF_CLOCKWISE   (0x06)//��˳���
#define    VISION_RBUFF_STAND   	(0x07)//��С��
#define    VISION_BBUFF_STAND   	(0x08)//��С��

#define NOW  0
#define LAST 1


#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus

	typedef struct
{
	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
	uint8_t   CmdID;		//ָ��
	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
}extVisionSendhand_t;
//STM32����
typedef struct
{
		/* ���� */
	float    INS_quat_send[4];
	float    INS_gyro_send[3];
	float    INS_accel_send[3];
	float    revolver_speed;
  float    firc_error;
//
	//float    temp;
  //float    firc_speed;


	uint16_t   Vision_empty_time;		//Ԥ��
	
	/* β */
	uint16_t  CRC16;
	
}extVisionSendDataFir_t;


typedef struct
{
	/* ���� */
	float		robots_position[14];
	/* β */
	uint16_t  CRC16;
	
}extVisionSendDataSed_t;


typedef struct
{
	/* ���� */
	float		robots_position[6];
  uint16_t robots_hp[10];
	uint16_t stage_remain_time;
	/* β */
	uint16_t  CRC16;
	
}extVisionSendDataThi_t;



//STM32����
typedef __packed struct
{
	/* ͷ */
	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
	uint8_t   CmdID;		//ָ��
	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
	/* ���� */
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//����
	uint8_t   is_switched;
	uint8_t   is_findtarget;
	uint8_t   is_spinning;
	uint8_t   is_predictl;
  uint8_t   is_shooting;//ȡ�����Ӿ��Ǳ��Ƿ�����Զ������־λ
	float     x_info;  //��������
	float     y_info;
	float     z_info;
	float     predict_x_info;  //Ԥ����������
	float     predict_y_info;
	float     predict_z_info;
//	uint8_t   is_middle;
	
	
	/* β */
	uint16_t  CRC16;  
}extVisionRecvDataFir_t;	

typedef __packed struct
{
	/* ͷ */
	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
	uint8_t   CmdID;		//ָ��
	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
	/* ���� */
	float     target_linear[3];
	float     target_angular[3];
	
	/* β */
	uint16_t  CRC16;       
	
}extVisionRecvDataSed_t;	

typedef __packed struct
{
	/* ͷ */
	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
	uint8_t   CmdID;		//ָ��
	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
	/* ���� */
	float     theta_gimbal;
	float     theta_chassis;
	
	/* β */
	uint16_t  CRC16;       
	
}extVisionRecvDataThi_t;	
	
//����֡��
typedef struct
{
	bool		    rx_data_update;		// ���������Ƿ����
	uint32_t 		rx_time_prev;		// �������ݵ�ǰһʱ��
	uint32_t 		rx_time_now;		// �������ݵĵ�ǰʱ��
	uint16_t 		rx_time_fps;		// ֡��

} Vision_State_t;

class vision_info_t
{
	public:
		extVisionRecvDataFir_t   RxPacketFir;
	  extVisionRecvDataSed_t   RxPacketSed;
	  extVisionRecvDataThi_t   RxPacketThi;
	
    extVisionSendDataFir_t   TxPacketFir;  //�����ǣ����٣�yaw
	  extVisionSendDataSed_t   TxPacketSed;	 //������λ�ã��ڱ���
    extVisionSendDataThi_t   TxPacketThi;
	
	  extVisionSendhand_t TxHandFir;
	  extVisionSendhand_t TxHandSed;
	  extVisionSendhand_t TxHandThi;
	
	  Vision_State_t Fir_State;
	  Vision_State_t Sed_State;
	  Vision_State_t Thi_State;
	  uint16_t Vision_empty_time;
	
	  void Vision_Send_Fir_Data(uint8_t CmdID );
	  void Vision_Send_Sed_Data(uint8_t CmdID);
	  void Vision_Send_Thi_Data(uint8_t CmdID);
	
	  float Vision_Error_Angle_Yaw(float *error);
	  float Vision_Error_Angle_Pitch(float *error);
	  void Vision_Get_Distance(float *distance);
		void Vision_Get_coordinate_system(float *x_axis,float *y_axis,float *z_axis);
	  void Vision_Get_predict_coordinate(float *predict_x,float *predict_y,float *predict_z);
};	
	
vision_info_t* vision_info_point(void);	
	
#endif
extern  void Vision_Read_Data(uint8_t *ReadFromUsart);
#ifdef __cplusplus
}	
#endif

#endif
