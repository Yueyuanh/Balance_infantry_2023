#include "vision.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "cstring"
#include "judge.h"


vision_info_t vision_info;


vision_info_t* vision_info_point(void)
{
    return &vision_info;
}


extern float Firc_L_firc3508_motor_speed,Firc_R_firc3508_motor_speed;
extern float Firc_L_tempreate,Firc_R_tempreate,gloab_bullet_speed;

extern float per_fir_delay;
bool vision_update_flag;

void Vision_Read_Data(uint8_t *ReadFromUsart)
{
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[0] == 0xA5)
	{
		//֡ͷCRC8У��
		if(Verify_CRC8_Check_Sum( ReadFromUsart, VISION_LEN_HEADER ) == true)
		{
			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum( ReadFromUsart, VISION_LEN_PACKED ) == true)
			{
				//�������ݿ���
				memcpy(&vision_info.RxPacketFir, ReadFromUsart, VISION_LEN_PACKED);	
				 //֡����
			  vision_info.Fir_State.rx_time_now  = xTaskGetTickCountFromISR();
				vision_info.Fir_State.rx_time_fps  = vision_info.Fir_State.rx_time_now - vision_info.Fir_State.rx_time_prev;
				vision_info.Fir_State.rx_time_prev = vision_info.Fir_State.rx_time_now;
				vision_info.Vision_empty_time=20;
				vision_info.Fir_State.rx_data_update = true;//����Ӿ����ݸ�����
			}
		}
	}
	else if(ReadFromUsart[0] == 0xB5)
	{
	  //֡ͷCRC8У��
		if(Verify_CRC8_Check_Sum( ReadFromUsart, VISION_LEN_HEADER ) == 1)
		{
			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum( ReadFromUsart, VISION_LEN_PACKED ) == 1)
			{
				//�������ݿ���
				memcpy( &vision_info.RxPacketSed, ReadFromUsart, VISION_LEN_PACKED);	
				 //֡����
			  vision_info.Sed_State.rx_time_now  = xTaskGetTickCountFromISR();
				vision_info.Sed_State.rx_time_fps  = vision_info.Sed_State.rx_time_now - vision_info.Sed_State.rx_time_prev;
				vision_info.Sed_State.rx_time_prev = vision_info.Sed_State.rx_time_now;
				vision_info.Vision_empty_time=20;
				vision_info.Sed_State.rx_data_update = true;//����Ӿ����ݸ�����
				
			}
		}
	
	
	}
  else if(ReadFromUsart[0] == 0xC5)
	{
	
	  //֡ͷCRC8У��
		if(Verify_CRC8_Check_Sum( ReadFromUsart, VISION_LEN_HEADER ) == 1)
		{
			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum( ReadFromUsart, VISION_LEN_PACKED ) == 1)
			{
				//�������ݿ���
				memcpy( &vision_info.RxPacketThi, ReadFromUsart, VISION_LEN_PACKED);	
        //֡����
			  vision_info.Thi_State.rx_time_now  = xTaskGetTickCountFromISR();
				vision_info.Thi_State.rx_time_fps  = vision_info.Thi_State.rx_time_now - vision_info.Thi_State.rx_time_prev;
				vision_info.Thi_State.rx_time_prev = vision_info.Thi_State.rx_time_now;
				vision_info.Vision_empty_time=20;
				vision_info.Thi_State.rx_data_update = true;//����Ӿ����ݸ�����
			}
		}
	}
	memset(ReadFromUsart, 0, VISION_LEN_PACKED);
}


void vision_info_t::Vision_Send_Fir_Data( uint8_t CmdID )
{
	uint8_t vision_sendfir_pack[64] = {0};//����22����

	
	TxHandFir.SOF = VISION_SOF;
	TxHandFir.CmdID = CmdID;//���Ӿ���˵����Ҫ������
	
	//д��֡ͷ
	memcpy( vision_sendfir_pack, &TxHandFir, VISION_LEN_HEADER );
	
	//֡ͷCRC8У��Э��
	Append_CRC8_Check_Sum( vision_sendfir_pack, VISION_LEN_HEADER );
	
	//�м����ݲ��ù�,�Ӿ��ò���,�õ���Ҳ�Ǻ��������Զ�����,�õ��ǶȲ�������
	TxPacketFir.INS_quat_send[0] = INS_quat[0];
	TxPacketFir.INS_quat_send[1] = INS_quat[1];
	TxPacketFir.INS_quat_send[2] = INS_quat[2];
	TxPacketFir.INS_quat_send[3] = INS_quat[3];
	TxPacketFir.INS_gyro_send[0] = INS_gyro[0];
	TxPacketFir.INS_gyro_send[1] = INS_gyro[1];
	TxPacketFir.INS_gyro_send[2] = INS_gyro[2];
	TxPacketFir.INS_accel_send[0] = INS_accel[0];
	TxPacketFir.INS_accel_send[1] = INS_accel[1];
	TxPacketFir.INS_accel_send[2] = INS_accel[2];
	TxPacketFir.revolver_speed =JUDGE_usGetSpeedHeat();//����
	TxPacketFir.firc_error=per_fir_delay;//���ӳ�
  //TxPacketFir.Color=BLUE;
  //TxPacketFir.Color=RED;



	memcpy( vision_sendfir_pack + 3, &TxPacketFir, VISION_LEN_DATA);
	
	//֡βCRC16У��Э��
	Append_CRC16_Check_Sum( vision_sendfir_pack, VISION_LEN_PACKED );
	
	//������õ����ݷ���
	
	CDC_Transmit_FS(vision_sendfir_pack, VISION_LEN_PACKED);
	
	memset(vision_sendfir_pack, 0, VISION_LEN_PACKED);
	
		if(Vision_empty_time <= 0)
		{
		  RxPacketSed.target_linear[0] = 0;
			RxPacketSed.target_linear[1] = 0;
			RxPacketSed.target_angular[2] = 0;
		}
		else 
		{
		  Vision_empty_time--;
		}
}



void vision_info_t::Vision_Send_Sed_Data(uint8_t CmdID)
{
   uint8_t vision_send_sentry_1_pack[64] = {0};//����22����
	 
   TxHandSed.SOF = VISION_SENTRY1_SOF;
	 TxHandSed.CmdID = CmdID;//���Ӿ���˵����Ҫ������
	 //д��֡ͷ
	 memcpy( vision_send_sentry_1_pack, &TxHandSed, VISION_LEN_HEADER );
   //֡ͷCRC8У��Э��
	 Append_CRC8_Check_Sum( vision_send_sentry_1_pack, VISION_LEN_HEADER );
	 //�м�����Ϊ���͸��ڱ� ��ȫ��λ��������Ϣ  14*4 56λ
	 for(int i=0;i<=13;i++)
	 {
	    TxPacketSed.robots_position[i]=1;
	 }
	 
	 memcpy( vision_send_sentry_1_pack + 3, &TxPacketSed, VISION_LEN_DATA);
	 
	 //֡βCRC16У��Э��
	 Append_CRC16_Check_Sum( vision_send_sentry_1_pack, VISION_LEN_PACKED );
	
	 //������õ����ݷ���
	 CDC_Transmit_FS(vision_send_sentry_1_pack, VISION_LEN_PACKED);
	 memset(vision_send_sentry_1_pack, 0, VISION_LEN_PACKED);
 }

 
void vision_info_t:: Vision_Send_Thi_Data(uint8_t CmdID) 
 {
	 uint8_t vision_send_sentry_pack[64] = {0};//����22����
	 TxHandThi.SOF = VISION_SENTRY2_SOF;
	 TxHandThi.CmdID = CmdID;//���Ӿ���˵����Ҫ������
	 //д��֡ͷ
	 memcpy( vision_send_sentry_pack, &TxHandThi, VISION_LEN_HEADER );
   //֡ͷCRC8У��Э��
	 Append_CRC8_Check_Sum( vision_send_sentry_pack, VISION_LEN_HEADER );
	 //�м�����Ϊ���͸��ڱ���ȫ��λ��������Ϣ  6*4 24λ
	 for(int a=0;a<=5;a++)
	 {
	    TxPacketThi.robots_position[a]=2;
	 }
	 //���͸��ڱ���ȫ��������Ѫ����Ϣ   10*2 20λ
	 
	 for(int j=0;j<=9;j++)
	 {
		 TxPacketThi.robots_hp[j]= 3;
	 }

   //ȫ��ʣ��ʱ�� 1*2 2λ
	 TxPacketThi.stage_remain_time= 4;
	 
	 memcpy( vision_send_sentry_pack + 3, &TxPacketThi, VISION_LEN_DATA);
	 
	 //֡βCRC16У��Э��
	 Append_CRC16_Check_Sum( vision_send_sentry_pack, VISION_LEN_PACKED );
	
	 //������õ����ݷ���
	 CDC_Transmit_FS(vision_send_sentry_pack, VISION_LEN_PACKED);
	 memset(vision_send_sentry_pack, 0, VISION_LEN_PACKED);
}


/**
  * @brief  ��ȡyaw���Ƕ�
  * @param  ���ָ��
  * @retval void
  */
float vision_info_t::Vision_Error_Angle_Yaw(float *error)
{
	*error = -(1.0f*RxPacketFir.yaw_angle +0.f) *PI / 180.f ;
	return *error;
}

/**
  * @brief  ��ȡpitch���Ƕ�
  * @param  ���ָ��
  */
float vision_info_t::Vision_Error_Angle_Pitch(float *error)
{	
	*error = -(1.0f*RxPacketFir.pitch_angle + 0.f) *PI / 180.f ;
  return *error;
}

/**
  * @brief  ��ȡ����
  * @param  void
  * @retval void
  * @attention  
  */

void vision_info_t::Vision_Get_Distance(float *distance)
{
	*distance =RxPacketFir.distance;

}

void vision_info_t::Vision_Get_coordinate_system(float *x_axis,float *y_axis,float *z_axis)
{
	*x_axis = RxPacketFir.x_info;
	*y_axis = RxPacketFir.y_info;
	*z_axis = RxPacketFir.z_info;
}

void vision_info_t::Vision_Get_predict_coordinate(float *predict_x,float *predict_y,float *predict_z)
{
	*predict_x = RxPacketFir.predict_x_info;
	*predict_y = RxPacketFir.predict_y_info;
	*predict_z = RxPacketFir.predict_z_info;
}
