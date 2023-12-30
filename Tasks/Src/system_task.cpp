#include "system_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "judge_task.h"
#include "tim.h"
#include "revolver_task.h"
#include "bsp_buzzer.h"
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


system_t sys;

/*****************ȫ�ֱ���*************************/
uint16_t speedlimit;																//��������
int steering_mode=0  ;        											//���ո�ģʽ 0Ϊ�رգ�1Ϊ��
int vision_mode=1;           												//��ģʽ1��1v1 �Զ�����  3��3v3 �ֶ�����
int spin_mode=0;																		//�Ƿ���С����


fp32 x_coordinate,y_coordinate;        							//����λ��ui
fp32 pre_x_coordinate,pre_y_coordinate; 						//����Ԥ��λ��ui
fp32 follow_radius,pre_radius;											//����װ�װ�뾶

/***************************************************/
/**
  * @brief          ϵͳ������
  * @param[in]      none
  */
void system_task(void const *pvParameters)
{
	vTaskDelay(201);
	uint32_t currentTime;
	while(1)
	{
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		sys.Mode_set();
		sys.Transit();
		sys.Key_set();
		vTaskDelayUntil(&currentTime, 1);
	}
}


/**
  * @brief 		ȡϵͳ��ָ��
  * @retval   ϵͳ����
  */
system_t *syspoint(void)
{
	return &sys;
}

/**
  * @brief          ���캯����ʼ��
  * @param[in]      null
  */
system_t::system_t()
{
	system_rc_ctrl = get_remote_control_point();
	rc_add_yaw =0.0f; 
	rc_add_pit = 0.0f;
	
	sys_mode = ZERO_FORCE;

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

	chassis_vx_set_temp=0;
	chassis_vy_set_temp=0;
}


/**
  * @brief          ��̨ģʽ����
  * @param[in]      null
  * @retval         null
  */
void system_t::Mode_set()
{	
	  Rc_vision();
		last_sys_mode=sys_mode;
		chassispoint()->SPIN_flag=0;//��ʼ��spin����
	  if(init_state==1)
		{
		  return;
		}
		else
		{
			if(sys_mode != ZERO_FORCE && (toe_is_error(DBUS_TOE) == 1)) //��ǰ������ģʽ��ң��������ʱ����ʧ��ģʽ
			{
				sys_mode = DBUS_MISSING_CONTROL;
				return;
			}
			else if(switch_is_up(system_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL])||spin_mode)//SPIN   
			{
					if((system_rc_ctrl->mouse.press_r != 1 && vision_auto_flag ==0))
					{
						sys_mode = SPIN;
					}
					else
					{
						sys_mode = SPIN_AUTO;
					}
			}
			else if(switch_is_down(system_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))  //����
			{
				sys_mode = ZERO_FORCE;
			}
			else if(system_rc_ctrl->mouse.press_r == 1 || vision_auto_flag ==1)//����
			{
					sys_mode=AUTO;
			}

			else if(switch_is_mid(sys.system_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))  //���̸�����̨
			{
				sys_mode = ABSOLUTE_ANGLE;
			}
			


			//��ʼ������
			if(last_sys_mode == ZERO_FORCE && sys_mode != ZERO_FORCE)
			{
				sys_mode = INIT;
				init_state=1;
			}
  	}
	//ͼ��ң����·�л�
	RC_Mode_Set();
}

/**
  * @brief          ��̨ģʽת��
  * @param[in]      null
  * @retval         null
  */
void system_t::Transit()
{
	 if(last_sys_mode != ABSOLUTE_ANGLE&&sys_mode ==ABSOLUTE_ANGLE)
		{
			gimbal_point()->Yaw_motor.absolute_angle_set =gimbal_point()->Yaw_motor.absolute_angle;
		  gimbal_point()->Pitch_motor.absolute_angle_set =gimbal_point()->Pitch_motor.absolute_angle;
		}
		else if(last_sys_mode != RELATIVE_ANGLE&&sys_mode ==RELATIVE_ANGLE)
		{
			gimbal_point()->Yaw_motor.relative_angle_set =gimbal_point()->Yaw_motor.relative_angle;
			gimbal_point()->Pitch_motor.relative_angle_set =gimbal_point()->Pitch_motor.relative_angle;
		}
		else if(last_sys_mode != SPIN&&sys_mode ==SPIN)
		{
			gimbal_point()->Yaw_motor.absolute_angle_set =gimbal_point()->Yaw_motor.absolute_angle;
			gimbal_point()->Pitch_motor.absolute_angle_set =gimbal_point()->Pitch_motor.absolute_angle;
		}	
		else if(last_sys_mode != AUTO&&sys_mode ==AUTO)
		{
			gimbal_point()->Yaw_motor.absolute_angle_set =gimbal_point()->Yaw_motor.absolute_angle;
			gimbal_point()->Pitch_motor.absolute_angle_set =gimbal_point()->Pitch_motor.absolute_angle;
		}
}




/**
  * @brief          ң�����ж��Ƿ��������ģʽ
  * @param[in]      null
  * @retval         True or False
  */
//int count_time_auto;
void system_t::Rc_vision()
{


//	//�����²����ֽ�����
//	if(rc_ctrl.rc.ch[4]>=100&&rc_ctrl.rc.ch[4]<500)
//	{
//			count_time_auto++;
//			if(count_time_auto>200)
//			{
//					vision_auto_flag=1;
//			}
//		  else
//	    {
//			    vision_auto_flag=0;
//	    }
//	}
//	else
//	{
//			count_time_auto=0;
//			vision_auto_flag=0;
//	}



//�L�J������
	static int16_t rc_vision_time;
	if (rc_ctrl.rc.ch[4]>=100&&rc_ctrl.rc.ch[4]<500)
	  rc_vision_time++;
	else
		rc_vision_time = 0;
	
	if(rc_vision_time >500)
	{	
		vision_flag = 1;
		if(vision_flag == 1 && vision_last_flag == 0)
			vision_auto_flag = !vision_auto_flag;
			buzzer_on(95,10000);
			osDelay(200);
			buzzer_off();
	}
	else
	  vision_flag = 0;

	if(vision_last_flag==1&&vision_flag==0)
		{
			buzzer_on(95,10000);
			osDelay(200);
			buzzer_off();
		}
  else if(vision_last_flag==0&&vision_flag==1)
		{
			buzzer_on(95,19999);
			osDelay(200);
			buzzer_off();
		}



  vision_last_flag = vision_flag;
	
	
}

/**
  * @brief		ͼ��ң����·�����͹رյ��л�
  * @attention	ͼ��ң����·������ֻ�Ǽ�����Ϣ��ͼ����·��ң�����Ĳ��˺�ҡ����Ȼ�п�������
  */
void system_t::RC_Mode_Set()
{
	static int rc_mode_set_time = 0;
	static bool_t rc_mode_set_flag = 0;

	if (switch_is_down(system_rc_ctrl->rc.s[0]) && switch_is_down(system_rc_ctrl->rc.s[1]) &&
		system_rc_ctrl->rc.ch[0] > 600 && system_rc_ctrl->rc.ch[1] > 600 && system_rc_ctrl->rc.ch[2] < -600 && system_rc_ctrl->rc.ch[3] > 600)
	{
		rc_mode_set_time++;
	}
	else
	{
		rc_mode_set_time = 0;
		rc_mode_set_flag = 0;
	}

	if (rc_mode_set_time > 100 && rc_mode_set_flag == 0)
	{
		Transfer_Image_Mode_Set();
		rc_mode_set_flag = 1;
	}
}

/**
  * @brief          ��̨ͨ��ֵ����
  * @param[in]      null
  */
//fp32 mouse_yaw, mouse_pitch;

void system_t ::Gimbal_value_calc()
{
	static fp32 mouse_yaw, mouse_pitch;

  static int16_t yaw_channel = 0, pitch_channel = 0;
	//����ӦYAW,PITCH��������
//	mouse_yaw = LPF(&yaw_lpf ,0.002,system_rc_ctrl->mouse.x * YAW_MOUSE_SEN, 14);
//	mouse_pitch = LPF(&pitch_lpf ,0.002,system_rc_ctrl->mouse.y * PITCH_MOUSE_SEN, 30);


  mouse_yaw = LPF(&yaw_lpf ,0.002,system_rc_ctrl->mouse.x , 12);
	mouse_pitch = LPF(&pitch_lpf ,0.002,system_rc_ctrl->mouse.y , 12);
	//��ң���������ݴ������� int16_t yaw_channel,pitch_channel
	rc_deadband_limit(system_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
  rc_deadband_limit(system_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

	rc_add_yaw = (yaw_channel * YAW_RC_SEN - 
		            mouse_yaw * YAW_MOUSE_SEN+//gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN + 
		           (system_rc_ctrl->key.v & GIMBAL_LEFT_KEY)*0.00012 - 
				  	   (system_rc_ctrl->key.v & GIMBAL_RIGHT_KEY)*0.00006)+vision_info_point()->RxPacketSed.target_angular[2]*CHASSIS_WZ_RE_SEN;
	
	rc_add_pit = (pitch_channel * PITCH_RC_SEN + 
								mouse_pitch* PITCH_MOUSE_SEN //gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN
							 );
}

/**
  * @brief          ����ͨ��ֵ����
  * @param[in]      null
  */
fp32 vx_set_channel, vy_set_channel;
void system_t::Chassis_value_calc()
{
	int16_t vx_channel, vy_channel;
	int16_t last_vx_channel,last_vy_channel;
	//�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
  rc_deadband_limit(system_rc_ctrl->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(system_rc_ctrl->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
	//��������
	vx_set_channel = -vx_channel * CHASSIS_VX_RC_SEN+vision_info_point()->RxPacketSed.target_linear[0]*CHASSIS_VX_RE_SEN;
  vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN+vision_info_point()->RxPacketSed.target_linear[1]*CHASSIS_VY_RE_SEN;
	
	if (system_rc_ctrl->key.v & CHASSIS_FRONT_KEY)
  {
    vx_set_channel = -NORMAL_MAX_CHASSIS_SPEED_X;
  }
  else if (system_rc_ctrl->key.v & CHASSIS_BACK_KEY)
  {
    vx_set_channel = NORMAL_MAX_CHASSIS_SPEED_X;
  }

  if (system_rc_ctrl->key.v & CHASSIS_LEFT_KEY)
  {
    vy_set_channel = -NORMAL_MAX_CHASSIS_SPEED_Y;
  }
  else if (system_rc_ctrl->key.v & CHASSIS_RIGHT_KEY)
  {
    vy_set_channel = NORMAL_MAX_CHASSIS_SPEED_Y;
  }
	
	chassis_vx_set_temp = RAMP_float(vx_set_channel,chassis_vx_set_temp,0.004f);
	chassis_vy_set_temp = RAMP_float(vy_set_channel,chassis_vy_set_temp,0.004f);
	//ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
  {
    chassis_vx_set_temp = 0.0f;
		if(abs(last_vx_channel)-abs(vx_channel)>0)//ң�˻���
		{
				chassis_vx_set_temp = vx_set_channel*500;
		}
  }

  if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
  {
    chassis_vy_set_temp = 0.0f;
		if(abs(last_vy_channel)-abs(vy_channel)>0)
		{
				chassis_vy_set_temp = -vy_set_channel*500;
		}
  }



	last_vx_channel=vx_channel;
	last_vy_channel=vy_channel;

}
/**
  * @brief          ���̰�������
  * @param[in]      null
  */
void system_t::Key_set()
{
		static uint32_t rc_keyZ_time=0;    							//����Z�İ���ʱ��
		static uint32_t rc_keyG_time=0;
	/************************************���ոǶ������*************************************/
			//����Z�ڵ��ոǿ��ؼ��л�		
if ( IF_KEY_PRESSED_Z||rc_ctrl.rc.ch[4]<=-630)
			{				
				if(rc_keyZ_time<500)//���ոǿ���ʱ��Ϊ500����
				 {
					 rc_keyZ_time++;
					 
					 if(rc_keyZ_time==500 && steering_mode==0)
					 {
						 steering_mode=1;  //��
					 }
					 else if(rc_keyZ_time==500 && steering_mode==1)
					 {
						 steering_mode=0;  //�ر�
					 }
					 
				 }
			}
			else
			{
				rc_keyZ_time=0;
			}
			
			if(steering_mode==1)
			{
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 700 );			
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 700 );			
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 700 );			
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 700 );			

			}				
			else if(steering_mode==0)
			{ 
				 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1850);
				 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1850);
				 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1850);
				 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1850);

			}



/************************************С���ݿ���*************************************/
if(!switch_is_down(system_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
{
if (IF_KEY_PRESSED_G)//G �л�spinģʽ
			{				

				if(rc_keyG_time<20)//ģʽת��ʱ��
				 {
					 rc_keyG_time++;
					 
					 if(rc_keyG_time==20 && spin_mode==0)
					 {
						 spin_mode=1;  //mode1
					 }
					 else if(rc_keyG_time==20 && spin_mode==1)
					 {
						 spin_mode=0;  //mode0
					 }
					 
				 }
			}
			else
			{
				rc_keyG_time=0;
			}
}
else{
			spin_mode=0;
}

}

