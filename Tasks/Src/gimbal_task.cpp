#include "gimbal_task.h"
#include "cmsis_os.h"
#include "task.h"
#include "revolver_task.h"
gimbal_control_t gimbal;	
	
	uint32_t currentTime;

/**
  * @brief          ȡ��ָ̨��
  * @param[in]      null
  * @retval         ��̨����
  */
gimbal_control_t *gimbal_point(void)
{
	return &gimbal;
}
/*********************************************/
extern bool auto_firc_flag_angle;

/*********************************************/
/**
  * @brief          ��̨������
  * @param[in]      null
  */		
void gimbal_task(void const *pvParameters)
{
	vTaskDelay(GIMBAL_TASK_INIT_TIME);


	while(1)
	{
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		gimbal.Judge_init_state();
		gimbal.Get_info();
		gimbal.Control();

		CAN2_1FF_cmd_motor(0,0,gimbal.Pitch_motor.given_current,0);
		CAN1_1FF_cmd_motor(gimbal.Yaw_motor.given_current,0,0,0);
		//vTaskDelay(2);
		osDelay(2);

	}
}	

/**
  * @brief          ��̨�๹�캯�����г�ʼ��
  * @param[in]      null
  */
gimbal_control_t::gimbal_control_t()
{
	add_pit = add_yaw = 0.0f;
	//�������ָ���ȡ
	Yaw_motor.get_gimbal_motor = get_motor_measure_class(YAW);
	Pitch_motor.get_gimbal_motor = get_motor_measure_class(PITCH);
	//����������ָ���ȡ
	gimbal_INT_angle_point = get_INS_angle_point();
	gimbal_INT_gyro_point = get_gyro_data_point();

	//LADRC_FDW��ʼ��
	Pitch_motor.LADRC_FDW.init(22, 0.0065, 150,30000,80,0);
  Yaw_motor.LADRC_FDW.init(35, 0.004, 245, 30000,70,0);


//����LADRC_FDW��ʼ��
	Pitch_motor.AUTO_LADRC_FDW.init(18, 0.004,100,30000,80,0.5);
	Yaw_motor.AUTO_LADRC_FDW.init(8, 0.0015,75, 30000,90,0.8);

////����LADRC_FDW��ʼ��
//	Pitch_motor.AUTO_LADRC_FDW.init(20, 0.0023,90,30000,80,0.45);
//	Yaw_motor.AUTO_LADRC_FDW.init(15, 0.005,85, 30000,90,0.3);

	//PITCH�������С��ԽǶ�����
	Pitch_motor.max_relative_angle =0.3f;
	Pitch_motor.min_relative_angle = -0.52f;
	//YAW�������С��ԽǶ�����
	Yaw_motor.max_relative_angle =  Motor_Ecd_to_Rad*2048.0f;
	Yaw_motor.min_relative_angle = Motor_Ecd_to_Rad*-2048.0f;
	//�����ֵ
	Yaw_motor.offset_ecd =3860;
	Pitch_motor.offset_ecd = 2024;
	//Yaw��Ƕ�����
	Yaw_motor.relative_angle_set = 0;
	//Pitch��Ƕ�����
	Pitch_motor.absolute_angle_set = 0;
	Pitch_motor.relative_angle_set = 0;
	//�������˲�����ʼ��

	/*PID�Ƕ�������,һ��*/
	gimbal_kalman.Yaw_Error_Vis_Kalman.KalmanCreate( 1,  0);
	gimbal_kalman.Pitch_Error_Vis_Kalman.KalmanCreate( 1,  0);
	gimbal_kalman.Yaw_Set_Gim_Kalman.KalmanCreate( 1,  400);
	gimbal_kalman.Pitch_Set_Gim_Kalman.KalmanCreate( 1,  100);
	gimbal_kalman.Vision_Distance_Kalman.KalmanCreate( 1,  10);
	gimbal_kalman.Gimbal_Yaw_Accle_Kalman.KalmanCreate( 1,  10);
	gimbal_kalman.Gimbal_Yaw_Gyro_Kalman.KalmanCreate( 1,  100);
}



/**
  * @brief          ���ݸ���
  * @param[in]      null
  */
void gimbal_control_t::Get_info()
{
	
	Pitch_motor.absolute_angle = -*(gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
	Roll_motor.absolute_angle= *(gimbal_INT_angle_point + INS_ROLL_ADDRESS_OFFSET);
	Yaw_motor.absolute_angle = *(gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
	
  Pitch_motor.relative_angle =  motor_ecd_to_angle_change(Pitch_motor.get_gimbal_motor->ecd,
	                                                                    Pitch_motor.offset_ecd);	
	Yaw_motor.relative_angle = motor_ecd_to_angle_change(Yaw_motor.get_gimbal_motor->ecd,
                                                                  Yaw_motor.offset_ecd);
	
	Pitch_motor.motor_gyro = -*(gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
	
	Yaw_motor.motor_gyro = arm_cos_f32(Pitch_motor.relative_angle) * (*(gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
													-arm_sin_f32(Roll_motor.absolute_angle) * (*(gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));

	Yaw_motor.yaw_error=Yaw_motor.absolute_angle_set-Yaw_motor.absolute_angle;
}


/**
  * @brief          ����ecd��offset_ecd֮�����ԽǶ�
  * @param[in]      ecd: �����ǰ����
  * @param[in]      offset_ecd: �����ֵ����
  * @retval         ��ԽǶȣ���λrad
  */
fp32 gimbal_control_t::motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}


/**
  * @brief          ���������
  * @param[in]      null
  */
void gimbal_control_t::Control()
{
	if(syspoint()->sys_mode == ZERO_FORCE )
	{
		Zero_force_control();
	}
	else if(syspoint()->sys_mode == DBUS_MISSING_CONTROL)
	{
		Dbus_missing_control();
	}
	else if(syspoint()->sys_mode == ABSOLUTE_ANGLE||syspoint()->sys_mode== NO_FOLLOW_YAW)
	{
		Absolute_angle_control();
	}
	else if(syspoint()->sys_mode == SPIN)
	{
		Spin_control();
	}
	else if(syspoint()->sys_mode == AUTO||syspoint()->sys_mode == SPIN_AUTO)
	{
		Auto_control();
	}
	else if(syspoint()->init_state == 1)
	{
		Init_control();
	}
}


/**
  * @brief          ��������
  * @param[in]      null
  */
void gimbal_control_t::Zero_force_control()
{

	Yaw_motor.current_set = 0;
	Pitch_motor.current_set = 0;
	Yaw_motor.given_current = (int16_t)(Yaw_motor.current_set);
	Pitch_motor.given_current = (int16_t)(Pitch_motor.current_set);
}

/**
  * @brief          ʧ��ģʽ����
  * @param[in]      null
  */
void gimbal_control_t::Dbus_missing_control()
{
	Yaw_motor.current_set = 0;
	Pitch_motor.current_set = 0;
	Yaw_motor.given_current = (int16_t)(Yaw_motor.current_set);
	Pitch_motor.given_current = (int16_t)(Pitch_motor.current_set);
}

/**
  * @brief          ��ʼ��ģʽ����
  * @param[in]      null
  */
void gimbal_control_t::Init_control()
{

	//Init_angle_set();

//	Yaw_motor.Relative_anlge_limit(add_yaw);
//	Pitch_motor.Relative_anlge_limit(add_pit);

  Yaw_motor.relative_angle_set=INIT_YAW_SET;
  Pitch_motor.relative_angle_set=INIT_Relative_PITCH_SET;

	Yaw_motor.Relative_adrc_control();
	Pitch_motor.Relative_adrc_control();
}

/**
  * @brief          ���ԽǶȿ��ƣ����̸�����̨��
  * @param[in]      null
  */
void gimbal_control_t::Absolute_angle_control()
{
  syspoint()->Gimbal_value_calc();
	add_yaw= syspoint()->rc_add_yaw;
	add_pit= syspoint()->rc_add_pit;

	Keyboard_angle_set();
	
	Yaw_motor.Absolute_angle_limit(add_yaw);
	Pitch_motor.Absolute_angle_limit(add_pit);

	Yaw_motor.Absolute_adrc_control();
	Pitch_motor.Absolute_adrc_control();
}


/**
  * @brief          С����ģʽ����
  * @param[in]      null
  */
void gimbal_control_t::Spin_control()
{
  syspoint()->Gimbal_value_calc();
	add_yaw= syspoint()->rc_add_yaw;
	add_pit= syspoint()->rc_add_pit;

	Yaw_motor.Absolute_angle_nolimit(add_yaw);
	Pitch_motor.Absolute_angle_limit(add_pit);
	
	Yaw_motor.Absolute_adrc_control();
	Pitch_motor.Absolute_adrc_control();
}


/**
  * @brief          ����ģʽ����
  * @param[in]      null
  */
void gimbal_control_t::Auto_control()
{
	Gimbal_auto_angle_get();
	Yaw_motor.Absolute_auto_adrc_control();
	Pitch_motor.Absolute_auto_adrc_control();
}



/**
  * @brief          ��ʼ��ģʽ�Ƕȿ���
  * @param[in]      null
  */
void gimbal_control_t::Init_angle_set()
{
	if(fabs(INIT_PITCH_SET - Pitch_motor.relative_angle) > GIMBAL_INIT_ANGLE_ERROR)
	{
		add_pit = (INIT_PITCH_SET - Pitch_motor.relative_angle) * GIMBAL_INIT_PITCH_SPEED;
    add_yaw = 0.0f;
	}
	else 	if(fabs(INIT_YAW_SET - Yaw_motor.relative_angle) > GIMBAL_INIT_ANGLE_ERROR)
	{
		add_pit =(INIT_PITCH_SET - Pitch_motor.relative_angle) * GIMBAL_INIT_PITCH_SPEED;
    add_yaw = (INIT_YAW_SET - Yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
	}
}

/**
  * @brief          ����ǶȻ�ȡ
  * @param[in]      null
  */
void gimbal_control_t::Gimbal_auto_angle_get()
{

//��ȡ�Ӿ�����
	  if(vision_info_point()->RxPacketFir.is_findtarget==1)
		{ 
			Yaw_motor.absolute_angle_set =  Vision_process_point()->data_kal.YawTarget_KF;
			Pitch_motor.absolute_angle_set =  Vision_process_point()->data_kal.PitchTarget_KF;
			
			if(isnan(Pitch_motor.absolute_angle_set))
			{
					Pitch_motor.absolute_angle_set=Pitch_motor.absolute_angle;
			}
			if(isnan(Yaw_motor.absolute_angle_set))
			{
					Yaw_motor.absolute_angle_set=Yaw_motor.absolute_angle;
			}
			
					

		}
    else
    {
			Yaw_motor.absolute_angle_set = Yaw_motor.absolute_angle;
			Pitch_motor.absolute_angle_set = Pitch_motor.absolute_angle;
		}


//��������
if(vision_info_point()->RxPacketFir.is_spinning==1)
{
		if(vision_info_point()->RxPacketFir.distance>3.0)
		{
			if(fabs(Yaw_motor.absolute_angle-Yaw_motor.absolute_angle_set)<0.01)
			{
					if(fabs(Pitch_motor.absolute_angle-Pitch_motor.absolute_angle_set)<0.01)
					{
						auto_firc_flag_angle=1;
					}
					else{auto_firc_flag_angle=0;}
			}
			else{auto_firc_flag_angle=0;}
		}else
		{	
			if(fabs(Yaw_motor.absolute_angle-Yaw_motor.absolute_angle_set)<0.02)
			{
					if(fabs(Pitch_motor.absolute_angle-Pitch_motor.absolute_angle_set)<0.02)
					{
						auto_firc_flag_angle=1;
					}
					else{auto_firc_flag_angle=0;}
			}
			else{auto_firc_flag_angle=0;}
		}
}
else
{
		auto_firc_flag_angle=1;
}
 
}

/**
  * @brief          һ����ͷ
  * @param[in]      null
  */
void gimbal_control_t::Keyboard_angle_set()
{
	 static uint16_t last_turn_keyboard = 0;
   static uint8_t gimbal_turn_flag = 0;
   static fp32 gimbal_end_angle = 0.0f;
	 int rc_keyCZ_time;
	 int turn_mode;


	 if((IF_KEY_PRESSED_CTRL&&IF_KEY_PRESSED_Z)||(rc_ctrl.rc.ch[4]<=-200&&(abs(rc_ctrl.rc.ch[0])>=10||abs(rc_ctrl.rc.ch[1]>=10)))||(rc_ctrl.mouse.z<-2))//
   {
     if (gimbal_turn_flag == 0)
     {
       gimbal_turn_flag = 1;
       //�����ͷ��Ŀ��ֵ
       gimbal_end_angle = rad_format(Yaw_motor.absolute_angle + PI);
     }
    }
    last_turn_keyboard = syspoint()->system_rc_ctrl->key.v ;

    if (gimbal_turn_flag)
    {
      //���Ͽ��Ƶ���ͷ��Ŀ��ֵ����ת����װ�����
				Yaw_motor.absolute_angle_set=RAMP_float(gimbal_end_angle,Yaw_motor.absolute_angle,0.45);
    }
    //����pi ��180�㣩��ֹͣ
    if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - Yaw_motor.absolute_angle)) < 0.01f)
    {
      gimbal_turn_flag = 0;
    }
}

/**
  * @brief          �жϳ�ʼ���Ƿ����
  * @param[in]      null
  */
void gimbal_control_t::Judge_init_state()
{
	if(syspoint()->init_state == 1)
	{
		static uint16_t init_time = 0;
    static uint16_t init_stop_time = 0;
    init_time++;
        
    if ((fabs(Yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
         fabs(Pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
    {        
      if (init_stop_time < GIMBAL_INIT_STOP_TIME)
      { 
        init_stop_time++;
      }
    }
    else
    {     
      if (init_time < GIMBAL_INIT_TIME)
      {
        init_time++;
      }
    }

    //������ʼ�����ʱ�䣬�����Ѿ��ȶ�����ֵһ��ʱ�䣬�˳���ʼ��״̬���ش��µ������ߵ���
    if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
        !switch_is_down(syspoint()->system_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) )//&& !toe_is_error(DBUS_TOE)
    {
      return;
    }
    else
    {
      init_stop_time = 0;
      init_time = 0;
			syspoint()->init_state =0;
    }
	}
	
}

/**
  * @brief          ���ԽǶ�����
  * @param[in]      fp32 add��ֵ
  */
void gimbal_motor_t::Absolute_angle_limit(fp32 add)
{
	static fp32 bias_angle;
  static fp32 angle_set;
	//��ǰ���ƽǶȣ����Ƕȣ�ѭ���޷�֮��ģ�
	bias_angle = rad_format(absolute_angle_set - absolute_angle);
	//relative angle + angle error + add_angle > max_relative angle
  //��̨��ԽǶ�+ ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�
  if (relative_angle + bias_angle + add > max_relative_angle)
  {
    //�����������е�Ƕȿ��Ʒ���
    if (add > 0.0f)
    {
      //calculate max add_angle
      //�����һ��������ӽǶȣ�
      add = max_relative_angle - relative_angle - bias_angle;
    }
  }
  else if (relative_angle + bias_angle + add < min_relative_angle)
  {
    if (add < 0.0f)
    {
      add = min_relative_angle - relative_angle - bias_angle;
    }
  }
  angle_set = absolute_angle_set;
  absolute_angle_set = rad_format(angle_set + add);
}

/**
  * @brief          �޾��ԽǶ�����
  * @param[in]      fp32 add��ֵ
  */
void gimbal_motor_t::Absolute_angle_nolimit(fp32 add)
{
	static fp32 angle_set;
	angle_set = absolute_angle_set;
  absolute_angle_set = rad_format(angle_set + add);
}

/**
  * @brief          ��ԽǶ�����
  * @param[in]      fp32 add��ֵ
  */
void gimbal_motor_t::Relative_anlge_limit(fp32 add)
{
	relative_angle_set += add;
	//�Ƿ񳬹���� ��Сֵ
	if (relative_angle_set > max_relative_angle)
  {
    relative_angle_set = max_relative_angle;
  }
  else if (relative_angle_set < min_relative_angle)
  {
    relative_angle_set = min_relative_angle;
  }

}

/**
  * @brief          ���������ԽǶȿ���
  * @param[in]      null
  */
void gimbal_motor_t::Absolute_adrc_control()
{
	current_set = LADRC_FDW.FDW_calc(absolute_angle ,absolute_angle_set ,motor_gyro);
	given_current = (int16_t)(current_set);
}

void gimbal_motor_t::Absolute_auto_adrc_control()
{
	current_set = AUTO_LADRC_FDW.FDW_calc(absolute_angle ,absolute_angle_set ,motor_gyro);
	given_current = (int16_t)(current_set);
}

/**
  * @brief          ��������ԽǶȿ���
  * @param[in]      null
  */
void gimbal_motor_t::Relative_adrc_control()
{
	current_set = LADRC_FDW.FDW_calc(relative_angle , relative_angle_set , motor_gyro);
	given_current = (int16_t)current_set;
}
