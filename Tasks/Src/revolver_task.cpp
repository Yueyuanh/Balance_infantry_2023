#include "system_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Revolver_task.h"
#include "judge_task.h"
#include "ui_task.h"

Revolver_Task  revolver; 

Revolver_Task* revolver_point(void)
{
	return &revolver;
}

/*********ȫ�ֱ���*************/

int stuck_mode_to_ui;
int v_firc_set;
extern int vision_mode;
bool auto_firc_flag_angle;
int num_firc;

/******************************/
/**
  * @brief          �����������
  * @param[in]      null
  */
void revolver_task(void const *pvParameters)
{
	//��ʱ
	vTaskDelay(REVOLVER_TASK_INIT_TIME);

	//��������ʼ��
	revolver.REVOLVER_Init();
	
	while(1)
	{
		//���ݸ���
		revolver.Revolver_Feedback_Update(); 

		//����ֵ�趨
		revolver.Friction_wheel_set.Friction_wheel_mode_set();
		revolver.Dial_set.Dial_behaviour_set();

		//�趨ֵ����
		revolver.Friction_wheel_calculating();
		revolver.Dial_calculating();

		//�����ķ���
		CAN2_200_cmd_motor(revolver.Firc_L_give_current,revolver.Firc_R_give_current,revolver.Dial_give_current,0);
		//CAN2_200_cmd_motor(0,0,revolver.Dial_give_current,0);
		vTaskDelay(2);
  }
}

/**
  * @brief          ������������ʼ��
  * @param[in]      null
  */

void Revolver_Task::REVOLVER_Init()

{
	Friction_wheel_set.Friction_Wheel_behaviour_init();
	Dial_set.Dial_mode_init();

	//��ʼ��PID
	fp32 Revolver_speed_pid[3] = {REVOLVER_SPEED_PID_KP, REVOLVER_SPEED_PID_KI, REVOLVER_SPEED_PID_KD};
	fp32 Revolver_position_pid[3] = {REVOLVER_POSITION_PID_KP, REVOLVER_POSITION_PID_KI, REVOLVER_POSITION_PID_KD};
	fp32 Firc_speed_pid[3] = {FIRC_SPEED_PID_KP, FIRC_SPEED_PID_KI, FIRC_SPEED_PID_KD};	 
	revolver_motor_speed_pid.init( PID_POSITION, Revolver_speed_pid,REVOLVER_SPEED_PID_MAX_OUT, REVOLVER_SPEED_PID_MAX_IOUT);
	revolver_motor_position_pid.init(PID_POSITION, Revolver_position_pid,REVOLVER_POSITION_PID_MAX_OUT, REVOLVER_POSITION_PID_MAX_IOUT);

	Firc_R_speed_pid.init(PID_POSITION, Firc_speed_pid,FIRC_SPEED_PID_MAX_OUT, FIRC_SPEED_PID_MAX_IOUT);
	Firc_L_speed_pid.init(PID_POSITION, Firc_speed_pid,FIRC_SPEED_PID_MAX_OUT, FIRC_SPEED_PID_MAX_IOUT);

	//б�º�����������λ��
	Dial_buff_ramp = AN_BULLET/30;
	//���ָ��
	revolver_motor_measure = get_motor_measure_class(TRIGGER);
	Firc_L_firc3508_motor_measure = get_motor_measure_class(FireL);
	Firc_R_firc3508_motor_measure = get_motor_measure_class(FireR);

	angle_out=0;
	//ģʽ��Ƕȳ�ʼ��
	Revolver_Feedback_Update();
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	//����PID
	fp32 speed_pid[3]={SPEED_PID_KP,SPEED_PID_KI,SPEED_PID_KD};
	Speed_pid.init(PID_POSITION,speed_pid,SPEED_PID_MAX_OUT,SPEED_PID_MAX_IOUT);
	
}

float Firc_L_firc3508_motor_speed,Firc_R_firc3508_motor_speed;
float Firc_L_tempreate,Firc_R_tempreate,gloab_bullet_speed;
/**
  * @brief          ����������ݸ���
  * @param[in]      null
  */

void Revolver_Task::Revolver_Feedback_Update()
{
	static fp32 speed_fliter_1 = 0.0f;
	static fp32 speed_fliter_2 = 0.0f;
	static fp32 speed_fliter_3 = 0.0f;

	//�����ֵ���ٶ��˲�һ��
	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
	//���׵�ͨ�˲�
	speed_fliter_1 = speed_fliter_2;
	speed_fliter_2 = speed_fliter_3;
	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (revolver_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
	Revolver_speed = speed_fliter_3;
	firc_l = Firc_L_firc3508_motor_measure->speed_rpm*Motor_RMP_TO_SPEED;
	firc_r = Firc_R_firc3508_motor_measure->speed_rpm*Motor_RMP_TO_SPEED;
	//���������Ƕ�
	Dial_set.angle = revolver_motor_measure->num*-8192 + revolver_motor_measure->ecd;
	

	Dial_set.heating = JUDGE_usGetRemoteHeat_id1_17mm();
	bullet_speed=JUDGE_usGetSpeedHeat();
	Dial_set.heat_max=JUDGE_usGetHeatLimit_id1_17mm();

	Dial_set.real_firc_info_update();
	judge_auto_firc();
}


/**
  * @brief          �������Ħ���ֿ���������
  * @param[in]      null
  */


/**
  * @brief          �������Ħ���ֿ���������
  * @param[in]      null
  */

void Revolver_Task::Friction_wheel_calculating()
{

		if(switch_is_down(Friction_wheel_set.Friction_wheel_RC->rc.s[RevChannel]) || switch_is_up(Friction_wheel_set.Friction_wheel_RC->rc.s[RevChannel]) 
			|| switch_is_down(Friction_wheel_set.Friction_wheel_RC->rc.s[0]))
		{
			Firc_L_give_current = Firc_L_speed_pid.calc( Firc_L_firc3508_motor_measure->speed_rpm,Friction_wheel_set.Firc_L.speed_set);
			Firc_R_give_current = Firc_R_speed_pid.calc( Firc_R_firc3508_motor_measure->speed_rpm,Friction_wheel_set.Firc_R.speed_set);

			Firc_L_speed_ramp_set = 0;
		  Firc_R_speed_ramp_set = 0;

		} 
		else if((switch_is_mid(Friction_wheel_set.Friction_wheel_RC->rc.s[RevChannel]) &&switch_is_mid(Friction_wheel_set.Friction_wheel_RC->rc.s[0]))
			   ||(switch_is_mid(Friction_wheel_set.Friction_wheel_RC->rc.s[RevChannel]) &&switch_is_up(Friction_wheel_set.Friction_wheel_RC->rc.s[0])))
		{
			Firc_L_speed_ramp_set = RAMP_float(Friction_wheel_set.Firc_L.speed_set,Firc_L_firc3508_motor_measure->speed_rpm,2500);
			Firc_R_speed_ramp_set = RAMP_float(Friction_wheel_set.Firc_R.speed_set, Firc_R_firc3508_motor_measure->speed_rpm,2500);			
			
			// �������ֵ
			Firc_L_give_current	=Firc_L_speed_pid.calc( Firc_L_firc3508_motor_measure->speed_rpm,Firc_L_speed_ramp_set);
			Firc_R_give_current	=	Firc_R_speed_pid.calc(Firc_R_firc3508_motor_measure->speed_rpm,Firc_R_speed_ramp_set);
		}		
		else 
		{
		  Firc_L_give_current=0;
	   	Firc_R_give_current=0;
			Firc_L_speed_ramp_set = 0;
		  Firc_R_speed_ramp_set = 0; 
		
		}
}



/**
  * @brief          Ħ����ģʽ��ʼ��
  * @param[in]      null
  */

void Friction_wheel_Mode_set::Friction_Wheel_behaviour_init()
{
	Friction_wheel_RC = get_remote_control_point();
	friction_wheel_mode = FRICTION_WHEEL_OFF;
	firc_mode=0;
	
}

/**
  * @brief          Ħ����ģʽ����
  * @param[in]      null
  */
	
void Friction_wheel_Mode_set::Friction_wheel_mode_set()
{
	if((switch_is_down(Friction_wheel_RC->rc.s[0]) == 0||syspoint()->sys_mode==AUTO||syspoint()->sys_mode==SPIN_AUTO)&&((switch_is_down(Friction_wheel_RC->rc.s[1]) == 0))&&syspoint()->sys_mode!=DBUS_MISSING_CONTROL)//���������� Ħ���ֿ���ģʽ
	{
		friction_wheel_mode = FRICTION_WHEEL_ON;
		Friction_wheel_mode_on_set();
		firc_mode=1;
	}
	else
	{
		friction_wheel_mode = FRICTION_WHEEL_OFF;
		Friction_wheel_mode_off_set();
		firc_mode=0;
	}
}

/**
  * @brief          Ħ���ֿ���ģʽ���趨ֵ����
  * @param[in]      null
  */

void Friction_wheel_Mode_set::Friction_wheel_mode_on_set()
{
	  //speedlimit = JUDGE_usGetSpeedLimit();
		speedlimit = 15;
		User_fix();

	//���ݵ������޽��е��ٲ���
	if(revolver.bullet_last_speed<revolver.bullet_speed)
	{
		if(speedlimit==15)
		{
      revolver.speed_adapt(revolver.bullet_speed , 13.5 , 14.5 ,&revolver.fix_num , 5 , 5);
		}
    
		else if(speedlimit==18)
		{
      revolver.speed_adapt(revolver.bullet_speed , 16.5 , 17.5 ,&revolver.fix_num , 5 , 5);
		}

		else if(speedlimit==30)
		{
      revolver.speed_adapt(revolver.bullet_speed , 28.5 , 29.5 ,&revolver.fix_num , 5 , 5);
		}
		else 
		{
		  revolver.speed_adapt(revolver.bullet_speed , 13.5 , 14.5 ,&revolver.fix_num , 5 , 5);
		}
	}
	else
	{
		revolver.fix_num = 0;
	}


	if(speedlimit==15)
	{ 
		Temp_Fix_15S();
		Firc_L.speed_set = -abs(v_fic_set);
		Firc_R.speed_set = abs(v_fic_set);
	}
	else if(speedlimit==18)
	{	
		Temp_Fix_18S();
		Firc_L.speed_set = -abs(v_fic_set);
		Firc_R.speed_set = abs(v_fic_set);
	}
	else
	{
		Temp_Fix_30S();
		Firc_L.speed_set = -abs(v_fic_set);
		Firc_R.speed_set = abs(v_fic_set);
	}
}


/**
  * @brief          ��������Ӧ
  * @param[in]      null
  */

void Revolver_Task::speed_adapt(fp32 real_S , fp32 min_S, fp32 max_S,fp32 *fix , fp32 up_num , fp32 down_num)
{
	static uint8_t SpeedErr_cnt=0;
  if(real_S < min_S && real_S > 20)
    SpeedErr_cnt++;
  else if(real_S >= min_S && real_S <= max_S )
	  SpeedErr_cnt = 0;

  if(SpeedErr_cnt == 1)//����ƫ��
  {
    SpeedErr_cnt = 0;
    *fix += up_num;
  }
  
  if(real_S > max_S)//����ƫ��
    *fix -= down_num;
}

/**
  * @brief           Ħ���ֹر�ģʽ���趨ֵ����
  * @param[in]       null
  */

void Friction_wheel_Mode_set::Friction_wheel_mode_off_set()
{
		Firc_L.speed_set=0;
		Firc_R.speed_set=0;
		Firc_L.speed_ramp_set = 0;
		Firc_R.speed_ramp_set = 0;
}


/**
  * @brief           �������ֶ�����Ħ����ת������
  * @param[in]       null
  */
void Friction_wheel_Mode_set::User_fix()
{
	if(IF_KEY_PRESSED_C&&IF_KEY_PRESSED_CTRL)//c�� v��
	{
		fix_mode = 1;
	}
	else if(IF_KEY_PRESSED_V&&IF_KEY_PRESSED_CTRL)
	{
		fix_mode = 2;
	}
	else 
	{
		fix_mode = 0;
	}
	if(fix_mode == 1&&fix_last_mode != 1)
	{
		fixed += 50;
		fix_times++;
		
	}
	else if(fix_mode == 2&&fix_last_mode != 2)
	{
		fixed -= 50;
		fix_times--;
	}
	fix_last_mode = fix_mode;

	v_firc_set=v_fic_set;//������
	//save
	Temp_fix_save();
}


/**
  * @brief           Ħ�����ֶ�����
  * @param[in]       null
  */

void Friction_wheel_Mode_set::Temp_Fix_15S()
{
   v_fic_set = FRICTION_L1_SPEED+fixed;	
}
/**
  * @brief           Ħ�����ֶ�����
  * @param[in]       null
  */
void Friction_wheel_Mode_set::Temp_Fix_18S()
{
   v_fic_set = FRICTION_L2_SPEED+fixed;	
}
/**
  * @brief           Ħ�����ֶ�����
  * @param[in]       null
  */
void Friction_wheel_Mode_set::Temp_Fix_30S()
{
   v_fic_set = FRICTION_L3_SPEED+fixed;	
	if(v_fic_set>FRICTION_L3_SPEED)
	{
			v_fic_set=FRICTION_L3_SPEED;
	}
}
/**
  * @brief           Ħ�����Զ�����/����
  * @param[in]       null
  */
void Friction_wheel_Mode_set::Temp_fix_save()
{
		if(revolver.bullet_speed>speedlimit-0.3)
		{
				v_fic_set-=100;
		}
	

}
/**
  * @brief           ����ģʽ��ʼ��
  * @param[in]       null
  */

void Dial_mode_set::Dial_mode_init()
{
		Dial_RC = get_remote_control_point();
		Revolver_mode=REVOL_POSI_MODE;
		set_angle =angle;
}


/**
  * @brief           ���̴���ʱ
  * @param[in]       null
  */
bool real_shoot;
bool start_firc;
int delay_count;
float per_fir_delay;
bool To_zero_flag;

void Dial_mode_set::real_firc_info_update()
{
		bullet_speed = JUDGE_usGetSpeedHeat();
		if(bullet_speed != bullet_last_speed)
		{
				real_shoot=1;
		}
		else
		{
				real_shoot=0;
		}
		bullet_last_speed=bullet_speed;

    if(start_firc)
		{
				++delay_count;
		}

		if(real_shoot&&start_firc)
		{
				per_fir_delay=delay_count*2;
				if(per_fir_delay>200)
				{per_fir_delay=100;}
				//per_fir_delay-=10;
				start_firc=0;
				delay_count=0;
				real_shoot=0;
		}

}

void Revolver_Task::judge_auto_firc()
{
	auto_flag = vision_info_point()->RxPacketFir.is_shooting;
	if(auto_flag == 1 && last_auto_flag == 0)
		auto_firc = 1;
	else
		auto_firc = 0;

  last_auto_flag = auto_flag;
}
/**
  * @brief           ����ģʽ���ü��趨ֵ����
  * @param[in]       null
  */


void Dial_mode_set::Dial_behaviour_set()
{
		static int16_t speed_time;
		int auto_firc_flag=0;



if(vision_info_point()->RxPacketFir.is_spinning==1)
{
		vision_mode=1;					//С�������Զ�����
}
else{
		vision_mode=3;					//����״̬���ֶ�����
}


/*********************************�Զ�����*******************************/
		if(vision_mode==1)
		{
				if(((syspoint()->sys_mode==AUTO||syspoint()->sys_mode==SPIN_AUTO)&&
				(fabs(revolver.Firc_L_firc3508_motor_measure->speed_rpm-revolver.Friction_wheel_set.Firc_L.speed_set)<100)&&
				(fabs(revolver.Firc_R_firc3508_motor_measure->speed_rpm-revolver.Friction_wheel_set.Firc_R.speed_set)<100)))
				{
						if(vision_info_point()->RxPacketFir.is_shooting==1&&auto_firc_flag_angle)
//						if(revolver.auto_firc && auto_firc_flag_angle)
						{
						speed_time++;
						num_firc=1;
						Revolver_Switch = REVOL_STEP1;
						if(speed_time>30)//300
						{
							//	Revolver_Switch = REVOL_STEP1;	//��������
							Revolver_Switch = REVOL_STEP2;	//

						}		
						}
						else if(vision_info_point()->RxPacketFir.is_shooting==0)
						{
								Revolver_Switch = REVOL_STEP0;
								key_ShootFlag = shoot_unready;
								speed_time = 0;

						}
				}
				//��������
				if ( (switch_is_up(Dial_RC->rc.s[RevChannel])||Dial_RC->mouse.press_l==1)&&((switch_is_down(Dial_RC->rc.s[0])!=1)||syspoint()->sys_mode==AUTO||syspoint()->sys_mode==SPIN_AUTO))
				{
						speed_time++;
						num_firc=1;
						Revolver_Switch = REVOL_STEP1;
						if(speed_time>300)//300
						{
								Revolver_Switch = REVOL_STEP2;	
						}			
				}
				else 
				{
						Revolver_Switch = REVOL_STEP0;
						key_ShootFlag = shoot_unready;
						speed_time = 0;
				}
		}
else if(vision_mode==3)
{
		num_firc=1;
//�󲦸˲���һ�λ�����������ΪSTEP1  �󲦸�һֱ���ϻ򳤰����ΪSTEP2
		if ( (switch_is_up(Dial_RC->rc.s[RevChannel])||Dial_RC->mouse.press_l==1)&&((switch_is_down(Dial_RC->rc.s[0])!=1)||syspoint()->sys_mode==AUTO||syspoint()->sys_mode==SPIN_AUTO))
		{
			speed_time++;
			Revolver_Switch = REVOL_STEP1;

			if(speed_time>300)//300
			{
				Revolver_Switch = REVOL_STEP2;	
			}			
		}
		else 
		{
				Revolver_Switch = REVOL_STEP0;
				key_ShootFlag = shoot_unready;
				speed_time = 0;
		}
}


		//STEP2�����ٶȻ�
		if(Revolver_Switch == REVOL_STEP2&&ifstuck_finish != stuck_unfinish)
				Revolver_mode =	REVOL_SPEED_MODE;
		else if((Revolver_Switch == REVOL_STEP1||Revolver_Switch == REVOL_STEP0)&&ifstuck_finish != stuck_unfinish)
			  Revolver_mode =	REVOL_POSI_MODE;
		


if(vision_mode==1)
{
//���˲���һ�λ��������һ�η���1 ����Ϊ0
		if((Revolver_Switch == REVOL_STEP1 && Revolver_last_Switch == REVOL_STEP0)||
        (vision_info_point()->RxPacketFir.is_shooting==1&&(syspoint()->sys_mode==AUTO||syspoint()->sys_mode==SPIN_AUTO)))
		{
			if(heating<heat_max-HEAT_ERROR|| heat_max == 0)//abs(revolver_control->set_angle- revolver_control->angle)<1000) || heat_max == 0)
			{
				key_ShootFlag = shoot_ready;
			}
		}
		else
		{
		key_ShootFlag = shoot_unready;
		}	
}
else
{
if((Revolver_Switch == REVOL_STEP1 && Revolver_last_Switch == REVOL_STEP0))
		{
			if(heating<heat_max-HEAT_ERROR|| heat_max == 0)//abs(revolver_control->set_angle- revolver_control->angle)<1000) || heat_max == 0)
			{
				key_ShootFlag = shoot_ready;
			}
		}
		else
		{
			key_ShootFlag = shoot_unready;
		}	
}
			
		Revolver_last_Switch = Revolver_Switch;

    //�������
		if(abs(set_angle - angle)> AN_BULLET/2)//����Ӧ����С��һ��ĵ��ӵ�λ���Ϊ����δ������
		{
			ifshoot_finsh = shoot_unfinish;
			unfinish_time++;
		}
		else
		{
			unfinish_time = 0;
			ifshoot_finsh = shoot_finish;
			ifstuck_finish = stuck_finish;
		}
		if(unfinish_time>500)
		{
			Revolver_mode = REVOL_REVERSAL_MODE;
			ifstuck_finish = stuck_unfinish;
			if_stucked = 1;
			stuck_mode = 1;
			stuck_mode_to_ui=1;
		}



		//�����뿨��ģʽ
		if(Revolver_mode == REVOL_REVERSAL_MODE&&last_Revolver_mode!=REVOL_REVERSAL_MODE)
		{
			set_angle += AN_BULLET*3/2;
		}

		else if(Revolver_mode == REVOL_POSI_MODE)//˫������
		{
			if(key_ShootFlag == shoot_ready&&ifshoot_finsh != shoot_unfinish)//&&abs(revolver_control->set_angle - revolver_control->last_angle)<1000)
			{
					set_angle -= AN_BULLET*num_firc;	
//					set_angle -= AN_BULLET;
					start_firc=1;
//					--num_firc;
				 if(if_stucked == 1)
				{
					set_angle += AN_BULLET/2;
					if_stucked = 0;
					stuck_mode=0;
					stuck_mode_to_ui=0;
				}
			}
		}
		//�������ֵ��������
		else if(Revolver_mode == REVOL_SPEED_MODE)
		{
			//������������
			if( ifshoot_finsh != shoot_unfinish && (heating<heat_max-HEAT_ERROR|| heat_max ==0))
				{
					if(if_stucked == 1)
					{
						set_angle+= AN_BULLET/2;
						if_stucked = 0;
						stuck_mode = 0;
						stuck_mode_to_ui=0;
					}
					else
					{
					//˫������
							set_angle-= AN_BULLET;
							start_firc=1;
					}
				}
		}


		if(auto_firc_flag==1)
		{
			//������������
			if( (heating<heat_max-HEAT_ERROR|| heat_max ==0))
				{
					if(if_stucked == 1)
					{
						set_angle+= AN_BULLET/2;
						if_stucked = 0;
						stuck_mode = 0;
						stuck_mode_to_ui=0;
					}
					else
					{
					  //˫������
						set_angle-= AN_BULLET*num_firc;
						start_firc=1;
					}
				}
		}


		 last_Revolver_mode = Revolver_mode;
		if(switch_is_down(Dial_RC->rc.s[RevChannel]))
		{
		  set_angle=angle;
		}
}



/**
  * @brief          ����������̿���������
  * @param[in]      null
  */
void Revolver_Task::Dial_calculating()
{
		if(Dial_set_ramp_angle != Dial_set.set_angle)//����ת��ȥ
		{
			Dial_set_ramp_angle = RAMP_float(Dial_set.set_angle, Dial_set_ramp_angle, Dial_buff_ramp);
		}
		//�ǶȻ����ٶȻ�����pid���ԣ������������ڲ������ۼ�ʱ�Ѿ���ɣ�
		angle_out = revolver_motor_position_pid.calc( Dial_set.angle, Dial_set_ramp_angle);
		Dial_give_current = revolver_motor_speed_pid.calc(Revolver_speed, angle_out);

		if(switch_is_down(Dial_set.Dial_RC->rc.s[RevChannel]))
		{
			Dial_give_current=0;
		}

}
