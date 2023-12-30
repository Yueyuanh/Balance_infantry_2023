#ifndef __SYSTEM_TASK_H
#define __SYSTEM_TASK_H

#include "remote_control.h"
#include "detect_task.h"
#include "ladrc_feedforward.h"
#include "user_lib.h"
#include "vision.h"
//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0
#define REVOLVER_MODE_CHANNEL 1
//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0

#define GIMBAL_CALC 0
#define CHASSIS_CALC 1

#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.00005f

//ң����ǰ������ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.005f
#define CHASSIS_VY_RC_SEN 0.005f

#define CHASSIS_VX_RE_SEN 0.65f 
#define CHASSIS_VY_RE_SEN 0.65f 


#define CHASSIS_WZ_RE_SEN      0.7f
//ң����ҡ�ˣ�max 660��ת���ɳ���yaw pitch�ٶȣ�m/s���ı���
#define YAW_RC_SEN    -0.00001f
#define PITCH_RC_SEN  -0.00000422f //0.005

//YAW����ת���ư�ť
#define GIMBAL_LEFT_KEY KEY_PRESSED_OFFSET_Q
#define GIMBAL_RIGHT_KEY KEY_PRESSED_OFFSET_E

//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_DEADBAND   0
//ҡ�����������̣�
#define CHASSIS_RC_DEADLINE 50

//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 5.0f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.0f

#define HEAT_ERROR 30    //��������

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
enum system_mode_t
{
	ZERO_FORCE = 0,
  INIT,
	SPIN,
	ABSOLUTE_ANGLE,
	RELATIVE_ANGLE,
	NO_FOLLOW_YAW,
	AUTO,
	CALI,
  DBUS_MISSING_CONTROL,
	SPIN_AUTO,
	GO_HEAD,
};	

enum shoot_mode_t
{
	Close = 0,
	Open_Fir,
	One_shoot,
};

class system_t
{	
	public:
		const RC_ctrl_t *system_rc_ctrl;

	  system_mode_t sys_mode;
    system_mode_t last_sys_mode;
	  shoot_mode_t revolver_mode;
	  
	  lpf_type_def yaw_lpf;
	  lpf_type_def pitch_lpf;
	
	  fp32 rc_add_yaw;
	  fp32 rc_add_pit;
	  uint8_t mode_change_type;
    bool_t vision_flag;
		bool_t vision_last_flag;
		bool_t vision_auto_flag;
		bool_t head_flag;
		bool_t head_last_flag;
		bool_t head_go_flag;
	  bool  init_state;
	
//    first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
//    first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
//	  
    fp32 chassis_vx_set_temp;
		fp32 chassis_vy_set_temp;
		fp32 vx_max_speed;  //max forward speed, unit m/s.ǰ����������ٶ� ��λm/s
    fp32 vx_min_speed;  //max backward speed, unit m/s.���˷�������ٶ� ��λm/s
    fp32 vy_max_speed;  //max letf speed, unit m/s.��������ٶ� ��λm/s
    fp32 vy_min_speed;  //max right speed, unit m/s.�ҷ�������ٶ� ��λm/s
	  
	  system_t();
		
	  void Mode_set();
    void Transit();
		void Set_control();
		
		void Rc_vision();
		void RC_Mode_Set();
		void Rc_head();
		int  Gimbal_mode_change_transit();
	  void Gimbal_value_calc();
	  void Chassis_value_calc();
		void Key_set();
};	


extern system_t *syspoint(void);	

//Ħ�����ٶȻ�PID����
#define FIRC_SPEED_PID_KP 15.50f//15.50f
#define FIRC_SPEED_PID_KI 0.000f
#define FIRC_SPEED_PID_KD 5.50f//1.50f
#define FIRC_SPEED_PID_MAX_OUT 16384.0f
#define FIRC_SPEED_PID_MAX_IOUT 5000.0f

//���ٻ�����
#define SPEED_PID_KP 15 
#define SPEED_PID_KI  0
#define SPEED_PID_KD  1
#define SPEED_PID_MAX_OUT 30
#define SPEED_PID_MAX_IOUT 10

//Ħ����ת���趨
#define FRICTION_L1_SPEED 4600    //15
#define FRICTION_L2_SPEED 4800		//18
#define FRICTION_L3_SPEED 7200		//30



#define REVOL_STEP0    0		
#define REVOL_STEP1    1		
#define REVOL_STEP2    2		


#define RevChannel 1

#define  	AN_BULLET         36864//38000//33903.370320855614973262032085562	//36864//33903.370320855614973262032085562		//�����ӵ����λ������ֵ


typedef enum
{
	shoot_unready  = 0,
	shoot_ready = 1,
}Key_ShootFlag;

typedef enum
{
	FRICTION_WHEEL_OFF,
	FRICTION_WHEEL_ON,
}Friction_Wheel_Mode;

typedef struct
{
  fp32 speed_set;
	fp32 speed_ramp_set;
} Firc3508_Motor_set;



class Friction_wheel_Mode_set
{
	public:
		Friction_Wheel_Mode  friction_wheel_mode;
		void Friction_Wheel_behaviour_init();
		const RC_ctrl_t *Friction_wheel_RC;               //Ħ����ʹ�õ�ң����ָ��
		void Friction_wheel_mode_set();										//Ħ����ģʽ����
		void Friction_wheel_mode_off_set();
		void Friction_wheel_mode_on_set();
		void User_fix();																	//ת���ֶ�����
		void Temp_Fix_30S();
		void Temp_Fix_18S();
		void Temp_Fix_15S();
		void Temp_fix_save();
		Firc3508_Motor_set Firc_L;
		Firc3508_Motor_set Firc_R;
		uint16_t speedlimit;
		int fix_mode;
		int fix_last_mode;
		int fix_times;
		fp32 fixed;
		int16_t v_fic_set;
		fp32 firc_mode;
};
	
	

typedef enum
{
	shoot_unfinish  = 0,
	shoot_finish = 1,
}Shoot_IFfinish;
typedef enum
{
	stuck_unfinish  = 0,
	stuck_finish = 1,
}Stuck_IFfinish;
typedef enum
{
	REVOL_POSI_MODE  = 0,
	REVOL_SPEED_MODE = 1,
	REVOL_REVERSAL_MODE =2,
}eRevolverCtrlMode;




class Dial_mode_set
{
	public:
		void Dial_mode_init();
		void Dial_behaviour_set();
		void real_firc_info_update();
		const RC_ctrl_t *Dial_RC;               //����ʹ�õ�ң����ָ��
		uint8_t	Revolver_Switch;
		uint8_t Revolver_last_Switch;
		int16_t speed_time;
		Key_ShootFlag key_ShootFlag;
		Shoot_IFfinish ifshoot_finsh;
		Stuck_IFfinish ifstuck_finish;
	  eRevolverCtrlMode Revolver_mode;
		eRevolverCtrlMode last_Revolver_mode;
		
		//����ϵͳ����
		uint16_t	heat_max;
		int16_t 	heating;
		fp32 angle;
		int64_t set_angle;
		int64_t last_angle;
		int16_t angle_out;
		int16_t unfinish_time;
		bool_t if_stucked;
		bool_t stuck_mode;

fp32 bullet_last_speed;
fp32 bullet_speed;
		
};
	
extern fp32 x_coordinate,y_coordinate;  
extern fp32 pre_x_coordinate,pre_y_coordinate;
extern fp32 follow_radius,pre_radius; 
#endif
extern void system_task(void const * argument);
//extern void system_t_task_init();
#ifdef __cplusplus
}	
#endif

#endif
