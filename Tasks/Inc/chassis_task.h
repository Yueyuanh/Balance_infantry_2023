#ifndef CHASSISTASK_H
#define CHASSISTASK_H

#include "system_task.h"	
#include "pid.h"
#include "motor.h"
#include "user_lib.h"
#include "gimbal_task.h"
#include "cmsis_os.h"
#include "judge.h"
#include "Bsp_can.h"
#include "judge_task.h"

#define CHASSIS_TASK_INIT_TIME 357 //����ʼ����һ��ʱ��

#define CHASSIS_CONTROL_TIME_MS 2  //����������Ƽ�� 2ms

#define M3508_MOTOR_SPEED_PID_MAX_OUT    16000.0f
#define M3508_MOTOR_SPEED_PID_MAX_IOUT   2000.0f
#define CHASSIS_MOTOR_SPEED_PID_KP 8000.0f
#define CHASSIS_MOTOR_SPEED_PID_KI 0.0f	
#define CHASSIS_MOTOR_SPEED_PID_KD 2.5f
		
		
#define MOTOR_YAW_PID_MAX_OUT    40.0f
#define MOTOR_YAW_PID_MAX_IOUT   0.2f

#define CHASSIS_MOTOR_YAW_PID_KP -20.0f //-16.8
#define CHASSIS_MOTOR_YAW_PID_KI   0.2f
#define CHASSIS_MOTOR_YAW_PID_KD -700.0f //-18.0

		
		
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.3f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.3f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.3f
	


#define CHASSIS_WZ_RC_SEN      0.007f      //��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_CHANNEL     2           //������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_SET_SCALE -0.072f//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ 0Ϊ�ڼ������ģ�����Ҫ����
#define MOTOR_DISTANCE_TO_CENTER 0.22f
#define MAX_WHEEL_SPEED 6.0f	//���̵������ٶ�



#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f //m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
typedef struct
{
  const motor_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  fp32 power;
	fp32 current_set;
	fp32 current;
	fp32 current_scale;
	int16_t give_current;
}chassis_motor_t;


typedef struct
{
  fp32 BusV;
  fp32 Current;
  fp32 Power;
} ina226_t;

class chassis_t
{
	private:
	system_mode_t chassis_mode;
	gimbal_motor_t chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
  gimbal_motor_t chassis_pitch_motor; //����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����	
	
	
	PID_t chassis_motor_speed_pid[4];
  PID_t chassis_angle_pid;              //���̸���Ƕ�pid
	PID_t chassis_angle_spin_pid;
	PID_t chassis_current_pid[4]; 
	
	first_order_filter_type_t chassis_cmd_slow_set_vx;
  first_order_filter_type_t chassis_cmd_slow_set_vy;

	fp32 vx;                         //�����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                         //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	
  fp32 vx_set;                     //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
	fp32 wz_set;                     //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	
	
	fp32 vx_follow_set;                     //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_follow_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
	
	fp32 vx_max_speed;               //ǰ����������ٶ� ��λm/s 
  fp32 vx_min_speed;               //ǰ��������С�ٶ� ��λm/s
  fp32 vy_max_speed;               //���ҷ�������ٶ� ��λm/s
  fp32 vy_min_speed;               //���ҷ�����С�ٶ� ��λm/s
	 
  fp32 chassis_relative_angle_set;

	
	fp32 angle_set;
	fp32 wheel_speed[4]; 
	public:
	fp32 chassis_real_power;
  fp32 chassis_max_power_limit;
	fp32 chassis_power;
	fp32 cap_absorb;
	int shift_flag;
	fp32 chassis_power_buffer;
	fp32 chassis_max_power;
  ina226_t ina226_data;	
	const Chassis_Power_t *chassis_cap_masure;
	chassis_motor_t chassis_motor[4];
	chassis_t();
	void Get_info();
	void Control();
	
	int SPIN_flag;
	void  chassis_zero_control_set(void);
	void  chassis_follow_gimbal_control_set(void);
	void  chassis_no_follow_gimbal_control_set(void);
	void  chassis_spin_control_set(void);
	
	void  chassis_control_loop(void);
	
	void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);
  void chassis_power_limit();
	fp32 scale_k;
	fp32 real_power_chassis;
	fp32 cap_power_fdb;
};
void C_RESET(void);
chassis_t* chassispoint(void);	
#endif
extern void chassis_task(void const * pvParameters);
#ifdef __cplusplus
}	
#endif
#endif

