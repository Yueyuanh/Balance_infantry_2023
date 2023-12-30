#ifndef _SYS_CONFIG_H_
#define _SYS_CONFIG_H_



/*
1.У׼��ʱ����Ҫ�������ݶ����밴һ����ѹ��У׼
2.����pid��ʱ���Ȱ�д��ɿأ��ٰ���ѹ��У׼
*/
#define abs(x) ((x)>0? (x):(-(x)))


/******************************PID default parameter*****************/
////////////////////////////		kp		ki		kd		
#define CHASSIS_Rot_PID_OFF 	{9.0f,	0.0f,	0.0f}       //Ŀǰ�����ã������޷�ͨ����λ�����ã�
#define CHASSIS_Vec_PID_OFF 	{40000.0f,	0.0f,	0.0f}   //���̵�������ٶ�pid
#define GIMBALP_Pos_PID_OFF 	{16.0f,	0.0f,	0.0f}         
#define GIMBALP_Vec_PID_OFF 	{8200.0f,	0.0f,	0.0f}

#define GIMBALY_Pos_PID_OFF 	{-14.0f,	0.0f,	0.0f}
#define GIMBALY_Vec_PID_OFF 	{-8200.0f,	0.0f,	0.0f}
#define SLIBLIN_Pos_PID_OFF 	{0.0f,	0.0f,	0.0f}
#define SLIBLIN_Vec_PID_OFF 	{1000.0f,	0.0f,	0.0f}  //���� �ٶȻ�


#define M3508_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define RADIAN_PER_RAD (57.29578f)


#define RC_KEY_LONG_TIME 500       //�����л�ģʽ����ʱ��




//#define SUPER_CAP_TEST




#endif


