#include "ui_task.h"
#include "main.h"
#include "struct_typedef.h"
#include "usart.h"
#include "judge.h"
#include "cmsis_os.h"
#include "main.h"
#include "string.h"
#include "stdbool.h"
#include "crc8_crc16.h"
#include "i2c.h"
#include "system_task.h"
#include "revolver_task.h"
#include "chassis_task.h"
#include "FreeRTOS.h"
//#include "vision.h"
//#include "Revolver_task.h"

extern uint8_t CliendTxBuffer[200];
UI_Info_t UI;
global_flag_t global_flag;
extern int steering_mode;
extern int IF_SUPER_OK;


void Judge_hitted(void);
void Client_graphic_follow_armour_init(void);
void Client_graphic_follow_armour_update(void);
void supercapacitor_energy_init(void);
void supercapacitor_energy_update(void);
void pitch_angle_init(void);
void pitch_angle_update(void);
void Client_graphic_pitch_Init(void);
void Client_graphic_super_Init(void);
void Client_graphic_pit_energy_init(void);
void Client_graphic_pit_energy_update(void);
void Client_graphic_circle_init(void);
void Client_graphic_circle_update(void);

void Client_graphic_cover_Init(void);
void Client_graphic_fir_Init(void);
void Client_graphic_auto_Init(void);
void Client_graphic_spin_Init(void);
void Client_graphic_mode_Init(void);
void Client_graphic_vision_Init(void);
void Client_graphic_vision_mode_Init(void);
void Client_graphic_vision_mode_Update(void);
void Client_bullet_send(void);

void circle_init(void);
void circle_update(void);



//��ʾ��������
char mode_line[10]= {"MODE"};
char super_line[10]= {"pCAP:"};//��������ʣ����,float
char cover_line[10]= {"COVER:"};
char fir_line[10]= {"FIRC:"};
char auto_line[10]= {"AUTO:"};
char spin_line[10]= {"SPIN:"};
char vision_line[10]= {"VISION:"};
char big_line[10]= {"BIG"};
char small_line[10]= {"SMALL"};
char autoaim_line[10]= {"AUTOAIM"};
char outpost_line[10]= {"OUTPOST"};


ext_charstring_data_t tx_client_pitch_char;
ext_charstring_data_t tx_client_super_char;
ext_charstring_data_t tx_client_cover_char;
ext_charstring_data_t tx_client_fir_char;
ext_charstring_data_t tx_client_auto_char;
ext_charstring_data_t tx_client_spin_char;
ext_charstring_data_t tx_client_mode_char;
ext_charstring_data_t tx_client_vision_char;
ext_charstring_data_t tx_client_vision_mode_char;


uint8_t change_char_flag = 0;

//�����װ�װ�
ext_graphic_seven_data_t tx_follow_armour_figure;

int target_flag;
fp32 angle =0;
uint8_t hitted_id;
uint8_t F_color,R_color,B_color,L_color;

//�������ݵ�����ʾ
float dump_energy;

//����pitch��Ƕ�
fp32 pitch_angle;


extern int steering_mode,vision_mode_ui,heat_mode,vision_mode,v_firc_set;


//ԲȦ��ɫ
uint32_t cover_color,spin_color,fir_color,auto_color;
uint16_t tx_aerial_id;
fp32 remain_bullet;
void UI_Init(void)
{
		
    F_color = R_color = B_color = L_color = GREEN;
    cover_color =spin_color = fir_color = auto_color = GREEN;
		if(is_red_or_blue())
			tx_aerial_id = 6;
		else
			tx_aerial_id = 106;
}
void UI_DataUpdate(void)
{
    global_flag.global_cover    =             steering_mode;
    global_flag.global_spin     =   	syspoint()->sys_mode==SPIN||syspoint()->sys_mode==SPIN_AUTO;
    global_flag.global_auto     =	  syspoint()->sys_mode==AUTO||syspoint()->sys_mode==SPIN_AUTO;
    global_flag.global_firc     = 								 revolver.Friction_wheel_set.firc_mode;
    dump_energy = (chassispoint()->chassis_cap_masure->Cap_voltage-18)*100.0f/3.5;
    angle = gimbal_point()->Yaw_motor.relative_angle*180.0f/PI;

		//�������ֵİ뾶��Ϣ����
		if(follow_radius>200)
{
			follow_radius = 200;
}
		else if(isinf(follow_radius))
		{
				follow_radius=0;
		}
		pre_radius = follow_radius - 10;
    if(pre_radius<0)
			pre_radius = 1;
		
		remain_bullet = JUDGE_usGetRadar_Robo_remain_17mm();
}	


void ui_task(void const *pvParameters)
{  
	
	/* ��̬UI���Ʊ��� */
	uint16_t UI_PushUp_Counter = 0;
  uint16_t Init_step = 0;
//	uint32_t currentTime;
	
	/* ����ϵͳ��ʼ�� */
	vTaskDelay(200);
	UI_Init();
  while(1)
	{	
		vTaskDelay(2);
		UI_PushUp_Counter++;
		if(Init_step  == 0)
		{
		  if(UI_PushUp_Counter <=20) 
		  {
		  	Client_graphic_cover_Init(); 				//cover��ʼ��
			  continue;
	  	}
		  if(UI_PushUp_Counter <=40)					
		  {
				Client_graphic_spin_Init();  				//spin��ʼ��
				continue;
			}
			if(UI_PushUp_Counter <=60)
			{
				Client_graphic_fir_Init();  				//firc��ʼ��  
				continue;
			}
			if(UI_PushUp_Counter <=90)
			{
				Client_graphic_auto_Init();					//auto��ʼ��
				_lowshort_aim_2();   								//����������
				continue;
			}
			if(UI_PushUp_Counter <=120)
			{
				_high_aim_();				 								//ʾ����
				_lowshort_aim_3();   								//��������
				continue;
			}
			if(UI_PushUp_Counter <=150)
			{
				Client_graphic_circle_init();				//Բ����ʼ��
				vision_range();          						//�Ӿ���Χ
				continue;
			}
			if(UI_PushUp_Counter <=200)
			{
				Client_graphic_vision_Init();				//�Ӿ�ģʽ��ʼ��
				continue;
			}
			if(UI_PushUp_Counter <=250)
			{
				Client_graphic_vision_mode_Init();  //modec��ʼ��
				continue;
			}
			if(UI_PushUp_Counter <=300)
			{
				Client_graphic_pit_energy_init();	  //�����������ֳ�ʼ��
				continue;
			}
      if(UI_PushUp_Counter<=350)
			{
				Client_graphic_super_Init();				//��������
				continue;
			}
			if(UI_PushUp_Counter<=400)
			{
				Client_graphic_mode_Init();					//mode
				continue;
			}
			if(UI_PushUp_Counter<=450) 						//�������ֳ�ʼ��
			{
				Client_graphic_follow_armour_init();  
				continue;
			}
			Init_step =1;
		}

		UI_DataUpdate();
		//����̨�ַ��͵�����Ϣ
		Client_bullet_send();
		
		//���������ٶ����ȼ�
		if(UI_PushUp_Counter % 7 == 0) //20ms
		{
			Client_graphic_circle_update();
			continue;
		}
		
		//�еȸ����ٶ����ȼ�
		if(UI_PushUp_Counter % 3 == 0) //20ms
		{
			Client_graphic_pit_energy_update();
			Client_graphic_vision_mode_Update();

			continue;
		}
		
		//�������ٶ����ȼ�-������Ҫ���ж�
		if(UI_PushUp_Counter % 2 == 0) //20ms
		{
			Client_graphic_follow_armour_update();
			continue;
		}

//UI����ˢ��
		if(UI_PushUp_Counter>60000)
		{
				UI_PushUp_Counter=0;
		}
//UI�ֶ�ˢ��
	  if(IF_KEY_PRESSED_B )
	  {
		  UI_PushUp_Counter = 0;
		  Init_step = 0;
	  }
  }

	
}

/**
  * @brief          ����UI����
  * @param[in]      null
  */
/****************************************************start**********************************************************/

void draw_follow_armour_init(fp32 x,fp32 y,fp32 pre_x,fp32 pre_y)
{
	//����
	Figure_Graphic(&tx_follow_armour_figure.clientData[0],"Tar",ADD,CIRCLE,2,GREEN,0,0,4,(x),(y),follow_radius,0,0);
	Figure_Graphic(&tx_follow_armour_figure.clientData[1],"Pre",ADD,CIRCLE,3,YELLOW,0,0,2,(pre_x),(pre_y),pre_radius,0,0);
	//�ƶ�װ�װ�
//	Figure_Graphic(&tx_follow_armour_figure.clientData[2],"F",ADD,CIRCLE,4,GREEN,0,0,3, 960+(int)340*arm_sin_f32((angle)*2*PI/360.0f),540+(int)340*arm_cos_f32((angle)*2*PI/360.0f),50,0,0);
//	Figure_Graphic(&tx_follow_armour_figure.clientData[3],"R",ADD,CIRCLE,4,GREEN,0,0,3, 960+(int)340*arm_sin_f32((angle+90)*2*PI/360.0f),540+(int)340*arm_cos_f32((angle+90)*2*PI/360.0f),50,0,0);
//	Figure_Graphic(&tx_follow_armour_figure.clientData[4],"B",ADD,CIRCLE,4,GREEN,0,0,3, 960+(int)340*arm_sin_f32((angle+180)*2*PI/360.0f),540+(int)340*arm_cos_f32((angle+180)*2*PI/360.0f),50,0,0);
//	Figure_Graphic(&tx_follow_armour_figure.clientData[5],"L",ADD,CIRCLE,4,GREEN,0,0,3, 960+(int)340*arm_sin_f32((angle+270)*2*PI/360.0f),540+(int)340*arm_cos_f32((angle+270)*2*PI/360.0f),50,0,0);
	//��addһ��ͼ��
//	Figure_Graphic(&tx_follow_armour_figure.clientData[6],"FL",ADD,LINE,4,GREEN,0,0,3, 960+(int)200*arm_sin_f32((angle)*2*PI/360.0f),540+(int)200*arm_cos_f32((angle)*2*PI/360.0f),0,960+(int)270*arm_sin_f32((angle)*2*PI/360.0f),540+(int)270*arm_cos_f32((angle)*2*PI/360.0f));
}

void draw_follow_armour_update(fp32 x,fp32 y,fp32 pre_x,fp32 pre_y)
{
	//����
	Figure_Graphic(&tx_follow_armour_figure.clientData[0],"Tar",MODIFY,CIRCLE,2,GREEN,0,0,4,(x),(y),follow_radius,0,0);
	Figure_Graphic(&tx_follow_armour_figure.clientData[1],"Pre",MODIFY,CIRCLE,3,YELLOW,0,0,2,(pre_x),(pre_y),pre_radius,0,0);
	//�ƶ�װ�װ�
//	Figure_Graphic(&tx_follow_armour_figure.clientData[2],"F",MODIFY,CIRCLE,4,GREEN,0,0,3, 960+(int)340*arm_sin_f32((angle)*2*PI/360.0f),540+(int)340*arm_cos_f32((angle)*2*PI/360.0f),50,0,0);
//	Figure_Graphic(&tx_follow_armour_figure.clientData[3],"R",MODIFY,CIRCLE,4,GREEN,0,0,3, 960+(int)340*arm_sin_f32((angle+90)*2*PI/360.0f),540+(int)340*arm_cos_f32((angle+90)*2*PI/360.0f),50,0,0);
//	Figure_Graphic(&tx_follow_armour_figure.clientData[4],"B",MODIFY,CIRCLE,4,GREEN,0,0,3, 960+(int)340*arm_sin_f32((angle+180)*2*PI/360.0f),540+(int)340*arm_cos_f32((angle+180)*2*PI/360.0f),50,0,0);
// 	Figure_Graphic(&tx_follow_armour_figure.clientData[5],"L",MODIFY,CIRCLE,4,GREEN,0,0,3, 960+(int)340*arm_sin_f32((angle+270)*2*PI/360.0f),540+(int)340*arm_cos_f32((angle+270)*2*PI/360.0f),50,0,0);
	//��addһ��ͼ��
//	Figure_Graphic(&tx_follow_armour_figure.clientData[6],"FL",MODIFY,LINE,4,GREEN,0,0,3, 960+(int)200*arm_sin_f32((angle)*2*PI/360.0f),540+(int)200*arm_cos_f32((angle)*2*PI/360.0f),0,960+(int)270*arm_sin_f32((angle)*2*PI/360.0f),540+(int)270*arm_cos_f32((angle)*2*PI/360.0f));
}

void Client_graphic_follow_armour_init()//��ʼ��
{
		//֡ͷ
		tx_follow_armour_figure.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_follow_armour_figure.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
		tx_follow_armour_figure.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_follow_armour_figure.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��

		//������
		tx_follow_armour_figure.CmdID = ID_robot_interactive_header_data;

		//���ݶ�ͷ�ṹ
		tx_follow_armour_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
		tx_follow_armour_figure.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_follow_armour_figure.dataFrameHeader.receiver_ID = REF.self_client;
	
		//���ݶ�
		draw_follow_armour_init(x_coordinate,y_coordinate,pre_x_coordinate,pre_y_coordinate);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_follow_armour_figure.CmdID, LEN_CMD_ID+tx_follow_armour_figure.txFrameHeader.DataLength);//���������볤��2

		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_follow_armour_figure));
		
    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_follow_armour_figure),200);

}

void Client_graphic_follow_armour_update()//��������λ�÷���
{
		//֡ͷ
		tx_follow_armour_figure.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_follow_armour_figure.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
		tx_follow_armour_figure.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_follow_armour_figure.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��

		//������
		tx_follow_armour_figure.CmdID = ID_robot_interactive_header_data;

		//���ݶ�ͷ�ṹ
		tx_follow_armour_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
		tx_follow_armour_figure.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_follow_armour_figure.dataFrameHeader.receiver_ID = REF.self_client;
	
		//���ݶ�
		draw_follow_armour_update(x_coordinate,y_coordinate,pre_x_coordinate,pre_y_coordinate);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_follow_armour_figure.CmdID, LEN_CMD_ID+tx_follow_armour_figure.txFrameHeader.DataLength);//���������볤��2

		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_follow_armour_figure));
		
    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_follow_armour_figure),200);

}

/*********************************************end**********************************************/



/**
  * @brief          ������������/����
  * @param[in]      null
  */
/*********************************************start**********************************************/

void Client_graphic_super_Init()							//pCAP   ��������ʣ��
{

		//֡ͷ
		tx_client_super_char.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_client_super_char.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_string_t);
		tx_client_super_char.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_client_super_char.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
		//������
		tx_client_super_char.CmdID = ID_robot_interactive_header_data;
		
		//���ݶ�ͷ�ṹ
		tx_client_super_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
		tx_client_super_char.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_client_super_char.dataFrameHeader.receiver_ID = REF.self_client;
		
		//���ݶ�
		Char_Graphic(&tx_client_super_char.clientData,"Cap",ADD,0,ORANGE,20,strlen(super_line),2,1620,(590),super_line);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_super_char.CmdID, LEN_CMD_ID+tx_client_super_char.txFrameHeader.DataLength);//���������볤��2
		
		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_super_char));
		
    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_client_super_char),200);
    
}


//��������ʾ ��������
ext_float_two_data_t tx_pit_super_figure;
ext_float_two_data_t tx_firc_speed_figure;

void draw_pit_energy_init()
{
	Float_Graphic(&tx_pit_super_figure.clientData[0],"Mode",ADD,FLOAT,4,CYAN_BLUE,30,1,3,1620,(660),1);
	//Float_Graphic(&tx_pit_super_figure.clientData[1],"Sc",ADD,FLOAT,4,CYAN_BLUE,30,1,3,1620,540,(float)(v_firc_set));
	Float_Graphic(&tx_pit_super_figure.clientData[1],"Sc",ADD,FLOAT,4,CYAN_BLUE,30,1,3,1620,540,(float)(dump_energy));
	//Float_Graphic(&tx_firc_speed_figure.clientData[0],"V",ADD,FLOAT,4,CYAN_BLUE,30,1,3,1620,740,(float)(v_firc_set));
}

void draw_pitch_energy_update()
{
//mode �ж�
	if(vision_mode==1)
	{
		Float_Graphic(&tx_pit_super_figure.clientData[0],"Mode",MODIFY,FLOAT,4,CYAN_BLUE,30,1,3,1620,(660),1);
	}
	else if(vision_mode==3)
	{
		Float_Graphic(&tx_pit_super_figure.clientData[0],"Mode",MODIFY,FLOAT,4,CYAN_BLUE,30,1,3,1620,(660),3);
	}

if(IF_SUPER_OK)
{
		Float_Graphic(&tx_pit_super_figure.clientData[1],"Sc",MODIFY,FLOAT,4,CYAN_BLUE,30,1,3,1620,540,(float)(int)(dump_energy));
//Float_Graphic(&tx_firc_speed_figure.clientData[0],"V",MODIFY,FLOAT,4,CYAN_BLUE,30,1,3,1620,740,(float)(v_firc_set));
}
else
{
	Float_Graphic(&tx_pit_super_figure.clientData[1],"Sc",MODIFY,FLOAT,4,FUCHSIA,30,1,3,1620,540,(float)(int)(dump_energy));
}
}



void Client_graphic_pit_energy_init()//���ͳ�ʼ������
{
		//֡ͷ
		tx_pit_super_figure.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_pit_super_figure.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*2;
		tx_pit_super_figure.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_pit_super_figure.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��

		//������
		tx_pit_super_figure.CmdID = ID_robot_interactive_header_data;

		//���ݶ�ͷ�ṹ
		tx_pit_super_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
		tx_pit_super_figure.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_pit_super_figure.dataFrameHeader.receiver_ID = REF.self_client;
	
		//���ݶ�
		draw_pit_energy_init();
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_pit_super_figure.CmdID, LEN_CMD_ID+tx_pit_super_figure.txFrameHeader.DataLength);//���������볤��2

		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_pit_super_figure));
		
	HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_pit_super_figure),200);
}



void Client_graphic_pit_energy_update()//���͸��º���
{
		//֡ͷ
		tx_pit_super_figure.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_pit_super_figure.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*2;
		tx_pit_super_figure.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_pit_super_figure.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��

		//������
		tx_pit_super_figure.CmdID = ID_robot_interactive_header_data;

		//���ݶ�ͷ�ṹ
		tx_pit_super_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
		tx_pit_super_figure.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_pit_super_figure.dataFrameHeader.receiver_ID = REF.self_client;
	
		//���ݶ�
		draw_pitch_energy_update();
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_pit_super_figure.CmdID, LEN_CMD_ID+tx_pit_super_figure.txFrameHeader.DataLength);//���������볤��2

		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_pit_super_figure));
		
	HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_pit_super_figure),200);
}

/**************************************************end*******************************************************/




/**
  * @brief          UI����
  * @param[in]      null
  */

/***********************************************�������****************************************************/
void Client_graphic_cover_Init()
{

		//֡ͷ
		tx_client_cover_char.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_client_cover_char.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_string_t);
		tx_client_cover_char.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_client_cover_char.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
		//������
		tx_client_cover_char.CmdID = ID_robot_interactive_header_data;
		
		//���ݶ�ͷ�ṹ
		tx_client_cover_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
		tx_client_cover_char.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_client_cover_char.dataFrameHeader.receiver_ID = REF.self_client;
		
		//���ݶ�
		Char_Graphic(&tx_client_cover_char.clientData,"Cov",ADD,0,ORANGE,20,strlen(cover_line),2,70,540,cover_line);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_cover_char.CmdID, LEN_CMD_ID+tx_client_cover_char.txFrameHeader.DataLength);//���������볤��2
		
		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_cover_char));
		
    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_client_cover_char),200);
    
}

void Client_graphic_fir_Init()
{

		//֡ͷ
		tx_client_fir_char.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_client_fir_char.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_string_t);
		tx_client_fir_char.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_client_fir_char.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
		//������
		tx_client_fir_char.CmdID = ID_robot_interactive_header_data;
		
		//���ݶ�ͷ�ṹ
		tx_client_fir_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
		tx_client_fir_char.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_client_fir_char.dataFrameHeader.receiver_ID = REF.self_client;
		
		//���ݶ�
		Char_Graphic(&tx_client_fir_char.clientData,"Firc",ADD,0,ORANGE,20,strlen(fir_line),2,70,640,fir_line);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_fir_char.CmdID, LEN_CMD_ID+tx_client_fir_char.txFrameHeader.DataLength);//���������볤��2
		
		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_fir_char));
		
    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_client_fir_char),200);
    
}

void Client_graphic_auto_Init()
{

		//֡ͷ
		tx_client_auto_char.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_client_auto_char.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_string_t);
		tx_client_auto_char.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_client_auto_char.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
		//������
		tx_client_auto_char.CmdID = ID_robot_interactive_header_data;
		
		//���ݶ�ͷ�ṹ
		tx_client_auto_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
		tx_client_auto_char.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_client_auto_char.dataFrameHeader.receiver_ID = REF.self_client;
		
		//���ݶ�
		Char_Graphic(&tx_client_auto_char.clientData,"Auto",ADD,0,ORANGE,20,strlen(auto_line),2,70,840,auto_line);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_auto_char.CmdID, LEN_CMD_ID+tx_client_auto_char.txFrameHeader.DataLength);//���������볤��2
		
		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_auto_char));
		
    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_client_auto_char),200);
    
}

void Client_graphic_spin_Init()
{

		//֡ͷ
		tx_client_spin_char.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_client_spin_char.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_string_t);
		tx_client_spin_char.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_client_spin_char.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
		//������
		tx_client_spin_char.CmdID = ID_robot_interactive_header_data;
		
		//���ݶ�ͷ�ṹ
		tx_client_spin_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
		tx_client_spin_char.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_client_spin_char.dataFrameHeader.receiver_ID = REF.self_client;
		
		//���ݶ�
		Char_Graphic(&tx_client_spin_char.clientData,"Spin",ADD,0,ORANGE,20,strlen(spin_line),2,70,740,spin_line);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_spin_char.CmdID, LEN_CMD_ID+tx_client_spin_char.txFrameHeader.DataLength);//���������볤��2
		
		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_spin_char));
		
    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_client_auto_char),200);
    
}



/***********************************************�Ҳ�����****************************************************/

void Client_graphic_vision_Init()
{

		//֡ͷ
		tx_client_vision_char.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_client_vision_char.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_string_t);
		tx_client_vision_char.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_client_vision_char.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
		//������
		tx_client_vision_char.CmdID = ID_robot_interactive_header_data;
		
		//���ݶ�ͷ�ṹ
		tx_client_vision_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
		tx_client_vision_char.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_client_vision_char.dataFrameHeader.receiver_ID = REF.self_client;
		
		//���ݶ�
		Char_Graphic(&tx_client_vision_char.clientData,"Vis",ADD,0,ORANGE,20,strlen(vision_line),2,1620,840,vision_line);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_vision_char.CmdID, LEN_CMD_ID+tx_client_vision_char.txFrameHeader.DataLength);//���������볤��2
		
		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_vision_char));
		
    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_client_vision_char),200);
    
}


void Client_graphic_vision_mode_Init()
{

		//֡ͷ
		tx_client_vision_mode_char.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_client_vision_mode_char.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_string_t);
		tx_client_vision_mode_char.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_client_vision_mode_char.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
		//������
		tx_client_vision_mode_char.CmdID = ID_robot_interactive_header_data;
		
		//���ݶ�ͷ�ṹ
		tx_client_vision_mode_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
		tx_client_vision_mode_char.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_client_vision_mode_char.dataFrameHeader.receiver_ID = REF.self_client;
		
		//���ݶ�
		Char_Graphic(&tx_client_vision_mode_char.clientData,"V_m",ADD,0,CYAN_BLUE,20,strlen(autoaim_line),2,1620,780,autoaim_line);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_vision_mode_char.CmdID, LEN_CMD_ID+tx_client_vision_mode_char.txFrameHeader.DataLength);//���������볤��2
		
		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_vision_mode_char));
		
    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_client_vision_mode_char),200);
    
}

void Client_graphic_vision_mode_Update()
{

		//֡ͷ
		tx_client_vision_mode_char.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_client_vision_mode_char.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_string_t);
		tx_client_vision_mode_char.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_client_vision_mode_char.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
		//������
		tx_client_vision_mode_char.CmdID = ID_robot_interactive_header_data;
		
		//���ݶ�ͷ�ṹ
		tx_client_vision_mode_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
		tx_client_vision_mode_char.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_client_vision_mode_char.dataFrameHeader.receiver_ID = REF.self_client;
		
		//���ݶ�
		if(vision_mode_ui==1)
		{
				Char_Graphic(&tx_client_vision_mode_char.clientData,"V_m",MODIFY,0,CYAN_BLUE,20,strlen(autoaim_line),2,1620,780,autoaim_line);
		}
		else if(vision_mode_ui==3)
		{
				Char_Graphic(&tx_client_vision_mode_char.clientData,"V_m",MODIFY,0,CYAN_BLUE,20,strlen(big_line),2,1620,780,big_line);
		}
		else if(vision_mode_ui==4)
		{
				Char_Graphic(&tx_client_vision_mode_char.clientData,"V_m",MODIFY,0,CYAN_BLUE,20,strlen(small_line),2,1620,780,small_line);
		}
		else if(vision_mode_ui==5)
		{
				Char_Graphic(&tx_client_vision_mode_char.clientData,"V_m",MODIFY,0,CYAN_BLUE,20,strlen(outpost_line),2,1620,780,outpost_line);
		}
		else if(vision_mode_ui==0)
		{
				Char_Graphic(&tx_client_vision_mode_char.clientData,"V_m",MODIFY,0,FUCHSIA,20,strlen(autoaim_line),2,1620,780,autoaim_line);
		}
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_vision_mode_char.CmdID, LEN_CMD_ID+tx_client_vision_mode_char.txFrameHeader.DataLength);//���������볤��2
		
		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_vision_mode_char));
		
    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_client_vision_mode_char),200);
    
}

void Client_graphic_mode_Init()
{

		//֡ͷ
		tx_client_mode_char.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_client_mode_char.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_string_t);
		tx_client_mode_char.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_client_mode_char.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��
	
		//������
		tx_client_mode_char.CmdID = ID_robot_interactive_header_data;
		
		//���ݶ�ͷ�ṹ
		tx_client_mode_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
		tx_client_mode_char.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_client_mode_char.dataFrameHeader.receiver_ID = REF.self_client;
		
		//���ݶ�
		Char_Graphic(&tx_client_mode_char.clientData,"Md",ADD,0,ORANGE,20,strlen(mode_line),2,1620,720,mode_line);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_mode_char.CmdID, LEN_CMD_ID+tx_client_mode_char.txFrameHeader.DataLength);//���������볤��2
		
		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_mode_char));
		
    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_client_mode_char),200);
    
}
/**********************************************end*****************************************************/






/**
  * @brief         ԲȦ״̬��ʾ
  * @param[in]      null
  */

/**********************************************start**************************************************/
//����ԲȦ
ext_graphic_seven_data_t tx_circle_figure;

void circle_init()
{
	//���ո�
	Figure_Graphic(&tx_circle_figure.clientData[0],"Co",ADD,CIRCLE,2,GREEN,0,0,4,X_C_O,540,20,0,0);
  //С����
	Figure_Graphic(&tx_circle_figure.clientData[1],"Sp",ADD,CIRCLE,2,GREEN,0,0,4,X_C_O,740,20,0,0);
	//Ħ����
	Figure_Graphic(&tx_circle_figure.clientData[2],"Fi",ADD,CIRCLE,2,GREEN,0,0,4,X_C_O,640,20,0,0);
  //����
  Figure_Graphic(&tx_circle_figure.clientData[3],"Au",ADD,CIRCLE,2,GREEN,0,0,4,X_C_O,840,20,0,0);
  //������ǰ��
  Figure_Graphic(&tx_circle_figure.clientData[4],"FL",ADD,LINE,2,GREEN,0,0,4, 960+(int)200*arm_sin_f32((angle)*2*PI/360.0f),540+(int)200*arm_cos_f32((angle)*2*PI/360.0f),0,960+(int)270*arm_sin_f32((angle)*2*PI/360.0f),540+(int)270*arm_cos_f32((angle)*2*PI/360.0f));

  //�������СȦȦ �ɴ������ҵ���������
//	Figure_Graphic(&tx_circle_figure.clientData[5],"Tar",ADD,CIRCLE,2,PINK,0,0,2,x_coordinate,y_coordinate,30,0,0);
//	Figure_Graphic(&tx_circle_figure.clientData[6],"Pre",ADD,CIRCLE,3,ORANGE,0,0,1,pre_x_coordinate,pre_y_coordinate,20,0,0);
}

void circle_update()
{

if(global_flag.global_cover==0)
{
		Figure_Graphic(&tx_circle_figure.clientData[0],"Co",MODIFY,CIRCLE,2,GREEN,0,0,4,X_C_O,540,20,0,0);
}
	else
{
		Figure_Graphic(&tx_circle_figure.clientData[0],"Co",MODIFY,CIRCLE,2,CYAN_BLUE,0,0,4,X_C_O,540,20,0,0);
}
if(global_flag.global_firc==0)
	{
		Figure_Graphic(&tx_circle_figure.clientData[2],"Fi",MODIFY,CIRCLE,2,GREEN,0,0,4,X_C_O,640,20,0,0);
	}
  else
	{
		Figure_Graphic(&tx_circle_figure.clientData[2],"Fi",MODIFY,CIRCLE,2,CYAN_BLUE,0,0,4,X_C_O,640,20,0,0);
	}

if(global_flag.global_spin==1)
	{
		Figure_Graphic(&tx_circle_figure.clientData[1],"Sp",MODIFY,CIRCLE,2,CYAN_BLUE,0,0,4,X_C_O,740,20,0,0);
	}
  else
	{
		Figure_Graphic(&tx_circle_figure.clientData[1],"Sp",MODIFY,CIRCLE,2,GREEN,0,0,4,X_C_O,740,20,0,0);
	}

if(global_flag.global_auto)
	{
		Figure_Graphic(&tx_circle_figure.clientData[3],"Au",MODIFY,CIRCLE,2,CYAN_BLUE,0,0,4,X_C_O,840,20,0,0);
	}
  else
	{
		Figure_Graphic(&tx_circle_figure.clientData[3],"Au",MODIFY,CIRCLE,2,GREEN,0,0,4,X_C_O,840,20,0,0);
	}

//������ǰ��
Figure_Graphic(&tx_circle_figure.clientData[4],"FL",MODIFY,LINE,2,GREEN,0,0,4, 960+(int)200*arm_sin_f32((angle)*2*PI/360.0f),540+(int)200*arm_cos_f32((angle)*2*PI/360.0f),0,960+(int)270*arm_sin_f32((angle)*2*PI/360.0f),540+(int)270*arm_cos_f32((angle)*2*PI/360.0f));
 //�������СȦȦ �ɴ������ҵ���������
//Figure_Graphic(&tx_circle_figure.clientData[5],"Tar",MODIFY,CIRCLE,2,PINK,0,0,2,x_coordinate,y_coordinate,30,0,0);
//Figure_Graphic(&tx_circle_figure.clientData[6],"Pre",MODIFY,CIRCLE,3,ORANGE,0,0,1,pre_x_coordinate,pre_y_coordinate,20,0,0);
}


void Client_graphic_circle_init()//ԲȦ״̬��ʼ��
{
		//֡ͷ
		tx_circle_figure.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_circle_figure.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
		tx_circle_figure.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_circle_figure.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��

		//������
		tx_circle_figure.CmdID = ID_robot_interactive_header_data;

		//���ݶ�ͷ�ṹ
		tx_circle_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
		tx_circle_figure.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_circle_figure.dataFrameHeader.receiver_ID = REF.self_client;
	
		//���ݶ�
		circle_init();
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_circle_figure.CmdID, LEN_CMD_ID+tx_circle_figure.txFrameHeader.DataLength);//���������볤��2

		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_circle_figure));
		
    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_circle_figure),200);

}

void Client_graphic_circle_update()//ԲȦ״̬����
{
		//֡ͷ
		tx_circle_figure.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_circle_figure.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
		tx_circle_figure.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_circle_figure.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��

		//������
		tx_circle_figure.CmdID = ID_robot_interactive_header_data;

		//���ݶ�ͷ�ṹ
		tx_circle_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
		tx_circle_figure.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_circle_figure.dataFrameHeader.receiver_ID = REF.self_client;
	
		//���ݶ�
		circle_update();
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_circle_figure.CmdID, LEN_CMD_ID+tx_circle_figure.txFrameHeader.DataLength);//���������볤��2

		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_circle_figure));
		
    HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_circle_figure),200);

}

/*****************************************************end************************************************/





/**
  * @brief          ����ͨ�ţ���������������̨��
  * @param[in]      null
  */

/***********************************************start****************************************************/

ext_CommunatianData_t tx_remain_bullrt_to_aerial;

void Client_bullet_send()//һ��ͼ�����
{
		//֡ͷ
		tx_remain_bullrt_to_aerial.txFrameHeader.SOF = JUDGE_FRAME_HEADER;
		tx_remain_bullrt_to_aerial.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(robot_interactive_data_t);
		tx_remain_bullrt_to_aerial.txFrameHeader.Seq = 0;//�����
		memcpy(CliendTxBuffer,&tx_remain_bullrt_to_aerial.txFrameHeader,sizeof(xFrameHeader));
		append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//ͷУ��

		//������
		tx_remain_bullrt_to_aerial.CmdID = ID_robot_interactive_header_data;

		//���ݶ�ͷ�ṹ
		tx_remain_bullrt_to_aerial.dataFrameHeader.data_cmd_id = 0x0201;
		tx_remain_bullrt_to_aerial.dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
		tx_remain_bullrt_to_aerial.dataFrameHeader.receiver_ID = tx_aerial_id;
	
		//���ݶ�
		tx_remain_bullrt_to_aerial.interactData.bullet = remain_bullet;
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_remain_bullrt_to_aerial.CmdID, LEN_CMD_ID+tx_remain_bullrt_to_aerial.txFrameHeader.DataLength);//���������볤��2

		//֡β
		append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_remain_bullrt_to_aerial));
		
	HAL_UART_Transmit(&huart1,CliendTxBuffer,sizeof(tx_remain_bullrt_to_aerial),200);
}

/*****************************************************end************************************************/

