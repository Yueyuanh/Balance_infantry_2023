//From ��������
#include "filter.h"
//#include "mymath.h"


// #define WIDTH_NUM 101
// #define FIL_ITEM  10

void anotc_filter_1(float base_hz,float gain_hz,float dT,float in,_filter_1_st *f1)
{
	LPF_1_(gain_hz,dT,(in - f1->out),f1->a); //��ͨ��ı仯��

	f1->b = my_pow(in - f1->out);

	f1->e_nr = LIMIT(safe_div(my_pow(f1->a),((f1->b) + my_pow(f1->a)),0),0,1); //�仯������Ч��
	
	LPF_1_(base_hz *f1->e_nr,dT,in,f1->out); //��ͨ����
}


 void Moving_Average(float moavarray[],uint16_t len ,uint16_t *fil_cnt,float in,float *out)
{
	uint16_t width_num;
	float last;

	width_num = len ;
	
	if( ++*fil_cnt >= width_num )	
	{
		*fil_cnt = 0; //now
	}
	
	last = moavarray[ *fil_cnt ];
	
	moavarray[ *fil_cnt ] = in;
	
	*out += ( in - ( last  ) )/(float)( width_num ) ;
	*out += 0.00001f *(in - *out);  //��Ҫ����
	
}




int32_t Moving_Median(int32_t moavarray[],uint16_t len ,uint16_t *fil_p,int32_t in)
{
	uint16_t width_num;
	uint16_t now_p;
	float t;
	int8_t pn=0;
	uint16_t start_p,i;
	int32_t sum = 0;

	width_num = len ;
	
	if( ++*fil_p >= width_num )	
	{
		*fil_p = 0; //now
	}
	
	now_p = *fil_p ;	
	
	moavarray[ *fil_p ] = in;
	
	if(now_p<width_num-1) //��֤�Ƚϲ�Խ��
	{
		while(moavarray[now_p] > moavarray[now_p + 1])
		{
			t = moavarray[now_p];
			moavarray[now_p] = moavarray[now_p + 1];
			moavarray[now_p + 1] = t;
			pn = 1;
			now_p ++;
			if(now_p == (width_num-1))
			{
				break;
			}
		}
	}
	
	if(now_p>0)  //��֤�Ƚϲ�Խ��
	{
		while(moavarray[now_p] < moavarray[now_p - 1])
		{
			t = moavarray[now_p];
			moavarray[now_p] = moavarray[now_p - 1];
			moavarray[now_p - 1] = t;
			pn = -1;
			now_p--;
			if(now_p == 0)
			{
				break;
			}
		}
	
	}
	
	if(*fil_p == 0 && pn == 1)
	{
		*fil_p = width_num - 1;
	}
	else if(*fil_p == width_num - 1 && pn == -1)
	{
		*fil_p = 0;
	}
	else
	{
		*fil_p -= pn;
	}
	
	start_p = (uint16_t)(0.25f * width_num );
	for(i = 0; i < width_num/2;i++)
	{
		sum += moavarray[start_p + i];
	}
	return (sum/(width_num/2));
}



float debug_speed;//�����Ҹ�,һ�㶼��1����,debug��
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2;//�����ٶ�

		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 300/*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0;//ʱ���������Ϊ�ٶȲ���
	}
	debug_speed = S->processed_speed;
	return S->processed_speed;//��������ٶ�
}






