#ifndef __LADRC_H
#define __LADRC_H

#include "struct_typedef.h"

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
class LADRC_t
{
	public:
		fp32 time_cons; //ʱ�䳣��
	
	  fp32 own_wo;//�۲�������
		fp32 own_b0;//�������
		fp32 z1;
		fp32 z2;
	
	  fp32 own_wc;//����������
	
    fp32 own_max_out;  //������
	  //����
    fp32 own_set;//�趨ֵ
    fp32 own_fdb;//����ֵ
		fp32 own_gyro;//���ٶ�
		fp32 own_err;
	
		fp32 u;
	
		void init(fp32 wc,fp32 b0,fp32 wo,fp32 max_out);
	  fp32 calc(fp32 ref, fp32 set, fp32 gyro);
};
	
	
#endif

#ifdef __cplusplus
}
#endif

#endif

