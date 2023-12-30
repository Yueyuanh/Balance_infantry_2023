#ifndef BSP_RC_H
#define BSP_RC_H

#ifdef __cplusplus
extern "C"{
#endif
	
#include "struct_typedef.h"
#include "remote_control.h"
extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void Transfer_Image_USART6_Init(void);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
	
#ifdef __cplusplus
}
#endif
	
#endif

