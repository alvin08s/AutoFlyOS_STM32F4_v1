#ifndef _RC_RC_H_
#define _RC_RC_H_
#include "stm32f4xx.h"

typedef struct int16_rcget{	
				u8 dir;
				u8 flag;
				uint8_t STATUS_OK;
				int16_t THROTTLE;
}RC_GETDATA;
extern RC_GETDATA Rc_Get;//���յ���RC����,1000~2000



#endif


