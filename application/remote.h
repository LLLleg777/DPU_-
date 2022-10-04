#ifndef _REMOTE_H
#define _REMOTE_H
#include "system.h"
#define nrf24l01_REC_NUM 100
void Receive_control(void);
typedef struct
{
	u8 s[15];
	float  ch[4];
}
Remote_typedef;
extern Remote_typedef control;

#endif
