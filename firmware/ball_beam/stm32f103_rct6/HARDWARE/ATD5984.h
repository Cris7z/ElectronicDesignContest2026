#ifndef __ATD5984_H
#define __ATD5984_H

#include "sys.h"

#define EN1		PBout(8)
#define DIR1	PBout(9)
#define EN2		PAout(1)
#define DIR2	PAout(2)


void ATD5984_Init(void);
void STEP12_PWM_Init(u16 arr, u16 psc);



#endif
