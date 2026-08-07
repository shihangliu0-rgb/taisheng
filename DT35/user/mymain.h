#ifndef MYMAIN_H
#define MYMAIN_H

#include "dt35.h"

extern dt35_data_t dt35_data_40;
extern dt35_data_t dt35_data_41;
extern HAL_StatusTypeDef dt35_state_40;
extern HAL_StatusTypeDef dt35_state_41;

void MyMain_Init(void);
void MyMain_Loop(void);

#endif
