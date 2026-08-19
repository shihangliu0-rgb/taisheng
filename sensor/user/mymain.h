#ifndef MYMAIN_H
#define MYMAIN_H

#include "dt35.h"
#include "pnp.h"

extern dt35_data_t dt35_data_F;
extern dt35_data_t dt35_data_L;
extern HAL_StatusTypeDef dt35_state_F;
extern HAL_StatusTypeDef dt35_state_L;

void MyMain_Init(void);
void MyMain_Loop(void);

#endif
