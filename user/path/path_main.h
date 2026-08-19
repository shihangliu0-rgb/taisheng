#ifndef PATH_MAIN_H
#define PATH_MAIN_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    PATH_STATE_IDLE = 0x10U,
    PATH_STATE_RUNNING = 0x11U,
    PATH_STATE_FINISHED = 0x12U,
    PATH_STATE_FAULT = 0x13U
} path_state_t;

typedef enum
{
    PATH_ERROR_NONE = 0U,
    PATH_ERROR_IMU = 1U,
    PATH_ERROR_DT35 = 2U,
    PATH_ERROR_CHASSIS = 3U,
    PATH_ERROR_REMOTE = 4U,
    PATH_ERROR_MANUAL = 5U
} path_error_t;

extern volatile path_state_t path_state;
extern volatile path_error_t path_error;
extern volatile uint8_t path_segment_index;
extern volatile bool path_mirrored;

void PathMain_Init(void);
void PathMain_Run(uint8_t remote_buttons, uint8_t remote_online);
void PathMain_Stop(void);
bool PathMain_IsRunning(void);

#endif
