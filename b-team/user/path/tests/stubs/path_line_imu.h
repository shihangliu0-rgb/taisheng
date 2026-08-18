#ifndef TEST_PATH_LINE_IMU_H
#define TEST_PATH_LINE_IMU_H

#include <stdbool.h>

#define PATH_LINE_IMU_WHEEL_DIAMETER_M 0.152f

typedef struct
{
    float encoder_body_velocity_x_mps;
    float encoder_body_velocity_y_mps;
    float fused_velocity_x_mps;
    float fused_velocity_y_mps;
    float fused_position_x_m;
    float fused_position_y_m;
    bool imu_solution_valid;
    bool encoder_solution_valid;
} path_line_imu_data_t;

bool PathLineImu_GetData(path_line_imu_data_t *data);

#endif
