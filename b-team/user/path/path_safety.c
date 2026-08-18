#include "path_safety.h"

#include <math.h>
#include <stddef.h>

float PathSafety_RequiredDistance(float speed_mps,
                                  float base_distance_m,
                                  float reaction_time_s,
                                  float brake_deceleration_mps2)
{
    if (speed_mps < 0.0f)
    {
        speed_mps = -speed_mps;
    }
    if (base_distance_m < 0.0f)
    {
        base_distance_m = 0.0f;
    }
    if (reaction_time_s < 0.0f)
    {
        reaction_time_s = 0.0f;
    }
    if (brake_deceleration_mps2 <= 0.0f)
    {
        return base_distance_m;
    }

    return base_distance_m + speed_mps * reaction_time_s +
           (speed_mps * speed_mps) / (2.0f * brake_deceleration_mps2);
}

float PathSafety_MaxAllowedSpeed(float measured_distance_m,
                                 float base_distance_m,
                                 float reaction_time_s,
                                 float brake_deceleration_mps2)
{
    float usable_distance_m;
    float reaction_term;

    if (base_distance_m < 0.0f)
    {
        base_distance_m = 0.0f;
    }
    if (reaction_time_s < 0.0f)
    {
        reaction_time_s = 0.0f;
    }
    usable_distance_m = measured_distance_m - base_distance_m;
    if ((usable_distance_m <= 0.0f) ||
        (brake_deceleration_mps2 <= 0.0f))
    {
        return 0.0f;
    }

    reaction_term = brake_deceleration_mps2 * reaction_time_s;
    return sqrtf(reaction_term * reaction_term +
                 2.0f * brake_deceleration_mps2 * usable_distance_m) -
           reaction_term;
}

int16_t PathSafety_LimitAxisCommand(int16_t command,
                                    float allowed_speed_mps,
                                    float command_to_mps)
{
    float command_speed_mps;
    int32_t allowed_command;

    if ((command == 0) || (command_to_mps <= 0.0f))
    {
        return command;
    }
    if (allowed_speed_mps <= 0.0f)
    {
        return 0;
    }

    command_speed_mps = fabsf((float)command) * command_to_mps;
    if (command_speed_mps <= allowed_speed_mps)
    {
        return command;
    }

    allowed_command = (int32_t)(allowed_speed_mps / command_to_mps);
    if (allowed_command <= 0)
    {
        return 0;
    }
    if (allowed_command > 32767)
    {
        allowed_command = 32767;
    }
    return (command < 0) ? (int16_t)-allowed_command :
                           (int16_t)allowed_command;
}

int PathSafety_LimitVectorCommand(int16_t *vx, int16_t *vy,
                                  float allowed_speed_mps,
                                  float command_to_mps)
{
    float magnitude_command;
    float magnitude_mps;
    float scale;

    if ((vx == NULL) || (vy == NULL) || (command_to_mps <= 0.0f))
    {
        return 0;
    }

    magnitude_command = sqrtf((float)(*vx) * (float)(*vx) +
                              (float)(*vy) * (float)(*vy));
    if (magnitude_command <= 0.0f)
    {
        return 0;
    }

    magnitude_mps = magnitude_command * command_to_mps;
    if ((allowed_speed_mps >= magnitude_mps) && (allowed_speed_mps > 0.0f))
    {
        return 0;
    }
    if (allowed_speed_mps <= 0.0f)
    {
        *vx = 0;
        *vy = 0;
        return 1;
    }

    scale = allowed_speed_mps / magnitude_mps;
    *vx = (int16_t)((float)*vx * scale);
    *vy = (int16_t)((float)*vy * scale);
    return 1;
}
