#include "path_localization.h"

#include "path_map.h"

#include <math.h>
#include <stddef.h>

#define PATH_LOCALIZATION_PI 3.14159265358979323846f
#define PATH_LOCALIZATION_WALL_EPSILON_M 0.0010f

static bool PathLocalization_IsFinite(float value)
{
    return !isnan(value) && !isinf(value);
}

bool PathLocalization_Calculate(float front_distance_m,
                                float left_distance_m,
                                float yaw_deg,
                                path_localization_result_t *result)
{
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float left_ray_origin_distance_m;

    /* 前 DT35 参数仅为兼容既有接口保留，不参与初始地图定位。 */
    (void)front_distance_m;

    if ((result == NULL) ||
        !PathLocalization_IsFinite(left_distance_m) ||
        !PathLocalization_IsFinite(yaw_deg) ||
        (left_distance_m <= 0.0f))
    {
        return false;
    }

    yaw_rad = yaw_deg * (PATH_LOCALIZATION_PI / 180.0f);
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    if (cos_yaw <= 0.0f)
    {
        return false;
    }

    left_ray_origin_distance_m =
        PATH_LOCALIZATION_LEFT_SENSOR_OFFSET_M + left_distance_m;

    /*
     * 初始中心 Y 恒为车长一半；左 DT35 随实测 yaw 旋转，并与
     * x=0.049 m 西边界内侧面相交，由此只反算中心 X。
     */
    result->map_x_m = PATH_MAP_WEST_INNER_X_M +
                      left_ray_origin_distance_m * cos_yaw;
    result->map_y_m = PATH_MAP_INITIAL_CENTER_Y_M;
    result->front_wall_hit_x_m = 0.0f;
    result->left_wall_hit_y_m = result->map_y_m -
                                left_ray_origin_distance_m * sin_yaw;

    if (!PathLocalization_IsFinite(result->map_x_m) ||
        !PathLocalization_IsFinite(result->map_y_m) ||
        (result->map_x_m < 0.0f) ||
        (result->map_x_m > PATH_MAP_FIELD_WIDTH_M) ||
        (result->map_y_m < 0.0f) ||
        (result->map_y_m > PATH_MAP_FIELD_HEIGHT_M) ||
        (result->left_wall_hit_y_m <
         -PATH_LOCALIZATION_WALL_EPSILON_M) ||
        (result->left_wall_hit_y_m >
         (PATH_MAP_FIELD_HEIGHT_M + PATH_LOCALIZATION_WALL_EPSILON_M)))
    {
        return false;
    }

    return true;
}
