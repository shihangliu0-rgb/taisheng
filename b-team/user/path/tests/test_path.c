#include "path_localization.h"
#include "path_map.h"
#include "path_safety.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define TEST_EPSILON 0.0002f

static void expect_close(float actual, float expected)
{
    assert(fabsf(actual - expected) <= TEST_EPSILON);
}

static void test_reference_map(void)
{
    const path_map_wall_t *walls;
    const path_map_route_segment_t *route;
    uint8_t count;

    walls = PathMap_GetWalls(&count);
    assert(count == 7U);
    expect_close(walls[4].x_min_m, 1.05f);
    expect_close(walls[4].y_min_m, 1.07f);
    expect_close(walls[4].x_max_m, 3.00f);
    expect_close(walls[4].y_max_m, 1.12f);
    expect_close(walls[5].x_max_m, 2.00f);
    expect_close(walls[5].y_min_m, 2.075f);
    expect_close(walls[6].x_min_m, 1.05f);
    expect_close(walls[6].y_max_m, 3.125f);

    route = PathMap_GetRoute(&count);
    assert(count == 6U);
    assert(route[0].axis == PATH_MAP_AXIS_Y);
    assert(route[0].direction == 1);
    expect_close(route[0].target_m, 1.60f);
    assert(route[3].axis == PATH_MAP_AXIS_X);
    assert(route[3].direction == -1);
    expect_close(route[3].target_m, 0.36f);
    expect_close(route[5].target_m, 0.50f);
}

static void test_segment_crossing(void)
{
    assert(!PathMap_SegmentReached(0U, 0.27f, 1.599f));
    assert(PathMap_SegmentReached(0U, 0.27f, 1.600f));
    assert(!PathMap_SegmentReached(1U, 2.479f, 1.65f));
    assert(PathMap_SegmentReached(1U, 2.480f, 1.65f));
    assert(!PathMap_SegmentReached(3U, 0.361f, 2.60f));
    assert(PathMap_SegmentReached(3U, 0.360f, 2.60f));
    assert(!PathMap_SegmentReached(PATH_MAP_ROUTE_SEGMENT_COUNT,
                                   0.0f, 0.0f));
}

static void test_map_clearance(void)
{
    float distance;

    /* 边界净空线与初始定位方式无关。 */
    expect_close(PathMap_RayClearance(PATH_MAP_BOUNDARY_MARGIN_X_M,
                                      PATH_MAP_BOUNDARY_MARGIN_Y_M,
                                      -1.0f, 0.0f), 0.0f);
    expect_close(PathMap_RayClearance(PATH_MAP_BOUNDARY_MARGIN_X_M,
                                      PATH_MAP_BOUNDARY_MARGIN_Y_M,
                                      0.0f, -1.0f), 0.0f);

    /* 西南净空线向前最先遇到膨胀后的墙 B。 */
    distance = PathMap_RayClearance(PATH_MAP_BOUNDARY_MARGIN_X_M,
                                    PATH_MAP_BOUNDARY_MARGIN_Y_M,
                                    0.0f, 1.0f);
    expect_close(distance, 1.7465f - PATH_MAP_BOUNDARY_MARGIN_Y_M);

    /* 第一条横向通道中向右只受场地东边界约束。 */
    distance = PathMap_RayClearance(0.50f, 1.65f, 1.0f, 0.0f);
    expect_close(distance,
                 (PATH_MAP_FIELD_WIDTH_M - PATH_MAP_BOUNDARY_MARGIN_X_M) -
                 0.50f);

    /* 墙 B 右侧向上时，最先遇到膨胀后的墙 C。 */
    distance = PathMap_RayClearance(2.48f, 1.65f, 0.0f, 1.0f);
    expect_close(distance, 2.7465f - 1.65f);

    /* 在膨胀墙边界或内部时，只允许朝最近边界恢复，不允许穿墙。 */
    expect_close(PathMap_RayClearance(1.00f, 1.7465f,
                                      0.0f, 1.0f), 0.0f);
    expect_close(PathMap_RayClearance(1.00f, 1.7465f,
                                      0.0f, -1.0f),
                 1.7465f - 1.4485f);
    expect_close(PathMap_RayClearance(1.00f, 1.80f,
                                      0.0f, 1.0f), 0.0f);
    expect_close(PathMap_RayClearance(1.00f, 1.80f,
                                      0.0f, -1.0f),
                 1.80f - 1.4485f);

    /* 已在膨胀带内时，沿墙切向平移不再被锁死（否则无法脱困）。 */
    expect_close(PathMap_RayClearance(1.00f, 1.80f, 1.0f, 0.0f),
                 (PATH_MAP_FIELD_WIDTH_M - PATH_MAP_BOUNDARY_MARGIN_X_M) -
                 1.00f);
    expect_close(PathMap_RayClearance(1.00f, 1.80f, -1.0f, 0.0f),
                 1.00f - PATH_MAP_BOUNDARY_MARGIN_X_M);

}

static void test_mirrored_map(void)
{
    const path_map_wall_t *walls;
    const path_map_route_segment_t *route;
    uint8_t count;

    PathMap_SetMirrored(true);
    assert(PathMap_IsMirrored());

    walls = PathMap_GetWalls(&count);
    assert(count == 7U);
    /* 4 面边界墙左右对称，保持不变。 */
    expect_close(walls[0].x_min_m, 0.000f);
    expect_close(walls[0].y_max_m, 0.049f);
    expect_close(walls[1].y_min_m, 5.951f);
    expect_close(walls[2].x_max_m, 0.049f);
    expect_close(walls[3].x_min_m, 2.951f);
    /* 内墙整体左右镜像：墙 1、墙 C 改贴西墙，墙 B 改贴东墙。 */
    expect_close(walls[4].x_min_m, 0.000f);
    expect_close(walls[4].x_max_m, 1.950f);
    expect_close(walls[4].y_min_m, 1.070f);
    expect_close(walls[4].y_max_m, 1.120f);
    expect_close(walls[5].x_min_m, 1.000f);
    expect_close(walls[5].x_max_m, 3.000f);
    expect_close(walls[5].y_min_m, 2.075f);
    expect_close(walls[5].y_max_m, 2.125f);
    expect_close(walls[6].x_min_m, 0.000f);
    expect_close(walls[6].x_max_m, 1.950f);
    expect_close(walls[6].y_min_m, 3.075f);
    expect_close(walls[6].y_max_m, 3.125f);

    route = PathMap_GetRoute(&count);
    assert(count == 6U);
    assert(route[0].axis == PATH_MAP_AXIS_Y);
    assert(route[0].direction == 1);
    expect_close(route[0].target_m, 1.600f);
    assert(route[1].axis == PATH_MAP_AXIS_X);
    assert(route[1].direction == -1);
    expect_close(route[1].target_m, 0.520f);
    assert(route[2].direction == 1);
    expect_close(route[2].target_m, 2.600f);
    assert(route[3].direction == 1);
    expect_close(route[3].target_m, 2.640f);
    assert(route[4].direction == 1);
    expect_close(route[4].target_m, 3.700f);
    assert(route[5].axis == PATH_MAP_AXIS_X);
    assert(route[5].direction == -1);
    expect_close(route[5].target_m, 2.500f);

    /* 镜像侧各段终点判断。 */
    assert(!PathMap_SegmentReached(0U, PATH_MAP_MIRRORED_START_X_M,
                                   1.599f));
    assert(PathMap_SegmentReached(0U, PATH_MAP_MIRRORED_START_X_M,
                                  1.600f));
    assert(!PathMap_SegmentReached(1U, 0.521f, 1.650f));
    assert(PathMap_SegmentReached(1U, 0.520f, 1.650f));
    assert(!PathMap_SegmentReached(3U, 2.639f, 2.600f));
    assert(PathMap_SegmentReached(3U, 2.640f, 2.600f));
    assert(!PathMap_SegmentReached(5U, 2.501f, 3.700f));
    assert(PathMap_SegmentReached(5U, 2.500f, 3.700f));

    PathMap_SetMirrored(false);
    assert(!PathMap_IsMirrored());
}

static void test_mirrored_map_clearance(void)
{
    float distance;

    PathMap_SetMirrored(true);

    /* 镜像起点（贴东墙）在墙 B 下方：向上最先遇到膨胀后的墙 B。 */
    distance = PathMap_RayClearance(PATH_MAP_MIRRORED_START_X_M,
                                    PATH_MAP_BOUNDARY_MARGIN_Y_M,
                                    0.0f, 1.0f);
    expect_close(distance, 1.7465f - PATH_MAP_BOUNDARY_MARGIN_Y_M);

    /* 第一条横向通道中向左只受场地西边界约束。 */
    distance = PathMap_RayClearance(2.50f, 1.65f, -1.0f, 0.0f);
    expect_close(distance,
                 2.50f - PATH_MAP_BOUNDARY_MARGIN_X_M);

    /* 墙 B 左侧向上时，最先遇到膨胀后的墙 C。 */
    distance = PathMap_RayClearance(0.52f, 1.65f, 0.0f, 1.0f);
    expect_close(distance, 2.7465f - 1.65f);

    /* 常规侧数据不受镜像选择影响。 */
    PathMap_SetMirrored(false);
    expect_close(PathMap_RayClearance(0.50f, 1.65f, 1.0f, 0.0f),
                 (PATH_MAP_FIELD_WIDTH_M -
                  PATH_MAP_BOUNDARY_MARGIN_X_M) - 0.50f);
}

static void test_initial_localization_geometry(void)
{
    path_localization_result_t result;
    path_localization_result_t result_without_front_sample;
    float yaw_rad = 10.0f * 3.14159265358979323846f / 180.0f;
    float left_center_to_wall_m =
        PATH_LOCALIZATION_LEFT_SENSOR_OFFSET_M + 0.15f;

    assert(PathLocalization_Calculate(0.18f, 0.15f, 10.0f, &result));
    expect_close(result.map_x_m,
                 PATH_MAP_WEST_INNER_X_M +
                 left_center_to_wall_m * cosf(yaw_rad));
    expect_close(result.map_y_m, PATH_MAP_INITIAL_CENTER_Y_M);
    expect_close(result.front_wall_hit_x_m, 0.0f);
    expect_close(result.left_wall_hit_y_m,
                 result.map_y_m - left_center_to_wall_m * sinf(yaw_rad));

    /* 前 DT35 无样本（0）甚至无效值时都不改变初始定位结果。 */
    assert(PathLocalization_Calculate(0.0f, 0.15f, 10.0f,
                                      &result_without_front_sample));
    expect_close(result_without_front_sample.map_x_m, result.map_x_m);
    expect_close(result_without_front_sample.map_y_m, result.map_y_m);
    /* 左光装在最左端：x = 0.049 + 半车宽 0.22 + 读数。 */
    assert(PathLocalization_Calculate(-1.0f, 0.15f, 0.0f, &result));
    expect_close(result.map_x_m, 0.4190f);
    expect_close(result.map_y_m, 0.3085f);
    expect_close(result.front_wall_hit_x_m, 0.0f);
    expect_close(result.left_wall_hit_y_m, 0.3085f);

    assert(!PathLocalization_Calculate(0.0f, -0.01f, 0.0f, &result));
    assert(!PathLocalization_Calculate(0.0f, 0.15f, 100.0f, &result));
    assert(!PathLocalization_Calculate(0.0f, 0.15f, 0.0f, NULL));
}

static void test_dynamic_distance(void)
{
    float required;
    float allowed;

    required = PathSafety_RequiredDistance(0.4f, 0.12f, 0.08f, 2.0f);
    expect_close(required, 0.192f);
    allowed = PathSafety_MaxAllowedSpeed(required, 0.12f, 0.08f, 2.0f);
    expect_close(allowed, 0.4f);
    expect_close(PathSafety_MaxAllowedSpeed(0.12f, 0.12f, 0.08f, 2.0f),
                 0.0f);
    expect_close(PathSafety_MaxAllowedSpeed(0.05f, 0.12f, 0.08f, 2.0f),
                 0.0f);
}

static void test_command_limits(void)
{
    int16_t vx;
    int16_t vy;

    assert(PathSafety_LimitAxisCommand(100, 0.50f, 0.01f) == 50);
    assert(PathSafety_LimitAxisCommand(-100, 0.50f, 0.01f) == -50);
    assert(PathSafety_LimitAxisCommand(20, 0.50f, 0.01f) == 20);
    assert(PathSafety_LimitAxisCommand(20, 0.0f, 0.01f) == 0);

    vx = 60;
    vy = 80;
    assert(PathSafety_LimitVectorCommand(&vx, &vy, 0.50f, 0.01f) == 1);
    assert(vx == 30);
    assert(vy == 40);

    vx = -30;
    vy = 0;
    assert(PathSafety_LimitVectorCommand(&vx, &vy, 0.50f, 0.01f) == 0);
    assert(vx == -30);
}

int main(void)
{
    test_reference_map();
    test_segment_crossing();
    test_map_clearance();
    test_mirrored_map();
    test_mirrored_map_clearance();
    test_initial_localization_geometry();
    test_dynamic_distance();
    test_command_limits();
    puts("path host tests: PASS");
    return 0;
}
