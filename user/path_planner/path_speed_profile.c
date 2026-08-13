/**
 ******************************************************************************
 * @file    path_speed_profile.c
 * @brief   离线速度剖面生成 + 最近点查表 + CSV 串口输出
 ******************************************************************************
 */
#include "path_speed_profile.h"

#include <math.h>
#include <string.h>

bool PathSpeedProfile_Build(path_point_t *points, uint16_t count,
                            const path_gridmap_t *real_map)
{
    uint16_t i;
    float v_limit;
    float ds;
    float ox;
    float oy;
    float dx;
    float dy;

    if ((points == NULL) || (count < 2U) || (real_map == NULL))
    {
        return false;
    }

    /* --- 1. 曲率限速 --- */
    for (i = 0U; i < count; i++)
    {
        float k_abs = fabsf(points[i].kappa);
        if (k_abs < PATH_KAPPA_MIN)
        {
            k_abs = PATH_KAPPA_MIN;
        }
        v_limit = sqrtf(PATH_A_LAT_MAX / k_abs);
        if (v_limit > PATH_V_MAX_MS)
        {
            v_limit = PATH_V_MAX_MS;
        }
        points[i].v_ref = v_limit;
    }

    /* --- 2. 前向扫描(加速能力) --- */
    points[0].v_ref = PATH_V_START_MS;
    for (i = 1U; i < count; i++)
    {
        ds = points[i].s_m - points[i - 1U].s_m;
        if (ds < 1e-6f)
        {
            continue;
        }
        float v_reach = sqrtf(points[i - 1U].v_ref * points[i - 1U].v_ref +
                              2.0f * PATH_A_LON_ACCEL * ds);
        if (v_reach < points[i].v_ref)
        {
            points[i].v_ref = v_reach;
        }
    }

    /* --- 3. 反向扫描(刹车能力) --- */
    points[count - 1U].v_ref = PATH_V_GOAL_MS;
    for (i = count - 1U; i > 0U; i--)
    {
        ds = points[i].s_m - points[i - 1U].s_m;
        if (ds < 1e-6f)
        {
            continue;
        }
        float v_brake = sqrtf(points[i].v_ref * points[i].v_ref +
                              2.0f * PATH_A_LON_BRAKE * ds);
        if (v_brake < points[i - 1U].v_ref)
        {
            points[i - 1U].v_ref = v_brake;
        }
    }

    /* 最低巡航速度(终点除外)。注意包含起点:否则 v[0]=0 会让机器人
     * 在轨迹索引 0 处查表得 0 速度,永远无法起步(死锁)。 */
    for (i = 0U; i + 1U < count; i++)
    {
        if (points[i].v_ref < PATH_V_MIN_MS)
        {
            points[i].v_ref = PATH_V_MIN_MS;
        }
    }

    /* --- 4. 期望激光表(yaw 锁定 0,车头朝 +y) --- */
    for (i = 0U; i < count; i++)
    {
        /* 前激光:挂 (0.225, 0),朝车体 +x(前方) */
        PathLaserRay(points[i].x_m, points[i].y_m, PATH_YAW_TARGET_RAD,
                     PATH_LASER_FRONT_X_M, PATH_LASER_FRONT_Y_M,
                     1.0f, 0.0f, &ox, &oy, &dx, &dy);
        points[i].exp_laser_front_m = PathGridMap_RayCast(real_map, ox, oy,
                                                          dx, dy,
                                                          PATH_LASER_MAX_RANGE_M);

        /* 左激光:挂 (0, 0.175),朝车体 +y(正左) */
        PathLaserRay(points[i].x_m, points[i].y_m, PATH_YAW_TARGET_RAD,
                     PATH_LASER_LEFT_X_M, PATH_LASER_LEFT_Y_M,
                     0.0f, 1.0f, &ox, &oy, &dx, &dy);
        points[i].exp_laser_left_m = PathGridMap_RayCast(real_map, ox, oy,
                                                         dx, dy,
                                                         PATH_LASER_MAX_RANGE_M);
    }

    return true;
}

uint16_t PathSpeedProfile_Nearest(const path_point_t *points, uint16_t count,
                                  float x, float y, uint16_t hint)
{
    uint16_t end;
    uint16_t i;
    uint16_t best_i;
    float best_d = 1e9f;
    float d;

    if ((points == NULL) || (count == 0U))
    {
        return 0U;
    }
    if (hint >= count)
    {
        hint = (uint16_t)(count - 1U);
    }

    end = hint + PATH_SEARCH_WINDOW;
    if (end >= count)
    {
        end = (uint16_t)(count - 1U);
    }

    best_i = hint;
    for (i = hint; i <= end; i++)
    {
        float dx = points[i].x_m - x;
        float dy = points[i].y_m - y;
        d = dx * dx + dy * dy;
        if (d < best_d)
        {
            best_d = d;
            best_i = i;
        }
    }

    /* 最近点已到窗口末尾且尚未到轨迹尾,整体前移,避免卡住 */
    if ((best_i >= end) && (end < count - 1U))
    {
        best_i = end;
    }

    return best_i;
}

/* ---------------- CSV 输出 ---------------- */

static void uart_puts(UART_HandleTypeDef *uart, const char *s)
{
    (void)HAL_UART_Transmit(uart, (uint8_t *)s, (uint16_t)strlen(s), 10U);
}

/* 手写浮点格式化(2 位小数),不依赖 printf/snprintf 重定向与 microlib 行为 */
static void uart_putf(UART_HandleTypeDef *uart, float v)
{
    char buf[24];
    uint8_t pos = 0U;
    char rev[12];
    uint8_t rpos = 0U;
    int32_t int_part;
    uint32_t frac;

    if (v < 0.0f)
    {
        buf[pos++] = '-';
        v = -v;
    }
    int_part = (int32_t)v;
    frac = (uint32_t)((v - (float)int_part) * 100.0f + 0.5f);
    if (frac >= 100U)
    {
        int_part++;
        frac = 0U;
    }
    if (int_part == 0)
    {
        buf[pos++] = '0';
    }
    else
    {
        while (int_part > 0)
        {
            rev[rpos++] = (char)('0' + (int_part % 10));
            int_part /= 10;
        }
        while (rpos > 0U)
        {
            buf[pos++] = rev[--rpos];
        }
    }
    buf[pos++] = '.';
    buf[pos++] = (char)('0' + (frac / 10U) % 10U);
    buf[pos++] = (char)('0' + frac % 10U);
    buf[pos] = '\0';
    uart_puts(uart, buf);
}

/* 手写 uint 转字符串 */
static void uart_putu(UART_HandleTypeDef *uart, uint16_t v)
{
    char rev[6];
    char buf[8];
    uint8_t rpos = 0U;
    uint8_t pos = 0U;

    if (v == 0U)
    {
        buf[pos++] = '0';
    }
    else
    {
        while (v > 0U)
        {
            rev[rpos++] = (char)('0' + (v % 10U));
            v /= 10U;
        }
        while (rpos > 0U)
        {
            buf[pos++] = rev[--rpos];
        }
    }
    buf[pos] = '\0';
    uart_puts(uart, buf);
}

void PathSpeedProfile_DumpCsv(const path_point_t *points, uint16_t count,
                              UART_HandleTypeDef *uart)
{
    uint16_t i;

    if ((points == NULL) || (uart == NULL))
    {
        return;
    }

    uart_puts(uart, "s_m,x_m,y_m,kappa,v_ref,exp_laser_front_m,exp_laser_left_m\r\n");
    for (i = 0U; i < count; i++)
    {
        uart_putu(uart, i);
        uart_puts(uart, ",");
        uart_putf(uart, points[i].s_m);
        uart_puts(uart, ",");
        uart_putf(uart, points[i].x_m);
        uart_puts(uart, ",");
        uart_putf(uart, points[i].y_m);
        uart_puts(uart, ",");
        uart_putf(uart, points[i].kappa);
        uart_puts(uart, ",");
        uart_putf(uart, points[i].v_ref);
        uart_puts(uart, ",");
        uart_putf(uart, points[i].exp_laser_front_m);
        uart_puts(uart, ",");
        uart_putf(uart, points[i].exp_laser_left_m);
        uart_puts(uart, "\r\n");
    }
}
