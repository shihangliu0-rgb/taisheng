/**
 ******************************************************************************
 * @file    path_spline.c
 * @brief   B 样条实现:de Boor 递推、导数、弧长、曲率、离墙外推
 ******************************************************************************
 */
#include "path_spline.h"

#include <math.h>
#include <stddef.h>

#define MAX_CTRL        (PATH_WAYPOINT_COUNT + 1U)  /* 控制点上限 */
#define MAX_KNOTS       (MAX_CTRL + PATH_SPLINE_DEGREE + 1U)
#define MAX_SAMPLES     PATH_SPLINE_SAMPLES

/* 前向声明:Build 末尾调用 */
static void update_arc_curvature(path_point_t *points, uint16_t count);

/* de Boor 求值:返回曲线值 out,导数 dout(dout 可为 NULL) */
static void deboor(const float *ctrl_x, const float *ctrl_y,
                   const float *knots, uint8_t n_ctrl, uint8_t degree,
                   float u, float *out_x, float *out_y)
{
    float dx[MAX_CTRL];
    float dy[MAX_CTRL];
    uint8_t k;
    uint8_t r;
    uint8_t j;
    float alpha;
    int16_t k_int;

    /* 找到 span k:u 在 [U_k, U_{k+1}) */
    k = degree;
    while ((k + 1U < n_ctrl + degree) && (u >= knots[k + 1U]))
    {
        k++;
    }

    for (j = 0U; j <= degree; j++)
    {
        k_int = (int16_t)(k - degree + j);
        if (k_int < 0)
        {
            k_int = 0;
        }
        dx[j] = ctrl_x[k_int];
        dy[j] = ctrl_y[k_int];
    }

    for (r = 1U; r <= degree; r++)
    {
        for (j = degree; j >= r; j--)
        {
            k_int = (int16_t)(k - degree + j);
            if (k_int < 0)
            {
                k_int = 0;
            }
            float denom = knots[j + k + 1U - r] - knots[k_int];
            if (denom < 1e-6f)
            {
                alpha = 0.0f;
            }
            else
            {
                alpha = (u - knots[k_int]) / denom;
            }
            dx[j] = (1.0f - alpha) * dx[j - 1U] + alpha * dx[j];
            dy[j] = (1.0f - alpha) * dy[j - 1U] + alpha * dy[j];
        }
    }

    *out_x = dx[degree];
    *out_y = dy[degree];
}

bool PathSpline_Build(const path_waypoint_t *waypoints, uint8_t n_wp,
                      path_point_t *out, uint16_t max_out,
                      uint16_t *out_count)
{
    float ctrl_x[MAX_CTRL];
    float ctrl_y[MAX_CTRL];
    float knots[MAX_KNOTS];
    float chord[MAX_CTRL];
    float total = 0.0f;
    uint8_t n = n_wp;
    uint8_t p = PATH_SPLINE_DEGREE;
    uint8_t i;
    uint16_t sample;
    uint16_t count;
    float u;
    float px;
    float py;
    float px_next;
    float py_next;
    float px_prev;
    float py_prev;
    float tx;
    float ty;

    if ((waypoints == NULL) || (out == NULL) || (out_count == NULL) ||
        (n < (p + 1U)) || (n > MAX_CTRL) || (max_out == 0U))
    {
        return false;
    }

    /* --- 1. 弦长参数化 --- */
    chord[0] = 0.0f;
    for (i = 0U; i < n; i++)
    {
        ctrl_x[i] = waypoints[i].x_m;
        ctrl_y[i] = waypoints[i].y_m;
        if (i > 0U)
        {
            float dx = ctrl_x[i] - ctrl_x[i - 1U];
            float dy = ctrl_y[i] - ctrl_y[i - 1U];
            total += sqrtf(dx * dx + dy * dy);
            chord[i] = total;
        }
    }
    if (total < 1e-6f)
    {
        return false;
    }
    for (i = 1U; i < n; i++)
    {
        chord[i] /= total;
    }

    /* --- 2. clamped 弦长平均 knot vector ---
     * U[0..p]=0,U[p+j]=chord[j](j=1..n-p-1),U[n..n+p]=1 */
    for (i = 0U; i <= p; i++)
    {
        knots[i] = 0.0f;
    }
    for (i = 1U; i <= (uint8_t)(n - p - 1U); i++)
    {
        knots[p + i] = chord[i];
    }
    for (i = n; i <= (uint8_t)(n + p); i++)
    {
        knots[i] = 1.0f;
    }

    /* --- 3. 采样并计算切线 --- */
    count = (max_out < MAX_SAMPLES) ? max_out : MAX_SAMPLES;
    for (sample = 0U; sample < count; sample++)
    {
        u = (count == 1U) ? 0.0f
                          : (float)sample / (float)(count - 1U);

        deboor(ctrl_x, ctrl_y, knots, n, p, u, &px, &py);

        /* 切线:用前后采样点中心差分(最后一个点与自身差分) */
        u += 1e-4f;
        if (u > 1.0f)
        {
            u = 1.0f;
        }
        deboor(ctrl_x, ctrl_y, knots, n, p, u, &px_next, &py_next);

        u = (count == 1U) ? 0.0f
                          : (float)sample / (float)(count - 1U);
        u -= 1e-4f;
        if (u < 0.0f)
        {
            u = 0.0f;
        }
        deboor(ctrl_x, ctrl_y, knots, n, p, u, &px_prev, &py_prev);

        tx = px_next - px_prev;
        ty = py_next - py_prev;

        out[sample].x_m = px;
        out[sample].y_m = py;
        out[sample].yaw_tangent = atan2f(ty, tx);
        out[sample].s_m = 0.0f;
        out[sample].kappa = 0.0f;
        out[sample].v_ref = 0.0f;
        out[sample].exp_laser_front_m = 0.0f;
        out[sample].exp_laser_left_m = 0.0f;
    }

    *out_count = count;
    update_arc_curvature(out, count);
    return true;
}

/* ---------------- 弧长/曲率(推离或平滑后需重算) ---------------- */
static void update_arc_curvature(path_point_t *points, uint16_t count)
{
    uint16_t sample;
    float ds;
    float dtheta;

    /* 切线:中心差分(端点用相邻两点方向) */
    for (sample = 1U; sample < count - 1U; sample++)
    {
        float tx = points[sample + 1U].x_m - points[sample - 1U].x_m;
        float ty = points[sample + 1U].y_m - points[sample - 1U].y_m;
        points[sample].yaw_tangent = atan2f(ty, tx);
    }
    points[0].yaw_tangent = atan2f(points[1].y_m - points[0].y_m,
                                   points[1].x_m - points[0].x_m);
    points[count - 1U].yaw_tangent =
        atan2f(points[count - 1U].y_m - points[count - 2U].y_m,
               points[count - 1U].x_m - points[count - 2U].x_m);

    /* 弧长 */
    points[0].s_m = 0.0f;
    for (sample = 1U; sample < count; sample++)
    {
        float dx = points[sample].x_m - points[sample - 1U].x_m;
        float dy = points[sample].y_m - points[sample - 1U].y_m;
        points[sample].s_m = points[sample - 1U].s_m +
                             sqrtf(dx * dx + dy * dy);
    }

    /* 曲率 κ = dθ/ds(中心差分 + 2 遍 3 点平滑) */
    for (sample = 1U; sample < count - 1U; sample++)
    {
        ds = points[sample + 1U].s_m - points[sample - 1U].s_m;
        if (ds < 1e-6f)
        {
            points[sample].kappa = 0.0f;
            continue;
        }
        dtheta = PathWrapAngle(points[sample + 1U].yaw_tangent -
                               points[sample - 1U].yaw_tangent);
        points[sample].kappa = dtheta / ds;
    }
    points[0].kappa = points[1].kappa;
    points[count - 1U].kappa = points[count - 2U].kappa;

    {
        uint8_t pass;
        for (pass = 0U; pass < 2U; pass++)
        {
            float prev = points[0].kappa;
            for (sample = 1U; sample < count - 1U; sample++)
            {
                float cur = points[sample].kappa;
                float nxt = points[sample + 1U].kappa;
                points[sample].kappa = 0.25f * prev + 0.5f * cur +
                                       0.25f * nxt;
                prev = cur;
            }
        }
    }
}

/* ---------------- 平滑与最终化 ---------------- */
static void smooth_xy(path_point_t *points, uint16_t count, uint8_t window)
{
    uint16_t sample;
    uint8_t half;
    int16_t lo;
    int16_t hi;
    uint16_t j;
    float sum_x;
    float sum_y;
    uint16_t n;

    if ((points == NULL) || (count < 3U) || (window < 3U))
    {
        return;
    }
    half = (uint8_t)(window / 2U);

    for (sample = 0U; sample < count; sample++)
    {
        lo = (int16_t)sample - (int16_t)half;
        hi = (int16_t)sample + (int16_t)half;
        if (lo < 0)
        {
            lo = 0;
        }
        if (hi >= (int16_t)count)
        {
            hi = (int16_t)(count - 1U);
        }
        sum_x = 0.0f;
        sum_y = 0.0f;
        n = 0U;
        for (j = (uint16_t)lo; j <= (uint16_t)hi; j++)
        {
            sum_x += points[j].x_m;
            sum_y += points[j].y_m;
            n++;
        }
        points[sample].x_m = sum_x / (float)n;
        points[sample].y_m = sum_y / (float)n;
    }
}

/* 曲率整形:把曲率超限(转弯半径过小)的点沿弯道外侧推开。
 * 说明:本场地的两个关键拐角(D 角与 wall_C 西侧拐角)墙角都在弯道
 * 内侧,内切圆弧会擦墙(离墙角 0.2~0.34m < 车角半径 0.379m),
 * 必须使用"外侧绕行"的弧线 —— 即把转弯点向弯道外侧推,让路径
 * 先远离墙再转弯。逐点外推 + 平滑交替迭代即可收敛成这种形状。 */
static void limit_curvature(path_point_t *points, uint16_t count,
                            float min_radius)
{
    uint16_t s;
    float k_max;

    if ((points == NULL) || (count < 3U))
    {
        return;
    }

    k_max = 1.0f / min_radius;
    for (s = 1U; s < count - 1U; s++)
    {
        float k = points[s].kappa;
        float th;
        float nx;
        float ny;
        float dir;

        if (fabsf(k) <= k_max)
        {
            continue;
        }

        th = points[s].yaw_tangent;
        nx = -sinf(th);   /* 切向的左法向 */
        ny = cosf(th);
        /* κ>0(左转)曲率中心在左侧 -> 向右外推;κ<0 反之 */
        dir = (k > 0.0f) ? -1.0f : 1.0f;
        points[s].x_m += dir * nx * PATH_PUSH_STEP_M;
        points[s].y_m += dir * ny * PATH_PUSH_STEP_M;
    }
}

void PathSpline_Finalize(path_point_t *points, uint16_t count,
                         const path_gridmap_t *inflated_map)
{
    uint8_t round;

    if ((points == NULL) || (inflated_map == NULL))
    {
        return;
    }

    /* 交替推离 + 平滑:收敛为绕膨胀墙的连续圆角 */
    for (round = 0U; round < PATH_PUSH_SMOOTH_ROUNDS; round++)
    {
        (void)PathSpline_PushAwayFromWalls(points, count, inflated_map);
        smooth_xy(points, count, PATH_SMOOTH_WINDOW);
    }
    /* 最后一轮只推离(平滑可能把点带回墙内) */
    (void)PathSpline_PushAwayFromWalls(points, count, inflated_map);

    /* 曲率整形:最小转弯半径约束(外侧绕行) */
    for (round = 0U; round < PATH_CURV_LIMIT_ITERS; round++)
    {
        update_arc_curvature(points, count);
        limit_curvature(points, count, PATH_MIN_TURN_RADIUS_M);
        smooth_xy(points, count, PATH_SMOOTH_WINDOW);
        (void)PathSpline_PushAwayFromWalls(points, count, inflated_map);
    }

    update_arc_curvature(points, count);
}


uint16_t PathSpline_PushAwayFromWalls(path_point_t *points, uint16_t count,
                                      const path_gridmap_t *inflated_map)
{
    uint16_t sample;
    uint16_t pushed = 0U;

    if ((points == NULL) || (inflated_map == NULL))
    {
        return 0U;
    }

    for (sample = 0U; sample < count; sample++)
    {
        if (PathGridMap_Contains(inflated_map,
                                 points[sample].x_m, points[sample].y_m))
        {
            if (PathGridMap_PushOut(inflated_map,
                                    &points[sample].x_m, &points[sample].y_m,
                                    PATH_PUSH_STEP_M, PATH_PUSH_MAX_ITERS))
            {
                pushed++;
            }
        }
    }

    return pushed;
}
