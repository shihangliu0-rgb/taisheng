/**
 ******************************************************************************
 * @file    path_main.c
 * @brief   曲线跑图实现(场地配置 path_config.h + 函数定义 path_main.h + 本文件)
 *
 * 完整运行流程(上电后自动执行):
 *
 *   INIT       -> 关闭 IMU 模块自带航向保持(本模块用自己的 yaw-lock)
 *   CALIB      -> 静止采 IMU 陀螺零偏(ImuMain_GetData,200 帧约 1s)
 *   WAIT_START -> 读小电脑 0x11 位置帧(PcLink_GetPosition)覆盖起点,最长等 3s
 *   BUILD      -> 路点(path_config.h)-> B 样条 -> 离墙外推 + 最小转弯半径整形
 *                 -> 速度剖面(曲率限速 + 双向扫描)+ 前/左激光期望距离表
 *   RUN(5ms)   -> 1) IMU 陀螺积分预测 yaw
 *                 2) 读小电脑位姿,30cm/20度门限 + 中值滤波融合(新帧去重)
 *                 3) 读前/左激光(dt35_link[].distance_cm,UART9 DT35 帧)
 *                 4) 最近点查表 v_ref
 *                 5) 前激光兜底:v_used = min(v_ref, sqrt(2*a_brake*(d-0.12)));
 *                    d<0.12 强制纵向停车,保留横向脱困平移
 *                 6) 纯追踪(曲率自适应前视)算世界系速度矢量
 *                 7) 世界系 -> 底盘系(R(yaw))-> Chassis_SetVelocity(RPM 单位)
 *                 8) 左激光横向微调 + yaw-lock 出 w + slew-rate 限幅
 *   ARRIVED    -> 距终点 0.15m 内停车
 *   STOPPED    -> 上位机丢失>500ms / IMU 离线 / 前激光<12cm / NaN / 超时 均停车
 *
 * 同时通过 PcLink_SetStatus 回传 0x20 状态帧,小电脑据此判断控制器在线。
 ******************************************************************************
 */
#include "path_main.h"

/* 仓库已有模块(全部真实外设数据) */
#include "chassis_main.h"
#include "dt35_pnp_link.h"
#include "imu_main.h"
#include "pc_link.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#if PATH_DEBUG
#include "usart.h"
#endif

#define DEG2RAD (float)(PATH_PI / 180.0)
#define RAD2DEG (float)(180.0 / PATH_PI)

/* ================================================================
 * 模块:几何工具:墙 / 膨胀 / 射线投射 / 坐标变换
 * ================================================================ */
/**
 ******************************************************************************
 * @file    path_geometry.c
 * @brief   几何工具实现:墙 / 距离场 / 射线投射 / 坐标变换
 ******************************************************************************
 */


/* field.yaml walls 段(wall_A 已注释,未启用) */
static const path_wall_t real_walls[PATH_WALL_COUNT] =
{
    PATH_WALLS_TABLE
};

/* 膨胀墙静态缓冲 */
static path_wall_t inflated_walls[PATH_WALL_COUNT];

void PathGridMap_BuildReal(path_gridmap_t *map)
{
    if (map == NULL)
    {
        return;
    }

    map->walls = real_walls;
    map->count = PATH_WALL_COUNT;
}

void PathGridMap_BuildInflated(path_gridmap_t *map)
{
    uint8_t i;

    if (map == NULL)
    {
        return;
    }

    for (i = 0U; i < PATH_WALL_COUNT; i++)
    {
        /* 矩形膨胀:yaw 锁定下机器人为轴对齐矩形 */
        inflated_walls[i].xmin = real_walls[i].xmin - PATH_INFLATE_DX_M;
        inflated_walls[i].ymin = real_walls[i].ymin - PATH_INFLATE_DY_M;
        inflated_walls[i].xmax = real_walls[i].xmax + PATH_INFLATE_DX_M;
        inflated_walls[i].ymax = real_walls[i].ymax + PATH_INFLATE_DY_M;
    }

    map->walls = inflated_walls;
    map->count = PATH_WALL_COUNT;
}

/* ------------------------------------------------------------------ */
static bool wall_contains(const path_wall_t *w, float x, float y)
{
    return (x >= w->xmin) && (x <= w->xmax) &&
           (y >= w->ymin) && (y <= w->ymax);
}

static float wall_dist_to(const path_wall_t *w, float x, float y)
{
    float dx;
    float dy;

    if (wall_contains(w, x, y))
    {
        return 0.0f;
    }

    dx = (x < w->xmin) ? (w->xmin - x) : ((x > w->xmax) ? (x - w->xmax) : 0.0f);
    dy = (y < w->ymin) ? (w->ymin - y) : ((y > w->ymax) ? (y - w->ymax) : 0.0f);
    return sqrtf(dx * dx + dy * dy);
}

/* AABB 射线相交(Slab 法),方向为单位向量 */
static float wall_ray_cast(const path_wall_t *w,
                           float ox, float oy,
                           float dx, float dy,
                           float max_range)
{
    float tx1;
    float tx2;
    float ty1;
    float ty2;
    float tmin;
    float tmax;
    float tmp;

    if (fabsf(dx) < 1e-9f)
    {
        /* 射线平行于 y 轴方向分量:若原点在墙 x 范围外则不相交 */
        if ((ox < w->xmin) || (ox > w->xmax))
        {
            return max_range;
        }
        tx1 = -1e9f;
        tx2 = 1e9f;
    }
    else
    {
        tx1 = (w->xmin - ox) / dx;
        tx2 = (w->xmax - ox) / dx;
        if (tx1 > tx2)
        {
            tmp = tx1;
            tx1 = tx2;
            tx2 = tmp;
        }
    }

    if (fabsf(dy) < 1e-9f)
    {
        if ((oy < w->ymin) || (oy > w->ymax))
        {
            return max_range;
        }
        ty1 = -1e9f;
        ty2 = 1e9f;
    }
    else
    {
        ty1 = (w->ymin - oy) / dy;
        ty2 = (w->ymax - oy) / dy;
        if (ty1 > ty2)
        {
            tmp = ty1;
            ty1 = ty2;
            ty2 = tmp;
        }
    }

    tmin = (tx1 > ty1) ? tx1 : ty1;
    tmax = (tx2 < ty2) ? tx2 : ty2;

    if ((tmax < 0.0f) || (tmin > tmax))
    {
        return max_range;
    }
    if (tmin > max_range)
    {
        return max_range;
    }

    return (tmin < 0.0f) ? 0.0f : tmin;
}

/* ------------------------------------------------------------------ */
static bool PathGridMap_Contains(const path_gridmap_t *map, float x, float y)
{
    uint8_t i;

    if (map == NULL)
    {
        return false;
    }

    for (i = 0U; i < map->count; i++)
    {
        if (wall_contains(&map->walls[i], x, y))
        {
            return true;
        }
    }

    return false;
}

float PathGridMap_DistTo(const path_gridmap_t *map, float x, float y)
{
    uint8_t i;
    float best = 1e9f;
    float d;

    if (map == NULL)
    {
        return 1e9f;
    }

    for (i = 0U; i < map->count; i++)
    {
        d = wall_dist_to(&map->walls[i], x, y);
        if (d < best)
        {
            best = d;
        }
    }

    return best;
}

float PathGridMap_RayCast(const path_gridmap_t *map,
                          float ox, float oy,
                          float dx, float dy,
                          float max_range)
{
    uint8_t i;
    float best = max_range;
    float d;

    if (map == NULL)
    {
        return max_range;
    }

    for (i = 0U; i < map->count; i++)
    {
        d = wall_ray_cast(&map->walls[i], ox, oy, dx, dy, max_range);
        if (d < best)
        {
            best = d;
        }
    }

    return best;
}

/* 找到包含该点的墙(点必须确实在某墙内),沿最小穿透轴推出 */
static bool push_out_one(const path_gridmap_t *map, float *x, float *y,
                         float step_m)
{
    uint8_t i;
    const path_wall_t *w;
    float dx_left;
    float dx_right;
    float dy_bottom;
    float dy_top;
    float min_pen;
    float margin = step_m;

    for (i = 0U; i < map->count; i++)
    {
        w = &map->walls[i];
        if (!wall_contains(w, *x, *y))
        {
            continue;
        }

        dx_left = *x - w->xmin;
        dx_right = w->xmax - *x;
        dy_bottom = *y - w->ymin;
        dy_top = w->ymax - *y;
        min_pen = dx_left;
        if (dx_right < min_pen) { min_pen = dx_right; }
        if (dy_bottom < min_pen) { min_pen = dy_bottom; }
        if (dy_top < min_pen) { min_pen = dy_top; }

        /* 沿穿透最小的方向,推出到墙外再留一步余量 */
        if (min_pen == dx_left)
        {
            *x = w->xmin - margin;
        }
        else if (min_pen == dx_right)
        {
            *x = w->xmax + margin;
        }
        else if (min_pen == dy_bottom)
        {
            *y = w->ymin - margin;
        }
        else
        {
            *y = w->ymax + margin;
        }
        return true;
    }

    return false;
}

static bool PathGridMap_PushOut(const path_gridmap_t *map, float *x, float *y,
                         float step_m, uint8_t max_iters)
{
    uint8_t iters;

    for (iters = 0U; iters < max_iters; iters++)
    {
        if (!PathGridMap_Contains(map, *x, *y))
        {
            return true;
        }
        (void)push_out_one(map, x, y, step_m);
    }

    return !PathGridMap_Contains(map, *x, *y);
}

/* ------------------------------------------------------------------ */
float PathWrapAngle(float angle_rad)
{
    while (angle_rad > PATH_PI)
    {
        angle_rad -= 2.0f * PATH_PI;
    }
    while (angle_rad < -PATH_PI)
    {
        angle_rad += 2.0f * PATH_PI;
    }
    return angle_rad;
}

static void PathBodyToWorld(float vx_b, float vy_b, float yaw_user,
                     float *vx_w, float *vy_w)
{
    /* θ = yaw + π/2:yaw=0(朝 +y)时车体 +x 应指向世界 +y */
    float theta = yaw_user + (float)(PATH_PI / 2.0);
    float c = cosf(theta);
    float s = sinf(theta);

    *vx_w = vx_b * c - vy_b * s;
    *vy_w = vx_b * s + vy_b * c;
}

static void PathWorldToBody(float vx_w, float vy_w, float yaw_user,
                     float *vx_b, float *vy_b)
{
    float theta = yaw_user + (float)(PATH_PI / 2.0);
    float c = cosf(theta);
    float s = sinf(theta);

    *vx_b = vx_w * c + vy_w * s;
    *vy_b = -vx_w * s + vy_w * c;
}

static void PathWorldToChassis(float vx_w, float vy_w, float yaw_user,
                        float *vx_c, float *vy_c)
{
    /* 底盘系 x=右、y=前,恰为世界系旋转 yaw(车体系再转 -90°) */
    float c = cosf(yaw_user);
    float s = sinf(yaw_user);

    *vx_c = vx_w * c + vy_w * s;
    *vy_c = -vx_w * s + vy_w * c;
}

void PathChassisToWorld(float vx_c, float vy_c, float yaw_user,
                        float *vx_w, float *vy_w)
{
    float c = cosf(yaw_user);
    float s = sinf(yaw_user);

    *vx_w = vx_c * c - vy_c * s;
    *vy_w = vx_c * s + vy_c * c;
}

void PathLaserRay(float robot_x, float robot_y, float yaw_user,
                  float mount_body_x, float mount_body_y,
                  float dir_body_x, float dir_body_y,
                  float *ox, float *oy, float *dx, float *dy)
{
    /* 挂载点:车体系 -> 世界系(R(yaw+π/2)) */
    PathBodyToWorld(mount_body_x, mount_body_y, yaw_user, ox, oy);
    *ox += robot_x;
    *oy += robot_y;

    /* 朝向:同变换作用于单位方向向量 */
    PathBodyToWorld(dir_body_x, dir_body_y, yaw_user, dx, dy);
}

/* ================================================================
 * 模块:B 样条平滑 + 离墙外推 + 最小转弯半径整形
 * ================================================================ */

/* 前向声明:PathSpline_Finalize 在前,PathSpline_PushAwayFromWalls 定义在后 */
static uint16_t PathSpline_PushAwayFromWalls(path_point_t *points,
                                             uint16_t count,
                                             const path_gridmap_t *inflated_map);

/**
 ******************************************************************************
 * @file    path_spline.c
 * @brief   B 样条实现:de Boor 递推、导数、弧长、曲率、离墙外推
 ******************************************************************************
 */


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

static bool PathSpline_Build(const path_waypoint_t *waypoints, uint8_t n_wp,
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

static void PathSpline_Finalize(path_point_t *points, uint16_t count,
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


static uint16_t PathSpline_PushAwayFromWalls(path_point_t *points, uint16_t count,
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

/* ================================================================
 * 模块:离线速度剖面 + 期望激光查表 + CSV 导出
 * ================================================================ */
/**
 ******************************************************************************
 * @file    path_speed_profile.c
 * @brief   离线速度剖面生成 + 最近点查表 + CSV 串口输出
 ******************************************************************************
 */


static bool PathSpeedProfile_Build(path_point_t *points, uint16_t count,
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

static uint16_t PathSpeedProfile_Nearest(const path_point_t *points, uint16_t count,
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

/* ================================================================
 * 模块:IMU + 上位机位姿互补融合
 * ================================================================ */
/**
 ******************************************************************************
 * @file    path_fusion.c
 * @brief   IMU + 上位机位姿互补融合实现
 ******************************************************************************
 */




typedef struct
{
    float x;
    float y;
    float yaw;                 /* 用户约定 yaw:0 朝 +y,CCW 为正 */
    float zero_offset_deg_s;   /* 静止标定得到的附加陀螺零偏 */
    bool have_upper;
    uint32_t last_upper_ms;
    uint16_t calib_count;
    float calib_sum;

    /* xy 中值滤波(单帧跳变剔除,配合 30cm 门限使用) */
    float median_x[PATH_FUSION_MEDIAN_WIN];
    float median_y[PATH_FUSION_MEDIAN_WIN];
    uint8_t median_i;
    uint8_t median_fill;

    uint32_t xy_rejects;
    uint32_t yaw_rejects;
    uint32_t upper_frames;
} path_fusion_t;

static path_fusion_t fusion;

static float median3(const float *buf, uint8_t n)
{
    float a;
    float b;
    float c;
    float tmp;

    if (n == 0U)
    {
        return 0.0f;
    }
    a = buf[0];
    b = (n > 1U) ? buf[1] : a;
    c = (n > 2U) ? buf[2] : b;

    /* 3 数排序取中值 */
    if (a > b) { tmp = a; a = b; b = tmp; }
    if (b > c) { tmp = b; b = c; c = tmp; }
    if (a > b) { tmp = a; a = b; b = tmp; }
    return b;
}

static void PathFusion_Init(void)
{
    (void)memset(&fusion, 0, sizeof(fusion));
}

static bool PathFusion_CalibrateSample(float gyro_z_deg_s)
{
    if (fusion.calib_count < PATH_FUSION_CALIB_SAMPLES)
    {
        fusion.calib_sum += gyro_z_deg_s;
        fusion.calib_count++;
    }

    if (fusion.calib_count >= PATH_FUSION_CALIB_SAMPLES)
    {
        /* 附加零偏 = -均值(抵消 IMU 内置零偏修正后的残余误差) */
        fusion.zero_offset_deg_s =
            -(fusion.calib_sum / (float)fusion.calib_count);
        return true;
    }

    return false;
}

static void PathFusion_Predict(float gyro_z_deg_s, float dt_s)
{
    float rate_deg_s = (gyro_z_deg_s + fusion.zero_offset_deg_s) *
                       PATH_GYRO_SIGN;

    fusion.yaw += rate_deg_s * DEG2RAD * dt_s;
    fusion.yaw = PathWrapAngle(fusion.yaw);
}

static bool PathFusion_UpdateUpper(float x_m, float y_m, float yaw_rad,
                            uint32_t now_ms)
{
    float med_x;
    float med_y;
    float dx;
    float dy;
    float yaw_err_rad;

    fusion.upper_frames++;
    fusion.last_upper_ms = now_ms;

    /* xy 跳变门限(>30cm 整帧拒绝):用原始样本与融合值比较,
     * 通过后才进入中值窗口 —— 离群帧不会污染窗口,不会造成冻结。 */
    if (fusion.have_upper)
    {
        dx = x_m - fusion.x;
        dy = y_m - fusion.y;
        if (sqrtf(dx * dx + dy * dy) > PATH_FUSION_XY_GATE_M)
        {
            fusion.xy_rejects++;
            return false;
        }
    }

    /* 中值滤波:吸收 20cm 量级的单帧跳变(未超 30cm 门限的离群帧) */
    fusion.median_x[fusion.median_i] = x_m;
    fusion.median_y[fusion.median_i] = y_m;
    fusion.median_i = (uint8_t)((fusion.median_i + 1U) %
                                PATH_FUSION_MEDIAN_WIN);
    if (fusion.median_fill < PATH_FUSION_MEDIAN_WIN)
    {
        fusion.median_fill++;
    }
    med_x = median3(fusion.median_x, fusion.median_fill);
    med_y = median3(fusion.median_y, fusion.median_fill);

    /* yaw 门限(与积分预测差 >20° 拒绝 yaw 分量,xy 照常融合) */
    yaw_err_rad = fabsf(PathWrapAngle(yaw_rad - fusion.yaw));
    if (fusion.have_upper &&
        (yaw_err_rad > PATH_FUSION_YAW_GATE_DEG * DEG2RAD))
    {
        fusion.yaw_rejects++;
    }
    else
    {
        /* yaw 低通拉回 */
        fusion.yaw += PATH_FUSION_YAW_GAIN *
                      PathWrapAngle(yaw_rad - fusion.yaw);
    }

    /* xy 直接覆盖 */
    fusion.x = med_x;
    fusion.y = med_y;
    fusion.have_upper = true;

    return true;
}

static bool PathFusion_IsUpperLost(uint32_t now_ms)
{
    if (!fusion.have_upper)
    {
        /* 尚未收到过任何上位机数据:在等待起点阶段不算丢失 */
        return false;
    }
    return (uint32_t)(now_ms - fusion.last_upper_ms) >
           PATH_FUSION_UPPER_TIMEOUT_MS;
}

static void PathFusion_Get(float *x_m, float *y_m, float *yaw_rad)
{
    if (x_m != NULL)
    {
        *x_m = fusion.x;
    }
    if (y_m != NULL)
    {
        *y_m = fusion.y;
    }
    if (yaw_rad != NULL)
    {
        *yaw_rad = fusion.yaw;
    }
}

static void PathFusion_GetStats(uint32_t *xy_rejects, uint32_t *yaw_rejects,
                         uint32_t *upper_frames)
{
    if (xy_rejects != NULL)
    {
        *xy_rejects = fusion.xy_rejects;
    }
    if (yaw_rejects != NULL)
    {
        *yaw_rejects = fusion.yaw_rejects;
    }
    if (upper_frames != NULL)
    {
        *upper_frames = fusion.upper_frames;
    }
}

/* ================================================================
 * 模块:纯追踪前视目标
 * ================================================================ */
/**
 ******************************************************************************
 * @file    path_pure_pursuit.c
 * @brief   纯追踪实现
 ******************************************************************************
 */



static void PathPurePursuit_Find(const path_point_t *points, uint16_t count,
                          float x, float y, float v_ref, float kappa,
                          uint16_t i_near, uint16_t *i_target,
                          float *tx, float *ty)
{
    float lookahead = PATH_LD_MIN_M + PATH_LD_K_S * v_ref;
    float kappa_cap = PATH_LD_KAPPA_MAX_M /
                      sqrtf(fabsf(kappa) + 0.05f);
    uint16_t i;

    /* 急弯处缩短前视距离,抑制抄近道 */
    if (lookahead > kappa_cap)
    {
        lookahead = kappa_cap;
    }

    if ((points == NULL) || (i_target == NULL))
    {
        return;
    }
    if (i_near >= count)
    {
        i_near = (uint16_t)(count - 1U);
    }

    /* 从最近点起向后(轨迹方向)找第一个距离 >= Ld 的点 */
    for (i = i_near; i < count; i++)
    {
        float dx = points[i].x_m - x;
        float dy = points[i].y_m - y;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist >= lookahead)
        {
            *i_target = i;
            if (tx != NULL) { *tx = points[i].x_m; }
            if (ty != NULL) { *ty = points[i].y_m; }
            return;
        }
    }

    /* 剩余段不足一个前视距离:直接取终点 */
    *i_target = (uint16_t)(count - 1U);
    if (tx != NULL) { *tx = points[count - 1U].x_m; }
    if (ty != NULL) { *ty = points[count - 1U].y_m; }
}

/* ================================================================
 * 模块:航向锁定
 * ================================================================ */
/**
 ******************************************************************************
 * @file    path_yaw_lock.c
 * @brief   航向锁定实现
 ******************************************************************************
 */




static float PathYawLock_Step(float yaw_rad, float speed_ms)
{
    float err_deg = PathWrapAngle(PATH_YAW_TARGET_RAD - yaw_rad) * RAD2DEG;
    float kp;
    float w_max;
    float w;

    /* 死区 ±1°:高速直线时避免抖动 */
    if (fabsf(err_deg) <= PATH_YAW_DEADZONE_DEG)
    {
        return 0.0f;
    }

    kp = (fabsf(err_deg) <= PATH_YAW_KP_THRES_DEG) ?
         PATH_YAW_KP_SMALL : PATH_YAW_KP_LARGE;
    w = kp * err_deg * DEG2RAD;

    /* 速度自适应限幅 */
    w_max = PATH_W_BASE_RAD_S - PATH_W_SLOPE * speed_ms;
    if (w_max < PATH_W_MIN_RAD_S)
    {
        w_max = PATH_W_MIN_RAD_S;
    }
    if (w > w_max)
    {
        w = w_max;
    }
    if (w < -w_max)
    {
        w = -w_max;
    }

    return w;
}

/* ================================================================
 * 模块:在线跟踪总控(对接仓库已有模块)
 * ================================================================ */
/**
 ******************************************************************************
 * @file    path_runner.c
 * @brief   在线跟踪总控实现(离线剖面查表 + 纯追踪 + 激光兜底 + 航向锁)
 ******************************************************************************
 */


/* 仓库已有模块(全部真实外设数据) */


#if PATH_DEBUG
#endif

/* ------------------------------------------------------------------ */
static path_point_t trajectory[PATH_SPLINE_SAMPLES];
static uint16_t trajectory_count;
static path_gridmap_t real_map;
static path_gridmap_t inflated_map;
static path_waypoint_t waypoints[PATH_WAYPOINT_COUNT] =
{
    PATH_WAYPOINTS_TABLE
};

static path_state_t state = PATH_STATE_INIT;
static path_reason_t reason = PATH_REASON_BOOT;
static path_debug_t debug;

static uint32_t state_start_ms;
static uint32_t run_start_ms;
static uint32_t last_step_ms;
#if PATH_DEBUG && defined(PATH_DEBUG_UART_HANDLE)
static uint32_t last_debug_ms;
#endif
static uint16_t last_i_near;
static uint32_t last_pc_frame_count;   /* 上位机帧去重:每帧只融合一次 */
static float last_cmd_vx;
static float last_cmd_vy;
static float last_cmd_w;

/* ------------------------------------------------------------------ */
static void runner_stop(path_reason_t why)
{
    state = PATH_STATE_STOPPED;
    reason = why;
    last_cmd_vx = 0.0f;
    last_cmd_vy = 0.0f;
    last_cmd_w = 0.0f;
    Chassis_StopAll();
}

/* 数值健康检查:NaN/Inf/量级异常一律判非法(v == v 可移植地判 NaN) */
static bool num_ok(float v)
{
    return (v == v) && (fabsf(v) < 1e6f);
}

/* 每个控制周期读一次真实传感器并做融合 */
static bool runner_read_and_fuse(uint32_t now_ms, float dt_s,
                                 imu_data_t *imu,
                                 float *laser_f_m, bool *laser_f_ok,
                                 float *laser_l_m, bool *laser_l_ok)
{
    bool imu_ok;
    pc_position_t upper;

    imu_ok = ImuMain_GetData(imu) &&
             imu->online && imu->yaw_valid && imu->gyro_valid &&
             (imu->state == IMU_STATE_READY);

    /* DT35 前/左激光(串口帧解析值,单位 cm -> m) */
    *laser_f_m = (float)dt35_link[SENSOR_LINK_F_INDEX].distance_cm * 0.01f;
    *laser_f_ok = (dt35_link[SENSOR_LINK_F_INDEX].online != 0U) &&
                  ((uint32_t)(now_ms -
                   dt35_link[SENSOR_LINK_F_INDEX].last_rx_ms) <
                   PATH_LASER_TIMEOUT_MS);
    *laser_l_m = (float)dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm * 0.01f;
    *laser_l_ok = (dt35_link[SENSOR_LINK_L_B_INDEX].online != 0U) &&
                  ((uint32_t)(now_ms -
                   dt35_link[SENSOR_LINK_L_B_INDEX].last_rx_ms) <
                   PATH_LASER_TIMEOUT_MS);

    if (imu_ok)
    {
        PathFusion_Predict(imu->gyro_z_deg_s, dt_s);
    }

    /* 上位机位姿(0x11 位置帧,field_w 为 yaw_rad;小电脑侧已修复,
     * 不再传四元数 W 分量)。
     * 去重:pc_link 保存最近一帧,本函数 5ms 调一次,同一帧会被读多次,
     * 若不去重,中值滤波窗口会被同一个跳变值填满而失去滤波意义,
     * 因此只在"位置帧序号变化"即真正收到新帧时才融合一次。 */
    {
        uint32_t pos_seq = PcLink_GetPositionSeq();
        if (PcLink_GetPosition(&upper) &&
            ((upper.flags & PC_LINK_FLAG_FIELD_VALID) != 0U) &&
            (pos_seq != last_pc_frame_count))
        {
            last_pc_frame_count = pos_seq;
            (void)PathFusion_UpdateUpper(upper.field_x_m, upper.field_y_m,
                                         upper.field_w, now_ms);
        }
    }

    return imu_ok;
}

/* ------------------------------------------------------------------ */
static void runner_step(uint32_t now_ms, float dt_s)
{
    imu_data_t imu;
    float laser_f;
    float laser_l;
    bool laser_f_ok;
    bool laser_l_ok;
    bool imu_ok;
    float fx;
    float fy;
    float fyaw;
    float lf;
    float v_laser;
    float v_ref;
    float v_used;
    float exp_l;
    float tx;
    float ty;
    float L;
    float vx_w;
    float vy_w;
    float vx_c;
    float vy_c;
    float w_cmd;
    float dv_max;
    float dw_max;
    uint16_t i_near;
    uint16_t i_target;
    int16_t rpm_x;
    int16_t rpm_y;
    int16_t z;

    imu_ok = runner_read_and_fuse(now_ms, dt_s, &imu,
                                  &laser_f, &laser_f_ok,
                                  &laser_l, &laser_l_ok);
    PathFusion_Get(&fx, &fy, &fyaw);

    /* ---- 安全检查(顺序即优先级) ---- */
    if (PathFusion_IsUpperLost(now_ms))
    {
        runner_stop(PATH_REASON_STOP_UPPER_LOST);
        return;
    }
    if (!imu_ok)
    {
        runner_stop(PATH_REASON_STOP_IMU_LOST);
        return;
    }
    if ((PATH_STOP_ON_LASER_LOSS != 0U) && !laser_f_ok)
    {
        runner_stop(PATH_REASON_STOP_LASER_LOST);
        return;
    }
    if ((uint32_t)(now_ms - run_start_ms) > PATH_MAX_RUN_MS)
    {
        runner_stop(PATH_REASON_STOP_TIMEOUT);
        return;
    }

    /* ---- 到达判定(只用融合位姿) ---- */
    {
        float dx = fx - PATH_GOAL_X_M;
        float dy = fy - PATH_GOAL_Y_M;
        if (sqrtf(dx * dx + dy * dy) <= PATH_ARRIVE_TOL_M)
        {
            state = PATH_STATE_ARRIVED;
            reason = PATH_REASON_ARRIVED;
            last_cmd_vx = 0.0f;
            last_cmd_vy = 0.0f;
            last_cmd_w = 0.0f;
            Chassis_StopAll();
            return;
        }
    }

    /* ---- 查表(只读,不修改离线剖面) ---- */
    i_near = PathSpeedProfile_Nearest(trajectory, trajectory_count,
                                      fx, fy, last_i_near);
    last_i_near = i_near;
    v_ref = trajectory[i_near].v_ref;
    exp_l = trajectory[i_near].exp_laser_left_m;

    /* ---- 纯追踪:目标点(先于激光兜底计算,激光停车时需要目标方向) ---- */
    PathPurePursuit_Find(trajectory, trajectory_count, fx, fy, v_ref,
                         trajectory[i_near].kappa,
                         i_near, &i_target, &tx, &ty);
    L = sqrtf((tx - fx) * (tx - fx) + (ty - fy) * (ty - fy));
    if (L <= 1e-3f)
    {
        L = 1e-3f;
    }

    /* ---- 前激光兜底 ---- */
    lf = laser_f;
    if (lf > PATH_LASER_MAX_RANGE_M)
    {
        lf = PATH_LASER_MAX_RANGE_M;
    }
    if (lf <= PATH_LASER_STOP_DIST_M)
    {
        /* 强制 vx=0:保留目标方向的横向分量做脱困平移(全向轮可侧移)。
         * 若目标几乎在正前方(横向分量过小)则原地等待,避免顶墙。 */
        float dir_bx;
        float dir_by;
        float lat_sign;

        PathWorldToBody((tx - fx) / L, (ty - fy) / L, fyaw,
                        &dir_bx, &dir_by);
        lat_sign = (dir_by > 0.05f) ? 1.0f :
                   ((dir_by < -0.05f) ? -1.0f : 0.0f);
        PathBodyToWorld(0.0f, lat_sign * PATH_LASER_STOP_LAT_MS, fyaw,
                        &vx_w, &vy_w);
        v_used = 0.0f;
        reason = PATH_REASON_STOP_LASER_FRONT;
    }
    else
    {
        v_laser = sqrtf(2.0f * PATH_A_LON_BRAKE *
                        (lf - PATH_LASER_STOP_DIST_M));
        v_used = (v_ref < v_laser) ? v_ref : v_laser;
        vx_w = v_used * (tx - fx) / L;
        vy_w = v_used * (ty - fy) / L;
        reason = (v_laser < (v_ref - 1e-3f)) ?
                 PATH_REASON_LASER_SLOW : PATH_REASON_RUN;
    }

    /* ---- 世界系 -> 底盘系(右/前),直接对应 Chassis_SetVelocity ---- */
    PathWorldToChassis(vx_w, vy_w, fyaw, &vx_c, &vy_c);

    /* ---- 左激光横向微调(err = laser_left - expected_left) ---- */
    if ((v_used > 0.0f) && laser_l_ok)
    {
        float err = laser_l - exp_l;
        float trim = PATH_LAT_TRIM_SIGN * PATH_LAT_TRIM_KP * err;
        if (trim > PATH_LAT_TRIM_MAX_MS)
        {
            trim = PATH_LAT_TRIM_MAX_MS;
        }
        if (trim < -PATH_LAT_TRIM_MAX_MS)
        {
            trim = -PATH_LAT_TRIM_MAX_MS;
        }
        /* 左墙太近:强制向右离开(底盘 +x = 向右) */
        if (laser_l < PATH_LAT_SAFE_M)
        {
            trim = PATH_LAT_TRIM_MAX_MS;
        }
        vx_c += trim;
    }

    /* ---- 航向锁 ---- */
    w_cmd = PathYawLock_Step(fyaw, v_used);

    /* ---- slew-rate 限幅(每周期变化不超过 max_accel * dt) ---- */
    dv_max = PATH_SLEW_XY_ACCEL_MS2 * dt_s;
    dw_max = PATH_SLEW_W_ACCEL_RADS2 * dt_s;
    if ((vx_c - last_cmd_vx) > dv_max) { vx_c = last_cmd_vx + dv_max; }
    if ((vx_c - last_cmd_vx) < -dv_max) { vx_c = last_cmd_vx - dv_max; }
    if ((vy_c - last_cmd_vy) > dv_max) { vy_c = last_cmd_vy + dv_max; }
    if ((vy_c - last_cmd_vy) < -dv_max) { vy_c = last_cmd_vy - dv_max; }
    if ((w_cmd - last_cmd_w) > dw_max) { w_cmd = last_cmd_w + dw_max; }
    if ((w_cmd - last_cmd_w) < -dw_max) { w_cmd = last_cmd_w - dw_max; }
    last_cmd_vx = vx_c;
    last_cmd_vy = vy_c;
    last_cmd_w = w_cmd;

    /* ---- 数值防护:任何环节产生 NaN/Inf 立即安全停车 ---- */
    if (!num_ok(vx_c) || !num_ok(vy_c) || !num_ok(w_cmd) ||
        !num_ok(v_ref) || !num_ok(v_used))
    {
        runner_stop(PATH_REASON_STOP_NUMERIC);
        return;
    }

    /* ---- 输出:底盘系(右/前) -> Chassis_SetVelocity(vx=右, vy=前, z=CCW) ---- */
    rpm_x = (int16_t)roundf(vx_c * PATH_RPM_PER_M_S);
    rpm_y = (int16_t)roundf(vy_c * PATH_RPM_PER_M_S);
    z = (int16_t)roundf(w_cmd * PATH_Z_PER_RAD_S);
    (void)Chassis_SetVelocity(rpm_x, rpm_y, z);

    /* ---- 调试信息 ---- */
    debug.i_near = i_near;
    debug.i_target = i_target;
    debug.v_ref = v_ref;
    debug.v_used = v_used;
    debug.laser_f_m = laser_f;
    debug.laser_l_m = laser_l;
    debug.exp_laser_l_m = exp_l;
    debug.fused_x = fx;
    debug.fused_y = fy;
    debug.fused_yaw_rad = fyaw;
    debug.cmd_vx_ch = vx_c;
    debug.cmd_vy_ch = vy_c;
    debug.cmd_w = w_cmd;
    debug.run_ms = now_ms - run_start_ms;
}

/* ------------------------------------------------------------------ */
void PathRunner_Init(void)
{
    (void)memset(&debug, 0, sizeof(debug));
    state = PATH_STATE_INIT;
    reason = PATH_REASON_BOOT;
    state_start_ms = HAL_GetTick();
    last_i_near = 0U;
    last_cmd_vx = 0.0f;
    last_cmd_vy = 0.0f;
    last_cmd_w = 0.0f;
    PathFusion_Init();
}

void PathRunner_Run(void)
{
    uint32_t now_ms = HAL_GetTick();
    float dt_s;

    /* 控制周期分频(commTask 1ms 调用) */
    if ((uint32_t)(now_ms - last_step_ms) < PATH_CONTROL_PERIOD_MS)
    {
        return;
    }
    dt_s = (float)(now_ms - last_step_ms) / 1000.0f;
    if (dt_s > 0.02f)
    {
        dt_s = 0.02f;
    }
    last_step_ms = now_ms;

    switch (state)
    {
    case PATH_STATE_INIT:
        /* 关闭 IMU 模块自带航向保持,避免它覆盖本模块的 z 指令 */
        ImuMain_EnableYawHold(false);
        state = PATH_STATE_CALIB;
        state_start_ms = now_ms;
        reason = PATH_REASON_CALIB;
        break;

    case PATH_STATE_CALIB:
    {
        imu_data_t imu;
        ImuMain_EnableYawHold(false);
        Chassis_StopAll();

        if (ImuMain_GetData(&imu) && imu.online &&
            imu.gyro_valid && (imu.state == IMU_STATE_READY))
        {
            if (PathFusion_CalibrateSample(imu.gyro_z_deg_s))
            {
                state = PATH_STATE_WAIT_START;
                state_start_ms = now_ms;
                reason = PATH_REASON_WAIT_START;
            }
        }
        /* IMU 异常兜底:标定超时也继续(零偏按 0 处理) */
        if ((uint32_t)(now_ms - state_start_ms) > 5000U)
        {
            state = PATH_STATE_WAIT_START;
            state_start_ms = now_ms;
            reason = PATH_REASON_WAIT_START;
        }
        break;
    }

    case PATH_STATE_WAIT_START:
    {
        imu_data_t imu;
        float laser_f;
        float laser_l;
        bool lf_ok;
        bool ll_ok;

        Chassis_StopAll();
        (void)runner_read_and_fuse(now_ms, dt_s, &imu,
                                   &laser_f, &lf_ok, &laser_l, &ll_ok);

        /* 起点由上电位姿覆盖(与 yaml 起点差 1m 以内才接受) */
        {
            pc_position_t upper;
            if (PcLink_GetPosition(&upper) &&
                ((upper.flags & PC_LINK_FLAG_FIELD_VALID) != 0U))
            {
                float dx = upper.field_x_m - waypoints[0].x_m;
                float dy = upper.field_y_m - waypoints[0].y_m;
                if (sqrtf(dx * dx + dy * dy) <= PATH_START_OVERRIDE_MAX_M)
                {
                    waypoints[0].x_m = upper.field_x_m;
                    waypoints[0].y_m = upper.field_y_m;
                }
                state = PATH_STATE_BUILD;
                state_start_ms = now_ms;
                break;
            }
        }
        if ((uint32_t)(now_ms - state_start_ms) > PATH_WAIT_START_MS)
        {
            /* 超时:用 yaml 占位起点 */
            state = PATH_STATE_BUILD;
            state_start_ms = now_ms;
        }
        break;
    }

    case PATH_STATE_BUILD:
    {
        PathGridMap_BuildReal(&real_map);
        PathGridMap_BuildInflated(&inflated_map);

        if (!PathSpline_Build(waypoints, PATH_WAYPOINT_COUNT,
                              trajectory, PATH_SPLINE_SAMPLES,
                              &trajectory_count))
        {
            runner_stop(PATH_REASON_STOP_BUILD);
            break;
        }
        /* 推离 + 平滑迭代,并重算弧长/曲率(避免点状外推撕裂拐角) */
        PathSpline_Finalize(trajectory, trajectory_count, &inflated_map);
        if (!PathSpeedProfile_Build(trajectory, trajectory_count, &real_map))
        {
            runner_stop(PATH_REASON_STOP_BUILD);
            break;
        }

        state = PATH_STATE_RUN;
        reason = PATH_REASON_RUN;
        run_start_ms = now_ms;
        last_i_near = 0U;
        break;
    }

    case PATH_STATE_RUN:
        runner_step(now_ms, dt_s);
        break;

    case PATH_STATE_ARRIVED:
    case PATH_STATE_STOPPED:
    default:
        Chassis_StopAll();
        break;
    }

    /* 调试信息公共部分 */
    debug.state = state;
    debug.reason = reason;
    PathFusion_GetStats(&debug.fusion_xy_rejects, &debug.fusion_yaw_rejects,
                        &debug.upper_frames);
    PcLink_GetStats(&debug.pc_frames, &debug.crc_errors);

    /* 回传小电脑状态帧(55 AA 20 | state | error):
     * state = 规划器状态机;error = 停止原因(STOPPED 时),正常为 0。
     * 上位机竞争网关用该帧判断控制器在线(500ms 未收到判离线)。 */
    PcLink_SetStatus((uint8_t)state,
                     (state == PATH_STATE_STOPPED) ? (uint8_t)reason : 0U);

#if PATH_DEBUG && defined(PATH_DEBUG_UART_HANDLE)
    if ((uint32_t)(now_ms - last_debug_ms) >= PATH_DEBUG_PERIOD_MS)
    {
        last_debug_ms = now_ms;
        PathRunner_DebugDump(&PATH_DEBUG_UART_HANDLE);
    }
#endif
}

void PathRunner_GetDebug(path_debug_t *out)
{
    if (out != NULL)
    {
        *out = debug;
    }
}

const path_point_t *PathRunner_GetTrajectory(uint16_t *count)
{
    /* 只要离线轨迹构建完成就返回(BUILD/RUN/ARRIVED/STOPPED 均可) */
    if (trajectory_count > 0U)
    {
        if (count != NULL)
        {
            *count = trajectory_count;
        }
        return trajectory;
    }
    if (count != NULL)
    {
        *count = 0U;
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
#if PATH_DEBUG && defined(PATH_DEBUG_UART_HANDLE)


static void dbg_puts(UART_HandleTypeDef *uart, const char *s)
{
    (void)HAL_UART_Transmit(uart, (uint8_t *)s, (uint16_t)strlen(s), 10U);
}

static void dbg_putf(UART_HandleTypeDef *uart, float v)
{
    char buf[24];
    uint8_t pos = 0U;
    char rev[12];
    uint8_t rpos = 0U;
    int32_t ip;
    uint32_t frac;

    if (v < 0.0f) { buf[pos++] = '-'; v = -v; }
    ip = (int32_t)v;
    frac = (uint32_t)((v - (float)ip) * 100.0f + 0.5f);
    if (frac >= 100U) { ip++; frac = 0U; }
    if (ip == 0) { buf[pos++] = '0'; }
    else
    {
        while (ip > 0) { rev[rpos++] = (char)('0' + (ip % 10)); ip /= 10; }
        while (rpos > 0U) { buf[pos++] = rev[--rpos]; }
    }
    buf[pos++] = '.';
    buf[pos++] = (char)('0' + (frac / 10U) % 10U);
    buf[pos++] = (char)('0' + frac % 10U);
    buf[pos] = '\0';
    dbg_puts(uart, buf);
}

/* 每 400ms 一行:
 * t_ms,state,reason,x,y,yaw,vx,vy,w,v_ref,v_used,laserF,laserL,crc */
void PathRunner_DebugDump(UART_HandleTypeDef *uart)
{
    dbg_puts(uart, "t=");
    dbg_putf(uart, (float)debug.run_ms);
    dbg_puts(uart, " st=");
    dbg_putf(uart, (float)debug.state);
    dbg_puts(uart, " rsn=");
    dbg_putf(uart, (float)debug.reason);
    dbg_puts(uart, " x=");
    dbg_putf(uart, debug.fused_x);
    dbg_puts(uart, " y=");
    dbg_putf(uart, debug.fused_y);
    dbg_puts(uart, " yaw=");
    dbg_putf(uart, debug.fused_yaw_rad * 57.29578f);
    dbg_puts(uart, " cmd=");
    dbg_putf(uart, debug.cmd_vx_ch);
    dbg_puts(uart, ",");
    dbg_putf(uart, debug.cmd_vy_ch);
    dbg_puts(uart, ",");
    dbg_putf(uart, debug.cmd_w);
    dbg_puts(uart, " vref=");
    dbg_putf(uart, debug.v_ref);
    dbg_puts(uart, " vused=");
    dbg_putf(uart, debug.v_used);
    dbg_puts(uart, " lf=");
    dbg_putf(uart, debug.laser_f_m);
    dbg_puts(uart, " ll=");
    dbg_putf(uart, debug.laser_l_m);
    dbg_puts(uart, " crc=");
    dbg_putf(uart, (float)debug.crc_errors);
    dbg_puts(uart, "\r\n");
}

#endif /* PATH_DEBUG && PATH_DEBUG_UART_HANDLE */
