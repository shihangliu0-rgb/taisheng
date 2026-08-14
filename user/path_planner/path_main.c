/* path_main.c - 跑曲线实现(流程/算法见 README.md) */
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

/* 几何工具 */

/* 墙表 */
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

/* 硬膨胀 */
static path_wall_t hard_walls[PATH_WALL_COUNT];

void PathGridMap_BuildHardInflated(path_gridmap_t *map)
{
    uint8_t i;
    float hx = 0.5f * PATH_ROBOT_WIDTH_M + PATH_HARD_MARGIN_M;
    float hy = 0.5f * PATH_ROBOT_LENGTH_M + PATH_HARD_MARGIN_M;

    if (map == NULL)
    {
        return;
    }
    for (i = 0U; i < PATH_WALL_COUNT; i++)
    {
        hard_walls[i].xmin = real_walls[i].xmin - hx;
        hard_walls[i].ymin = real_walls[i].ymin - hy;
        hard_walls[i].xmax = real_walls[i].xmax + hx;
        hard_walls[i].ymax = real_walls[i].ymax + hy;
    }
    map->walls = hard_walls;
    map->count = PATH_WALL_COUNT;
}

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

/* AABB 射线相交(Slab 法) */
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
        /* 射线平行于 y 轴方向分量 */
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

/* 找包含该点的墙 */
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
    /* θ = yaw + π/2 */
    float theta = yaw_user + (float)(PATH_PI / 2.0);
    float c = cosf(theta);
    float s = sinf(theta);

    *vx_w = vx_b * c - vy_b * s;
    *vy_w = vx_b * s + vy_b * c;
}

static void PathWorldToChassis(float vx_w, float vy_w, float yaw_user,
                        float *vx_c, float *vy_c)
{

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
    /* 挂载点 */
    PathBodyToWorld(mount_body_x, mount_body_y, yaw_user, ox, oy);
    *ox += robot_x;
    *oy += robot_y;

    /* 朝向:同变换作用于单位方向向量 */
    PathBodyToWorld(dir_body_x, dir_body_y, yaw_user, dx, dy);
}

/* B 样条平滑 */

static uint16_t PathSpline_PushAwayFromWalls(path_point_t *points,
                                             uint16_t count,
                                             const path_gridmap_t *inflated_map);

#define MAX_CTRL        (PATH_WAYPOINT_COUNT + 1U)  /* 控制点上限 */
#define MAX_KNOTS       (MAX_CTRL + PATH_SPLINE_DEGREE + 1U)
#define MAX_SAMPLES     PATH_SPLINE_SAMPLES

/* 前向声明:Build 末尾调用 */
static void update_arc_curvature(path_point_t *points, uint16_t count);

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

    /* 找到 span k:u 在 [U_k, U_{k+1} */
    k = degree;
    while ((k + 1U < n_ctrl + degree) && (u >= knots[k + 1U]))
    {
        k++;
    }
    if (k >= n_ctrl)
    {
        k = (uint8_t)(n_ctrl - 1U);
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

        /* 切线 */
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

    /* 曲率 κ = dθ/ds */
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

static void smooth_xy(path_point_t *points, uint16_t count, uint8_t passes)
{
    uint16_t sample;
    uint8_t pass;

    if ((points == NULL) || (count < 3U))
    {
        return;
    }

    /* 拉普拉斯平滑 */
    for (pass = 0U; pass < passes; pass++)
    {
        float prev_x = points[0].x_m;
        float prev_y = points[0].y_m;

        for (sample = 1U; sample + 1U < count; sample++)
        {
            float cur_x = points[sample].x_m;
            float cur_y = points[sample].y_m;

            points[sample].x_m = 0.5f * cur_x +
                                 0.25f * (prev_x + points[sample + 1U].x_m);
            points[sample].y_m = 0.5f * cur_y +
                                 0.25f * (prev_y + points[sample + 1U].y_m);
            prev_x = cur_x;
            prev_y = cur_y;
        }
    }
}

/* 曲率整形:把曲率超限(转弯半径过小)的点沿弯道外侧推开 */
static void PathSpline_Finalize(path_point_t *points, uint16_t count,
                         const path_gridmap_t *inflated_map)
{
    uint8_t round;

    if ((points == NULL) || (inflated_map == NULL))
    {
        return;
    }

    /* 推离(硬约束) */
    for (round = 0U; round < PATH_PUSH_SMOOTH_ROUNDS; round++)
    {
        (void)PathSpline_PushAwayFromWalls(points, count, inflated_map);
        smooth_xy(points, count, 1U);
    }
    (void)PathSpline_PushAwayFromWalls(points, count, inflated_map);
    /* 平滑轮数为 0 时只推离:密集弧点路点下样条紧贴圆弧 */
    if (PATH_PUSH_SMOOTH_ROUNDS > 0U)
    {
        smooth_xy(points, count, 1U);
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

/* 离线速度剖面 */

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

    /* 3. 反向扫描 */
    points[count - 1U].v_ref = PATH_V_GOAL_MS;
    for (i = count - 1U; i > 0U; i--)
    {
        ds = points[i].s_m - points[i - 1U].s_m;
        if (ds < 1e-6f)
        {
            continue;
        }
        float v_brake = sqrtf(points[i].v_ref * points[i].v_ref +
                              2.0f * PATH_PROFILE_BRAKE_MS2 * ds);
        if (v_brake < points[i - 1U].v_ref)
        {
            points[i - 1U].v_ref = v_brake;
        }
    }

    /* 最低巡航速度 */
    for (i = 0U; i + 1U < count; i++)
    {
        if (points[i].v_ref < PATH_V_MIN_MS)
        {
            points[i].v_ref = PATH_V_MIN_MS;
        }
    }

    /* 曲率封顶(0.8m 前视窗) */
    for (i = 0U; i < count; i++)
    {
        float k_abs = fabsf(points[i].kappa);
        float v_curve;
        uint16_t j;

        for (j = i + 1U; j < count; j++)
        {
            float kk;

            if ((points[j].s_m - points[i].s_m) > PATH_CURV_LOOKAHEAD_M)
            {
                break;
            }
            kk = fabsf(points[j].kappa);
            if (kk > k_abs)
            {
                k_abs = kk;
            }
        }
        if (k_abs < PATH_KAPPA_MIN)
        {
            k_abs = PATH_KAPPA_MIN;
        }
        v_curve = sqrtf(PATH_A_LAT_MAX / k_abs);
        if (points[i].v_ref > v_curve)
        {
            points[i].v_ref = v_curve;
        }
    }

    /* 4. 期望激光表 */
    for (i = 0U; i < count; i++)
    {

        PathLaserRay(points[i].x_m, points[i].y_m, PATH_YAW_TARGET_RAD,
                     PATH_LASER_FRONT_X_M, PATH_LASER_FRONT_Y_M,
                     1.0f, 0.0f, &ox, &oy, &dx, &dy);
        points[i].exp_laser_front_m = PathGridMap_RayCast(real_map, ox, oy,
                                                          dx, dy,
                                                          PATH_EXPECTED_FRONT_RANGE_M);

        PathLaserRay(points[i].x_m, points[i].y_m, PATH_YAW_TARGET_RAD,
                     PATH_LASER_LEFT_X_M, PATH_LASER_LEFT_Y_M,
                     0.0f, 1.0f, &ox, &oy, &dx, &dy);
        points[i].exp_laser_left_m = PathGridMap_RayCast(real_map, ox, oy,
                                                         dx, dy,
                                                         PATH_EXPECTED_SIDE_RANGE_M);
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
    uint16_t start;

    if (hint >= count)
    {
        hint = (uint16_t)(count - 1U);
    }

    /* 允许小幅回退:过冲后能找回最近点 */
    start = (hint > PATH_SEARCH_BACK_WINDOW) ?
            (uint16_t)(hint - PATH_SEARCH_BACK_WINDOW) : 0U;
    end = hint + PATH_SEARCH_WINDOW;
    if (end >= count)
    {
        end = (uint16_t)(count - 1U);
    }

    best_i = hint;
    for (i = start; i <= end; i++)
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

    /* 最近点已到窗口末尾且尚未到轨迹尾 */
    if ((best_i >= end) && (end < count - 1U))
    {
        best_i = end;
    }

    return best_i;
}

static void uart_puts(UART_HandleTypeDef *uart, const char *s)
{
    (void)HAL_UART_Transmit(uart, (uint8_t *)s, (uint16_t)strlen(s), 10U);
}

/* 手写浮点格式化 */
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

    uart_puts(uart, "index,s_m,x_m,y_m,kappa,v_ref,exp_laser_front_m,exp_laser_left_m\r\n");
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

/* IMU */

typedef struct
{
    float x;
    float y;
    float yaw;                 /* 用户约定 yaw:0 朝 +y,CCW 为正 */
    float zero_offset_deg_s;   /* 静止标定得到的附加陀螺零偏 */
    bool have_upper;
    uint32_t last_upper_ms;    /* 链路存活:任何 CRC 有效位置帧都刷新 */
    uint32_t last_data_ms;     /* 数据可用:通过门限的帧才刷新 */
    uint16_t calib_count;
    float calib_sum;

    uint32_t xy_rejects;
    uint32_t yaw_rejects;
    uint32_t upper_frames;
    uint16_t yaw_rej_run;       /* 连续一致偏出门限的帧数(P0-1 重捕获) */
    int8_t   yaw_rej_sign;
} path_fusion_t;

static path_fusion_t fusion;

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
        /* 附加零偏 = -均值 */
        fusion.zero_offset_deg_s =
            -(fusion.calib_sum / (float)fusion.calib_count);
        return true;
    }

    return false;
}

static void PathFusion_Predict(float gyro_z_deg_s, float dt_s,
                            float vx_ch_ms, float vy_ch_ms)
{
    float rate_deg_s = (gyro_z_deg_s + fusion.zero_offset_deg_s) *
                       PATH_GYRO_SIGN;
    float c;
    float s;
    float vx_w;
    float vy_w;

    /* yaw 积分(陀螺) */
    fusion.yaw += rate_deg_s * DEG2RAD * dt_s;
    fusion.yaw = PathWrapAngle(fusion.yaw);

    /* 位置前馈 */
    c = cosf(fusion.yaw);
    s = sinf(fusion.yaw);
    vx_w = vx_ch_ms * c - vy_ch_ms * s;
    vy_w = vx_ch_ms * s + vy_ch_ms * c;
    fusion.x += vx_w * dt_s;
    fusion.y += vy_w * dt_s;
}

static bool PathFusion_UpdateUpper(float x_m, float y_m, float yaw_rad,
                            uint32_t now_ms)
{
    float dx;
    float dy;

    /* 数值/场地/单位校验:非法帧不参与融合。
     * yaw 量级门(|yaw| <= 2pi + 0.1):挡住"把度当 rad"的单位混用
     * (如 yaw=30 度被当 30rad,会在 20 度门限连续拒绝后被重捕获
     * 强制接受,yaw-lock 直接发疯——单位错误是 90% 玄学的来源) */
    if ((x_m != x_m) || (y_m != y_m) || (yaw_rad != yaw_rad) ||
        (fabsf(yaw_rad) > (2.0f * PATH_PI + 0.1f)) ||
        (x_m < PATH_POSE_X_MIN_M) || (x_m > PATH_POSE_X_MAX_M) ||
        (y_m < PATH_POSE_Y_MIN_M) || (y_m > PATH_POSE_Y_MAX_M))
    {
        fusion.xy_rejects++;
        return false;
    }

    /* xy 门限 */
    if (fusion.have_upper &&
        ((uint32_t)(now_ms - fusion.last_data_ms) > PATH_FUSION_REACQ_MS))
    {
        /* 强制重捕获:直接落入下方覆盖逻辑 */
    }
    else if (fusion.have_upper)
    {
        dx = x_m - fusion.x;
        dy = y_m - fusion.y;
        if (sqrtf(dx * dx + dy * dy) > PATH_FUSION_XY_GATE_M)
        {
            fusion.xy_rejects++;
            return false;   /* 被拒帧不更新 last_upper_ms */
        }
    }

    /* yaw:首帧或 REACQ(数据过旧)直接全量覆盖 */
    if (!fusion.have_upper ||
        ((uint32_t)(now_ms - fusion.last_data_ms) > PATH_FUSION_REACQ_MS))
    {
        fusion.yaw = yaw_rad;
        fusion.yaw_rej_run = 0U;
    }
    else
    {
        float yaw_err = PathWrapAngle(yaw_rad - fusion.yaw);
        if (fabsf(yaw_err) > PATH_FUSION_YAW_GATE_DEG * DEG2RAD)
        {
            int8_t sign = (yaw_err >= 0.0f) ? 1 : -1;
            fusion.yaw_rejects++;
            /* P0-1:连续一致方向偏出门限(打滑/侧撞后测量恒偏),
             * 计数达标强制整帧覆盖,不再永久锁死 */
            if ((fusion.yaw_rej_run > 0U) &&
                (sign == fusion.yaw_rej_sign))
            {
                fusion.yaw_rej_run++;
            }
            else
            {
                fusion.yaw_rej_run = 1U;
                fusion.yaw_rej_sign = sign;
            }
            if (fusion.yaw_rej_run >= PATH_FUSION_YAW_REACQ_N)
            {
                fusion.yaw = yaw_rad;
                fusion.yaw_rej_run = 0U;
            }
        }
        else
        {
            /* yaw 低通拉回 */
            fusion.yaw_rej_run = 0U;
            fusion.yaw += PATH_FUSION_YAW_GAIN * yaw_err;
        }
    }

    /* xy 直接覆盖 */
    fusion.x = x_m;
    fusion.y = y_m;
    fusion.have_upper = true;
    fusion.last_data_ms = now_ms;
    fusion.upper_frames++;

    return true;
}

/* 链路存活刷新 */
static void PathFusion_TouchLink(uint32_t now_ms)
{
    fusion.last_upper_ms = now_ms;
}

static bool PathFusion_IsUpperLost(uint32_t now_ms)
{

    if (!fusion.have_upper)
    {
        return true;
    }
    return (uint32_t)(now_ms - fusion.last_upper_ms) >
           PATH_FUSION_UPPER_TIMEOUT_MS;
}

static uint32_t PathFusion_DataAge(uint32_t now_ms)
{
    if (!fusion.have_upper)
    {
        return 0xFFFFFFFFU;
    }
    return now_ms - fusion.last_data_ms;
}

static bool PathFusion_HasUpper(void)
{
    return fusion.have_upper;
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

/* 纯追踪前视目标 ================================================================ */

static void PathPurePursuit_Find(const path_point_t *points, uint16_t count,
                          float x, float y, float v_ref, float kappa,
                          uint16_t i_near, uint16_t *i_target,
                          float *tx, float *ty)
{
    float lookahead = PATH_LD_MIN_M + PATH_LD_K_S * v_ref;
    float kappa_max;
    float kappa_cap;
    uint16_t i;

    /* 前视窗口内最大曲率 */
    kappa_max = fabsf(kappa);
    for (i = i_near + 1U;
         (i < count) && (i <= i_near + 15U); i++)
    {
        float kk = fabsf(points[i].kappa);
        if (kk > kappa_max)
        {
            kappa_max = kk;
        }
    }
    kappa_cap = PATH_LD_KAPPA_MAX_M / sqrtf(kappa_max + 0.05f);
    if (kappa_cap < PATH_LD_CAP_MIN_M)
    {
        kappa_cap = PATH_LD_CAP_MIN_M;
    }

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

/* 航向锁定 ================================================================ */

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

/* 在线跟踪总控(对接仓库已有模块) ================================================================ */

/* 仓库已有模块(全部真实外设数据) */

#if PATH_DEBUG
#endif

static path_point_t trajectory[PATH_SPLINE_SAMPLES];
static uint16_t trajectory_count;
static path_gridmap_t real_map;
static path_gridmap_t inflated_map;
static path_gridmap_t hard_map;
static const path_waypoint_t route_template[PATH_WAYPOINT_COUNT] =
{
    PATH_WAYPOINTS_TABLE
};
static path_waypoint_t raw_route[24];      /* 区域路由(未加密) */
static path_waypoint_t waypoints[PATH_WAYPOINT_COUNT];

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
static float build_start_x;            /* 本次 BUILD 使用的融合起点 */
static float build_start_y;
static uint16_t imu_fault_cycles;      /* 连续故障周期计数(去抖,P0-3) */
static uint16_t motor_fault_cycles;
static uint16_t laser_fault_cycles;
static uint32_t recover_healthy_since; /* 0=未开始计时 */
static uint8_t  recover_count;
static float last_cmd_vx;
static float last_cmd_vy;
static float last_cmd_w;

/* P0-4:按起点区域动态路由(路由表见 README 附录)。
 * 模板布局:[0]起点 [1](0.5,1.65) [2](1.0,1.65) [3](2.0,1.65)D弧起点
 * [4..14]D弧 [15](2.0,2.60) [16](1.0,2.60) [17](0.375,2.60)
 * [18](0.36,2.66) [19](0.36,3.70) [20]目标点 */
static uint8_t build_route(void)
{
    float sx = raw_route[0].x_m;
    float sy = raw_route[0].y_m;

    if (sy >= 3.45f)
    {
        /* 终点区:直行到目标 */
        raw_route[1].x_m = PATH_GOAL_X_M;
        raw_route[1].y_m = PATH_GOAL_Y_M;
        return 2U;
    }
    if (sy >= 2.45f)
    {
        if (sx <= 0.45f)
        {
            /* B0:缺口列内,直行北上,终点锚定目标 */
            raw_route[1].x_m = sx;
            raw_route[1].y_m = 3.70f;
            raw_route[2].x_m = PATH_GOAL_X_M;
            raw_route[2].y_m = PATH_GOAL_Y_M;
            return 3U;
        }
        if ((sy >= 2.75f) && (sx >= 0.9f))
        {
            /* 墙C 邻近带:墙C x>=1.05(硬膨胀西缘 0.815),
             * x<0.9 的起点已在缺口列内,可直行北上 */
            return 0U;
        }
        /* B1:通道2:西行到缺口列再北上,终点锚定目标 */
        raw_route[1].x_m = 0.375f;
        raw_route[1].y_m = sy;
        raw_route[2].x_m = 0.36f;
        raw_route[2].y_m = sy + 0.06f;
        raw_route[3].x_m = 0.36f;
        raw_route[3].y_m = 3.70f;
        raw_route[4].x_m = PATH_GOAL_X_M;
        raw_route[4].y_m = PATH_GOAL_Y_M;
        return 5U;
    }
    if (sy >= 1.12f)
    {
        if (sy > 2.125f)
        {
            /* C0:墙B 以北:直接西行绕缺口(同 B1) */
            raw_route[1].x_m = 0.375f;
            raw_route[1].y_m = sy;
            raw_route[2].x_m = 0.36f;
            raw_route[2].y_m = sy + 0.06f;
            raw_route[3].x_m = 0.36f;
            raw_route[3].y_m = 3.70f;
            raw_route[4].x_m = PATH_GOAL_X_M;
            raw_route[4].y_m = PATH_GOAL_Y_M;
            return 5U;
        }
        if (sx <= 2.0f)
        {
            /* C1:墙B 西侧:并入 y=1.65 走标准 D 弧 */
            if (fabsf(sy - 1.65f) < 0.15f)
            {
                /* 已在通道线上:直接东行入弧 */
                raw_route[1].x_m = 2.0f;
                raw_route[1].y_m = 1.65f;
                (void)memmove(&raw_route[2], &raw_route[4],
                              17U * sizeof(path_waypoint_t));
                return 19U;
            }
            if (sy < 1.53f)
            {
                /* 低位:北爬到通道1,倒角并入 */
                raw_route[1].x_m = sx;
                raw_route[1].y_m = 1.53f;
                raw_route[2].x_m = sx + 0.12f;
                if (raw_route[2].x_m > 2.0f) { raw_route[2].x_m = 2.0f; }
                raw_route[2].y_m = 1.65f;
                return 21U;
            }
        }
        /* C2:D 弧东侧或墙B 下沿:东移安全列(xcol >= 2.35,
         * 避开墙B 东端硬膨胀 2.235),北上,倒角西行绕缺口 */
        {
            float xcol = (sx < 2.35f) ? 2.35f : sx;
            raw_route[1].x_m = xcol;
            raw_route[1].y_m = sy;
            raw_route[2].x_m = xcol;
            raw_route[2].y_m = 2.48f;
            raw_route[3].x_m = xcol - 0.12f;
            raw_route[3].y_m = 2.60f;
            raw_route[4].x_m = 1.0f;
            raw_route[4].y_m = 2.60f;
            raw_route[5].x_m = 0.375f;
            raw_route[5].y_m = 2.60f;
            raw_route[6].x_m = 0.36f;
            raw_route[6].y_m = 2.66f;
            raw_route[7].x_m = 0.36f;
            raw_route[7].y_m = 3.70f;
            raw_route[8].x_m = PATH_GOAL_X_M;
            raw_route[8].y_m = PATH_GOAL_Y_M;
            return 9U;
        }
    }
    if (sx <= 0.8f)
    {
        /* 西南:倒角 L 形并入通道1 */
        raw_route[1].x_m = sx;
        raw_route[1].y_m = 1.53f;
        raw_route[2].x_m = sx + 0.12f;
        raw_route[2].y_m = 1.65f;
        return 21U;
    }
    /* 东南:先西行 x=0.5,倒角后北上(与标称同形) */
    raw_route[1].x_m = 0.5f;
    raw_route[1].y_m = sy;
    raw_route[2].x_m = 0.5f;
    raw_route[2].y_m = 1.53f;
    raw_route[3].x_m = 0.62f;
    raw_route[3].y_m = 1.65f;
    /* 先复制模板 D 弧(源还在),再覆盖 raw[4];顺序反了会丢 D 弧
     * 第一点,路由出现重复点 + 跳点(BUILD 必败,仿真复现) */
    (void)memmove(&raw_route[5], &raw_route[4],
                  17U * sizeof(path_waypoint_t));
    raw_route[4].x_m = 2.0f;
    raw_route[4].y_m = 1.65f;
    return 22U;
}

/* 路由加密:直线段按 PATH_ROUTE_STEP_M 插入中间点。长腿+少点的
 * 弦长参数化会让样条在拐角折叠(kappa 数百),加密后节点均匀,
 * 拐角由样条自然圆滑(与静态 D 弧表同一原理) */
static uint8_t densify_route(const path_waypoint_t *src, uint8_t n,
                             path_waypoint_t *dst, uint8_t max)
{
    uint8_t out = 0U;
    uint8_t i;

    if ((n == 0U) || (max < n))
    {
        return 0U;
    }
    dst[out++] = src[0];
    for (i = 1U; i < n; i++)
    {
        float dx = src[i].x_m - src[i - 1U].x_m;
        float dy = src[i].y_m - src[i - 1U].y_m;
        float len = sqrtf(dx * dx + dy * dy);
        uint16_t steps = (uint16_t)(len / PATH_ROUTE_STEP_M);
        uint16_t k;

        /* 仅对 <=3 点的短路线每段至少插 2 个中间点(死区起点补足
         * 4 控制点);长路线短腿(D 弧密集点)保持自然步数,否则点数
         * 超预算导致 densify 失败(仿真复现) */
        if ((steps < 2U) && (n <= 3U)) { steps = 2U; }

        for (k = 1U; k <= steps; k++)
        {
            float t = (float)k / (float)(steps + 1U);
            /* 溢出判失败而非静默截断(截断会丢终点,车跑完
             * 全程在终点前干等超时) */
            if (out >= max) { return 0U; }
            dst[out].x_m = src[i - 1U].x_m + t * dx;
            dst[out].y_m = src[i - 1U].y_m + t * dy;
            out++;
        }
        if (out >= max) { return 0U; }
        dst[out++] = src[i];
    }
    return out;
}

/* P0-4:弧长均匀重采样。u 域均匀采样在长路径/推离后弧长步长爆炸,
 * 导致采样间距验收失败;重采样保证步长恒定,并再推离一次 */
static void resample_uniform(path_point_t *pts, uint16_t *count)
{
    static path_point_t temp[PATH_SPLINE_SAMPLES];
    float total;
    uint16_t n_old;
    uint16_t n_new;
    uint16_t i;
    uint16_t j;

    if ((pts == NULL) || (count == NULL) || (*count < 2U))
    {
        return;
    }
    n_old = *count;
    total = pts[n_old - 1U].s_m;
    if (total < 1e-3f)
    {
        return;
    }

    /* 点数按长度自适应:标准路线(>=6m)300 点;极短路线(<0.5m)
     * 5cm/点(30 点对 0.25m 直线是 8mm 间隔,推离微扰动被放大成
     * kappa=165 噪声,仿真复现);其余约 3cm/点 */
    if (total >= 6.0f)
    {
        n_new = PATH_SPLINE_SAMPLES;
    }
    else if (total < 0.5f)
    {
        n_new = (uint16_t)(total / 0.05f);
        if (n_new < 4U) { n_new = 4U; }
    }
    else
    {
        n_new = (uint16_t)(total / 0.03f);
        if (n_new < 30U) { n_new = 30U; }
        if (n_new > PATH_SPLINE_SAMPLES) { n_new = PATH_SPLINE_SAMPLES; }
    }

    (void)memcpy(temp, pts, n_old * sizeof(path_point_t));
    for (i = 1U; i < n_new - 1U; i++)
    {
        float s_t = total * (float)i / (float)(n_new - 1U);
        j = 1U;
        while ((j < n_old) && (temp[j].s_m < s_t))
        {
            j++;
        }
        {
            float s0 = temp[j - 1U].s_m;
            float s1 = temp[j].s_m;
            float t = ((s1 - s0) > 1e-6f) ?
                      ((s_t - s0) / (s1 - s0)) : 0.0f;
            pts[i].x_m = temp[j - 1U].x_m +
                         t * (temp[j].x_m - temp[j - 1U].x_m);
            pts[i].y_m = temp[j - 1U].y_m +
                         t * (temp[j].y_m - temp[j - 1U].y_m);
        }
    }
    pts[n_new - 1U] = temp[n_old - 1U];
    if (total >= 0.5f)
    {
        /* 短路线跳过推离:终点区直线远离墙,推离只引入扰动 */
        (void)PathSpline_PushAwayFromWalls(pts, n_new, &inflated_map);
    }
    update_arc_curvature(pts, n_new);
    *count = n_new;
}

static void runner_stop(path_reason_t why)
{
    state = PATH_STATE_STOPPED;
    reason = why;
    last_cmd_vx = 0.0f;
    last_cmd_vy = 0.0f;
    last_cmd_w = 0.0f;
    Chassis_StopAll();
}

/* 四轮电机是否全部在线(P0-3 恢复判定) */
static bool motors_online(void)
{
    uint8_t wheel;

    for (wheel = 0U; wheel < CHASSIS_WHEEL_COUNT; wheel++)
    {
        vesc_motor_status_t st;
        if (!Chassis_GetStatus((chassis_wheel_t)wheel, &st) ||
            !st.online)
        {
            return false;
        }
    }
    return true;
}

/* 线段中点穿透检查 */
static bool point_penetrates(const path_gridmap_t *map, float x, float y,
                             float eps)
{
    uint8_t i;

    for (i = 0U; i < map->count; i++)
    {
        const path_wall_t *w = &map->walls[i];
        if ((x > (w->xmin + eps)) && (x < (w->xmax - eps)) &&
            (y > (w->ymin + eps)) && (y < (w->ymax - eps)))
        {
            return true;
        }
    }
    return false;
}

/* 轨迹验收(P0-2):进入 RUN 前强制校验 */
static bool validate_trajectory(const path_point_t *pts, uint16_t count,
                                const path_gridmap_t *hard_map)
{
    uint16_t i;

    if ((pts == NULL) || (hard_map == NULL) || (count < 3U))
    {
        return false;
    }

    for (i = 0U; i < count; i++)
    {
        float k_abs = fabsf(pts[i].kappa);

        if ((pts[i].x_m != pts[i].x_m) || (pts[i].y_m != pts[i].y_m) ||
            (pts[i].kappa != pts[i].kappa) ||
            (pts[i].v_ref != pts[i].v_ref))
        {
            return false;
        }
        /* 曲率验收 */
        if (k_abs > PATH_KAPPA_HARD_MAX)
        {
            return false;
        }
        if ((pts[i].v_ref * pts[i].v_ref * k_abs) >
            (PATH_A_LAT_MAX * PATH_LAT_ACC_TOL))
        {
            return false;
        }
        if (PathGridMap_DistTo(hard_map, pts[i].x_m, pts[i].y_m) <
            PATH_MIN_CLEARANCE_M)
        {
            return false;
        }
    }

    for (i = 1U; i < count; i++)
    {
        float dx = pts[i].x_m - pts[i - 1U].x_m;
        float dy = pts[i].y_m - pts[i - 1U].y_m;
        float mx = 0.5f * (pts[i].x_m + pts[i - 1U].x_m);
        float my = 0.5f * (pts[i].y_m + pts[i - 1U].y_m);

        if (sqrtf(dx * dx + dy * dy) > PATH_SAMPLE_STEP_MAX_M)
        {
            return false;
        }
        if (point_penetrates(hard_map, mx, my, PATH_SEGMENT_CUT_EPS_M))
        {
            return false;
        }
    }

    /* 末点必须落在目标容差内(路由截断/终点丢失的兜底检测,
     * 宁可 STOP_BUILD 不出车,也不跑 8 米干等超时) */
    {
        float gx = pts[count - 1U].x_m - PATH_GOAL_X_M;
        float gy = pts[count - 1U].y_m - PATH_GOAL_Y_M;
        if (sqrtf(gx * gx + gy * gy) > PATH_ARRIVE_TOL_M)
        {
            return false;
        }
    }

    /* 起点校验:轨迹首点必须与本次 BUILD 使用的融合起点一致
     * (0.05m 容差覆盖"贴软膨胀墙起点被推离"的几厘米位移) */
    {
        float sx = pts[0].x_m - build_start_x;
        float sy = pts[0].y_m - build_start_y;
        if (sqrtf(sx * sx + sy * sy) > 0.05f)
        {
            return false;
        }
    }

    return true;
}

/* 数值健康检查 */
static bool num_ok(float v)
{
    return (v == v) && (fabsf(v) < 1e6f);
}

/* 每个控制周期读一次真实传感器并做融合 */
static bool runner_read_and_fuse(uint32_t now_ms, float dt_s,
                                 float vx_ch_ms, float vy_ch_ms,
                                 imu_data_t *imu,
                                 float *laser_f_m, bool *laser_f_ok,
                                 float *laser_l_m, bool *laser_l_ok)
{
    bool imu_ok;
    pc_position_t upper;

    imu_ok = ImuMain_GetData(imu) &&
             imu->online && imu->yaw_valid && imu->gyro_valid &&
             (imu->state == IMU_STATE_READY);

    /* DT35 前/左激光 */
    *laser_f_m = (float)dt35_link[SENSOR_LINK_F_INDEX].distance_cm * 0.01f;
    if ((PATH_LASER_NO_ECHO_FREE != 0U) && (*laser_f_m <= 0.001f))
    {
        *laser_f_m = PATH_LASER_MAX_RANGE_M;
    }
    *laser_f_ok = (dt35_link[SENSOR_LINK_F_INDEX].online != 0U) &&
                  ((uint32_t)(now_ms -
                   dt35_link[SENSOR_LINK_F_INDEX].last_rx_ms) <
                   PATH_LASER_TIMEOUT_MS);
    *laser_l_m = (float)dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm * 0.01f;
    if ((PATH_LASER_NO_ECHO_FREE != 0U) && (*laser_l_m <= 0.001f))
    {
        *laser_l_m = PATH_LASER_MAX_RANGE_M;
    }
    *laser_l_ok = (dt35_link[SENSOR_LINK_L_B_INDEX].online != 0U) &&
                  ((uint32_t)(now_ms -
                   dt35_link[SENSOR_LINK_L_B_INDEX].last_rx_ms) <
                   PATH_LASER_TIMEOUT_MS);

    if (imu_ok)
    {
        /* 指令速度位置前馈 */
        PathFusion_Predict(imu->gyro_z_deg_s, dt_s,
                           vx_ch_ms, vy_ch_ms);
    }

    /* 上位机位姿 */
    {
        uint32_t pos_seq = PcLink_GetPositionSeq();
        if (pos_seq != last_pc_frame_count)
        {
            last_pc_frame_count = pos_seq;
            /* CRC 有效帧即刷新链路存活(P1-2) */
            PathFusion_TouchLink(now_ms);
            if (PcLink_GetPosition(&upper) &&
                ((upper.flags & PC_LINK_FLAG_FIELD_VALID) != 0U))
            {
                float mx = upper.field_x_m;
                float my = upper.field_y_m;
                /* P0-2:链路延迟使测量滞后 v*L,按配置延迟用指令
                 * 速度把测量点前推,消除大部分滞后误差 */
                if (PATH_POSE_LATENCY_MS > 0U)
                {
                    float c;
                    float s;
                    float vxw;
                    float vyw;
                    float fyaw_l;

                    PathFusion_Get(NULL, NULL, &fyaw_l);
                    c = cosf(fyaw_l);
                    s = sinf(fyaw_l);
                    vxw = vx_ch_ms * c - vy_ch_ms * s;
                    vyw = vx_ch_ms * s + vy_ch_ms * c;
                    mx += vxw * (PATH_POSE_LATENCY_MS * 0.001f);
                    my += vyw * (PATH_POSE_LATENCY_MS * 0.001f);
                }
                (void)PathFusion_UpdateUpper(mx, my,
                                             upper.field_w, now_ms);
            }
        }
    }

    return imu_ok;
}

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
    float exp_f;
    float dir_body_x;
    bool target_lateral;
    bool expected_wall;
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

    imu_ok = runner_read_and_fuse(now_ms, dt_s,
                                  last_cmd_vx, last_cmd_vy,
                                  &imu,
                                  &laser_f, &laser_f_ok,
                                  &laser_l, &laser_l_ok);
    PathFusion_Get(&fx, &fy, &fyaw);
    /* ---- 安全检查(顺序即优先级,P0-3 去抖) ---- */
    if (!PathFusion_HasUpper() || PathFusion_IsUpperLost(now_ms))
    {
        runner_stop(PATH_REASON_STOP_UPPER_LOST);
        return;
    }
    if (PathFusion_DataAge(now_ms) > PATH_UPPER_DATA_STOP_MS)
    {
        runner_stop(PATH_REASON_STOP_UPPER_LOST);
        return;
    }

    /* IMU 去抖:连续累计超 PATH_FAULT_IMU_MS 才停,单次抖动不误停 */
    if (!imu_ok)
    {
        if (imu_fault_cycles < 0xFFF0U) { imu_fault_cycles++; }
    }
    else
    {
        imu_fault_cycles = 0U;
    }
    if ((uint32_t)imu_fault_cycles * PATH_CONTROL_PERIOD_MS >=
        PATH_FAULT_IMU_MS)
    {
        runner_stop(PATH_REASON_STOP_IMU_LOST);
        return;
    }

    /* 前激光去抖(下位板 I2C 重试 1000ms,窗口取 1500ms) */
    if ((PATH_STOP_ON_LASER_LOSS != 0U) && !laser_f_ok)
    {
        if (laser_fault_cycles < 0xFFF0U) { laser_fault_cycles++; }
    }
    else
    {
        laser_fault_cycles = 0U;
    }
    if ((uint32_t)laser_fault_cycles * PATH_CONTROL_PERIOD_MS >=
        PATH_FAULT_LASER_MS)
    {
        runner_stop(PATH_REASON_STOP_LASER_LOST);
        return;
    }

    /* 电机去抖 */
    if ((PATH_REQUIRE_MOTORS != 0U) && !motors_online())
    {
        if (motor_fault_cycles < 0xFFF0U) { motor_fault_cycles++; }
    }
    else
    {
        motor_fault_cycles = 0U;
    }
    if ((uint32_t)motor_fault_cycles * PATH_CONTROL_PERIOD_MS >=
        PATH_FAULT_MOTOR_MS)
    {
        runner_stop(PATH_REASON_STOP_MOTOR_LOST);
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
    exp_f = trajectory[i_near].exp_laser_front_m;   /* 此前算出未用,现参与期望墙门控 */

    /* 数据降级限速(在查表后、激光兜底前统一钳制) */
    if (PathFusion_DataAge(now_ms) > PATH_UPPER_DEGRADE_MS)
    {
        if (v_ref > PATH_UPPER_DEGRADE_V_MS)
        {
            v_ref = PATH_UPPER_DEGRADE_V_MS;
        }
    }

    /* 纯追踪目标点 */
    PathPurePursuit_Find(trajectory, trajectory_count, fx, fy, v_ref,
                         trajectory[i_near].kappa,
                         i_near, &i_target, &tx, &ty);
    L = sqrtf((tx - fx) * (tx - fx) + (ty - fy) * (ty - fy));
    if (L <= 1e-3f)
    {
        L = 1e-3f;
    }

    /* 前激光兜底 */
    lf = laser_f;
    if (lf > PATH_LASER_MAX_RANGE_M)
    {
        lf = PATH_LASER_MAX_RANGE_M;
    }

    /* 车体纵向分量 */
    {
        float fwd_x = -sinf(fyaw);
        float fwd_y = cosf(fyaw);
        dir_body_x = ((tx - fx) / L) * fwd_x + ((ty - fy) / L) * fwd_y;
    }
    target_lateral = (fabsf(dir_body_x) <= PATH_LASER_LATERAL_DIR_MAX);
    expected_wall = (exp_f <= (PATH_LASER_MAX_RANGE_M + 0.03f));

    if (lf >= (PATH_LASER_MAX_RANGE_M - 1e-4f))
    {
        v_used = v_ref;
        reason = PATH_REASON_RUN;
    }
    else if (expected_wall &&
             (lf >= (exp_f - PATH_LASER_EXPECTED_MARGIN_M)))
    {
        v_used = v_ref;
        reason = PATH_REASON_RUN;
    }
    else if (lf <= PATH_LASER_STOP_DIST_M)
    {
        if (expected_wall && target_lateral)
        {
            v_used = (v_ref < PATH_LASER_RECOVERY_V_MS) ?
                     v_ref : PATH_LASER_RECOVERY_V_MS;
            reason = PATH_REASON_LASER_SLOW;
        }
        else
        {
            v_used = 0.0f;
            reason = PATH_REASON_STOP_LASER_FRONT;
        }
    }
    else
    {
        v_laser = sqrtf(2.0f * PATH_A_LON_BRAKE *
                        (lf - PATH_LASER_STOP_DIST_M));
        v_used = (v_ref < v_laser) ? v_ref : v_laser;
        reason = PATH_REASON_LASER_SLOW;
    }

    vx_w = v_used * (tx - fx) / L;
    vy_w = v_used * (ty - fy) / L;

    PathWorldToChassis(vx_w, vy_w, fyaw, &vx_c, &vy_c);

    /* 左激光横向微调(默认关,见 PATH_LAT_TRIM_ENABLE) */
    if ((PATH_LAT_TRIM_ENABLE != 0U) &&
        (v_used > 0.0f) && laser_l_ok &&
        (exp_l <= PATH_LASER_MAX_RANGE_M))
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

    /* ---- slew-rate 限幅 */
    dv_max = PATH_SLEW_XY_ACCEL_MS2 * dt_s;
    dw_max = PATH_SLEW_W_ACCEL_RADS2 * dt_s;
    {
        float dvx = vx_c - last_cmd_vx;
        float dvy = vy_c - last_cmd_vy;
        float dv_mag = sqrtf(dvx * dvx + dvy * dvy);
        if (dv_mag > dv_max)
        {
            float scale = dv_max / dv_mag;
            vx_c = last_cmd_vx + dvx * scale;
            vy_c = last_cmd_vy + dvy * scale;
        }
    }
    if ((w_cmd - last_cmd_w) > dw_max) { w_cmd = last_cmd_w + dw_max; }
    if ((w_cmd - last_cmd_w) < -dw_max) { w_cmd = last_cmd_w - dw_max; }
    last_cmd_vx = vx_c;
    last_cmd_vy = vy_c;
    last_cmd_w = w_cmd;

    /* ---- 数值防护 */
    if (!num_ok(vx_c) || !num_ok(vy_c) || !num_ok(w_cmd) ||
        !num_ok(v_ref) || !num_ok(v_used))
    {
        runner_stop(PATH_REASON_STOP_NUMERIC);
        return;
    }

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
        /* 关闭 IMU 模块自带航向保持 */
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
        /* 等待 IMU READY */
        if (imu.state == IMU_STATE_ERROR)
        {
            runner_stop(PATH_REASON_STOP_IMU_LOST);
            break;
        }
        if ((uint32_t)(now_ms - state_start_ms) > PATH_CALIB_TIMEOUT_MS)
        {
            runner_stop(PATH_REASON_STOP_IMU_LOST);
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

        /* 起点=小电脑实测位姿 */
        {
            pc_position_t upper;
            uint32_t pos_seq = PcLink_GetPositionSeq();
            if ((pos_seq != last_pc_frame_count) &&
                PcLink_GetPosition(&upper) &&
                ((upper.flags & PC_LINK_FLAG_FIELD_VALID) != 0U))
            {
                last_pc_frame_count = pos_seq;
                PathFusion_TouchLink(now_ms);

                if (fabsf(PathWrapAngle(upper.field_w)) >
                    (PATH_START_YAW_LIMIT_DEG * DEG2RAD))
                {
                    runner_stop(PATH_REASON_STOP_HEADING);
                    break;
                }
                /* UpdateUpper 会做数值/场地范围校验 */
                if (!PathFusion_UpdateUpper(upper.field_x_m,
                                            upper.field_y_m,
                                            upper.field_w, now_ms))
                {
                    break;
                }
                waypoints[0].x_m = upper.field_x_m;
                waypoints[0].y_m = upper.field_y_m;
                state = PATH_STATE_BUILD;
                state_start_ms = now_ms;
                break;
            }
        }

        (void)runner_read_and_fuse(now_ms, dt_s, 0.0f, 0.0f,
                                   &imu,
                                   &laser_f, &lf_ok, &laser_l, &ll_ok);
        break;
    }

    case PATH_STATE_BUILD:
    {
        uint8_t wp_count;
        float fx0;
        float fy0;

        PathGridMap_BuildReal(&real_map);
        PathGridMap_BuildInflated(&inflated_map);
        PathGridMap_BuildHardInflated(&hard_map);

        /* 模板拷贝 + 起点=融合位姿 + 区域路由 + 加密 */
        (void)memcpy(raw_route, route_template,
                     21U * sizeof(path_waypoint_t));
        PathFusion_Get(&fx0, &fy0, NULL);
        build_start_x = fx0;
        build_start_y = fy0;
        if (PathGridMap_Contains(&hard_map, fx0, fy0))
        {
            /* 起点车体与墙重叠(如贴墙 2cm):非法,直接拒绝 */
            runner_stop(PATH_REASON_STOP_BUILD);
            break;
        }
        raw_route[0].x_m = fx0;
        raw_route[0].y_m = fy0;
        wp_count = build_route();
        if (wp_count < 2U)
        {
            runner_stop(PATH_REASON_STOP_BUILD);
            break;
        }
        wp_count = densify_route(raw_route, wp_count,
                                 waypoints, PATH_WAYPOINT_COUNT);
        if (wp_count < 2U)
        {
            runner_stop(PATH_REASON_STOP_BUILD);
            break;
        }

        if (!PathSpline_Build(waypoints, wp_count,
                              trajectory, PATH_SPLINE_SAMPLES,
                              &trajectory_count))
        {
            runner_stop(PATH_REASON_STOP_BUILD);
            break;
        }
        /* 整形 + 弧长重采样 + 验收循环 */
        {
            uint8_t attempt;
            bool built = false;

            for (attempt = 0U; attempt < PATH_BUILD_MAX_ATTEMPTS; attempt++)
            {
                PathSpline_Finalize(trajectory, trajectory_count,
                                    &inflated_map);
                resample_uniform(trajectory, &trajectory_count);
                if (!PathSpeedProfile_Build(trajectory, trajectory_count,
                                            &real_map))
                {
                    break;
                }
                if (validate_trajectory(trajectory, trajectory_count,
                                        &hard_map))
                {
                    built = true;
                    break;
                }
            }
            if (!built)
            {
                runner_stop(PATH_REASON_STOP_BUILD);
                break;
            }
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
        /* 到达:锁存,不自动恢复 */
        break;

    case PATH_STATE_STOPPED:
        /* P0-3 瞬态故障恢复:连续健康 PATH_RECOVER_MS 后重新布防
         * (重新按当前位置规划),限 PATH_RECOVER_MAX_TIMES 次;
         * BUILD/NUMERIC/HEADING/TIMEOUT 等确定性故障不自动恢复 */
        {
            bool transient = (reason == PATH_REASON_STOP_UPPER_LOST) ||
                             (reason == PATH_REASON_STOP_IMU_LOST) ||
                             (reason == PATH_REASON_STOP_LASER_LOST) ||
                             (reason == PATH_REASON_STOP_MOTOR_LOST);

            if (transient && (recover_count < PATH_RECOVER_MAX_TIMES))
            {
                imu_data_t imu;
                float lf;
                float ll;
                bool lf_ok;
                bool ll_ok;
                bool imu_ok;
                bool healthy;

                imu_ok = runner_read_and_fuse(now_ms, dt_s, 0.0f, 0.0f,
                                              &imu, &lf, &lf_ok, &ll, &ll_ok);
                healthy = imu_ok && lf_ok && motors_online() &&
                          !PathFusion_IsUpperLost(now_ms) &&
                          PathFusion_HasUpper();

                if (healthy)
                {
                    if (recover_healthy_since == 0U)
                    {
                        recover_healthy_since = now_ms;
                    }
                    else if ((uint32_t)(now_ms - recover_healthy_since) >=
                             PATH_RECOVER_MS)
                    {
                        recover_healthy_since = 0U;
                        recover_count++;
                        imu_fault_cycles = 0U;
                        motor_fault_cycles = 0U;
                        laser_fault_cycles = 0U;
                        last_i_near = 0U;
                        state = PATH_STATE_WAIT_START;
                        reason = PATH_REASON_WAIT_START;
                        state_start_ms = now_ms;
                    }
                }
                else
                {
                    recover_healthy_since = 0U;
                }
            }
        }
        break;

    default:
        break;
    }

    /* 调试信息公共部分 */
    debug.state = state;
    debug.reason = reason;
    PathFusion_GetStats(&debug.fusion_xy_rejects, &debug.fusion_yaw_rejects,
                        &debug.upper_frames);
    PcLink_GetStats(&debug.pc_frames, &debug.crc_errors);

    /* 回传状态帧 */
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

bool PathPlanner_OwnsChassis(void)
{
    /* 指令仲裁(P0-5):RUN 期间规划器独占底盘 */
    return (state == PATH_STATE_RUN);
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

#endif /* PATH_DEBUG */
