/**
 ******************************************************************************
 * @file    path_geometry.c
 * @brief   几何工具实现:墙 / 距离场 / 射线投射 / 坐标变换
 ******************************************************************************
 */
#include "path_geometry.h"

#include <math.h>
#include <stddef.h>

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
bool PathGridMap_Contains(const path_gridmap_t *map, float x, float y)
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

bool PathGridMap_PushOut(const path_gridmap_t *map, float *x, float *y,
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

void PathBodyToWorld(float vx_b, float vy_b, float yaw_user,
                     float *vx_w, float *vy_w)
{
    /* θ = yaw + π/2:yaw=0(朝 +y)时车体 +x 应指向世界 +y */
    float theta = yaw_user + (float)(PATH_PI / 2.0);
    float c = cosf(theta);
    float s = sinf(theta);

    *vx_w = vx_b * c - vy_b * s;
    *vy_w = vx_b * s + vy_b * c;
}

void PathWorldToBody(float vx_w, float vy_w, float yaw_user,
                     float *vx_b, float *vy_b)
{
    float theta = yaw_user + (float)(PATH_PI / 2.0);
    float c = cosf(theta);
    float s = sinf(theta);

    *vx_b = vx_w * c + vy_w * s;
    *vy_b = -vx_w * s + vy_w * c;
}

void PathWorldToChassis(float vx_w, float vy_w, float yaw_user,
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
