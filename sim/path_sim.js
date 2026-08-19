/* 与 user/path/path_main.c 对齐的路径仿真（浏览器 / Node 共用）。 */
(function (root, factory) {
    if (typeof module === "object" && module.exports) {
        module.exports = factory();
    } else {
        root.PathSim = factory();
    }
}(typeof self !== "undefined" ? self : this, function () {
    "use strict";

    var CFG = {
        CONTROL_PERIOD_MS: 10,
        SETTLE_MS: 65,
        LASER_STOP_CM: 10,
        FRONT_ARRIVE_CM: 68,
        LEFT_NEAR_CM: 69,
        LEFT_FAR_CM: 200,
        MIRROR_LEFT_CM: 100,
        ARC_ENABLE: 1,
        LEFT_NEAR_ARC_CM: 110,
        LEFT_FAR_ARC_CM: 160,
        SEGMENT_COUNT: 4,
        SPEED_MAX: 170,
        SPEED_MIN: 45,
        PID_KP: 2.6,
        PID_KI: 1.2,
        PID_KD: 0.05,
        PID_I_LIMIT: 30,
        PID_DT_S: 0.01,
        ALIGN_DONE_DEG: 2,
        ALIGN_TIMEOUT_MS: 4000,
        FIELD_W: 280,
        FIELD_H: 430,
        ROBOT_R: 15,
        LASER_MAX: 400,
        CMD_TO_CM_S: 1
    };

    var STATE = {
        IDLE: 0x10,
        RUNNING: 0x11,
        FINISHED: 0x12,
        FAULT: 0x13
    };

    function clamp(value, low, high) {
        if (value < low) {
            return low;
        }
        if (value > high) {
            return high;
        }
        return value;
    }

    function wrapDeg(angle) {
        while (angle >= 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    function selectHoldYaw(yaw) {
        var toZero = Math.abs(wrapDeg(yaw));
        var to180 = Math.abs(wrapDeg(yaw - 180));
        return (toZero <= to180) ? 0 : 180;
    }

    function segIntersect(ax, ay, bx, by, cx, cy, dx, dy) {
        var rxs = (bx - ax) * (dy - cy) - (by - ay) * (dx - cx);
        if (Math.abs(rxs) < 1e-9) {
            return null;
        }
        var t = ((cx - ax) * (dy - cy) - (cy - ay) * (dx - cx)) / rxs;
        var u = ((cx - ax) * (by - ay) - (cy - ay) * (bx - ax)) / rxs;
        if (t < 0 || t > 1 || u < 0 || u > 1) {
            return null;
        }
        return { x: ax + t * (bx - ax), y: ay + t * (by - ay), t: t };
    }

    function distPointSeg(px, py, ax, ay, bx, by) {
        var dx = bx - ax;
        var dy = by - ay;
        var len2 = dx * dx + dy * dy;
        var t = 0;
        if (len2 > 1e-9) {
            t = clamp(((px - ax) * dx + (py - ay) * dy) / len2, 0, 1);
        }
        var qx = ax + t * dx;
        var qy = ay + t * dy;
        return Math.hypot(px - qx, py - qy);
    }

    function makeField(mirrored) {
        var w = CFG.FIELD_W;
        var h = CFG.FIELD_H;
        var walls = [
            { x1: 0, y1: 0, x2: w, y2: 0 },
            { x1: w, y1: 0, x2: w, y2: h },
            { x1: w, y1: h, x2: 0, y2: h },
            { x1: 0, y1: h, x2: 0, y2: 0 }
        ];
        /* 通道1尽头挡板：常规挡左边、镜像挡右边，开口让车滑进通道2。 */
        if (mirrored) {
            walls.push({ x1: 142, y1: 198, x2: w, y2: 198 });
        } else {
            walls.push({ x1: 0, y1: 198, x2: 138, y2: 198 });
        }
        return walls;
    }

    function raycast(walls, ox, oy, dx, dy, maxDist) {
        var best = maxDist;
        var hit = null;
        var i;
        var n = Math.hypot(dx, dy);
        if (n < 1e-9) {
            return { dist: maxDist, hit: null };
        }
        dx /= n;
        dy /= n;
        for (i = 0; i < walls.length; i++) {
            var w = walls[i];
            var p = segIntersect(ox, oy, ox + dx * maxDist, oy + dy * maxDist,
                                 w.x1, w.y1, w.x2, w.y2);
            if (p && p.t * maxDist < best) {
                best = p.t * maxDist;
                hit = p;
            }
        }
        return { dist: best, hit: hit };
    }

    function circleHitsWalls(walls, x, y, r) {
        var i;
        for (i = 0; i < walls.length; i++) {
            var w = walls[i];
            if (distPointSeg(x, y, w.x1, w.y1, w.x2, w.y2) < r) {
                return true;
            }
        }
        return false;
    }

    function createController(cfg) {
        cfg = cfg || CFG;
        var s = {
            state: STATE.IDLE,
            error: 0,
            segment: 0,
            mirrored: false,
            startCount: 0,
            aligning: false,
            holdYaw: 0,
            alignStartMs: 0,
            arcLatched: false,
            lastControlMs: 0,
            segmentChangeMs: 0,
            pid: { integral: 0, lastError: 0, started: false }
        };

        function resetPid() {
            s.pid.integral = 0;
            s.pid.lastError = 0;
            s.pid.started = false;
        }

        function arcActive() {
            if (!cfg.ARC_ENABLE) {
                return false;
            }
            if (s.mirrored) {
                return cfg.LEFT_NEAR_ARC_CM > cfg.LEFT_NEAR_CM;
            }
            return cfg.LEFT_FAR_ARC_CM < cfg.LEFT_FAR_CM;
        }

        function arcTrigger(leftCm) {
            if (!arcActive()) {
                return false;
            }
            if (s.mirrored) {
                return leftCm <= cfg.LEFT_NEAR_ARC_CM;
            }
            return leftCm >= cfg.LEFT_FAR_ARC_CM;
        }

        function runPid(errorCm) {
            var derivative = 0;
            var output;
            if (errorCm <= 0) {
                resetPid();
                return 0;
            }
            if (s.pid.started) {
                derivative = (errorCm - s.pid.lastError) / cfg.PID_DT_S;
            } else {
                s.pid.started = true;
            }
            s.pid.integral = clamp(
                s.pid.integral + cfg.PID_KI * errorCm * cfg.PID_DT_S,
                -cfg.PID_I_LIMIT, cfg.PID_I_LIMIT);
            s.pid.lastError = errorCm;
            output = cfg.PID_KP * errorCm + s.pid.integral +
                     cfg.PID_KD * derivative;
            return Math.round(clamp(output, cfg.SPEED_MIN, cfg.SPEED_MAX));
        }

        function arrived(frontCm, leftCm) {
            var lateralDone;
            if (s.segment === 0 || s.segment === 2) {
                return frontCm <= cfg.FRONT_ARRIVE_CM;
            }
            if (s.segment === 1) {
                lateralDone = s.mirrored ?
                    (leftCm <= cfg.LEFT_NEAR_CM) :
                    (leftCm >= cfg.LEFT_FAR_CM);
                if (arcActive()) {
                    return lateralDone && (frontCm <= cfg.FRONT_ARRIVE_CM);
                }
                return lateralDone;
            }
            if (s.segment === 3) {
                return s.mirrored ?
                    (leftCm >= cfg.LEFT_FAR_CM) :
                    (leftCm <= cfg.LEFT_NEAR_CM);
            }
            return true;
        }

        function command(frontCm, leftCm) {
            var vx = 0;
            var vy = 0;
            var remaining = 0;
            var frontRem;
            var vyCm;

            if (s.segment === 0 || s.segment === 2) {
                remaining = frontCm - cfg.FRONT_ARRIVE_CM;
                vy = runPid(remaining);
            } else if (s.segment === 1) {
                if (s.mirrored) {
                    remaining = leftCm - cfg.LEFT_NEAR_CM;
                    vx = -runPid(remaining);
                } else {
                    remaining = cfg.LEFT_FAR_CM - leftCm;
                    vx = runPid(remaining);
                }
                if (arcTrigger(leftCm)) {
                    s.arcLatched = true;
                }
                if (s.arcLatched && (frontCm > cfg.FRONT_ARRIVE_CM)) {
                    frontRem = frontCm - cfg.FRONT_ARRIVE_CM;
                    if (remaining > 1) {
                        vyCm = Math.abs(vx) * (frontRem / remaining);
                    } else {
                        vyCm = cfg.PID_KP * frontRem;
                    }
                    vy = Math.round(clamp(vyCm, 0, cfg.SPEED_MAX));
                }
            } else if (s.segment === 3) {
                if (s.mirrored) {
                    remaining = cfg.LEFT_FAR_CM - leftCm;
                    vx = runPid(remaining);
                } else {
                    remaining = leftCm - cfg.LEFT_NEAR_CM;
                    vx = -runPid(remaining);
                }
            }

            if ((frontCm < cfg.LASER_STOP_CM) && (vy > 0)) {
                vy = 0;
            }
            if ((leftCm < cfg.LASER_STOP_CM) && (vx < 0)) {
                vx = 0;
            }
            return { vx: vx, vy: vy };
        }

        function start(nowMs, leftCm, yawDeg) {
            s.holdYaw = selectHoldYaw(yawDeg);
            s.mirrored = leftCm >= cfg.MIRROR_LEFT_CM;
            s.segment = 0;
            s.segmentChangeMs = nowMs;
            s.lastControlMs = nowMs - cfg.CONTROL_PERIOD_MS;
            resetPid();
            s.startCount += 1;
            s.aligning = s.startCount > 1;
            s.alignStartMs = nowMs;
            s.arcLatched = false;
            s.state = STATE.RUNNING;
            s.error = 0;
        }

        function step(nowMs, frontCm, leftCm, yawDeg) {
            var cmd = { vx: 0, vy: 0, z: 0 };
            var err;

            if (s.state !== STATE.RUNNING) {
                return cmd;
            }
            if (nowMs - s.lastControlMs < cfg.CONTROL_PERIOD_MS) {
                return null;
            }
            s.lastControlMs = nowMs;

            if (s.aligning) {
                err = wrapDeg(yawDeg - s.holdYaw);
                if ((Math.abs(err) <= cfg.ALIGN_DONE_DEG) ||
                    (nowMs - s.alignStartMs >= cfg.ALIGN_TIMEOUT_MS)) {
                    s.aligning = false;
                    s.segmentChangeMs = nowMs;
                    resetPid();
                }
                return cmd;
            }

            if (arrived(frontCm, leftCm)) {
                if ((s.segment === 1) && arcActive()) {
                    s.segment = 3;
                } else {
                    s.segment += 1;
                }
                s.arcLatched = false;
                s.segmentChangeMs = nowMs;
                resetPid();
                if (s.segment >= cfg.SEGMENT_COUNT) {
                    s.state = STATE.FINISHED;
                }
                return cmd;
            }
            if (nowMs - s.segmentChangeMs < cfg.SETTLE_MS) {
                return cmd;
            }
            return command(frontCm, leftCm);
        }

        return {
            s: s,
            start: start,
            step: step,
            resetPid: resetPid,
            arcActive: arcActive
        };
    }

    function createWorld(options) {
        options = options || {};
        var cfg = {};
        var key;
        for (key in CFG) {
            if (Object.prototype.hasOwnProperty.call(CFG, key)) {
                cfg[key] = CFG[key];
            }
        }
        if (options.cfg) {
            for (key in options.cfg) {
                if (Object.prototype.hasOwnProperty.call(options.cfg, key)) {
                    cfg[key] = options.cfg[key];
                }
            }
        }

        var mirrored = !!options.mirrored;
        var startYaw = (options.startYaw !== undefined) ? options.startYaw : 0;
        var world = {
            cfg: cfg,
            mirrored: mirrored,
            walls: makeField(mirrored),
            x: mirrored ? cfg.LEFT_FAR_CM : cfg.LEFT_NEAR_CM,
            y: 55,
            yaw: startYaw,
            vx: 0,
            vy: 0,
            nowMs: 0,
            trail: [],
            collisions: 0,
            usedSeg2: false,
            maxSeg: 0,
            frontCm: 0,
            leftCm: 0,
            frontHit: null,
            leftHit: null,
            finished: false,
            failed: false,
            failReason: "",
            ctrl: createController(cfg)
        };

        function dirs() {
            var rad = world.yaw * Math.PI / 180;
            return {
                fx: Math.sin(rad),
                fy: Math.cos(rad),
                lx: -Math.cos(rad),
                ly: -Math.sin(rad)
            };
        }

        function sense() {
            var d = dirs();
            var front = raycast(world.walls, world.x, world.y, d.fx, d.fy,
                                cfg.LASER_MAX);
            var left = raycast(world.walls, world.x, world.y, d.lx, d.ly,
                               cfg.LASER_MAX);
            world.frontCm = Math.max(1, Math.round(front.dist));
            world.leftCm = Math.max(1, Math.round(left.dist));
            world.frontHit = front.hit;
            world.leftHit = left.hit;
        }

        function holdYaw(dt) {
            var err = wrapDeg(world.ctrl.s.holdYaw - world.yaw);
            var omega;
            if (Math.abs(err) < 0.1) {
                world.yaw = world.ctrl.s.holdYaw;
                return;
            }
            omega = clamp(err * 4, -120, 120);
            world.yaw = wrapDeg(world.yaw + omega * dt);
        }

        function tick() {
            var dt = cfg.CONTROL_PERIOD_MS / 1000;
            var cmd;
            var nx;
            var ny;

            if (world.failed || world.finished) {
                return;
            }
            world.nowMs += cfg.CONTROL_PERIOD_MS;
            sense();
            cmd = world.ctrl.step(world.nowMs, world.frontCm, world.leftCm,
                                  world.yaw);
            if (cmd) {
                world.vx = cmd.vx || 0;
                world.vy = cmd.vy || 0;
            }
            if (world.ctrl.s.segment === 2) {
                world.usedSeg2 = true;
            }
            if (world.ctrl.s.segment > world.maxSeg) {
                world.maxSeg = world.ctrl.s.segment;
            }

            holdYaw(dt);
            /* yaw=0: 右=+X, 前=+Y，与 path_main / 底盘约定一致。 */
            nx = world.x + (world.vx * Math.cos(world.yaw * Math.PI / 180) -
                 world.vy * Math.sin(world.yaw * Math.PI / 180)) * cfg.CMD_TO_CM_S * dt;
            ny = world.y + (world.vx * Math.sin(world.yaw * Math.PI / 180) +
                 world.vy * Math.cos(world.yaw * Math.PI / 180)) * cfg.CMD_TO_CM_S * dt;

            if (circleHitsWalls(world.walls, nx, ny, cfg.ROBOT_R)) {
                world.collisions += 1;
                world.failed = true;
                world.failReason = "撞墙 seg=" + world.ctrl.s.segment +
                    " x=" + nx.toFixed(1) + " y=" + ny.toFixed(1);
                return;
            }
            world.x = nx;
            world.y = ny;
            if ((world.trail.length === 0) ||
                (Math.hypot(world.x - world.trail[world.trail.length - 1].x,
                            world.y - world.trail[world.trail.length - 1].y) > 2)) {
                world.trail.push({
                    x: world.x,
                    y: world.y,
                    seg: world.ctrl.s.segment,
                    arc: world.ctrl.s.arcLatched
                });
            }
            if (world.ctrl.s.state === STATE.FINISHED) {
                world.finished = true;
            }
        }

        function start() {
            sense();
            world.ctrl.start(world.nowMs, world.leftCm, world.yaw);
        }

        function run(maxMs) {
            var limit = maxMs || 30000;
            start();
            while (!world.finished && !world.failed && world.nowMs < limit) {
                tick();
            }
            if (!world.finished && !world.failed) {
                world.failed = true;
                world.failReason = "超时 " + world.nowMs + "ms seg=" +
                    world.ctrl.s.segment;
            }
            return summarize();
        }

        function summarize() {
            return {
                ok: world.finished && !world.failed && (world.collisions === 0),
                finished: world.finished,
                failed: world.failed,
                reason: world.failReason,
                mirrored: world.mirrored,
                arc: !!cfg.ARC_ENABLE && world.ctrl.arcActive(),
                usedSeg2: world.usedSeg2,
                collisions: world.collisions,
                timeMs: world.nowMs,
                x: world.x,
                y: world.y,
                yaw: world.yaw,
                frontCm: world.frontCm,
                leftCm: world.leftCm,
                segment: world.ctrl.s.segment,
                trail: world.trail
            };
        }

        world.sense = sense;
        world.tick = tick;
        world.start = start;
        world.run = run;
        world.summarize = summarize;
        world.dirs = dirs;
        sense();
        return world;
    }

    function verifyAll() {
        var cases = [
            { name: "常规+原S弯", mirrored: false, cfg: { ARC_ENABLE: 0 } },
            { name: "常规+画弧", mirrored: false, cfg: { ARC_ENABLE: 1 } },
            { name: "镜像+原S弯", mirrored: true, cfg: { ARC_ENABLE: 0 } },
            { name: "镜像+画弧", mirrored: true, cfg: { ARC_ENABLE: 1 } },
            { name: "常规+阈值相等无弧", mirrored: false,
              cfg: { ARC_ENABLE: 1, LEFT_NEAR_ARC_CM: 69, LEFT_FAR_ARC_CM: 200 } },
            { name: "第二次起步先对准", mirrored: false, startYaw: 18,
              secondStart: true, cfg: { ARC_ENABLE: 1 } }
        ];
        return cases.map(function (item) {
            var world = createWorld({
                mirrored: item.mirrored,
                startYaw: item.startYaw || 0,
                cfg: item.cfg
            });
            var result;
            if (item.secondStart) {
                world.ctrl.s.startCount = 1;
            }
            result = world.run(40000);
            result.name = item.name;
            if (item.cfg.ARC_ENABLE && world.ctrl.arcActive() && result.ok &&
                result.usedSeg2) {
                result.ok = false;
                result.reason = "画弧仍走进了段2";
            }
            if ((!item.cfg.ARC_ENABLE || !world.ctrl.arcActive()) &&
                result.ok && !result.usedSeg2) {
                result.ok = false;
                result.reason = "关闭画弧却跳过了段2";
            }
            return result;
        });
    }

    return {
        CFG: CFG,
        STATE: STATE,
        createWorld: createWorld,
        createController: createController,
        makeField: makeField,
        verifyAll: verifyAll
    };
}));
