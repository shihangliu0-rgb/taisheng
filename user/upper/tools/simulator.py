#!/usr/bin/env python3
"""下位机模拟器：没有硬件也能完整测试上位机。

用法：
  1) TCP 模式（推荐，最省事）：
        python tools/simulator.py --tcp 8888
     上位机里选择「网络 TCP」，host=127.0.0.1，port=8888

  2) 虚拟串口对（Linux，需要 socat）：
        socat -d -d pty,raw,echo=0 pty,raw,echo=0
        python tools/simulator.py --serial /dev/pts/X
     上位机连另一个 /dev/pts/Y

模拟器行为：模拟一阶电机 + PID 闭环，回 ACK / PID_VALUE / TELEMETRY / PID_CURVE。
"""

from __future__ import annotations

import argparse
import math
import random
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from upper_computer import protocol as P  # noqa: E402


class Plant:
    """一阶惯性对象 + PID，用来产生像样的调试曲线。"""

    def __init__(self):
        self.pid = {}      # RAM 中当前生效的参数
        self.flash = {}    # Flash 中已固化的参数（掉电不丢的那份）
        for pid_id in (0, 1, 2, 3, 0x10, 0x11, 0x20, 0x21):
            self.pid[pid_id] = dict(kp=1.2, ki=0.08, kd=0.01, i_limit=500.0, out_limit=8000.0)
            self.flash[pid_id] = dict(self.pid[pid_id])
        self.target = {k: 0.0 for k in self.pid}
        self.fb = {k: 0.0 for k in self.pid}
        self.integ = {k: 0.0 for k in self.pid}
        self.prev_err = {k: 0.0 for k in self.pid}
        self.out = {k: 0.0 for k in self.pid}

    def step(self, pid_id: int, dt: float):
        p = self.pid[pid_id]
        err = self.target[pid_id] - self.fb[pid_id]
        self.integ[pid_id] += err * dt
        if p["i_limit"] > 0:
            self.integ[pid_id] = max(-p["i_limit"], min(p["i_limit"], self.integ[pid_id]))
        d = (err - self.prev_err[pid_id]) / dt if dt > 0 else 0.0
        self.prev_err[pid_id] = err
        o = p["kp"] * err + p["ki"] * self.integ[pid_id] + p["kd"] * d
        if p["out_limit"] > 0:
            o = max(-p["out_limit"], min(p["out_limit"], o))
        self.out[pid_id] = o
        tau = 0.15
        self.fb[pid_id] += (o * 0.12 - self.fb[pid_id]) * (dt / tau)


class Sim:
    def __init__(self, write):
        self.write = write
        self.parser = P.FrameParser()
        self.plant = Plant()
        self.t0 = time.monotonic()
        self.vx = self.vy = self.omega = 0
        self.yaw = 0.0
        self.telem_on = False
        self.telem_period = 0.05
        self.telem_mask = 0x03
        self.last_telem = 0.0
        self.last_curve = 0.0
        # 数据流订阅
        self.stream_on = False
        self.stream_period = 0.02
        self.stream_channels = []
        self.last_stream = 0.0
        self.odom_x = self.odom_y = self.odom_dist = 0.0
        self.roll = self.pitch = 0.0
        self.last_rx = time.monotonic()
        self.cur_pid = 0
        self.seq = 0

    def ms(self):
        return int((time.monotonic() - self.t0) * 1000)

    def send(self, cmd, values):
        self.seq = (self.seq + 1) & 0xFF
        self.write(P.encode(cmd, values, self.seq))

    def ack(self, cmd, seq, status=0):
        self.send(P.Cmd.ACK, {"ack_cmd": int(cmd), "ack_seq": seq, "status": status})

    def feed(self, data: bytes):
        for fr in self.parser.feed(data):
            self.last_rx = time.monotonic()
            self.handle(fr)

    def handle(self, fr: P.Frame):
        d = fr.parse()
        c = fr.cmd
        if c == int(P.Cmd.MOTION):
            self.vx, self.vy, self.omega = d["vx"], d["vy"], d["omega"]
            if d["flags"] & 0x01:
                self.vx = self.vy = self.omega = 0
        elif c == int(P.Cmd.BRAKE):
            self.vx = self.vy = self.omega = 0
            self.ack(c, fr.seq)
            print(f"[sim] BRAKE level={d['level']}")
        elif c == int(P.Cmd.MODE):
            print(f"[sim] MODE -> {d['mode']}")
            self.ack(c, fr.seq)
        elif c == int(P.Cmd.PID_READ):
            src = d.get("source", 0)
            store = self.plant.flash if src == 1 else self.plant.pid
            ids = list(store) if d["pid_id"] == 0xFF else [d["pid_id"]]
            for i in ids:
                if i in store:
                    v = dict(store[i])
                    v["pid_id"] = i
                    v["source"] = src
                    v["dirty"] = 0 if self.plant.pid[i] == self.plant.flash[i] else 1
                    self.send(P.Cmd.PID_VALUE, v)
        elif c == int(P.Cmd.PID_WRITE):
            i = d["pid_id"]
            if i in self.plant.pid:
                for k in ("kp", "ki", "kd", "i_limit", "out_limit"):
                    self.plant.pid[i][k] = d[k]
                self.cur_pid = i
                where = "RAM"
                if d.get("target", 0) == 1:          # 一步到位写 Flash
                    self.plant.flash[i] = dict(self.plant.pid[i])
                    where = "RAM+FLASH"
                self.ack(c, fr.seq)
                print(f"[sim] PID_WRITE[{where}] 0x{i:02X} "
                      f"kp={d['kp']:.3f} ki={d['ki']:.3f} kd={d['kd']:.3f}")
            else:
                self.ack(c, fr.seq, 3)
        elif c == int(P.Cmd.PID_SAVE):
            ids = list(self.plant.pid) if d["pid_id"] == 0xFF else [d["pid_id"]]
            ok = True
            for i in ids:
                if i in self.plant.pid:
                    self.plant.flash[i] = dict(self.plant.pid[i])
                else:
                    ok = False
            time.sleep(0.02)                          # 模拟 Flash 擦写耗时
            self.ack(c, fr.seq, 0 if ok else 3)
            print(f"[sim] PID_SAVE -> FLASH {[hex(i) for i in ids]} {'OK' if ok else 'ERR'}")
            self.send(P.Cmd.LOG, {"level": 1, "text": f"PID saved to flash x{len(ids)}"})
        elif c == int(P.Cmd.PID_REVERT):
            ids = list(self.plant.flash) if d["pid_id"] == 0xFF else [d["pid_id"]]
            for i in ids:
                if i in self.plant.flash:
                    self.plant.pid[i] = dict(self.plant.flash[i])
            self.ack(c, fr.seq)
            print(f"[sim] PID_REVERT <- FLASH {[hex(i) for i in ids]}")
            for i in ids:                              # 主动回报，刷新上位机界面
                if i in self.plant.pid:
                    v = dict(self.plant.pid[i])
                    v.update(pid_id=i, source=0, dirty=0)
                    self.send(P.Cmd.PID_VALUE, v)
        elif c == int(P.Cmd.PID_TARGET):
            i = d["pid_id"]
            if i in self.plant.target:
                self.plant.target[i] = d["target"]
                self.cur_pid = i
        elif c == int(P.Cmd.TELEM_CTRL):
            self.telem_on = bool(d["enable"])
            self.telem_period = max(d["period_ms"], 5) / 1000.0
            self.telem_mask = d["mask"]
            self.ack(c, fr.seq)
            print(f"[sim] TELEM {'ON' if self.telem_on else 'OFF'} {d['period_ms']}ms mask=0x{d['mask']:02X}")
        elif c == int(P.Cmd.STREAM_CFG):
            self.stream_on = bool(d["enable"])
            self.stream_period = max(d["period_ms"], 5) / 1000.0
            self.stream_channels = list(d["channels"])[:d["count"]]
            self.ack(c, fr.seq)
            names = ",".join(P.CHANNELS[i].name for i in self.stream_channels
                             if i in P.CHANNELS)
            print(f"[sim] STREAM {'ON' if self.stream_on else 'OFF'} "
                  f"{d['period_ms']}ms count={d['count']} [{names}]")
        elif c == int(P.Cmd.STREAM_LIST):
            for ch in P.CHANNELS.values():
                self.send(P.Cmd.STREAM_INFO,
                          {"channel_id": ch.id, "unit": 0, "name": ch.name})
        elif c == int(P.Cmd.RAW_TEXT):
            print(f"[sim] TEXT: {d['text']}")
            self.send(P.Cmd.LOG, {"level": 1, "text": "echo: " + d["text"]})
        elif c == int(P.Cmd.HEARTBEAT):
            pass

    def channel_value(self, cid: int) -> float:
        """按通道 ID 产生一个像样的模拟值。"""
        t = time.monotonic()
        n = random.gauss(0, 0.02)
        w = self.omega / 1000.0            # rad/s
        table = {
            0x01: self.roll + n, 0x02: self.pitch + n, 0x03: self.yaw + n,
            0x04: math.cos(math.radians(self.yaw) / 2), 0x05: 0.0 + n,
            0x06: 0.0 + n, 0x07: math.sin(math.radians(self.yaw) / 2),
            0x10: 2.0 * math.sin(t * 2) + n, 0x11: 2.0 * math.cos(t * 2) + n,
            0x12: math.degrees(w) + n,
            0x20: self.vx / 1000.0 * 2 + n, 0x21: self.vy / 1000.0 * 2 + n,
            0x22: 9.81 + n,
            0x30: 22.0 * math.cos(math.radians(self.yaw)) + n,
            0x31: 22.0 * math.sin(math.radians(self.yaw)) + n,
            0x32: -40.0 + n, 0x33: 36.5 + 0.5 * math.sin(t * 0.2),
            0x40: self.odom_x, 0x41: self.odom_y, 0x42: self.yaw,
            0x43: self.odom_dist,
            0x44: self.vx * 0.95 + n * 10, 0x45: self.vy * 0.95 + n * 10,
            0x46: self.omega * 0.95 + n * 10,
            0x60: 12.0 + 0.2 * math.sin(t), 0x61: 1.2 + abs(self.vx) / 2000.0,
            0x62: 35.0 + 5 * math.sin(t * 0.5), 0x63: 1.0 + 0.1 * abs(n),
        }
        if cid in table:
            return float(table[cid])
        if 0x50 <= cid <= 0x53:            # 实际轮速
            k = (self.vx + (1 if cid in (0x51, 0x53) else -1) * self.vy) / 20.0
            return k + n * 5
        if 0x54 <= cid <= 0x57:            # 目标轮速
            return (self.vx + (1 if cid in (0x55, 0x57) else -1) * self.vy) / 20.0
        if 0x70 <= cid <= 0x77:            # 用户通道
            return math.sin(t * (1 + (cid - 0x70) * 0.3)) * 100
        if 0x80 <= cid <= 0x8A:            # DT35 激光测距
            return self.dt35_value(cid)

        return 0.0

    # ---- DT35 激光测距模拟 ----
    # 假设车在一个 4m x 4m 的房间里，四周装了 DT35，距离随位置/航向变化
    ROOM = 4000.0    # mm

    def dt35_value(self, cid: int) -> float:
        n = random.gauss(0, 1.5)                 # DT35 精度约 ±1~2mm
        x = self.odom_x * 1000.0 + self.ROOM / 2
        y = self.odom_y * 1000.0 + self.ROOM / 2
        x = min(max(x, 100.0), self.ROOM - 100)
        y = min(max(y, 100.0), self.ROOM - 100)
        front, back = self.ROOM - x, x
        left, right = self.ROOM - y, y
        # 航向偏斜会让测距变长（1/cos 修正）
        k = 1.0 / max(math.cos(math.radians(self.yaw % 90)), 0.3)
        table = {
            0x80: front * k, 0x81: back * k, 0x82: left * k, 0x83: right * k,
            0x84: math.hypot(front, left) * 0.8, 0x85: math.hypot(front, right) * 0.8,
            0x86: math.hypot(back, left) * 0.8, 0x87: math.hypot(back, right) * 0.8,
        }
        if cid in table:
            v = table[cid] + n
            return float(min(max(v, 50.0), 35000.0))   # DT35 量程 50mm~35m
        if cid == 0x88:      # 模拟量输出 4~20mA -> 0~10V，这里给 mV
            d = min(max(front, 50.0), 10000.0)
            return d / 10000.0 * 10000.0 + n
        if cid == 0x89:      # 左右差值算贴边角度
            dl = table[0x82] + random.gauss(0, 1)
            dr = table[0x83] + random.gauss(0, 1)
            return math.degrees(math.atan2(dl - dr, self.ROOM)) 
        if cid == 0x8A:      # 有效性位图：全部有效
            return float(0xFF)
        return 0.0

    def tick(self, dt: float):
        for i in self.plant.pid:
            self.plant.step(i, dt)
        # 看门狗
        if time.monotonic() - self.last_rx > 1.0:
            self.vx = self.vy = self.omega = 0
        self.yaw = (self.yaw + self.omega / 1000.0 * dt * 57.2958) % 360.0
        # 里程计积分
        th = math.radians(self.yaw)
        self.odom_x += (self.vx * math.cos(th) - self.vy * math.sin(th)) / 1000.0 * dt
        self.odom_y += (self.vx * math.sin(th) + self.vy * math.cos(th)) / 1000.0 * dt
        self.odom_dist += math.hypot(self.vx, self.vy) / 1000.0 * dt
        self.roll = 3.0 * math.sin(time.monotonic() * 1.3)
        self.pitch = 2.0 * math.sin(time.monotonic() * 0.7)
        now = time.monotonic()
        if self.stream_on and self.stream_channels and now - self.last_stream >= self.stream_period:
            self.last_stream = now
            vals = [self.channel_value(i) for i in self.stream_channels]
            self.send(P.Cmd.STREAM_DATA, {"timestamp_ms": self.ms(), "values": vals})
        if self.telem_on and (self.telem_mask & 0x01) and now - self.last_telem >= self.telem_period:
            self.last_telem = now
            self.send(P.Cmd.TELEMETRY, {
                "timestamp_ms": self.ms(),
                "vx": int(self.vx * 0.95), "vy": int(self.vy * 0.95),
                "omega": int(self.omega * 0.95),
                "yaw": int(self.yaw * 100) % 36000,
                "battery_mv": 12000 + int(200 * math.sin(now)),
                "state": 1 if (self.vx or self.vy or self.omega) else 0,
                "err_code": 0})
        if self.telem_on and (self.telem_mask & 0x02) and now - self.last_curve >= 0.01:
            self.last_curve = now
            i = self.cur_pid
            self.send(P.Cmd.PID_CURVE, {
                "pid_id": i, "timestamp_ms": self.ms(),
                "target": self.plant.target[i],
                "feedback": self.plant.fb[i],
                "output": self.plant.out[i]})


def run_tcp(port: int):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    print(f"[sim] TCP 监听 0.0.0.0:{port} —— 上位机选『网络 TCP』连 127.0.0.1:{port}")
    while True:
        conn, addr = srv.accept()
        print(f"[sim] 上位机已连接 {addr}")
        conn.settimeout(0.005)
        sim = Sim(lambda b: conn.sendall(b))
        last = time.monotonic()
        try:
            while True:
                try:
                    data = conn.recv(4096)
                    if not data:
                        break
                    sim.feed(data)
                except socket.timeout:
                    pass
                now = time.monotonic()
                sim.tick(now - last)
                last = now
                time.sleep(0.002)
        except Exception as e:
            print(f"[sim] 连接结束: {e}")
        finally:
            conn.close()


def run_serial(dev: str, baud: int):
    import serial
    ser = serial.Serial(dev, baud, timeout=0.005)
    print(f"[sim] 串口模拟器运行于 {dev}@{baud}")
    sim = Sim(lambda b: ser.write(b))
    last = time.monotonic()
    while True:
        d = ser.read(ser.in_waiting or 1)
        if d:
            sim.feed(d)
        now = time.monotonic()
        sim.tick(now - last)
        last = now


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--tcp", type=int, default=None, help="TCP 监听端口")
    ap.add_argument("--serial", type=str, default=None, help="串口设备")
    ap.add_argument("--baud", type=int, default=115200)
    a = ap.parse_args()
    if a.serial:
        run_serial(a.serial, a.baud)
    else:
        run_tcp(a.tcp or 8888)
