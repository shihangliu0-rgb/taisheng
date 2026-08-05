import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from upper_computer import protocol as P
from upper_computer.config import MotionConfig
from upper_computer.motion import MotionEngine, MotionCommand


def test_crc_known_vector():
    assert P.crc16(b"123456789") == 0x4B37


def test_roundtrip_all_messages():
    parser = P.FrameParser()
    for cmd, msg in P.MESSAGES.items():
        vals = {}
        for f in msg.fields:
            if f.type == "str":
                vals[f.name] = "abc"
            elif f.type == "u8[]":
                vals[f.name] = [1, 2, 3]
            elif f.type == "f32[]":
                vals[f.name] = [1.5, 2.5]
            elif f.type == "f32":
                vals[f.name] = 2.5
            else:
                vals[f.name] = 7
        data = P.encode(P.Cmd(cmd), vals, seq=9)
        frames = parser.feed(data)
        assert len(frames) == 1, msg.name
        fr = frames[0]
        assert fr.cmd == cmd and fr.seq == 9
        out = fr.parse()
        for f in msg.fields:
            if f.type == "f32":
                assert abs(out[f.name] - 2.5) < 1e-6
            elif f.type == "f32[]":
                assert all(abs(a - b) < 1e-6
                           for a, b in zip(out[f.name], vals[f.name]))
            else:
                assert out[f.name] == vals[f.name], (msg.name, f.name)


def test_parser_handles_garbage_and_split():
    parser = P.FrameParser()
    good = P.encode(P.Cmd.MOTION, {"vx": 100, "vy": -50, "omega": 3, "flags": 1, "seq_echo": 0})
    stream = b"\x00\xffnoise" + good + b"\xaa" + good
    out = []
    for i in range(0, len(stream), 3):        # 模拟分片到达
        out += parser.feed(stream[i:i + 3])
    assert len(out) == 2
    d = out[0].parse()
    assert d["vx"] == 100 and d["vy"] == -50 and d["flags"] == 1


def test_crc_error_detected():
    parser = P.FrameParser()
    data = bytearray(P.encode(P.Cmd.BRAKE, {"level": 1}))
    data[-1] ^= 0xFF
    assert parser.feed(bytes(data)) == []
    assert parser.stat_crc_err == 1


def test_text_frame():
    b = P.encode_text(P.Cmd.MOTION, {"vx": 500, "vy": 0, "omega": -1200, "flags": 0, "seq_echo": 0})
    name, args = P.parse_text_line(b.decode())
    assert name == "MOTION" and args[0] == "500" and args[2] == "-1200"


def test_motion_diagonal_normalized():
    cfg = MotionConfig(max_vx=1000, max_vy=1000, accel_ms=0, decel_ms=0)
    e = MotionEngine(cfg)
    e.press("forward")
    e.press("left")
    e.update()
    c = e.update()
    assert abs((c.vx ** 2 + c.vy ** 2) ** 0.5 - 1000) < 20


def test_motion_combo_and_brake():
    cfg = MotionConfig(accel_ms=0, decel_ms=0)
    e = MotionEngine(cfg)
    e.press("forward")
    e.press("cw")
    e.update()
    c = e.update()
    assert c.vx > 0 and c.omega < 0          # 边走边顺时针转
    e.press("brake")
    e.update()
    c = e.update()
    assert c.is_zero() is False and c.flags & MotionCommand.FLAG_BRAKE
    assert c.vx == 0 and c.omega == 0


def test_boost_slow_scaling():
    cfg = MotionConfig(max_vx=1000, boost_scale=2.0, slow_scale=0.5, accel_ms=0, decel_ms=0)
    e = MotionEngine(cfg)
    e.press("forward")
    e.update()
    base = e.update().vx
    e.press("boost")
    e.update()
    assert e.update().vx == base * 2
    e.release("boost")
    e.press("slow")
    e.update()
    assert e.update().vx == base // 2


def test_doc_generation_matches_protocol():
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "tools"))
    import gen_protocol_doc as g
    text = g.gen()
    for msg in P.MESSAGES.values():
        assert msg.name in text
        for f in msg.fields:
            assert f"`{f.name}`" in text


# ---------------------------------------------------------------------------
# v1.1 新增：数据流订阅 / 通道表
# ---------------------------------------------------------------------------
def test_stream_cfg_variable_channels():
    ids = [0x03, 0x40, 0x41]
    data = P.encode(P.Cmd.STREAM_CFG, {"enable": 1, "period_ms": 20,
                                       "count": len(ids), "channels": ids})
    fr = P.FrameParser().feed(data)[0]
    d = fr.parse()
    assert d["enable"] == 1 and d["count"] == 3 and d["channels"] == ids
    # payload = enable(1) + period(2) + count(1) + 3 通道
    assert len(fr.payload) == 4 + 3


def test_stream_data_float_array():
    vals = [12.5, -3.25, 0.125]
    data = P.encode(P.Cmd.STREAM_DATA, {"timestamp_ms": 9999, "values": vals})
    d = P.FrameParser().feed(data)[0].parse()
    assert d["timestamp_ms"] == 9999
    assert all(abs(a - b) < 1e-6 for a, b in zip(d["values"], vals))
    assert len(d["values"]) == 3


def test_stream_bandwidth_scales_with_subscription():
    """只订阅需要的通道，帧长应显著小于全量。"""
    few = P.encode(P.Cmd.STREAM_DATA, {"timestamp_ms": 1, "values": [0.0] * 3})
    many = P.encode(P.Cmd.STREAM_DATA, {"timestamp_ms": 1, "values": [0.0] * 40})
    assert len(few) == P.FRAME_OVERHEAD + 4 + 12
    assert len(many) > 4 * len(few)


def test_channel_registry_covers_imu():
    names = set(P.CHANNEL_BY_NAME)
    for must in ("imu_roll", "imu_pitch", "imu_yaw", "gyro_x", "gyro_y", "gyro_z",
                 "accel_x", "accel_y", "accel_z", "mag_x", "mag_z",
                 "odom_x", "odom_y", "odom_dist", "vel_x"):
        assert must in names, must
    assert len(P.CHANNELS) >= 40
    # ID 唯一
    assert len(P.CHANNELS) == len({c.id for c in P.CHANNELS.values()})
    assert len(P.channel_groups()) >= 6


def test_all_channels_have_simulator_values():
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "tools"))
    import simulator
    sim = simulator.Sim(lambda b: None)
    for cid in P.CHANNELS:
        v = sim.channel_value(cid)
        assert isinstance(v, float)
