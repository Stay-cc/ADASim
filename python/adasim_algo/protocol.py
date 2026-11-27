"""ADASim 线上协议编解码。

本模块是 C++ 网关 `src/infrastructure/net/TcpUdpGateway.cpp` 的镜像实现，
两端的字节布局必须完全一致。因此所有 `struct` 定义只出现在本文件，
其余模块（perception / planning / node）只使用这里给出的数据类与函数，
不直接接触字节偏移——协议一旦变更，改动被限制在一个文件内。

两条通道的分工：
  * TCP：结构化状态，需要可靠有序。4 字节大端长度前缀 + 紧凑 JSON。
  * UDP：高频点云，允许丢帧。24 字节分片头 + 若干 12 字节点记录。
"""

from __future__ import annotations

import array
import json
import struct
import sys
from dataclasses import dataclass, field

# ---- 线上格式常量（与 C++ 侧逐一对应）----
TCP_LENGTH = struct.Struct(">I")          # 长度前缀，大端 uint32
CLOUD_HEADER = struct.Struct(">2sBBHHQfHH")  # 24 字节分片头
POINT = struct.Struct(">fff")             # 每个点 12 字节：x, y, intensity

MAGIC = b"AS"
VERSION = 1

#: 与 C++ 侧一致的上限：超过即视为协议失步，不能当作合法长度继续解析。
MAX_PAYLOAD = 8 * 1024 * 1024


@dataclass(slots=True)
class EgoState:
    """自车位姿。来自定位而非感知，算法链路可以放心使用。"""

    x: float
    y: float
    theta: float
    speed: float


@dataclass(slots=True)
class TruthObstacle:
    """场景真值障碍物。

    只在 `--use-truth` 对照实验中使用。默认链路完全走点云感知，
    否则"传感器级闭环"这个设计目标就成了空话。
    """

    id: int
    kind: str
    x: float
    y: float
    theta: float
    speed: float
    length: float
    width: float


@dataclass(slots=True)
class StateFrame:
    frame: int
    sim_time: float
    ego: EgoState
    obstacles: list[TruthObstacle] = field(default_factory=list)
    # 道路曲率，1/m，正值左弯。老版本底座不发这个字段，缺省按直道处理。
    curvature: float = 0.0
    # 分段道路（超大地图）。为空时算法沿用单曲率 curvature。
    road_segments: list = field(default_factory=list)
    # 自车沿道路里程，用于在分段道路上定位参考点。
    odometer: float = 0.0
    # 自车目标车道中心相对道路中心线的横向偏移。横向控制跟踪此偏移，
    # 使自车保持在初始车道中心而非压着道路中心线。
    target_lateral: float = 0.0
    # 道路车道数与车道宽，变道避障据此判断邻道是否存在及邻道中心位置。
    lane_count: int = 2
    lane_width: float = 3.5
    # 是否允许自车变道避障（由场景下发）。false 时即使前车慢/近也不变道。
    lane_change_enabled: bool = False
    ack_seq: int = -1


@dataclass(slots=True)
class CloudFrame:
    """一帧完整点云。

    xs / ys / intensity 是 `array('f')` 而不是 tuple：它们由批量解码一次切出，
    支持下标与步长切片（`xs[::stride]`，感知抽稀要用），构建成本几乎为零。
    下游只当作"可下标、可切片、有 len"的序列使用，不依赖具体类型。
    """

    frame: int
    sim_time: float
    xs: object
    ys: object
    intensity: object

    def __len__(self) -> int:
        return len(self.xs)


def parse_state(payload: bytes) -> StateFrame | None:
    """解析一帧 TCP 状态报文。字段缺失或类型不符时返回 None。

    刻意做成"容错解析"而不是抛异常：算法进程不能因为一条畸形报文
    就退出，否则仿真侧一次偶发错误会连带整个闭环中断。
    """
    try:
        obj = json.loads(payload)
    except (ValueError, UnicodeDecodeError):
        return None
    if not isinstance(obj, dict) or obj.get("type") != "state":
        return None

    ego_raw = obj.get("ego") or {}
    try:
        ego = EgoState(
            x=float(ego_raw.get("x", 0.0)),
            y=float(ego_raw.get("y", 0.0)),
            theta=float(ego_raw.get("theta", 0.0)),
            speed=float(ego_raw.get("v", 0.0)),
        )
    except (TypeError, ValueError):
        return None

    obstacles: list[TruthObstacle] = []
    for o in obj.get("obstacles") or ():
        if not isinstance(o, dict):
            continue
        try:
            obstacles.append(
                TruthObstacle(
                    id=int(o.get("id", 0)),
                    kind=str(o.get("kind", "unknown")),
                    x=float(o.get("x", 0.0)),
                    y=float(o.get("y", 0.0)),
                    theta=float(o.get("theta", 0.0)),
                    speed=float(o.get("v", 0.0)),
                    length=float(o.get("l", 4.5)),
                    width=float(o.get("w", 1.9)),
                )
            )
        except (TypeError, ValueError):
            continue

    try:
        frame = int(obj.get("frame", 0))
        sim_time = float(obj.get("t", 0.0))
        curvature = float(obj.get("curvature", 0.0))
        odometer = float(obj.get("odometer", 0.0))
        target_lateral = float(obj.get("targetLateral", 0.0))
        lane_count = int(obj.get("laneCount", 2))
        lane_width = float(obj.get("laneWidth", 3.5))
        lane_change_enabled = bool(obj.get("laneChangeEnabled", False))
        ack_seq = int(obj.get("ackSeq", -1))
    except (TypeError, ValueError):
        return None

    road_segments = []
    for seg in obj.get("roadSegments") or ():
        if isinstance(seg, dict):
            road_segments.append({
                "startS": float(seg.get("startS", 0.0)),
                "endS": float(seg.get("endS", 0.0)),
                "curvature": float(seg.get("curvature", 0.0)),
                "laneCount": int(seg.get("laneCount", 2)),
                "laneWidth": float(seg.get("laneWidth", 3.5)),
            })

    return StateFrame(frame=frame, sim_time=sim_time, ego=ego,
                      obstacles=obstacles, curvature=curvature,
                      road_segments=road_segments, odometer=odometer,
                      target_lateral=target_lateral,
                      lane_count=lane_count, lane_width=lane_width,
                      lane_change_enabled=lane_change_enabled,
                      ack_seq=ack_seq)


def encode_command(seq: int, frame: int, sim_time: float,
                   steer: float, speed: float, accel: float,
                   reason: str = "", rtt_ms: float | None = None) -> bytes:
    """编码一帧控制指令：长度前缀 + 紧凑 JSON。

    字段名必须与 `TcpUdpGateway::handlePayload` 读取的键一致：
    type / seq / frame / t / steer / speed / accel，外加两个可选的
    reason（决策原因）与 rtt（本端实测往返时延）。两个都是给 HUD 显示用的，
    底座不据此改变任何控制行为——若 reason 参与了控制，"算法说自己在跟驰"
    就会变成既能改显示又能改行为的两面量。
    """
    body = {
        "type": "cmd",
        "seq": int(seq),
        "frame": int(frame),
        "t": float(sim_time),
        "steer": float(steer),
        "speed": float(speed),
        "accel": float(accel),
    }
    if reason:
        body["reason"] = str(reason)
    if rtt_ms is not None and rtt_ms >= 0.0:
        body["rtt"] = float(rtt_ms)
    payload = json.dumps(body, separators=(",", ":")).encode("utf-8")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"指令报文超过上限：{len(payload)} 字节")
    return TCP_LENGTH.pack(len(payload)) + payload


class FrameDecoder:
    """TCP 字节流 → 完整报文列表。

    TCP 是字节流，不自行分帧就会出现"两次发送粘成一个包"或
    "一个包被拆成两段"。剩余半包留在缓冲区里等下一批字节。
    """

    __slots__ = ("_buf", "resyncs")

    def __init__(self) -> None:
        self._buf = bytearray()
        self.resyncs = 0

    def feed(self, data: bytes) -> list[bytes]:
        self._buf += data
        buf = self._buf
        out: list[bytes] = []
        while len(buf) >= TCP_LENGTH.size:
            (length,) = TCP_LENGTH.unpack_from(buf, 0)
            if length == 0 or length > MAX_PAYLOAD:
                # 与 C++ 侧同一策略：长度非法即判定失步，清空缓冲重新同步。
                # 若不这样做，会一直卡在这个非法长度上空转。
                self.resyncs += 1
                buf.clear()
                break
            if len(buf) < TCP_LENGTH.size + length:
                break  # 半包，等下一批
            start = TCP_LENGTH.size
            out.append(bytes(buf[start:start + length]))
            del buf[:start + length]
        return out


class CloudReassembler:
    """UDP 分片重组。

    分片可能乱序、可能丢片。策略是：
      * 收齐 chunkCount 片才产出完整帧，半成品不交给感知；
      * 只保留最近若干个未完成帧，防止丢片导致内存无界增长；
      * 单分片帧（chunkCount == 1）走快路径，省掉一次字典操作。

    这里**不逐点解包**。十万点会分成 858 片，逐片 `iter_unpack` 再跨片拼接
    tuple，实测单帧 32.6 ms——占十万点净时延的三分之一，比感知本身还贵。
    改成"先拼字节、再一次性 `array.frombytes` + 分列切片"后同一帧 1.9 ms：
    两处的差别只是把 12 万次 Python 层循环换成了 1 次 C 层转换。
    分片载荷按原样暂存，等齐了才解码，因此每帧只付一次转换成本。
    """

    __slots__ = ("_pending", "_max_pending", "dropped", "corrupt")

    def __init__(self, max_pending: int = 4) -> None:
        self._pending: dict[int, dict] = {}
        self._max_pending = max(1, max_pending)
        self.dropped = 0
        self.corrupt = 0

    def feed(self, datagram: bytes) -> CloudFrame | None:
        decoded = _decode_cloud(datagram)
        if decoded is None:
            self.corrupt += 1
            return None

        frame_id, sim_time, index, total, payload, count = decoded

        if total == 1:
            xs, ys, inten = _split_payload(payload, count)
            return CloudFrame(frame_id, sim_time, xs, ys, inten)
        if index >= total:
            self.corrupt += 1
            return None

        slot = self._pending.get(frame_id)
        if slot is None:
            if len(self._pending) >= self._max_pending:
                # dict 保持插入序，弹出最早的那个即为最旧的未完成帧。
                oldest = next(iter(self._pending))
                del self._pending[oldest]
                self.dropped += 1
            slot = {"t": sim_time, "total": total, "parts": {}}
            self._pending[frame_id] = slot

        slot["parts"][index] = payload
        if len(slot["parts"]) < slot["total"]:
            return None

        del self._pending[frame_id]
        parts = slot["parts"]
        blob = b"".join(parts[i] for i in range(slot["total"]))
        xs, ys, inten = _split_payload(blob, len(blob) // POINT.size)
        return CloudFrame(frame_id, slot["t"], xs, ys, inten)


def _split_payload(payload: bytes, count: int):
    """把 count 个 (x, y, intensity) 记录一次性拆成三列。

    `array('f')` 按本机字节序解释内存，而线上是大端，所以在小端机上要
    `byteswap()`——这一步在 C 层完成，比逐点 `unpack` 快两个数量级。
    返回的是 array('f') 而不是 tuple：它支持下标与切片（抽稀要用 `[::stride]`），
    内存占用与 tuple 相当，构建成本几乎为零。
    """
    arr = array.array("f")
    arr.frombytes(payload[:count * POINT.size])
    if sys.byteorder == "little":
        arr.byteswap()
    return arr[0::3], arr[1::3], arr[2::3]


def _decode_cloud(datagram: bytes):
    """解析单个 UDP 分片头，返回 (frame, t, index, total, payload, count)。

    只切到头，不碰点数据：点数据的解码推迟到整帧收齐之后（见 _split_payload）。
    """
    if len(datagram) < CLOUD_HEADER.size:
        return None
    magic, version, _flags, index, total, frame, sim_time, count, _reserved = \
        CLOUD_HEADER.unpack_from(datagram, 0)
    if magic != MAGIC or version != VERSION or total == 0:
        return None

    need = CLOUD_HEADER.size + count * POINT.size
    if len(datagram) < need:
        return None
    return frame, sim_time, index, total, bytes(datagram[CLOUD_HEADER.size:need]), count
