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

import json
import struct
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


@dataclass(slots=True)
class CloudFrame:
    frame: int
    sim_time: float
    xs: tuple
    ys: tuple
    intensity: tuple

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
    except (TypeError, ValueError):
        return None

    return StateFrame(frame=frame, sim_time=sim_time, ego=ego, obstacles=obstacles)


def encode_command(seq: int, frame: int, sim_time: float,
                   steer: float, speed: float, accel: float) -> bytes:
    """编码一帧控制指令：长度前缀 + 紧凑 JSON。

    字段名必须与 `TcpUdpGateway::handlePayload` 读取的键一致：
    type / seq / frame / t / steer / speed / accel。
    """
    payload = json.dumps(
        {
            "type": "cmd",
            "seq": int(seq),
            "frame": int(frame),
            "t": float(sim_time),
            "steer": float(steer),
            "speed": float(speed),
            "accel": float(accel),
        },
        separators=(",", ":"),
    ).encode("utf-8")
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

        frame_id, sim_time, index, total, xs, ys, inten = decoded

        if total == 1:
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

        slot["parts"][index] = (xs, ys, inten)
        if len(slot["parts"]) < slot["total"]:
            return None

        del self._pending[frame_id]
        order = range(slot["total"])
        return CloudFrame(
            frame_id,
            slot["t"],
            tuple(p for i in order for p in slot["parts"][i][0]),
            tuple(p for i in order for p in slot["parts"][i][1]),
            tuple(p for i in order for p in slot["parts"][i][2]),
        )


def _decode_cloud(datagram: bytes):
    """解析单个 UDP 分片，返回 (frame, t, index, total, xs, ys, intensity)。"""
    if len(datagram) < CLOUD_HEADER.size:
        return None
    magic, version, _flags, index, total, frame, sim_time, count, _reserved = \
        CLOUD_HEADER.unpack_from(datagram, 0)
    if magic != MAGIC or version != VERSION or total == 0:
        return None

    need = CLOUD_HEADER.size + count * POINT.size
    if len(datagram) < need:
        return None

    xs: list[float] = []
    ys: list[float] = []
    inten: list[float] = []
    for x, y, i in POINT.iter_unpack(datagram[CLOUD_HEADER.size:need]):
        xs.append(x)
        ys.append(y)
        inten.append(i)
    return frame, sim_time, index, total, tuple(xs), tuple(ys), tuple(inten)
