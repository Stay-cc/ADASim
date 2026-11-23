#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "domain/model/WorldState.h"

namespace adasim::domain {

/// 环形缓冲中的一条记录。
struct FrameRecord {
    WorldState state{};
    bool isSnapshot{false};
};

/// 回溯定位方案：从哪一帧的完整快照出发，需要重推演多少步。
struct SeekPlan {
    FrameId baseFrame{0};
    WorldState baseState{};
    FrameId targetFrame{0};
    std::uint64_t replaySteps{0};
};

/// 滚动窗口回放缓冲。
///
/// 并发模型：单写多读。物理线程是唯一写者，按 50 Hz 顺序 append；
/// 渲染线程与回溯路径只读已提交的帧。读取方必须在写指针绕回覆盖之前完成拷贝，
/// 该约束由窗口长度（默认 1500 帧 / 30 s）自然满足。
///
/// 快照策略：每 snapshotEvery 帧存一份完整状态，其余帧只存自身状态。
/// 回溯时若命中快照则直接返回，否则从最近快照重推演——这是"内存占用"
/// 与"重推演长度"之间的折中，默认 25 帧约为 0.5 s。
class TimelineBuffer {
public:
    TimelineBuffer(FrameId capacity, std::uint64_t snapshotEvery);

    /// 追加一帧。帧号由缓冲分配，返回该帧号。仅允许单线程调用。
    FrameId append(WorldState state);

    [[nodiscard]] bool empty() const noexcept { return nextFrame_ == begin_; }
    [[nodiscard]] std::size_t size() const noexcept { return static_cast<std::size_t>(nextFrame_ - begin_); }
    [[nodiscard]] FrameId capacity() const noexcept { return static_cast<FrameId>(ring_.size()); }
    [[nodiscard]] std::uint64_t snapshotEvery() const noexcept { return snapshotEvery_; }

    /// 窗口内最旧的有效帧号。
    [[nodiscard]] FrameId earliest() const noexcept { return begin_; }
    /// 最新已提交的帧号；缓冲为空时无意义（用 empty() 判断）。
    [[nodiscard]] FrameId latest() const noexcept { return nextFrame_ == 0 ? 0 : nextFrame_ - 1; }

    /// 回溯后继续仿真：写指针回退到 frame，其后的帧被丢弃。
    /// 这是"覆盖式单时间轴"语义——调试时改参数后从回退点重跑，
    /// 而不是分叉出第二条时间线。
    void rewindTo(FrameId frame);

    [[nodiscard]] std::optional<FrameRecord> at(FrameId id) const;

    /// 返回不晚于 id 的最近一个完整快照。
    [[nodiscard]] std::optional<FrameRecord> nearestSnapshot(FrameId id) const;

    /// 生成回溯方案：给出重推演起点与步数。目标帧不在窗口内时返回 nullopt。
    [[nodiscard]] std::optional<SeekPlan> planSeek(FrameId target) const;

    /// 粗略内存占用估算，用于界面显示与实验记录。
    [[nodiscard]] std::size_t approximateBytes() const;

    void clear();

private:
    [[nodiscard]] std::optional<std::size_t> indexOf(FrameId id) const;

    std::vector<FrameRecord> ring_;
    std::uint64_t snapshotEvery_;
    FrameId nextFrame_{0};  ///< 下一帧号，仅写者推进
    FrameId begin_{0};      ///< 窗口内最旧帧号
};

}  // namespace adasim::domain
