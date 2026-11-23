#include "domain/timeline/TimelineBuffer.h"

#include <stdexcept>
#include <utility>

namespace adasim::domain {

TimelineBuffer::TimelineBuffer(FrameId capacity, std::uint64_t snapshotEvery)
    : snapshotEvery_(snapshotEvery == 0 ? 1 : snapshotEvery) {
    if (capacity == 0) {
        throw std::invalid_argument("TimelineBuffer: capacity 必须大于 0");
    }
    ring_.resize(static_cast<std::size_t>(capacity));
}

FrameId TimelineBuffer::append(WorldState state) {
    const FrameId id = nextFrame_;
    state.frameId = id;

    FrameRecord& slot = ring_[static_cast<std::size_t>(id % ring_.size())];
    slot.state = std::move(state);
    slot.isSnapshot = (id % snapshotEvery_ == 0);

    ++nextFrame_;
    if (nextFrame_ - begin_ > ring_.size()) {
        begin_ = nextFrame_ - static_cast<FrameId>(ring_.size());
    }
    return id;
}

void TimelineBuffer::rewindTo(FrameId frame) {
    if (frame >= nextFrame_) {
        return;  // 还没写到这里，无需回退
    }
    if (frame < begin_) {
        return;  // 该帧已被环形覆盖，无法回到更早的位置
    }
    nextFrame_ = frame + 1;
}

std::optional<std::size_t> TimelineBuffer::indexOf(FrameId id) const {
    if (nextFrame_ == begin_) {
        return std::nullopt;
    }
    if (id < begin_ || id >= nextFrame_) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(id % ring_.size());
}

std::optional<FrameRecord> TimelineBuffer::at(FrameId id) const {
    const auto idx = indexOf(id);
    if (!idx.has_value()) {
        return std::nullopt;
    }
    const FrameRecord& rec = ring_[*idx];
    // 槽位可能已被更晚的帧覆盖，用帧号二次确认
    if (rec.state.frameId != id) {
        return std::nullopt;
    }
    return rec;
}

std::optional<FrameRecord> TimelineBuffer::nearestSnapshot(FrameId id) const {
    if (nextFrame_ == begin_ || id < begin_) {
        return std::nullopt;
    }
    FrameId probe = (id > latest()) ? latest() : id;
    while (true) {
        const auto rec = at(probe);
        if (rec.has_value() && rec->isSnapshot) {
            return rec;
        }
        if (probe == earliest()) {
            return std::nullopt;
        }
        --probe;
    }
}

std::optional<SeekPlan> TimelineBuffer::planSeek(FrameId target) const {
    const auto base = nearestSnapshot(target);
    if (!base.has_value()) {
        return std::nullopt;
    }
    const FrameId baseFrame = base->state.frameId;
    if (target < baseFrame) {
        return std::nullopt;
    }

    SeekPlan plan;
    plan.baseFrame = baseFrame;
    plan.baseState = base->state;
    plan.targetFrame = target;
    plan.replaySteps = target - baseFrame;
    return plan;
}

std::size_t TimelineBuffer::approximateBytes() const {
    std::size_t bytes = ring_.size() * sizeof(FrameRecord);
    for (FrameId f = begin_; f < nextFrame_; ++f) {
        const auto idx = indexOf(f);
        if (idx.has_value()) {
            bytes += ring_[*idx].state.obstacles.capacity() * sizeof(EntityState);
        }
    }
    return bytes;
}

void TimelineBuffer::clear() {
    for (auto& rec : ring_) {
        rec = FrameRecord{};
    }
    nextFrame_ = 0;
    begin_ = 0;
}

}  // namespace adasim::domain
