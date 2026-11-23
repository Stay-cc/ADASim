#pragma once

#include <cstddef>
#include <vector>

#include "domain/model/WorldState.h"

namespace adasim::domain {

/// 交给渲染层的只读快照。
///
/// 点云按 SoA（structure of arrays）存放：三个分量分别连续存储，
/// 便于渲染端做向量化处理与批量提交，也便于按着色档位切分区间。
/// 该对象以 shared_ptr<const WorldSnapshot> 传递，跨线程只读共享，避免深拷贝。
struct WorldSnapshot {
    WorldState state{};
    std::vector<float> px;
    std::vector<float> py;
    std::vector<float> intensity;

    [[nodiscard]] std::size_t pointCount() const noexcept { return px.size(); }

    void clearPoints() {
        px.clear();
        py.clear();
        intensity.clear();
    }
};

}  // namespace adasim::domain
