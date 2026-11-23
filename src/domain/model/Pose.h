#pragma once

#include <cmath>

namespace adasim::domain {

/// 平面位姿。角度单位为弧度，逆时针为正，原点与朝向由场景坐标系约定。
struct Pose {
    double x{0.0};
    double y{0.0};
    double theta{0.0};

    [[nodiscard]] double distanceTo(const Pose& other) const noexcept {
        return std::hypot(x - other.x, y - other.y);
    }
};

}  // namespace adasim::domain
