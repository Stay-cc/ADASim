#pragma once

namespace adasim::domain {

/// 视口参数。缩放与平移只改这里，不重算点坐标——
/// 渲染端把世界坐标到屏幕坐标的映射收敛为一个仿射变换。
struct Viewport {
    double centerX{0.0};
    double centerY{0.0};
    double metersPerPixel{0.1};  ///< 缩放：每像素代表多少米
    int widthPx{800};
    int heightPx{600};
};

}  // namespace adasim::domain
