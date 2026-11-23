#pragma once

namespace adasim::domain {

/// 车辆与积分参数。默认值取自开题报告 5.3.4 的标定结果。
struct VehicleParams {
    double wheelBase{2.7};      ///< 轴距，m
    double maxSteer{0.6};       ///< 最大前轮转角，rad
    double accelMin{-6.0};      ///< m/s^2
    double accelMax{3.0};       ///< m/s^2
    double speedMin{0.0};       ///< m/s，本平台暂不支持倒车
    double speedMax{40.0};      ///< m/s
};

}  // namespace adasim::domain
