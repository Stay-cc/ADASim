#include "domain/simulation/PhysicsEngine.h"

#include <algorithm>
#include <cmath>

#include "domain/simulation/ActorBehavior.h"

namespace adasim::domain {
namespace {
constexpr double kPi = 3.14159265358979323846;

struct Derivative {
    double dx{0.0};
    double dy{0.0};
    double dtheta{0.0};
};

/// 运动学自行车模型的连续时间导数：
///   ẋ = v·cosθ， ẏ = v·sinθ， θ̇ = v·tanδ / L
Derivative evaluate(const Pose& pose, double speed, double steer, double wheelBase) {
    Derivative d;
    d.dx = speed * std::cos(pose.theta);
    d.dy = speed * std::sin(pose.theta);
    d.dtheta = speed * std::tan(steer) / wheelBase;
    return d;
}

Pose advance(const Pose& pose, const Derivative& d, double h) {
    return Pose{pose.x + d.dx * h, pose.y + d.dy * h, pose.theta + d.dtheta * h};
}

Pose combine(const Pose& pose, const Derivative& k1, const Derivative& k2,
             const Derivative& k3, const Derivative& k4, double h) {
    const double sixth = h / 6.0;
    return Pose{
        pose.x + sixth * (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx),
        pose.y + sixth * (k1.dy + 2.0 * k2.dy + 2.0 * k3.dy + k4.dy),
        pose.theta + sixth * (k1.dtheta + 2.0 * k2.dtheta + 2.0 * k3.dtheta + k4.dtheta),
    };
}

}  // namespace

PhysicsEngine::PhysicsEngine(VehicleParams params) : params_(params) {}

Pose PhysicsEngine::integrate(const Pose& pose, double speed, double steer, double dt) const {
    const double L = params_.wheelBase;

    const Derivative k1 = evaluate(pose, speed, steer, L);
    const Derivative k2 = evaluate(advance(pose, k1, dt * 0.5), speed, steer, L);
    const Derivative k3 = evaluate(advance(pose, k2, dt * 0.5), speed, steer, L);
    const Derivative k4 = evaluate(advance(pose, k3, dt), speed, steer, L);

    return combine(pose, k1, k2, k3, k4, dt);
}

WorldState PhysicsEngine::step(const WorldState& in, double dt,
                               const ControlCommand& cmd) const {
    WorldState out = in;
    out.frameId = in.frameId + 1;
    out.simTime = in.simTime + dt;

    // 控制量限幅：转角受机械限制，加速度受动力与制动能力限制
    const double steer = std::clamp(cmd.steer, -params_.maxSteer, params_.maxSteer);
    const double accel = std::clamp(cmd.targetAccel, params_.accelMin, params_.accelMax);

    out.egoSpeed = std::clamp(in.egoSpeed + accel * dt, params_.speedMin, params_.speedMax);
    out.egoAccel = accel;
    out.steer = steer;
    out.ego = integrate(in.ego, out.egoSpeed, steer, dt);
    out.ego.theta = std::remainder(out.ego.theta, 2.0 * kPi);

    // 障碍物按各自行为推进（匀速 / 跟驰 / 切入）。
    // 行为推进与自车积分分开：自车由控制指令驱动，参与者由行为模型驱动。
    advanceActors(in, out, dt);

    return out;
}

}  // namespace adasim::domain
