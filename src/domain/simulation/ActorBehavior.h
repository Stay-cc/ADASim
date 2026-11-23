#pragma once

#include "domain/model/WorldState.h"

namespace adasim::domain {

/// 按各自行为推进全部交通参与者 dt 秒。
///
/// `in` 是推进之前的世界状态，`out` 在其基础上被改写。两者分开传参不是冗余：
/// 跟驰要找"前车"，若边推进边查前车，推进顺序就会影响结果；
/// 一律以步进前的状态为准，结果与参与者排列顺序无关。
void advanceActors(const WorldState& in, WorldState& out, double dt);

/// 推进单个参与者。导出是为了让用例能单独验证某一种行为，
/// 不必为了测切入而把整个世界跑起来。
void advanceActor(const WorldState& in, const EntityState& before, EntityState& after,
                  double dt);

/// 计算某个跟随者按 IDM 应有的纵向加速度（前车从状态里自动查找）。
///
/// 参与者推进用它，控制用例也用它。共用一套公式与参数，是为了避免
/// "场景里那辆车怎么减速"和"我认为它应该怎么减速"两处各写一份而分叉——
/// 这类分叉在数值上不会报错，只会让对照实验悄悄失真。
[[nodiscard]] double idmAccelFor(const WorldState& state, const EntityState& self);

/// 切入的横向位移曲线（余弦型，端点横向速度为零）。
///
/// 单独暴露的原因：它决定了点云里那辆车的航向角，是最容易被怀疑"穿模/瞬移"的地方，
/// 用例需要直接核对曲线而不是隔着 20 秒闭环去猜。
/// @param progress 0..1，触发后的进度
/// @return 位移占总横向距离的比例
[[nodiscard]] double cutInLateralRatio(double progress);

}  // namespace adasim::domain
