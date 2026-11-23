#pragma once

#include <memory>

#include "domain/model/Viewport.h"
#include "domain/model/WorldSnapshot.h"

namespace adasim::domain {

/// 渲染端口。领域层只表达"把这一帧画出来"，不规定用什么画。
///
/// 快照以 shared_ptr<const WorldSnapshot> 传入：
///   1. 渲染线程读到的对象不可变，无需加锁；
///   2. 物理线程下一帧会新建快照，不会复用正在被读取的那一份。
/// 这构成单写多读（single-writer / multi-reader）的基础。
struct IRenderPort {
    virtual ~IRenderPort() = default;

    virtual void submit(std::shared_ptr<const WorldSnapshot> snapshot) = 0;
    virtual void setViewport(const Viewport& viewport) = 0;
    virtual void requestFrame() = 0;
};

}  // namespace adasim::domain
