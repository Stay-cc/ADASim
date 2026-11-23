# ---------------------------------------------------------------------------
# 目标 1：adasim_domain（L3 领域层）
#
# 硬性约束：本目标不 find_package(Qt6)，也不链接任何 Qt 组件。
# 一旦有人在 domain/ 下引入 Qt 符号，链接阶段就会失败——这是分层的最终保障。
# ---------------------------------------------------------------------------

set(ADASIM_DOMAIN_SOURCES
    model/ForwardGap.cpp
    timeline/TimelineBuffer.cpp
    simulation/PhysicsEngine.cpp)

add_library(adasim_domain STATIC ${ADASIM_DOMAIN_SOURCES})

target_include_directories(adasim_domain
    PUBLIC ${PROJECT_SOURCE_DIR}/src)

target_compile_features(adasim_domain PUBLIC cxx_std_17)

# 第一道防线：字面检查，早于编译器
adasim_enforce_layer("领域层 L3" "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
    "#include <Q"
    "#include <Qt"
    "#include <QtCore"
    "#include \"infrastructure"
    "#include \"presentation"
    "#include \"application"
    "QTcpSocket"
    "QJsonDocument"
    "QWidget")

adasim_enforce_layer("领域层 L3" "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "#include <Q"
    "#include \"infrastructure"
    "#include \"presentation"
    "#include \"application")

message(STATUS "ADASim: adasim_domain -> 纯 C++17 静态库（无 Qt 依赖）")
