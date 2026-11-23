# ---------------------------------------------------------------------------
# 目标 2：adasim_app（L2 应用层）
#
# 只链接 adasim_domain。没有 Qt、看不到 adasim_infra——
# 因此应用层无法 #include 任何渲染器或网关实现，只能操作端口接口。
# ---------------------------------------------------------------------------

set(ADASIM_APP_SOURCES
    SimController.cpp
    StandbyGuard.cpp
    ScenarioService.cpp
    ExperimentRunner.cpp)

add_library(adasim_app STATIC ${ADASIM_APP_SOURCES})

target_link_libraries(adasim_app PUBLIC adasim_domain)

target_compile_features(adasim_app PUBLIC cxx_std_17)

adasim_enforce_layer("应用层 L2" "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
    "#include <Q"
    "#include <Qt"
    "#include \"infrastructure"
    "#include \"presentation")

adasim_enforce_layer("应用层 L2" "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "#include <Q"
    "#include \"infrastructure"
    "#include \"presentation")

message(STATUS "ADASim: adasim_app -> 静态库，仅依赖 adasim_domain")
