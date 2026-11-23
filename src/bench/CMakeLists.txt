# ---------------------------------------------------------------------------
# 目标 5：adasim_bench（点云压测）
#
# 它链接 domain 与 infra，不链接 app、也不碰表现层：
# 压测关心的是"生成一帧点云"和"把一帧点云画出来"这两件事的代价，
# 不需要控制器与界面参与。这样测出来的数字才是可比的——
# 混进界面的控件刷新与事件循环反而会把结论带偏。
# ---------------------------------------------------------------------------

add_executable(adasim_bench main_bench.cpp)

target_include_directories(adasim_bench PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(adasim_bench PRIVATE adasim_infra adasim_domain)

set_target_properties(adasim_bench PROPERTIES
    AUTOMOC ON
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

message(STATUS "ADASim: 可执行目标 adasim_bench 已生成（点云压测）")
