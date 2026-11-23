# ---------------------------------------------------------------------------
# 目标 0：adasim_headless（无界面闭环，不需要 Qt）
#
# 它只链接 adasim_app 与 adasim_domain，所以本机没有 Qt 时也能构建运行。
# 用途：核心逻辑回归、吞吐基线、以及"分层是否真的解耦"的活证据——
# 如果哪天领域层偷偷引入了 Qt，这个目标会第一个编译失败。
# ---------------------------------------------------------------------------

add_executable(adasim_headless main_headless.cpp)

target_include_directories(adasim_headless PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(adasim_headless PRIVATE adasim_app adasim_domain)

set_target_properties(adasim_headless PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

message(STATUS "ADASim: 可执行目标 adasim_headless 已生成（无 Qt 依赖）")
