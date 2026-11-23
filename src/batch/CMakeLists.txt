# ---------------------------------------------------------------------------
# 目标 7：adasim_batch（批量评估进程，失效搜索的评价函数端点）
#
# 需要链接 infra：场景是 JSON，解析走 Qt 的 QJsonDocument（与主程序共用
# 同一套仓储实现）。多这一层依赖换来的是"Python 写出的场景，与界面上
# 加载的场景由同一份代码解析"，不会出现两边对同一份文件理解不一致。
#
# 接口是 stdin/stdout 的行式 JSON，见 main_batch.cpp 头部说明。
# ---------------------------------------------------------------------------

add_executable(adasim_batch main_batch.cpp)

target_include_directories(adasim_batch PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(adasim_batch PRIVATE adasim_app adasim_infra)

set_target_properties(adasim_batch PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

message(STATUS "ADASim: 可执行目标 adasim_batch 已生成（行式 JSON 评估服务）")
