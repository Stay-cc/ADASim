# ---------------------------------------------------------------------------
# 四个目标在此汇合
#
#   adasim_domain  静态库  L3  纯 C++17，不链接 Qt，被所有上层依赖
#   adasim_app     静态库  L2  只依赖 adasim_domain，看不到任何实现类
#   adasim_infra   静态库  L4  实现 L3 端口，依赖 Qt6
#   adasim         可执行  L1  + 装配根，链接 app 与 infra
#   adasim_bench   可执行  —   点云压测（domain + infra，不经控制器与界面）
#
# 依赖关系刻意做成单向：app 不认识 infra，infra 不认识 app，
# 二者只通过 domain 的端口交接，连接点在 main.cpp。
# ---------------------------------------------------------------------------

add_subdirectory(domain)
add_subdirectory(application)

# 无界面闭环：不依赖 Qt，先于 GUI 判断加入，保证精简构建也有可运行产物
add_subdirectory(headless)

if(NOT ADASIM_WITH_GUI)
    message(STATUS "ADASim: 跳过 adasim_infra 与 adasim 可执行目标（ADASIM_WITH_GUI=OFF）")
    return()
endif()

add_subdirectory(infrastructure)

# 压测工具需要 Qt（渲染器与网关），因此和 GUI 同进退
add_subdirectory(bench)

# 表现层不单独成库：它只能被最终可执行程序使用，不允许被其他目标依赖
set(ADASIM_PRESENTATION_SOURCES
    presentation/MainWindow.cpp
    presentation/ViewportWidget.cpp
    main.cpp)

add_executable(adasim ${ADASIM_PRESENTATION_SOURCES})

target_include_directories(adasim PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(adasim PRIVATE adasim_app adasim_infra)

# 表现层的 MainWindow / ViewportWidget 带 Q_OBJECT，必须为可执行目标开启 AUTOMOC，
# 否则会在链接阶段报 "undefined reference to vtable for ..."。
set_target_properties(adasim PROPERTIES
    AUTOMOC ON
    AUTOUIC ON
    AUTORCC ON
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

message(STATUS "ADASim: 可执行目标 adasim 已生成，输出到 ${CMAKE_BINARY_DIR}/bin")
