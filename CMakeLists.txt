cmake_minimum_required(VERSION 3.20)

project(ADASim
    VERSION 0.1.0
    DESCRIPTION "轻量级自动驾驶算法仿真平台"
    LANGUAGES CXX)

# ---------------------------------------------------------------------------
# 全局编译设置
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "构建类型" FORCE)
endif()

# ---------------------------------------------------------------------------
# 构建选项
# ---------------------------------------------------------------------------
option(ADASIM_WITH_GUI "构建 Qt 渲染、通信与窗口，需要 Qt6" ON)
option(ADASIM_BUILD_TESTS "构建回归测试（CTest）" ON)
option(ADASIM_WARNINGS_AS_ERRORS "把编译告警视为错误" OFF)

# Qt 缺失时自动降级：保证领域层与应用层始终可独立构建
# 这一条是分层设计的实际体现——没有 Qt，核心逻辑依然能编译、能测。
if(ADASIM_WITH_GUI)
    find_package(Qt6 6.2 QUIET COMPONENTS Widgets Network)
    if(NOT Qt6_FOUND)
        message(WARNING
            "未找到 Qt6 (>= 6.2)，已自动关闭 GUI 目标。\n"
            "  本次仅构建 adasim_domain 与 adasim_app。\n"
            "  如需完整构建，请安装 Qt6 并用 -DCMAKE_PREFIX_PATH=<Qt路径> 指定。")
        set(ADASIM_WITH_GUI OFF CACHE BOOL "构建 Qt 渲染、通信与窗口，需要 Qt6" FORCE)
    endif()
endif()

if(ADASIM_WARNINGS_AS_ERRORS AND NOT MSVC)
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# ---------------------------------------------------------------------------
# 分层守卫：编译期之外的静态检查，违规即配置失败
# ---------------------------------------------------------------------------
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/LayerGuard.cmake)

add_subdirectory(src)

# 测试放在 src 之后：用例链接 adasim_app / adasim_domain，也复用 adasim_headless
if(ADASIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# 构建摘要
# ---------------------------------------------------------------------------
if(ADASIM_WITH_GUI)
    message(STATUS "ADASim 目标: adasim_domain | adasim_app | adasim_infra | adasim (exe) | adasim_headless")
else()
    message(STATUS "ADASim 目标: adasim_domain | adasim_app | adasim_headless  (GUI 已关闭)")
endif()
