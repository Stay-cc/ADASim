# ---------------------------------------------------------------------------
# 目标 3：adasim_infra（L4 基础设施层）
#
# 实现 L3 声明的端口：IRenderPort / ITransportPort / IPersistence / IClock / ILogger。
# 依赖方向在此处倒置：infra 依赖 domain（为了拿到接口定义），
# 但 domain 完全不知道 infra 的存在。
# ---------------------------------------------------------------------------

find_package(Qt6 REQUIRED COMPONENTS Widgets Network)

set(ADASIM_INFRA_SOURCES
    render/QtRenderer.cpp
    net/TcpUdpGateway.cpp
    persist/JsonScenarioRepository.cpp)

add_library(adasim_infra STATIC ${ADASIM_INFRA_SOURCES})

target_include_directories(adasim_infra
    PUBLIC ${PROJECT_SOURCE_DIR}/src)

# PUBLIC：可执行目标链接本库时才能拿到 Qt 头文件与宏定义
target_link_libraries(adasim_infra
    PUBLIC adasim_domain
    PUBLIC Qt6::Widgets
    PUBLIC Qt6::Network)

set_target_properties(adasim_infra PROPERTIES AUTOMOC ON)

message(STATUS "ADASim: adasim_infra -> 静态库，实现 domain 端口，依赖 Qt6 Widgets/Network")
