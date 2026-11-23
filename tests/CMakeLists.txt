# ---------------------------------------------------------------------------
# 回归测试（CTest）
#
# 三类：
#   C++   领域/应用层逻辑，不需要 Qt，可在精简构建下跑
#   Python 算法进程离线自检与场景生成自检，不连底座
#   闭环   adasim_headless 的确定性检查（有前车、有弯道、可回溯）
#
# 命名约定：C++ 用例失败返回 1 而不是断言崩溃——一次跑完看到全部失败项，
# 比在第一个失败处 abort 更有用。
# ---------------------------------------------------------------------------

# --- C++ 用例 -------------------------------------------------------------

set(ADASIM_CPP_TESTS
    standby_guard_check
    cut_in_check
    replay_check)

foreach(name ${ADASIM_CPP_TESTS})
    add_executable(${name} ${CMAKE_CURRENT_SOURCE_DIR}/${name}.cpp)
    target_include_directories(${name} PRIVATE ${PROJECT_SOURCE_DIR}/src)
    target_link_libraries(${name} PRIVATE adasim_app adasim_domain)
    set_target_properties(${name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES
        LABELS "unit"
        TIMEOUT 120)
endforeach()

# --- 无界面闭环 -----------------------------------------------------------

# 只链接 app + domain，即使 ADASIM_WITH_GUI=OFF 也能跑：
# 这条测试同时是"领域层没有偷偷依赖 Qt"的守门员。
if(TARGET adasim_headless)
    add_test(NAME headless_closed_loop COMMAND adasim_headless)
    set_tests_properties(headless_closed_loop PROPERTIES
        LABELS "integration"
        TIMEOUT 180)
endif()

# --- Python 用例 ----------------------------------------------------------

# 有 Python 就挂上，没有则跳过而不是配置失败——纯粹的 C++ 使用者不该被卡住。
#
# 版本下限 3.10 是硬要求：protocol.py 用了 dataclass(slots=True)，3.9 及更低
# 会直接 TypeError。本机 CMake 会优先找到 Visual Studio 自带的那份 3.9，
# 所以这里显式带上版本约束，并留一个变量给使用者手动指定。
set(ADASIM_PYTHON_EXECUTABLE "" CACHE FILEPATH
    "指定用于自检的 Python 解释器（需 >= 3.10）；留空则自动探测")

find_package(Python3 3.10 COMPONENTS Interpreter QUIET)

if(ADASIM_PYTHON_EXECUTABLE)
    set(ADASIM_PYTHON "${ADASIM_PYTHON_EXECUTABLE}")
elseif(Python3_Interpreter_FOUND)
    set(ADASIM_PYTHON "${Python3_EXECUTABLE}")
else()
    set(ADASIM_PYTHON "")
endif()

if(ADASIM_PYTHON)
    message(STATUS "ADASim: 自检使用 Python ${Python3_VERSION}（${ADASIM_PYTHON}）")

    add_test(NAME algo_selftest
        COMMAND ${ADASIM_PYTHON} ${PROJECT_SOURCE_DIR}/python/run.py --self-test)
    set_tests_properties(algo_selftest PROPERTIES
        LABELS "python"
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/python
        ENVIRONMENT "PYTHONIOENCODING=utf-8;PYTHONPATH=${PROJECT_SOURCE_DIR}/python"
        TIMEOUT 120)

    add_test(NAME ai_selftest
        COMMAND ${ADASIM_PYTHON} ${PROJECT_SOURCE_DIR}/ai/selftest.py)
    set_tests_properties(ai_selftest PROPERTIES
        LABELS "python"
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/ai
        ENVIRONMENT "PYTHONIOENCODING=utf-8;PYTHONPATH=${PROJECT_SOURCE_DIR}/ai"
        TIMEOUT 180)
else()
    message(STATUS "ADASim: 未找到 Python >= 3.10，跳过算法与场景生成自检")
endif()

# 刻意不用 PASS_REGULAR_EXPRESSION：一旦设了它，CTest 只看输出不看退出码，
# 用例中途崩溃但已经打印完"全部通过"就会被判通过。退出码才是判据。

message(STATUS "ADASim: 测试目标 ${ADASIM_CPP_TESTS} 已生成，用 ctest 运行")
