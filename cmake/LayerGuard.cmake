# 分层守卫
#
# 目的：把"领域层不得依赖 Qt / 基础设施层"从文档约定变成构建期检查。
# 用法：
#   adasim_enforce_layer("<层名>" "<glob 表达式>" "<禁止出现的片段1>" ["<片段2>" ...])
#
# 说明：这是编译期之外的第一道防线，做的是字面检查，成本极低；
#       第二道防线是链接期——adasim_domain 不链接 Qt，引入 Qt 符号必然失败。

function(adasim_enforce_layer layer_name glob_expr)
    if(NOT ARGN)
        return()
    endif()

    file(GLOB_RECURSE _layer_files ${glob_expr})
    if(NOT _layer_files)
        return()
    endif()

    foreach(_file ${_layer_files})
        file(READ "${_file}" _content)
        foreach(_needle ${ARGN})
            string(FIND "${_content}" "${_needle}" _hit)
            if(NOT _hit EQUAL -1)
                message(FATAL_ERROR
                    "分层违规：${layer_name} 中出现了被禁止的依赖 ${_needle}\n"
                    "  文件：${_file}\n"
                    "  处理建议：把该依赖收敛到端口接口之后，由基础设施层提供实现。")
            endif()
        endforeach()
    endforeach()
endfunction()
