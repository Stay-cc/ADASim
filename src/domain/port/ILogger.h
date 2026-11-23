#pragma once

#include <string>

namespace adasim::domain {

enum class LogLevel {
    Debug = 0,
    Info,
    Warn,
    Error,
};

/// 日志端口。领域层允许表达"这里发生了异常"，
/// 但不决定写文件、写控制台还是上报。
struct ILogger {
    virtual ~ILogger() = default;
    virtual void log(LogLevel level, const std::string& message) = 0;
};

}  // namespace adasim::domain
