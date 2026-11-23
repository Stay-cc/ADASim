#pragma once

#include <cstdio>
#include <mutex>
#include <string>

#include "domain/port/ILogger.h"

namespace adasim::infra {

/// 控制台日志。带互斥：物理线程、通信线程与界面线程都会写，
/// 不加锁会让整行日志互相穿插，排查时无法阅读。
class ConsoleLogger final : public domain::ILogger {
public:
    void log(domain::LogLevel level, const std::string& message) override {
        static const char* const kTag[] = {"DEBUG", "INFO ", "WARN ", "ERROR"};
        const auto idx = static_cast<std::size_t>(level);
        const char* tag = idx < 4 ? kTag[idx] : "?????";

        std::lock_guard<std::mutex> guard(mutex_);
        std::fprintf(level >= domain::LogLevel::Warn ? stderr : stdout,
                     "[%s] %s\n", tag, message.c_str());
        std::fflush(level >= domain::LogLevel::Warn ? stderr : stdout);
    }

private:
    std::mutex mutex_;
};

}  // namespace adasim::infra
