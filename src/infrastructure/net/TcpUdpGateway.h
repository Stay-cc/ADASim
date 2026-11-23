#pragma once

#include <QByteArray>
#include <QElapsedTimer>
#include <QObject>
#include <atomic>
#include <deque>
#include <mutex>

#include "domain/port/ITransportPort.h"

class QTcpServer;
class QTcpSocket;
class QUdpSocket;
class QTimer;

namespace adasim::infra {

/// 网关配置。刻意放在命名空间作用域而不是嵌套在类里：
/// 若写成嵌套类，`Config cfg = {}` 这样的默认实参在类内会被拒绝——
/// 因为默认成员初始化器的求值上下文止于外层类的右花括号，
/// 此时外层类尚未完整。挪出来就没有这个问题。
struct GatewayConfig {
    quint16 tcpPort{45100};        ///< 底座监听，算法进程连接
    quint16 udpPort{45101};        ///< 点云发往此端口
    int heartbeatTimeoutMs{500};   ///< 超过此值未见字节即判定断链
    /// 每片点数。按以太网 MTU 反推：1500 − IP 头 20 − UDP 头 8 = 1472 字节，
    /// 减 24 字节分片头后每点 12 字节，正好 120 点（1464 字节）。
    ///
    /// 早期取 2000 点（约 24 KB）——那是按 UDP 协议上限 65507 留的余量，
    /// 漏掉了链路 MTU 这一层。环回上跑不出问题，一旦经真实网卡发出，
    /// IP 层会把它切成 17 个分片：丢任一分片整片作废，接收端还要等重组，
    /// 丢包率被放大十几倍。压测（docs/性能压测报告.md）里两档都测了。
    int cloudChunkPoints{120};
};

/// 双通道通信网关（TCP 可靠状态 + UDP 时效点云）。
///
/// 关键设计：
///   1. TCP 用 4 字节大端长度前缀分帧。TCP 是字节流，不自行分帧就会出现
///      "两次 send 粘成一个包"或"一个包被拆成两段"的经典问题。
///   2. UDP 单包上限 65507 字节，十万点（每点 12 字节）必须分片。
///      分片头携带 (frameId, chunkIndex, chunkCount)，接收方可据此重组，
///      也可以只取第一片用于快速预览。
///   3. 心跳不用独立报文，直接用"最近一次收到字节的时刻"判断。
///      这样算法进程只要在跑就一定有心跳，不会出现"心跳正常但数据断了"。
class TcpUdpGateway final : public QObject, public domain::ITransportPort {
    Q_OBJECT

public:
    explicit TcpUdpGateway(GatewayConfig cfg = GatewayConfig{}, QObject* parent = nullptr);
    ~TcpUdpGateway() override;

    /// 开始监听。失败时返回 false 并置 errorString()。
    bool listen();
    [[nodiscard]] bool listening() const;
    [[nodiscard]] const QString& errorString() const noexcept { return error_; }
    [[nodiscard]] quint16 boundTcpPort() const;

    void publishCloud(const domain::PointCloudPacket& packet) override;
    void publishState(const domain::StatePacket& packet) override;
    [[nodiscard]] bool pollCommand(domain::ControlCommand& out) override;
    [[nodiscard]] bool linkAlive() const noexcept override;

    /// 往返时延估计（毫秒）：由指令的 echoFrameId 与当前帧号差换算。
    [[nodiscard]] double lastRoundTripMs() const noexcept { return lastRttMs_; }
    [[nodiscard]] quint64 droppedCloudChunks() const noexcept { return droppedChunks_; }

signals:
    void linkStateChanged(bool alive);
    void commandReceived();
    void peerConnected(const QString& peer);

private:
    void onReadyRead();
    void drainFrames();
    void handlePayload(const QByteArray& payload);
    void checkHeartbeat();

    GatewayConfig cfg_;
    QTcpServer* server_{nullptr};
    QTcpSocket* peer_{nullptr};
    QUdpSocket* udp_{nullptr};
    QTimer* heartbeat_{nullptr};

    QByteArray rxBuffer_;
    mutable std::mutex queueMutex_;
    std::deque<domain::ControlCommand> pending_;

    QElapsedTimer clock_;
    qint64 lastRxMs_{0};
    std::atomic<bool> alive_{false};
    double lastRttMs_{0.0};
    quint64 droppedChunks_{0};
    QString error_;
};

}  // namespace adasim::infra
