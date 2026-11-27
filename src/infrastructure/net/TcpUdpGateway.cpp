#include "infrastructure/net/TcpUdpGateway.h"

#include <QAbstractSocket>
#include <QHostAddress>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include <QUdpSocket>
#include <QVariant>
#include <QtEndian>
#include <algorithm>
#include <cstring>

namespace adasim::infra {
namespace {

// ---- 分片头布局（24 字节，全部大端）----
//   0  : 'A'        magic[0]
//   1  : 'S'        magic[1]
//   2  : version    uint8
//   3  : flags      uint8（bit0 = 帧尾片）
//   4  : chunkIndex uint16
//   6  : chunkCount uint16
//   8  : frameId    uint64
//   16 : simTime    float
//   20 : count      uint16
//   22 : reserved   uint16
constexpr int kCloudHeaderBytes = 24;
constexpr quint8 kCloudVersion = 1;

inline void appendU16(QByteArray& out, quint16 v) {
    char buf[2];
    qToBigEndian<quint16>(v, buf);
    out.append(buf, 2);
}

inline void appendU32(QByteArray& out, quint32 v) {
    char buf[4];
    qToBigEndian<quint32>(v, buf);
    out.append(buf, 4);
}

inline void appendU64(QByteArray& out, quint64 v) {
    char buf[8];
    qToBigEndian<quint64>(v, buf);
    out.append(buf, 8);
}

inline void appendF32(QByteArray& out, float v) {
    quint32 bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    appendU32(out, bits);
}

QString kindToString(domain::EntityKind kind) {
    switch (kind) {
        case domain::EntityKind::Car: return QStringLiteral("car");
        case domain::EntityKind::Truck: return QStringLiteral("truck");
        case domain::EntityKind::Pedestrian: return QStringLiteral("pedestrian");
        case domain::EntityKind::Cyclist: return QStringLiteral("cyclist");
        case domain::EntityKind::Unknown: break;
    }
    return QStringLiteral("unknown");
}

}  // namespace

TcpUdpGateway::TcpUdpGateway(GatewayConfig cfg, QObject* parent)
    : QObject(parent), cfg_(cfg), clock_() {
    clock_.start();
    lastRxMs_ = clock_.elapsed();

    server_ = new QTcpServer(this);
    udp_ = new QUdpSocket(this);
    heartbeat_ = new QTimer(this);
    heartbeat_->setInterval(100);

    connect(server_, &QTcpServer::newConnection, this, [this]() {
        QTcpSocket* sock = server_->nextPendingConnection();
        if (peer_ != nullptr) {
            // 只服务一个算法进程：后到的连接直接断开，避免状态机出现两个写者。
            sock->disconnectFromHost();
            sock->deleteLater();
            return;
        }
        peer_ = sock;
        rxBuffer_.clear();
        lastRxMs_ = clock_.elapsed();
        connect(peer_, &QTcpSocket::readyRead, this, &TcpUdpGateway::onReadyRead);
        connect(peer_, &QTcpSocket::disconnected, this, [this]() {
            emit peerConnected(QString());
            if (peer_ != nullptr) {
                peer_->deleteLater();
                peer_ = nullptr;
            }
            alive_ = false;
            emit linkStateChanged(false);
        });
        emit peerConnected(QStringLiteral("%1:%2")
                               .arg(peer_->peerAddress().toString())
                               .arg(peer_->peerPort()));
    });

    connect(heartbeat_, &QTimer::timeout, this, &TcpUdpGateway::checkHeartbeat);
}

TcpUdpGateway::~TcpUdpGateway() = default;

bool TcpUdpGateway::listen() {
    if (!server_->listen(QHostAddress::LocalHost, cfg_.tcpPort)) {
        error_ = server_->errorString();
        return false;
    }
    // 点云通道只发不收，绑定到任意端口即可。
    if (!udp_->bind(QHostAddress::LocalHost, 0)) {
        error_ = udp_->errorString();
        return false;
    }
    heartbeat_->start();
    return true;
}

bool TcpUdpGateway::listening() const { return server_ != nullptr && server_->isListening(); }

quint16 TcpUdpGateway::boundTcpPort() const {
    return server_ != nullptr ? server_->serverPort() : 0;
}

void TcpUdpGateway::publishState(const domain::StatePacket& packet) {
    if (peer_ == nullptr || peer_->state() != QAbstractSocket::ConnectedState) {
        return;
    }

    QJsonObject root;
    root.insert(QStringLiteral("type"), QStringLiteral("state"));
    root.insert(QStringLiteral("frame"), static_cast<qint64>(packet.frameId));
    root.insert(QStringLiteral("t"), packet.simTime);
    // 曲率取 0 时也照发：算法侧不该靠"字段缺失"去猜这是直道，
    // 显式给 0 才能把"直道"与"老版本底座不发这个字段"区分开。
    root.insert(QStringLiteral("curvature"), packet.curvature);
    root.insert(QStringLiteral("odometer"), packet.egoOdometer);
    root.insert(QStringLiteral("targetLateral"), packet.egoTargetLateral);
    root.insert(QStringLiteral("laneCount"), packet.laneCount);
    root.insert(QStringLiteral("laneWidth"), packet.laneWidth);
    root.insert(QStringLiteral("laneChangeEnabled"), packet.laneChangeEnabled);

    // 分段道路（超大地图）：逐段下发。为空则数组为空，算法侧回退到单曲率。
    QJsonArray segs;
    for (const auto& s : packet.roadSegments) {
        QJsonObject o;
        o.insert(QStringLiteral("startS"), s.startS);
        o.insert(QStringLiteral("endS"), s.endS);
        o.insert(QStringLiteral("curvature"), s.curvature);
        o.insert(QStringLiteral("laneCount"), s.laneCount);
        o.insert(QStringLiteral("laneWidth"), s.laneWidth);
        segs.append(o);
    }
    root.insert(QStringLiteral("roadSegments"), segs);

    QJsonObject ego;
    ego.insert(QStringLiteral("x"), packet.ego.x);
    ego.insert(QStringLiteral("y"), packet.ego.y);
    ego.insert(QStringLiteral("theta"), packet.ego.theta);
    ego.insert(QStringLiteral("v"), packet.egoSpeed);
    root.insert(QStringLiteral("ego"), ego);

    QJsonArray obstacles;
    for (const auto& e : packet.obstacles) {
        QJsonObject o;
        o.insert(QStringLiteral("id"), static_cast<qint64>(e.id));
        o.insert(QStringLiteral("kind"), kindToString(e.kind));
        o.insert(QStringLiteral("x"), e.pose.x);
        o.insert(QStringLiteral("y"), e.pose.y);
        o.insert(QStringLiteral("theta"), e.pose.theta);
        o.insert(QStringLiteral("v"), e.speed);
        o.insert(QStringLiteral("l"), e.length);
        o.insert(QStringLiteral("w"), e.width);
        obstacles.append(o);
    }
    root.insert(QStringLiteral("obstacles"), obstacles);

    // 回带最近一次收到的指令序号。算法侧据此计算"发出指令 → 看到它被消费"
    // 的往返时延，不需要两端时钟对齐——两边各用自己的单调时钟即可，
    // 这是跨进程测时延唯一不依赖时钟同步的做法。
    root.insert(QStringLiteral("ackSeq"), static_cast<qint64>(lastAckSeq_));

    const QByteArray json = QJsonDocument(root).toJson(QJsonDocument::Compact);

    // 4 字节长度前缀 + 载荷。长度前缀让接收方能从字节流里切出完整报文，
    // 不依赖换行符或空字符——JSON 里可能含转义序列，用分隔符分帧并不可靠。
    QByteArray frame;
    frame.reserve(4 + json.size());
    appendU32(frame, static_cast<quint32>(json.size()));
    frame.append(json);

    peer_->write(frame);
}

void TcpUdpGateway::publishCloud(const domain::PointCloudPacket& packet) {
    if (udp_ == nullptr) {
        return;
    }
    const int total = static_cast<int>(packet.pointCount());
    // 点数为 0 也必须发一帧"空点云"：算法侧靠"收到一帧空点云"来让感知航迹
    // 逐帧 miss 并最终删除。若这里 return，障碍清除后算法再也收不到任何点云，
    // `_latest_cloud` 停在最后一次非空帧，感知走"同一帧复用"永不更新航迹，
    // 幽灵障碍永远存在，车停在原地不动。
    if (total == 0) {
        QByteArray dgram;
        dgram.reserve(kCloudHeaderBytes);
        dgram.append('A');
        dgram.append('S');
        dgram.append(static_cast<char>(kCloudVersion));
        dgram.append(static_cast<char>(0x01));   // 单分片、末片
        appendU16(dgram, 0);                     // chunkIndex
        appendU16(dgram, 1);                     // chunkCount = 1
        appendU64(dgram, packet.frameId);
        appendF32(dgram, static_cast<float>(packet.simTime));
        appendU16(dgram, 0);                     // count = 0
        appendU16(dgram, 0);
        const QHostAddress dest(QHostAddress::LocalHost);
        if (udp_->writeDatagram(dgram, dest, cfg_.udpPort) < 0) {
            ++droppedChunks_;
        }
        return;
    }

    const int perChunk = cfg_.cloudChunkPoints > 0 ? cfg_.cloudChunkPoints : 120;
    const int chunkCount = (total + perChunk - 1) / perChunk;
    if (chunkCount > 65535) {
        // 点数过多时丢弃，而不是静默截断——截断会让感知算法看到半辆车。
        ++droppedChunks_;
        return;
    }

    const QHostAddress dest(QHostAddress::LocalHost);
    for (int c = 0; c < chunkCount; ++c) {
        const int begin = c * perChunk;
        const int end = std::min(begin + perChunk, total);
        const int count = end - begin;

        QByteArray dgram;
        dgram.reserve(kCloudHeaderBytes + count * 12);
        dgram.append('A');
        dgram.append('S');
        dgram.append(static_cast<char>(kCloudVersion));
        dgram.append(static_cast<char>(c == chunkCount - 1 ? 0x01 : 0x00));
        appendU16(dgram, static_cast<quint16>(c));
        appendU16(dgram, static_cast<quint16>(chunkCount));
        appendU64(dgram, packet.frameId);
        appendF32(dgram, static_cast<float>(packet.simTime));
        appendU16(dgram, static_cast<quint16>(count));
        appendU16(dgram, 0);

        for (int i = begin; i < end; ++i) {
            appendF32(dgram, packet.px[static_cast<std::size_t>(i)]);
            appendF32(dgram, packet.py[static_cast<std::size_t>(i)]);
            appendF32(dgram, packet.intensity[static_cast<std::size_t>(i)]);
        }

        if (udp_->writeDatagram(dgram, dest, cfg_.udpPort) < 0) {
            ++droppedChunks_;
        }
    }
}

bool TcpUdpGateway::pollCommand(domain::ControlCommand& out) {
    std::lock_guard<std::mutex> guard(queueMutex_);
    if (pending_.empty()) {
        return false;
    }
    out = pending_.front();
    pending_.pop_front();
    return true;
}

bool TcpUdpGateway::linkAlive() const noexcept { return alive_.load(std::memory_order_relaxed); }

void TcpUdpGateway::onReadyRead() {
    if (peer_ == nullptr) {
        return;
    }
    const QByteArray chunk = peer_->readAll();
    if (chunk.isEmpty()) {
        return;
    }
    lastRxMs_ = clock_.elapsed();
    rxBuffer_.append(chunk);
    drainFrames();
}

void TcpUdpGateway::drainFrames() {
    while (rxBuffer_.size() >= 4) {
        const auto* raw = reinterpret_cast<const uchar*>(rxBuffer_.constData());
        const quint32 len = qFromBigEndian<quint32>(raw);
        if (len == 0 || len > 8u * 1024u * 1024u) {
            // 长度字段异常：协议已失步，清空缓冲重新同步，
            // 否则会一直卡在这个非法长度上死循环。
            rxBuffer_.clear();
            return;
        }
        if (static_cast<quint32>(rxBuffer_.size()) < 4 + len) {
            return;  // 半包，等下一批字节
        }
        const QByteArray payload = rxBuffer_.mid(4, static_cast<int>(len));
        rxBuffer_.remove(0, 4 + static_cast<int>(len));
        handlePayload(payload);
    }
}

void TcpUdpGateway::handlePayload(const QByteArray& payload) {
    QJsonParseError err{};
    const QJsonDocument doc = QJsonDocument::fromJson(payload, &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) {
        return;
    }
    const QJsonObject obj = doc.object();
    const QString type = obj.value(QStringLiteral("type")).toString();
    if (type != QLatin1String("cmd")) {
        return;
    }

    domain::ControlCommand cmd;
    cmd.seq = static_cast<std::uint32_t>(obj.value(QStringLiteral("seq")).toInt());
    cmd.echoFrameId = static_cast<domain::FrameId>(
        obj.value(QStringLiteral("frame")).toVariant().toLongLong());
    cmd.simTime = obj.value(QStringLiteral("t")).toDouble();
    cmd.steer = obj.value(QStringLiteral("steer")).toDouble();
    cmd.targetSpeed = static_cast<double>(obj.value(QStringLiteral("speed")).toDouble());
    cmd.targetAccel = static_cast<double>(obj.value(QStringLiteral("accel")).toDouble());
    // 可选字段：老版本算法进程不发这两个键，缺省即"未上报"，不影响控制。
    cmd.reason = obj.value(QStringLiteral("reason")).toString().toStdString();
    cmd.algoRttMs = obj.value(QStringLiteral("rtt")).toDouble(-1.0);

    {
        std::lock_guard<std::mutex> guard(queueMutex_);
        // 队列设上限：算法进程若瞬时突发，宁可丢旧指令也不能让内存无界增长。
        if (pending_.size() > 64) {
            pending_.pop_front();
        }
        pending_.push_back(cmd);
    }
    lastAckSeq_ = static_cast<qint64>(cmd.seq);
    emit commandReceived();
}

void TcpUdpGateway::checkHeartbeat() {
    const bool connected = (peer_ != nullptr && peer_->state() == QAbstractSocket::ConnectedState);
    const qint64 since = clock_.elapsed() - lastRxMs_;
    const bool aliveNow = connected && since <= cfg_.heartbeatTimeoutMs;

    if (aliveNow != alive_.load(std::memory_order_relaxed)) {
        alive_ = aliveNow;
        emit linkStateChanged(aliveNow);
    }
}

}  // namespace adasim::infra
