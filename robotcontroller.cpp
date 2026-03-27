#include "robotcontroller.h"
#include "SerialPort.h"

#include <cmath>
#include <QDateTime>
#include <chrono>
#include <thread>

///////////////////////////// ControlWorker 实现 /////////////////////////////

ControlWorker::ControlWorker(QObject *parent)
    : QObject(parent)
{
}

void ControlWorker::start()
{
    if (running_.load()) {
        emit logMessage(QStringLiteral("控制线程已在运行"));
        return;
    }

    running_.store(true);
    startTime_ = getCurrentTime();

    emit logMessage(QStringLiteral("控制线程已启动，周期: %1ms").arg(controlPeriodMs_));

    // 执行控制循环
    controlLoop();

    emit logMessage(QStringLiteral("控制线程已停止"));
}

void ControlWorker::stop()
{
    running_.store(false);
}

void ControlWorker::updateJointState(const JointState &state)
{
    QMutexLocker locker(&stateMutex_);
    if (state.jointIndex >= 1 && state.jointIndex <= 3) {
        jointStates_[state.jointIndex] = state;
    }
}

void ControlWorker::setControlParams(const ControlParams &params)
{
    QMutexLocker locker(&paramsMutex_);
    params_ = params;
    controlPeriodMs_ = params.controlPeriod;
}

void ControlWorker::initTrajectory()
{
    trajectoryInitialized_.store(true);
    startTime_ = getCurrentTime();
    emit logMessage(QStringLiteral("轨迹跟踪已初始化"));
}
//控制循环函数，持续计算控制命令并发送
void ControlWorker::controlLoop()
{
    auto nextWakeTime = std::chrono::steady_clock::now();

    while (running_.load()) {
        // 计算下一次唤醒时间
        nextWakeTime += std::chrono::milliseconds(controlPeriodMs_);

        // 获取当前时间
        float currentTime = getCurrentTime();
        float elapsedTime = currentTime - startTime_;

        // 检查是否超出轨迹时长
        {
            QMutexLocker locker(&paramsMutex_);
            if (elapsedTime > params_.trajectoryDuration) {
                emit logMessage(QStringLiteral("轨迹跟踪已完成，时长: %1秒").arg(elapsedTime, 0, 'f', 2));
                emit controlStatusChanged(false);
                running_.store(false);
                break;
            }
        }

        // 计算期望轨迹
        QVector<float> desiredTrajectory = computeDesiredTrajectory(elapsedTime);

        // 获取关节状态
        std::array<JointState, 4> currentStates;
        {
            QMutexLocker locker(&stateMutex_);
            currentStates = jointStates_;
        }

        // 对每个关节执行控制
        for (int jointIndex = 1; jointIndex <= 3; ++jointIndex) {
            const JointState &state = currentStates[jointIndex];

            if (state.jointIndex == 0) {
                continue;  // 关节未初始化
            }

            // 计算控制量
            auto [targetPos, targetVel] = computeControl(jointIndex, state, desiredTrajectory);

            // 发送控制命令信号
            emit controlCommandSent(jointIndex, targetPos, targetVel);
        }

        // 精确休眠到下一个控制周期
        std::this_thread::sleep_until(nextWakeTime);
    }

    emit controlStatusChanged(false);
}

QVector<float> ControlWorker::computeDesiredTrajectory(float t) const
{
    QMutexLocker locker(&paramsMutex_);

    QVector<float> desired(4, 0.0f);  // 索引0不使用，1-3对应3个关节

    // 使用参数d1, d2, d3生成轨迹
    // 示例：简单的正弦轨迹
    float omega = 2.0f * M_PI / params_.trajectoryDuration;  // 角频率

    // 关节1
    desired[1] = params_.d1 * std::sin(omega * t);

    // 关节2
    desired[2] = params_.d2 * std::sin(omega * t + M_PI / 4.0f);

    // 关节3 (取负，与硬件特性对应)
    desired[3] = -params_.d3 * std::sin(omega * t + M_PI / 2.0f);

    return desired;
}

std::pair<float, float> ControlWorker::computeControl(
    int jointIndex,
    const JointState &state,
    const QVector<float> &desired) const
{
    QMutexLocker locker(&paramsMutex_);

    // 获取期望位置和速度
    float desiredPos = (jointIndex <= desired.size()) ? desired[jointIndex] : 0.0f;
    float desiredVel = 0.0f;  // 可以根据需要计算期望速度

    // 计算位置误差
    float posError = desiredPos - state.position;

    // 计算速度误差
    float velError = desiredVel - state.velocity;

    // 控制算法：简单的PD控制
    float targetPos = state.position + params_.k1 * posError;
    float targetVel = params_.k2 * velError + desiredVel * params_.k3;

    // 限制输出范围（根据电机限制调整）
    const float MAX_POS = 12.5f;
    const float MAX_VEL = 30.0f;

    targetPos = std::clamp(targetPos, -MAX_POS, MAX_POS);
    targetVel = std::clamp(targetVel, -MAX_VEL, MAX_VEL);

    return {targetPos, targetVel};
}

void ControlWorker::sendMotorCommand(int canId, float position, float velocity)
{
    // 这个方法实际上不在这里发送，而是通过信号发送到RobotController
    // RobotController负责实际的串口发送
    Q_UNUSED(canId)
    Q_UNUSED(position)
    Q_UNUSED(velocity)
}

float ControlWorker::getCurrentTime() const
{
    return static_cast<float>(QDateTime::currentMSecsSinceEpoch()) / 1000.0f;
}

///////////////////////////// RobotController 实现 /////////////////////////////

RobotController::RobotController(QObject *parent)
    : QObject(parent)
{
    // 创建控制线程和工作对象
    controlThread_ = new QThread(this);
    worker_ = new ControlWorker();

    // 将工作对象移到控制线程
    worker_->moveToThread(controlThread_);

    // 连接信号槽
    connect(worker_, &ControlWorker::controlCommandSent,
            this, &RobotController::onControlCommandSent);
    connect(worker_, &ControlWorker::controlStatusChanged,
            this, &RobotController::controlStatusChanged);
    connect(worker_, &ControlWorker::logMessage,
            this, &RobotController::logMessage);

    // 启动线程
    controlThread_->start();
}

RobotController::~RobotController()
{
    stopControl();

    if (worker_) {
        worker_->stop();
        delete worker_;
        worker_ = nullptr;
    }

    if (controlThread_) {
        controlThread_->quit();
        controlThread_->wait();
        delete controlThread_;
        controlThread_ = nullptr;
    }
}

void RobotController::setSerialPort(SerialPort *serialPort)
{
    serialPort_ = serialPort;
}

void RobotController::updateJointState(int jointIndex, float position, float velocity)
{
    if (!worker_) {
        return;
    }

    JointState state(jointIndex, position, velocity, getCurrentTime());

    // 转发到工作线程
    QMetaObject::invokeMethod(worker_, "updateJointState",
                             Qt::QueuedConnection,
                             Q_ARG(JointState, state));

    // 同时发送状态更新信号
    emit jointStateChanged(state);
}

void RobotController::startControl()
{
    if (controlRunning_.load()) {
        emit logMessage(QStringLiteral("控制循环已在运行"));
        return;
    }

    if (!worker_) {
        emit logMessage(QStringLiteral("错误：控制工作对象未初始化"));
        return;
    }

    controlRunning_.store(true);

    // 启动工作线程的控制循环
    QMetaObject::invokeMethod(worker_, "start", Qt::QueuedConnection);
}

void RobotController::stopControl()
{
    if (!controlRunning_.load()) {
        return;
    }

    if (worker_) {
        worker_->stop();
    }

    controlRunning_.store(false);
}

void RobotController::setControlParams(const ControlParams &params)
{
    if (worker_) {
        QMetaObject::invokeMethod(worker_, "setControlParams",
                                 Qt::QueuedConnection,
                                 Q_ARG(ControlParams, params));
    }

    emit logMessage(QStringLiteral("控制参数已更新"));
}

ControlParams RobotController::getControlParams() const
{
    // 注意：这个方法可能不能获取最新值，因为参数在工作线程中
    // 如果需要精确获取，可以添加getter信号
    static ControlParams defaultParams;
    return defaultParams;
}

void RobotController::initTrajectoryTracking()
{
    if (worker_) {
        QMetaObject::invokeMethod(worker_, "initTrajectory", Qt::QueuedConnection);
    }
}

void RobotController::enableMotors()
{
    if (!serialPort_) {
        emit logMessage(QStringLiteral("错误：未设置串口对象"));
        return;
    }

    // 使能3个电机（CAN ID: 1, 2, 3）
    const uint8_t enableCmd[9] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x88};
    const uint8_t canIds[3] = {0x01, 0x02, 0x03};
    const uint8_t head[21] = {0x55, 0xaa, 0x1e, 0x01, 0x01, 0x00, 0x00, 0x00,
                               0x0a, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
                               0x00, 0x00, 0x08, 0x00, 0x00};

    uint8_t frame[30];

    for (int i = 0; i < 3; ++i) {
        memset(frame, 0, 30);
        memcpy(frame, head, 21);
        frame[13] = canIds[i];
        memcpy(frame + 21, enableCmd, 9);

        serialPort_->send(frame, 30);
        usleep(1000);  // 防止CAN丢帧
    }

    emit logMessage(QStringLiteral("电机使能命令已发送 (ID: 1, 2, 3)"));
}

void RobotController::disableMotors()
{
    if (!serialPort_) {
        emit logMessage(QStringLiteral("错误：未设置串口对象"));
        return;
    }

    // 失能3个电机
    const uint8_t disableCmd[9] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0x77};
    const uint8_t canIds[3] = {0x01, 0x02, 0x03};
    const uint8_t head[21] = {0x55, 0xaa, 0x1e, 0x01, 0x01, 0x00, 0x00, 0x00,
                               0x0a, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
                               0x00, 0x00, 0x08, 0x00, 0x00};

    uint8_t frame[30];

    for (int i = 0; i < 3; ++i) {
        memset(frame, 0, 30);
        memcpy(frame, head, 21);
        frame[13] = canIds[i];
        memcpy(frame + 21, disableCmd, 9);

        serialPort_->send(frame, 30);
        usleep(1000);
    }

    emit logMessage(QStringLiteral("电机失能命令已发送 (ID: 1, 2, 3)"));
}

JointState RobotController::getJointState(int jointIndex) const
{
    return JointState();
}

void RobotController::onControlCommandSent(int jointIndex, float targetPos, float targetVel)
{
    if (!serialPort_) {
        return;
    }

    // 构造CAN帧（根据达妙电机协议）
    int canId = (jointIndex == 3) ? 0x03 : jointIndex;
    uint32_t canIdWithMode = canId + 0x100;  // 位置速度控制模式

    uint8_t head[21] = {0x55, 0xaa, 0x1e, 0x01, 0x01, 0x00, 0x00, 0x00,
                        0x0a, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
                        0x00, 0x00, 0x08, 0x00, 0x00};

    uint8_t frame[30];
    memset(frame, 0, 30);
    memcpy(frame, head, 21);

    // 写入CAN ID (little-endian)
    frame[13] = canIdWithMode & 0xFF;
    frame[14] = (canIdWithMode >> 8) & 0xFF;
    frame[15] = (canIdWithMode >> 16) & 0xFF;
    frame[16] = (canIdWithMode >> 24) & 0xFF;

    // 写入位置（float）
    memcpy(frame + 21, &targetPos, sizeof(float));

    // 写入速度（float）
    memcpy(frame + 25, &targetVel, sizeof(float));

    // 发送
    serialPort_->send(frame, 30);

    // 转发信号给MainWindow
    emit controlCommandSent(jointIndex, targetPos, targetVel);
}

float RobotController::getCurrentTime() const
{
    return static_cast<float>(QDateTime::currentMSecsSinceEpoch()) / 1000.0f;
}
