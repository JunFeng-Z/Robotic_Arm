#include "robotcontroller.h"
#include "SerialPort.h"
#include "control_worker.h"

#include <cmath>
#include <chrono>
#include <thread>

// 辅助函数：获取当前时间（毫秒）
static uint64_t currentMillis() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

///////////////////////////// RobotController 实现 /////////////////////////////

RobotController::RobotController(QObject *parent)
    : QObject(parent)
{
    // 设置ControlWorker的回调函数
    ControlWorker::Callbacks callbacks;
    callbacks.log = [this](const std::string& message) {
        emit logMessage(QString::fromStdString(message));
    };
    callbacks.torqueCommand = [this](int jointIndex, float torque) {
        // 直接调用槽函数，确保在正确的线程中执行
        QMetaObject::invokeMethod(this, "onTorqueCommandSent",
                                 Qt::QueuedConnection,
                                 Q_ARG(int, jointIndex),
                                 Q_ARG(float, torque));
    };
    callbacks.controlCommand = [this](int jointIndex, float targetPos, float targetVel) {
        // 直接调用槽函数
        QMetaObject::invokeMethod(this, "onControlCommandSent",
                                 Qt::QueuedConnection,
                                 Q_ARG(int, jointIndex),
                                 Q_ARG(float, targetPos),
                                 Q_ARG(float, targetVel));
    };
    callbacks.controlStatus = [this](bool running) {
        emit controlStatusChanged(running);
    };

    // 创建控制工作对象
    worker_ = std::make_unique<ControlWorker>(callbacks);

    // 默认使用重力补偿算法
    SwitchControlAlgorithm(ControlAlgorithm::GravityCompensation);
}

RobotController::~RobotController()
{
    stopControl();
    // worker_的析构函数会自动停止线程并等待
}

void RobotController::setSerialPort(SerialPort *serialPort)
{
    serialPort_ = serialPort;
}

//拿到底层通信线程解析出的关节状态，转发到工作线程，并发送状态到界面显示
void RobotController::updateJointState(int jointIndex, float position, float velocity)
{
    if (!worker_) {
        return;
    }

    JointState state(jointIndex, position, velocity, 0, getCurrentTime());

    // 转发到工作线程（线程安全）
    worker_->updateJointState(state);

    // 同时发送状态更新信号
    emit jointStateChanged(state);
}

void RobotController::SwitchControlAlgorithm(ControlAlgorithm algorithm)
{
    currentAlgorithm_ = algorithm;
    if (worker_) {
        worker_->switchControlAlgorithm(algorithm);
    }
    else
    {
        emit logMessage(QStringLiteral("错误：worker对象未初始化"));
    }
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
    worker_->start();
}

void RobotController::clearMoveIndex()
{
    if (worker_) {
        worker_->clearMoveIndex();
    }
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
        worker_->setControlParams(params);
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
        worker_->initTrajectory();
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
    return static_cast<float>(currentMillis()) / 1000.0f;
}

uint16_t RobotController::floatToUint(float x, float x_min, float x_max, uint8_t bits) const
{
    float span = x_max - x_min;
    float data_norm = (x - x_min) / span;
    uint16_t data_uint = static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
    return data_uint;
}

std::array<uint8_t, 2> RobotController::floatToBytes(float x, float x_min, float x_max, uint8_t bits) const
{
    uint16_t x_int = floatToUint(x, x_min, x_max, bits);
    std::array<uint8_t, 2> bytes;
    if (bits > 12)
    {
        bytes[1] = static_cast<uint8_t>(x_int & 0xff);
        bytes[0] = static_cast<uint8_t>((x_int & 0xff00) >> 8);
    }
    else
    {
        bytes[1] = static_cast<uint8_t>(x_int & 0xff);
        bytes[0] = static_cast<uint8_t>((x_int & 0x0f00) >> 8);
    }
    return bytes;
}

void RobotController::fillTorqueFrame(damiao::can_send_frame& frame, uint8_t motorId, float torque) const
{
    // 初始化帧为默认值（使用结构体默认初始化）
    frame = damiao::can_send_frame();
    // 设置命令为转发CAN数据帧（0x01）
    frame.CMD = 0x01;
    // 发送次数和时间间隔使用默认值（1和10）
    // 设置CAN ID为电机ID
    frame.canId = motorId;
    // 数据长度为8
    frame.len = 0x08;
    // 构建Turn数组（9字节）
    const uint8_t turn[9] = {0x7f, 0xff, 0x7f, 0xf0, 0x00, 0x00, 0x07, 0xff, 0x88};
    // 将扭矩转换为字节
    float torque_min = -10.0f;
    float torque_max = 10.0f;
    int torque_bits = 12;
    std::array<uint8_t, 2> torque_bytes = floatToBytes(torque, torque_min, torque_max, torque_bits);
    // 复制Turn数组到data和crc字段
    std::copy(turn, turn + 8, frame.data); // 前8字节到data
    frame.crc = turn[8]; // 第9字节到crc
    // 替换data中的第6、7字节（索引6,7）
    frame.data[6] = torque_bytes[0];
    frame.data[7] = torque_bytes[1];
}

void RobotController::sendTorqueCommand(uint8_t motorId, float torque)
{
    // 扭矩限制 (与C#一致)
    if (torque > 2.0f || torque < -1.5f)
    {
        torque = 0.0f;
    }

    damiao::can_send_frame frame;
    fillTorqueFrame(frame, motorId, torque);

    // 发送
    if (serialPort_) {
        serialPort_->send(reinterpret_cast<uint8_t*>(&frame), sizeof(frame));
    } else {
        // 可以记录错误或发出信号
        emit logMessage(QStringLiteral("错误：未设置串口对象"));
    }
}

void RobotController::onTorqueCommandSent(int jointIndex, float torque)
{
    // 关节索引1-3对应电机ID 1-3
    if (jointIndex >= 1 && jointIndex <= 3) {
        sendTorqueCommand(static_cast<uint8_t>(jointIndex), torque);
    }

    // 转发信号给MainWindow
    emit torqueCommandSent(jointIndex, torque);
}

std::array<uint8_t, 30> RobotController::buildCommandFrame(uint8_t motorId, const std::array<uint8_t, 9>& data) const
{
    damiao::can_send_frame frame;
    // 使用默认初始化
    frame = damiao::can_send_frame();
    frame.CMD = 0x01; // 转发CAN数据帧
    frame.canId = motorId;
    frame.len = 0x08;
    // 复制数据到data和crc
    std::copy(data.begin(), data.begin() + 8, frame.data);
    frame.crc = data[8];
    // 将结构体转换为字节数组
    std::array<uint8_t, 30> byteArray;
    memcpy(byteArray.data(), &frame, sizeof(frame));
    return byteArray;
}