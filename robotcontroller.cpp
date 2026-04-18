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
    //params_构造时全部使用的默认值
    // 创建默认的机器人模型和轨迹生成器
    robotModel_ = std::make_unique<RobotModel>(params_.robotParams);
    trajectoryGenerator_ = std::make_unique<TrajectoryGenerator>(params_.trajectory);
    GetReferenceTorques() ;

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
    //这里是否考虑加上时间辍，三个电机的时间辍应当匹配才能组合为一个状态
    QMutexLocker locker(&stateMutex_);
    std::cout<<"Received joint state: index=" << state.jointIndex << ", pos=" << state.position << ", vel=" << state.velocity << ", acc=" << state.acceleration << std::endl;
    if (state.jointIndex >= 1 && state.jointIndex <= 3) {
        jointStates_[state.jointIndex] = state;
    }
}

void ControlWorker::setControlParams(const ControlParams &params)
{
    QMutexLocker locker(&paramsMutex_);
    params_ = params;
    controlPeriodMs_ = params.controlPeriod;

    // 更新模型和生成器
    updateModelFromParams();
}

void ControlWorker::initTrajectory()
{
    trajectoryInitialized_.store(true);
    startTime_ = getCurrentTime();
    emit logMessage(QStringLiteral("轨迹跟踪已初始化"));
}

void ControlWorker::updateModelFromParams()
{
    QMutexLocker locker(&modelMutex_);

    if (robotModel_) {
        robotModel_->setParameters(params_.robotParams);
    }

    if (trajectoryGenerator_) {
        trajectoryGenerator_->setParameters(params_.trajectory);
    }
}
//控制循环函数，持续计算控制命令并发送
void ControlWorker::controlLoop()
{
    try {
        auto nextWakeTime = std::chrono::steady_clock::now();

        std::cout<<running_.load()<<std::endl;
        while (running_.load()) 
        {
            nextWakeTime += std::chrono::milliseconds(controlPeriodMs_);

            switch (currentAlgorithm_) 
            {
            case ControlAlgorithm::GravityCompensation: 
            {
                if (!GravityCompensation()) {
                    throw std::runtime_error("Failed to compute control command using gravity compensation");
                }
                break;
            }
            case ControlAlgorithm::SlidingMode: 
            {
                float currentTime = getCurrentTime();
                float elapsedTime = currentTime - startTime_;

                {
                    QMutexLocker locker(&paramsMutex_);
                    if (elapsedTime > params_.trajectory.duration) {
                        emit logMessage(QStringLiteral("轨迹跟踪已完成，时长: %1秒")
                                            .arg(elapsedTime, 0, 'f', 2));
                        running_.store(false);
                        break;
                    }
                }

                if (!Slidingmode_controller(elapsedTime)) {
                    throw std::runtime_error("Failed to compute control command using sliding mode controller");
                }

                break;
            }
            case ControlAlgorithm::PID: 
            {
                break;
            }
            default:
            {
                break;
            }
            }

            std::this_thread::sleep_until(nextWakeTime);
        }
    } catch (const std::exception& e) {
        emit logMessage(QStringLiteral("控制线程异常: %1").arg(e.what()));
        running_.store(false);
    }

    emit controlStatusChanged(false);
}

void ControlWorker::GetReferenceTorques() 
{
    torque_d.clear();
    q_d.clear();
    qd_d.clear();
    qdd_d.clear();
    int numPoints = params_.trajectory.duration * 1000 / controlPeriodMs_;
    for(int i=0;i<numPoints;i++)
    {
        float t = i * controlPeriodMs_ / 1000.0f;
        Eigen::Vector3f q, qd, qdd;
        Eigen::Vector3f tau;
        bool success = this->computeJointStatesFromTrajectory(t, q, qd, qdd);
        if (!success) 
        {
            emit logMessage(QStringLiteral("时间 t=%1 时计算失败").arg(t, 0, 'f', 3));
            throw std::runtime_error("Failed to compute joint states from trajectory");
        }
        tau = this->computeTorques(q, qd, qdd);
        torque_d.push_back(tau);
        q_d.push_back(q);
        qd_d.push_back(qd);
        qdd_d.push_back(qdd);
    }
}

void ControlWorker::SwitchControlAlgorithm(ControlAlgorithm algorithm)
{
    currentAlgorithm_ = algorithm;
    emit logMessage(QStringLiteral("控制算法已切换"));
}

bool ControlWorker::GravityCompensation()
{
    //emit logMessage(QStringLiteral("执行重力补偿控制算法"));
    // 获取当前关节状态
    std::array<JointState, 4> currentStates;
    {
        QMutexLocker locker(&stateMutex_);
        currentStates = jointStates_;
    }
    Eigen::Vector3f tau = robotModel_->gravityCompensation(
        Eigen::Vector3f(currentStates[1].position, currentStates[2].position, currentStates[3].position)
    );
    std::cout<<"Joint positions: " << currentStates[1].position << ", " << currentStates[2].position << ", " << currentStates[3].position << std::endl;
    // 发送扭矩命令到电机
    std::cout << "Gravity Compensation Torques: " << tau.transpose() << std::endl;
    emit torqueCommandSent(1, tau[0]);
    emit torqueCommandSent(2, tau[1]);
    emit torqueCommandSent(3, tau[2]);
    return true;
}

bool ControlWorker::Slidingmode_controller(float t)
{
    // 获取时间索引
    int index = static_cast<int>(t * 1000 / controlPeriodMs_);

    // 检查索引是否在预定义轨迹范围内
    if (index >= torque_d.size()) {
        emit logMessage(QStringLiteral("时间 t=%1 超出预定义轨迹范围").arg(t, 0, 'f', 3));
        return false;
    }


    // 获取期望关节状态（从预定义轨迹）
    Eigen::Vector3f q_desired = q_d[index];
    Eigen::Vector3f qd_desired = qd_d[index];
    Eigen::Vector3f qdd_desired = qdd_d[index];

    // 获取当前关节状态
    std::array<JointState, 4> currentStates;
    {
        QMutexLocker locker(&stateMutex_);
        currentStates = jointStates_;
    }

    // 提取当前关节状态到Eigen向量
    Eigen::Vector3f q_current, qd_current, qdd_current;
    q_current << currentStates[1].position, currentStates[2].position, currentStates[3].position;
    qd_current << currentStates[1].velocity, currentStates[2].velocity, currentStates[3].velocity;
    qdd_current << currentStates[1].acceleration, currentStates[2].acceleration, currentStates[3].acceleration;

    // 计算误差
    Eigen::Vector3f e = q_desired - q_current;
    Eigen::Vector3f de = qd_desired - qd_current;

    // 计算滑模面
    Eigen::Vector3f s = de + params_.K.cwiseProduct(e);

    // 计算切换函数
    Eigen::Vector3f Yd;
    Yd[0] = (54.0f + params_.sliding_a1) * std::tanh(s[0] / 3.0f);
    Yd[1] = (50.0f + params_.sliding_a2) * std::tanh(s[1]);
    // 符号函数实现
    Yd[2] = (110.0f + params_.sliding_b1) * ((s[2] > 0) ? 1.0f : ((s[2] < 0) ? -1.0f : 0.0f));

    // 计算控制量（期望加速度 + 补偿）
    Eigen::Vector3f V = qdd_desired + params_.K.cwiseProduct(de) + Yd;

    // 估计扭矩（通过逆向动力学）
    Eigen::Vector3f estimateT;
    {
        QMutexLocker locker(&modelMutex_);
        estimateT = robotModel_->inverseDynamicsL(q_current, qd_current, V);
    }

    // 获取参考扭矩
    Eigen::Vector3f referenceT = torque_d[index];

    // 计算控制扭矩并进行限幅
    Eigen::Vector3f ControlT = estimateT;

    // 关节扭矩限幅
    for (int i = 0; i < 3; ++i) {
        float upper_limit = referenceT[i] + params_.detlta[i];
        float lower_limit = referenceT[i] - params_.detlta[i];

        if (ControlT[i] > upper_limit) ControlT[i] = upper_limit;
        if (ControlT[i] < lower_limit) ControlT[i] = lower_limit;
    }

    // 发送扭矩命令到电机
    emit torqueCommandSent(1, ControlT[0]);
    emit torqueCommandSent(2, ControlT[1]);
    emit torqueCommandSent(3, ControlT[2]);
    return true;
}

Eigen::Vector3f ControlWorker::computeTorques(const Eigen::Vector3f& q,
                                                    const Eigen::Vector3f& qd,
                                                    const Eigen::Vector3f& qdd) {
    // 直接调用机器人的逆向动力学计算
    Eigen::Vector3f tau = robotModel_->inverseDynamics(q, qd, qdd);
    return tau;
}

bool ControlWorker::computeJointStatesFromTrajectory(float t,
                                                           Eigen::Vector3f& q,
                                                           Eigen::Vector3f& qd,
                                                           Eigen::Vector3f& qdd)
{
    // 1. 生成轨迹点
    TrajectoryPoint point = trajectoryGenerator_->generatePoint(t);

    // 2. 使用完整的逆运动学计算关节状态
    return computeInverseKinematicsFull(point.position, point.velocity, point.acceleration,
                                        q, qd, qdd);
}

bool ControlWorker::computeInverseKinematicsFull(const Eigen::Vector3f& position,
                                                       const Eigen::Vector3f& velocity,
                                                       const Eigen::Vector3f& acceleration,
                                                       Eigen::Vector3f& q,
                                                       Eigen::Vector3f& qd,
                                                       Eigen::Vector3f& qdd,
                                                       int elbow)
{
    // 步骤1: 逆运动学计算关节角度
    if (!robotModel_->inverseKinematics(position, q, elbow)) {
        return false;
    }

    // 步骤2: 计算雅可比矩阵
    Eigen::Matrix3f J = robotModel_->computeJacobian(q);

    // 步骤3: 逆速度计算 (使用QR分解，参考Matlab代码)
    if (!robotModel_->inverseVelocityQR(J, velocity, qd)) {
        // 如果QR分解失败，尝试其他方法
        if (!robotModel_->inverseVelocity(J, velocity, qd)) {
            return false;
        }
    }

    // 步骤4: 计算雅可比矩阵导数
    Eigen::Matrix3f dJ = robotModel_->computeJacobianDerivative(q, qd);

    // 步骤5: 逆加速度计算 (使用QR分解，参考Matlab代码)
    if (!robotModel_->inverseAccelerationQR(q, qd, acceleration, qdd)) {
        // 如果QR分解失败，尝试其他方法
        if (!robotModel_->inverseAcceleration(q, qd, acceleration, qdd)) {
            return false;
        }
    }

    return true;
}





//界面按钮清除数据时调用，UI图清除，这里重置预定轨迹的起始运行索引
void ControlWorker::clearMoveIndex()
{
    moveIndex_.store(0);
    emit logMessage(QStringLiteral("预定轨迹索引已重置"));
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
    connect(worker_, &ControlWorker::torqueCommandSent,
            this, &RobotController::onTorqueCommandSent);
    connect(worker_, &ControlWorker::controlStatusChanged,
            this, &RobotController::controlStatusChanged);
    connect(worker_, &ControlWorker::logMessage,
            this, &RobotController::logMessage);

    // 启动线程循环，等待调用startControl()后才执行控制循环
    controlThread_->start();
    SwitchControlAlgorithm(ControlAlgorithm::GravityCompensation);  // 默认使用重力补偿算法
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
//拿到底层通信线程解析出的关节状态，转发到工作线程，并发送状态到界面显示
void RobotController::updateJointState(int jointIndex, float position, float velocity)
{
    if (!worker_) {
        return;
    }

    JointState state(jointIndex, position, velocity,0, getCurrentTime());

    // 转发到工作线程
    QMetaObject::invokeMethod(worker_, "updateJointState",
                             Qt::QueuedConnection,
                             Q_ARG(JointState, state));

    // 同时发送状态更新信号
    emit jointStateChanged(state);
}
void RobotController::SwitchControlAlgorithm(ControlAlgorithm algorithm)
{
    currentAlgorithm_ = algorithm;
    if (worker_) {
        QMetaObject::invokeMethod(worker_, "SwitchControlAlgorithm",
                                 Qt::QueuedConnection,
                                 Q_ARG(ControlAlgorithm, algorithm));
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

    // 启动工作线程的控制循环
    QMetaObject::invokeMethod(worker_, "start", Qt::QueuedConnection);
}

void RobotController::clearMoveIndex()
{

    if (worker_) {
        QMetaObject::invokeMethod(worker_, "clearMoveIndex", Qt::QueuedConnection);
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
    // if (worker_) {
    //     QMetaObject::invokeMethod(worker_, "setControlParams",
    //                              Qt::QueuedConnection,
    //                              Q_ARG(ControlParams, params));
    // }

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

