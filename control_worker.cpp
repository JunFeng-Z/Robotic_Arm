#include "control_worker.h"
#include <cmath>
#include <chrono>
#include <thread>
#include <cstdarg>
#include <cstdio>
#include <iostream>

// 辅助函数：获取当前时间（毫秒）
static uint64_t currentMillis() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

ControlWorker::ControlWorker(const Callbacks& callbacks)
    : callbacks_(callbacks)
{
    // params_构造时全部使用的默认值
    // 创建默认的机器人模型和轨迹生成器
    robotModel_ = std::make_unique<RobotModel>(params_.robotParams);
    trajectoryGenerator_ = std::make_unique<TrajectoryGenerator>(params_.trajectory);
    getReferenceTorques();
}

ControlWorker::~ControlWorker()
{
    stop();
}

void ControlWorker::log(const std::string& message)
{
    if (callbacks_.log) {
        callbacks_.log(message);
    }
}

void ControlWorker::log(const char* format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    log(std::string(buffer));
}

void ControlWorker::start()
{
    if (running_.load()) {
        log("控制线程已在运行");
        return;
    }

    running_.store(true);
    startTime_ = getCurrentTime();

    log("控制线程已启动，周期: %dms", controlPeriodMs_);

    // 启动工作线程执行控制循环
    workerThread_ = std::thread([this]() { controlLoop(); });
}

void ControlWorker::stop()
{
    if (!running_.load()) {
        return;
    }

    running_.store(false);

    if (workerThread_.joinable()) {
        workerThread_.join();
    }

    log("控制线程已停止");
}

void ControlWorker::updateJointState(const JointState &state)
{
    // 这里是否考虑加上时间戳，三个电机的时间戳应当匹配才能组合为一个状态
    std::lock_guard<std::mutex> locker(stateMutex_);
    std::cout << "Received joint state: index=" << state.jointIndex
              << ", pos=" << state.position << ", vel=" << state.velocity
              << ", acc=" << state.acceleration << std::endl;
    if (state.jointIndex >= 1 && state.jointIndex <= 3) {
        jointStates_[state.jointIndex] = state;
    }
}

void ControlWorker::setControlParams(const ControlParams &params)
{
    std::lock_guard<std::mutex> locker(paramsMutex_);
    params_ = params;
    controlPeriodMs_ = params.controlPeriod;

    // 更新模型和生成器
    updateModelFromParams();
}

void ControlWorker::initTrajectory()
{
    trajectoryInitialized_.store(true);
    startTime_ = getCurrentTime();
    log("轨迹跟踪已初始化");
}

void ControlWorker::updateModelFromParams()
{
    std::lock_guard<std::mutex> locker(modelMutex_);

    if (robotModel_) {
        robotModel_->setParameters(params_.robotParams);
    }

    if (trajectoryGenerator_) {
        trajectoryGenerator_->setParameters(params_.trajectory);
    }
}

void ControlWorker::clearMoveIndex()
{
    moveIndex_.store(0);
    log("预定轨迹索引已重置");
}

void ControlWorker::switchControlAlgorithm(ControlAlgorithm algorithm)
{
    currentAlgorithm_ = algorithm;
    log("控制算法已切换");
}

// 控制循环函数，持续计算控制命令并发送
void ControlWorker::controlLoop()
{
    try {
        auto nextWakeTime = std::chrono::steady_clock::now();

        std::cout << running_.load() << std::endl;
        while (running_.load())
        {
            nextWakeTime += std::chrono::milliseconds(controlPeriodMs_);

            switch (currentAlgorithm_)
            {
            case ControlAlgorithm::GravityCompensation:
            {
                if (!gravityCompensation()) {
                    throw std::runtime_error("Failed to compute control command using gravity compensation");
                }
                break;
            }
            case ControlAlgorithm::SlidingMode:
            {
                float currentTime = getCurrentTime();
                float elapsedTime = currentTime - startTime_;

                {
                    std::lock_guard<std::mutex> locker(paramsMutex_);
                    if (elapsedTime > params_.trajectory.duration) {
                        log("轨迹跟踪已完成，时长: %.2f秒", elapsedTime);
                        running_.store(false);
                        break;
                    }
                }

                if (!slidingModeController(elapsedTime)) {
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
        log("控制线程异常: %s", e.what());
        running_.store(false);
    }

    if (callbacks_.controlStatus) {
        callbacks_.controlStatus(false);
    }
}

void ControlWorker::getReferenceTorques()
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
            log("时间 t=%.3f 时计算失败", t);
            throw std::runtime_error("Failed to compute joint states from trajectory");
        }
        tau = this->computeTorques(q, qd, qdd);
        torque_d.push_back(tau);
        q_d.push_back(q);
        qd_d.push_back(qd);
        qdd_d.push_back(qdd);
    }
}

bool ControlWorker::gravityCompensation()
{
    // 获取当前关节状态
    std::array<JointState, 4> currentStates;
    {
        std::lock_guard<std::mutex> locker(stateMutex_);
        currentStates = jointStates_;
    }
    Eigen::Vector3f tau = robotModel_->gravityCompensation(
        Eigen::Vector3f(currentStates[1].position, currentStates[2].position, currentStates[3].position)
    );
    std::cout << "Joint positions: " << currentStates[1].position << ", "
              << currentStates[2].position << ", " << currentStates[3].position << std::endl;
    // 发送扭矩命令到电机
    std::cout << "Gravity Compensation Torques: " << tau.transpose() << std::endl;
    if (callbacks_.torqueCommand) {
        callbacks_.torqueCommand(1, tau[0]);
        callbacks_.torqueCommand(2, tau[1]);
        callbacks_.torqueCommand(3, tau[2]);
    }
    return true;
}

bool ControlWorker::slidingModeController(float t)
{
    // 获取时间索引
    int index = static_cast<int>(t * 1000 / controlPeriodMs_);

    // 检查索引是否在预定义轨迹范围内
    if (index >= torque_d.size()) {
        log("时间 t=%.3f 超出预定义轨迹范围", t);
        return false;
    }

    // 获取期望关节状态（从预定义轨迹）
    Eigen::Vector3f q_desired = q_d[index];
    Eigen::Vector3f qd_desired = qd_d[index];
    Eigen::Vector3f qdd_desired = qdd_d[index];

    // 获取当前关节状态
    std::array<JointState, 4> currentStates;
    {
        std::lock_guard<std::mutex> locker(stateMutex_);
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
        std::lock_guard<std::mutex> locker(modelMutex_);
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
    if (callbacks_.torqueCommand) {
        callbacks_.torqueCommand(1, ControlT[0]);
        callbacks_.torqueCommand(2, ControlT[1]);
        callbacks_.torqueCommand(3, ControlT[2]);
    }
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

float ControlWorker::getCurrentTime() const
{
    return static_cast<float>(currentMillis()) / 1000.0f;
}