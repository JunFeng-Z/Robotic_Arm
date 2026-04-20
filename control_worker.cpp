#include "control_worker.h"
#include <cmath>
#include <chrono>
#include <thread>
#include <cstdarg>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>

// 辅助函数：获取当前时间（毫秒）
static uint64_t currentMillis() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

ControlWorker::ControlWorker(const Callbacks& callbacks)
    : callbacks_(callbacks), logFileName_("")
{
    // params_构造时全部使用的默认值
    // 创建默认的机器人模型和轨迹生成器
    robotModel_ = std::make_unique<RobotModel>(params_.robotParams);
    trajectoryGenerator_ = std::make_unique<TrajectoryGenerator>(params_.trajectory);
    jointTrajectoryPlanner_ = std::make_unique<JointTrajectoryPlanner>();
    getReferenceTorques();
}

ControlWorker::~ControlWorker()
{
    stop();
    if (logFile_.is_open()) {
        logFile_.close();
    }
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

void ControlWorker::initLogFile()
{
    // 固定文件名
    logFileName_ = "control_log.txt";

    // 检查文件是否存在，存在则删除
    if (std::filesystem::exists(logFileName_)) {
        std::filesystem::remove(logFileName_);
    }

    // 打开文件进行写入
    logFile_.open(logFileName_, std::ios::out);
    if (!logFile_.is_open()) {
        log("无法打开记录文件: %s", logFileName_.c_str());
        return;
    }

    // 写入标题行
    logFile_ << "time,"
             << "q1,q2,q3," << "qd1,qd2,qd3," << "qdd1,qdd2,qdd3,"
             << "q_des1,q_des2,q_des3," << "qd_des1,qd_des2,qd_des3," << "qdd_des1,qdd_des2,qdd_des3,"
             << "ref_tau1,ref_tau2,ref_tau3," << "control_tau1,control_tau2,control_tau3," << "final_tau1,final_tau2,final_tau3" << std::endl;

    log("记录文件已创建: %s", logFileName_.c_str());
}

void ControlWorker::start()
{
    if (running_.load()) {
        log("控制线程已在运行");
        return;
    }

    running_.store(true);
    startTime_ = getCurrentTime();

    // 初始化记录文件
    initLogFile();

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
    //std::cout << "控制线程已停止" << std::endl;
}

void ControlWorker::updateJointState(const JointState &state)
{
    // 这里是否考虑加上时间戳，三个电机的时间戳应当匹配才能组合为一个状态
    std::lock_guard<std::mutex> locker(stateMutex_);
    static bool firstUpdate = true;
    if (firstUpdate) {
        std::cout << "Received joint state: index=" << state.jointIndex
                    << ", pos=" << state.position << ", vel=" << state.velocity
                    << ", acc=" << state.acceleration << std::endl;
        firstUpdate = false;
    }

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

        //std::cout << running_.load() << std::endl;
        while (running_.load())
        {
            //std::cout << running_.load() << std::endl;
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
            case ControlAlgorithm::PlannedTrajectoryTracking:
            {
                if(lastAlgorithm_ != ControlAlgorithm::PlannedTrajectoryTracking) {
                    startTime_ = getCurrentTime(); // 切换到预定轨迹跟踪控制时重置开始时间
                    lastAlgorithm_ = ControlAlgorithm::PlannedTrajectoryTracking;
                }
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
            case ControlAlgorithm::TeachingTrajectoryTracking:
            {
                break;
            }
            case ControlAlgorithm::JointPositionControl:
            {
                if(lastAlgorithm_ != ControlAlgorithm::JointPositionControl) {
                    startTime_ = getCurrentTime(); // 切换到关节位置控制时重置开始时间
                    lastAlgorithm_ = ControlAlgorithm::JointPositionControl;
                }
                double currentTime = getCurrentTime();
                double elapsedTime = currentTime - startTime_;
                //std::cout << "Elapsed time: " << elapsedTime << std::endl;
                if (!jointPositionControl(elapsedTime)) {
                    throw std::runtime_error("Failed to compute control command using joint position control");
                }
                break;
            }
            default:
            {
                log("警告：未知的控制算法");
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
    //std::cout << "Joint positions: " << currentStates[1].position << ", "
    //          << currentStates[2].position << ", " << currentStates[3].position << std::endl;
    // 发送扭矩命令到电机
    //std::cout << "Gravity Compensation Torques: " << tau.transpose() << std::endl;
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

    const float phi1 = 0.05f;
    const float phi2 = 0.05f;
    const float phi3 = 0.05f;

    Eigen::Vector3f Yd;
    Yd[0] = params_.sliding_Yd1 * std::tanh(s[0] / phi1);
    Yd[1] = params_.sliding_Yd2 * std::tanh(s[1] / phi2);
    Yd[2] = params_.sliding_Yd3 * std::tanh(s[2] / phi3);

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

    // 记录数据
    if (logFile_.is_open()) {
        float currentTime = getCurrentTime();
        logFile_ << std::fixed << std::setprecision(6)
                 << t << ","
                 << q_current[0] << "," << q_current[1] << "," << q_current[2] << ","
                 << qd_current[0] << "," << qd_current[1] << "," << qd_current[2] << ","
                 << qdd_current[0] << "," << qdd_current[1] << "," << qdd_current[2] << ","
                 << q_desired[0] << "," << q_desired[1] << "," << q_desired[2] << ","
                 << qd_desired[0] << "," << qd_desired[1] << "," << qd_desired[2] << ","
                 << qdd_desired[0] << "," << qdd_desired[1] << "," << qdd_desired[2] << ","
                 << referenceT[0] << "," << referenceT[1] << "," << referenceT[2] << ","
                 << estimateT[0] << "," << estimateT[1] << "," << estimateT[2] << ","
                 << ControlT[0] << "," << ControlT[1] << "," << ControlT[2] <<","
                 << e[0] << "," << e[1] << "," << e[2] << ","
                 << V[0] << "," << V[1] << "," << V[2] << ","
                 << de[0] << "," << de[1] << "," << de[2] << std::endl;
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

double ControlWorker::getCurrentTime() const
{
    return static_cast<double>(currentMillis()) / 1000.0;
}

bool ControlWorker::setTargetJointAngles(const Eigen::Vector3f& targetAngles, float duration)
{
    if (duration <= 0.0f) {
        log("错误：轨迹持续时间必须为正数");
        return false;
    }

    // 切换到关节位置控制算法
    switchControlAlgorithm(ControlAlgorithm::JointPositionControl);

    // 规划轨迹
    if (!planJointTrajectory(targetAngles, duration)) {
        log("错误：规划关节轨迹失败");
        return false;
    }

    jointTrajectoryPlanned_.store(true);

    log("关节位置控制已设置：目标角度=[%.3f, %.3f, %.3f] rad，持续时间=%.2f s",
        targetAngles[0], targetAngles[1], targetAngles[2], duration);

    return true;
}

bool ControlWorker::planJointTrajectory(const Eigen::Vector3f& targetAngles, float duration)
{ 
       // 获取当前关节位置
    Eigen::Vector3f currentAngles;
    {
        std::lock_guard<std::mutex> lock1(stateMutex_);
        currentAngles << jointStates_[1].position, jointStates_[2].position, jointStates_[3].position;
    }
    //std::cout << "Current joint angles: " << currentAngles.transpose() << std::endl;
    log("当前关节位置: [%.3f, %.3f, %.3f] rad", currentAngles[0]*180.0/M_PI, currentAngles[1]*180.0/M_PI, currentAngles[2]*180.0/M_PI);
    // 设置轨迹规划器
    try {
        jointTrajectoryPlanner_->setTrajectory(currentAngles, targetAngles, duration);
    } catch (const std::exception& e) {
        log("轨迹规划异常：%s", e.what());
        return false;
    }

    // 预计算完整轨迹（用于滑模控制）
    auto trajectory = jointTrajectoryPlanner_->generateTrajectory(controlPeriodMs_ / 1000.0f);

    // 清空现有轨迹数据
    torque_d.clear();
    q_d.clear();
    qd_d.clear();
    qdd_d.clear();

    // 填充预计算轨迹
    for (const auto& [q, qd, qdd] : trajectory) {
        // 计算所需扭矩
        Eigen::Vector3f tau = computeTorques(q, qd, qdd);

        torque_d.push_back(tau);
        q_d.push_back(q);
        qd_d.push_back(qd);
        qdd_d.push_back(qdd);
        //std::cout << q.transpose() << " " << qd.transpose() << " " << qdd.transpose() << " " << tau.transpose() << std::endl;
    }

    log("关节轨迹规划完成：%d 个轨迹点，步长=%d ms",
        static_cast<int>(trajectory.size()), controlPeriodMs_);
 
    return true;
}

bool ControlWorker::jointPositionControl(float t)
{
    // 检查轨迹是否已规划
    if (!jointTrajectoryPlanned_.load()) {
        log("错误：关节轨迹未规划");
        return false;
    }

    // 检查轨迹是否完成
    {
        std::lock_guard<std::mutex> lock(jointTrajectoryMutex_);
        if (jointTrajectoryPlanner_->isTrajectoryFinished(t)) {
            log("关节位置控制已完成，时长: %.2f秒", t);

            // 切换到重力补偿模式以保持位置
            switchControlAlgorithm(ControlAlgorithm::GravityCompensation);
            jointTrajectoryPlanned_.store(false);

            // 通知轨迹完成，但不停止控制循环
            return true;
        }
    }

    // 使用滑模控制器跟踪预计算轨迹
    return slidingModeController(t);
}