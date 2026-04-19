#ifndef CONTROL_WORKER_H
#define CONTROL_WORKER_H

#include <array>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <vector>

#include "robot_common.h"
#include "robot_model.h"
#include "trajectory_generator.h"

/**
 * @brief 控制工作线程（独立线程执行控制算法）
 * 使用标准C++线程，不依赖Qt
 */
class ControlWorker
{
public:
    // 回调函数类型定义
    using LogCallback = std::function<void(const std::string&)>;
    using TorqueCommandCallback = std::function<void(int jointIndex, float torque)>;
    using ControlCommandCallback = std::function<void(int jointIndex, float targetPos, float targetVel)>;
    using ControlStatusCallback = std::function<void(bool running)>;

    // 回调结构体，由RobotController设置
    struct Callbacks {
        LogCallback log;
        TorqueCommandCallback torqueCommand;
        ControlCommandCallback controlCommand;
        ControlStatusCallback controlStatus;
    };

    explicit ControlWorker(const Callbacks& callbacks);
    ~ControlWorker();

    // 禁止拷贝和移动
    ControlWorker(const ControlWorker&) = delete;
    ControlWorker& operator=(const ControlWorker&) = delete;

    // 公共接口，线程安全
    void start();
    void stop();
    void updateJointState(const JointState &state);
    void setControlParams(const ControlParams &params);
    void initTrajectory();
    void clearMoveIndex();
    void switchControlAlgorithm(ControlAlgorithm algorithm);

    // 获取当前控制算法
    ControlAlgorithm currentAlgorithm() const { return currentAlgorithm_; }

    // 检查是否正在运行
    bool isRunning() const { return running_.load(); }

private:
    // 控制循环函数
    void controlLoop();

    // 算法实现
    bool computeJointStatesFromTrajectory(float t, Eigen::Vector3f& q, Eigen::Vector3f& qd, Eigen::Vector3f& qdd);
    bool computeInverseKinematicsFull(const Eigen::Vector3f& position,
                                      const Eigen::Vector3f& velocity,
                                      const Eigen::Vector3f& acceleration,
                                      Eigen::Vector3f& q,
                                      Eigen::Vector3f& qd,
                                      Eigen::Vector3f& qdd,
                                      int elbow = -1);
    Eigen::Vector3f computeTorques(const Eigen::Vector3f& q,
                                   const Eigen::Vector3f& qd,
                                   const Eigen::Vector3f& qdd);
    void getReferenceTorques();
    float getCurrentTime() const;
    void updateModelFromParams();
    bool slidingModeController(float t);
    bool gravityCompensation();

    // 辅助函数：记录日志（线程安全通过回调）
    void log(const std::string& message);
    void log(const char* format, ...);

    // 回调函数
    Callbacks callbacks_;

    // 数据成员
    std::array<JointState, 4> jointStates_;  // 索引1-3对应关节1-3
    mutable std::mutex stateMutex_;  // 保护关节状态
    ControlParams params_;
    mutable std::mutex paramsMutex_;  // 保护参数

    // 机器人模型和轨迹生成器
    std::unique_ptr<RobotModel> robotModel_;
    std::unique_ptr<TrajectoryGenerator> trajectoryGenerator_;
    mutable std::mutex modelMutex_;  // 保护模型和生成器

    std::atomic_bool running_{false};
    std::atomic_bool trajectoryInitialized_{false};
    float startTime_ = 0.0f;
    int controlPeriodMs_ = 1; // 单位毫秒

    std::atomic_int moveIndex_{0};  // 预定义轨迹点的起始索引
    std::vector<Eigen::Vector3f> torque_d;
    std::vector<Eigen::Vector3f> q_d;
    std::vector<Eigen::Vector3f> qd_d;
    std::vector<Eigen::Vector3f> qdd_d;
    ControlAlgorithm currentAlgorithm_;

    // 线程管理
    std::thread workerThread_;
};

#endif // CONTROL_WORKER_H