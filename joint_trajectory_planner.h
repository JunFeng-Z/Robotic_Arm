#ifndef JOINT_TRAJECTORY_PLANNER_H
#define JOINT_TRAJECTORY_PLANNER_H

#include "robot_common.h"
#include <vector>
#include <fstream>
#include <string>

/**
 * @brief 插值类型枚举
 */
enum class InterpolationType {
    Cubic,      // 三次多项式插值 (保证位置、速度连续)
    Quintic     // 五次多项式插值 (保证位置、速度、加速度连续)
};

/**
 * @brief 关节空间轨迹规划器 - 生成点到点的平滑关节轨迹
 *
 * 支持三次和五次多项式插值
 * 支持3个关节的独立规划
 */
class JointTrajectoryPlanner
{
public:
    /**
     * @brief 默认构造函数
     */
    JointTrajectoryPlanner();

    /**
     * @brief 设置插值类型
     * @param type 插值类型 (Cubic 或 Quintic)
     */
    void setInterpolationType(InterpolationType type);

    /**
     * @brief 设置轨迹参数
     * @param startAngles 起始关节角度 [q1, q2, q3] (rad)
     * @param targetAngles 目标关节角度 [q1, q2, q3] (rad)
     * @param duration 轨迹持续时间 (秒)
     * @param startVelocities 起始关节速度 (可选，默认为0) [qd1, qd2, qd3] (rad/s)
     * @param targetVelocities 目标关节速度 (可选，默认为0) [qd1, qd2, qd3] (rad/s)
     */
    void setTrajectory(const Eigen::Vector3f& startAngles,
                       const Eigen::Vector3f& targetAngles,
                       float duration,
                       const Eigen::Vector3f& startVelocities = Eigen::Vector3f::Zero(),
                       const Eigen::Vector3f& targetVelocities = Eigen::Vector3f::Zero());

    /**
     * @brief 生成轨迹点
     * @param t 时间 (秒)，从轨迹开始计时
     * @return 关节状态 [q1, q2, q3], [qd1, qd2, qd3], [qdd1, qdd2, qdd3]
     */
    void generatePoint(float t, Eigen::Vector3f& q, Eigen::Vector3f& qd, Eigen::Vector3f& qdd) const;

    /**
     * @brief 预计算完整轨迹
     * @param dt 时间步长 (秒)
     * @return 轨迹点序列
     */
    std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f>>
    generateTrajectory(float dt = 0.001f) const;

    /**
     * @brief 检查轨迹是否完成
     * @param t 当前时间 (秒)
     * @return true 如果轨迹已完成
     */
    bool isTrajectoryFinished(float t) const;

    /**
     * @brief 获取轨迹持续时间
     */
    float getDuration() const { return duration_; }

    /**
     * @brief 获取起始关节角度
     */
    const Eigen::Vector3f& getStartAngles() const { return startAngles_; }

    /**
     * @brief 获取目标关节角度
     */
    const Eigen::Vector3f& getTargetAngles() const { return targetAngles_; }

    /**
     * @brief 检查轨迹是否已设置
     */
    bool isTrajectorySet() const { return trajectorySet_; }

    /**
     * @brief 清除当前轨迹
     */
    void clearTrajectory();

private:
    // 计算多项式系数（根据当前插值类型）
    void computeCoefficients();

    // 轨迹参数
    Eigen::Vector3f startAngles_;
    Eigen::Vector3f targetAngles_;
    Eigen::Vector3f startVelocities_;
    Eigen::Vector3f targetVelocities_;
    float duration_;

    // 插值类型
    InterpolationType interpolationType_;

    // 三次多项式系数矩阵 (3x4): 每行对应一个关节，每列对应系数 a0, a1, a2, a3
    // q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    Eigen::Matrix<float, 3, 4> cubicCoeffs_;

    // 五次多项式系数矩阵 (3x6): 每行对应一个关节，每列对应系数 c0, c1, c2, c3, c4, c5
    // q(τ) = c0 + c1*τ + c2*τ² + c3*τ³ + c4*τ⁴ + c5*τ⁵, 其中 τ = t/duration ∈ [0,1]
    Eigen::Matrix<float, 3, 6> quinticCoeffs_;

    bool trajectorySet_;

    // 轨迹记录文件
    mutable std::ofstream trajectoryLogFile_;
    std::string trajectoryLogFileName_;

    // 初始化轨迹记录文件
    void initTrajectoryLogFile() const;
};

#endif // JOINT_TRAJECTORY_PLANNER_H