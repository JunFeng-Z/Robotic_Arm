#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include "robot_common.h"
#include <vector>
#include <memory>

/**
 * @brief 机器人类 - 包含运动学、动力学和控制算法
 */
class RobotModel
{
public:
    /**
     * @brief 构造函数
     * @param params 机器人参数
     */
    explicit RobotModel(const RobotParams& params = RobotParams());

    /**
     * @brief 设置机器人参数
     */
    void setParameters(const RobotParams& params);

    /**
     * @brief 获取机器人参数
     */
    const RobotParams& getParameters() const { return params_; }

    // ==================== 运动学计算 ====================

    /**
     * @brief 正运动学：关节空间 -> 工作空间
     * @param q 关节角度 [q1, q2, q3] (rad)
     * @param position 输出末端位置 (x, y, z) (m)
     * @return 是否成功
     */
    bool forwardKinematics(const Vector3f& q, Vector3f& position) const;

    /**
     * @brief 逆运动学：工作空间 -> 关节空间
     * @param position 末端位置 (x, y, z) (m)
     * @param q 输出关节角度 [q1, q2, q3] (rad)
     * @param elbow 肘部配置 (+1 或 -1)
     * @return 是否成功
     */
    bool inverseKinematics(const Vector3f& position, Vector3f& q, int elbow = -1) const;

    /**
     * @brief 计算雅可比矩阵
     * @param q 关节角度 [q1, q2, q3] (rad)
     * @return 3x3 雅可比矩阵
     */
    Matrix3f computeJacobian(const Vector3f& q) const;

    /**
     * @brief 计算雅可比矩阵的导数
     * @param q 关节角度 [q1, q2, q3] (rad)
     * @param qd 关节速度 [qd1, qd2, qd3] (rad/s)
     * @return 3x3 雅可比矩阵导数
     */
    Matrix3f computeJacobianDerivative(const Vector3f& q, const Vector3f& qd) const;

    // ==================== 动力学计算 ====================

    /**
     * @brief 计算质量矩阵 M(q)
     * @param q 关节角度 [q1, q2, q3] (rad)
     * @return 3x3 质量矩阵
     */
    Matrix3f computeMassMatrix(const Vector3f& q) const;

    /**
     * @brief 计算科里奥利和离心力矩阵 C(q, qd)
     * @param q 关节角度 [q1, q2, q3] (rad)
     * @param qd 关节速度 [qd1, qd2, qd3] (rad/s)
     * @return 3x3 科里奥利矩阵
     */
    Matrix3f computeCoriolisMatrix(const Vector3f& q, const Vector3f& qd) const;

    /**
     * @brief 计算重力向量 G(q)
     * @param q 关节角度 [q1, q2, q3] (rad)
     * @return 3x1 重力向量
     */
    Vector3f computeGravityVector(const Vector3f& q) const;

    /**
     * @brief 计算完整动力学：M(q)qdd + C(q,qd)qd + G(q) = tau
     * @param q 关节角度 [q1, q2, q3] (rad)
     * @param qd 关节速度 [qd1, qd2, qd3] (rad/s)
     * @param qdd 关节加速度 [qdd1, qdd2, qdd3] (rad/s²)
     * @return 3x1 力矩向量 tau
     */
    Vector3f computeInverseDynamics(const Vector3f& q, const Vector3f& qd, const Vector3f& qdd) const;

    /**
     * @brief 计算正向动力学：给定力矩计算加速度
     * @param q 关节角度 [q1, q2, q3] (rad)
     * @param qd 关节速度 [qd1, qd2, qd3] (rad/s)
     * @param tau 关节力矩 [tau1, tau2, tau3] (N·m)
     * @return 3x1 关节加速度 qdd
     */
    Vector3f computeForwardDynamics(const Vector3f& q, const Vector3f& qd, const Vector3f& tau) const;

    /**
     * @brief 从C#代码移植的力矩计算函数 (Closed_Arm_Modle_decoup)
     * @param jointstate [q1, q2, q3, qd1, qd2, qd3, v1, v2, v3]
     * @return [tau1, tau2, tau3] 力矩
     */
    Vector3f computeTorqueDecoupled(const std::vector<float>& jointstate) const;

    /**
     * @brief 重力补偿力矩计算 (Closed_Arm_Modle_thetch)
     * @param q 关节角度 [q1, q2, q3] (rad)
     * @return [tau1, tau2, tau3] 重力补偿力矩
     */
    Vector3f computeGravityCompensation(const Vector3f& q) const;

    // ==================== 控制算法 ====================

    /**
     * @brief PD控制器
     * @param q_desired 期望关节角度
     * @param qd_desired 期望关节速度
     * @param q_current 当前关节角度
     * @param qd_current 当前关节速度
     * @param Kp 位置增益矩阵 (3x3)
     * @param Kd 速度增益矩阵 (3x3)
     * @return 控制力矩
     */
    Vector3f computePDControl(const Vector3f& q_desired,
                             const Vector3f& qd_desired,
                             const Vector3f& q_current,
                             const Vector3f& qd_current,
                             const Matrix3f& Kp,
                             const Matrix3f& Kd) const;

    /**
     * @brief 计算力矩控制 (计算力矩法)
     * @param q_desired 期望关节角度
     * @param qd_desired 期望关节速度
     * @param qdd_desired 期望关节加速度
     * @param q_current 当前关节角度
     * @param qd_current 当前关节速度
     * @param Kp 位置增益矩阵
     * @param Kd 速度增益矩阵
     * @return 控制力矩
     */
    Vector3f computeTorqueControl(const Vector3f& q_desired,
                                 const Vector3f& qd_desired,
                                 const Vector3f& qdd_desired,
                                 const Vector3f& q_current,
                                 const Vector3f& qd_current,
                                 const Matrix3f& Kp,
                                 const Matrix3f& Kd) const;

    /**
     * @brief 滑模控制器 (从C#代码移植)
     * @param q_desired 期望关节角度
     * @param qd_desired 期望关节速度
     * @param q_current 当前关节角度
     * @param qd_current 当前关节速度
     * @param K 滑模面增益
     * @param eta 切换增益
     * @return 控制力矩
     */
    Vector3f computeSlidingModeControl(const Vector3f& q_desired,
                                      const Vector3f& qd_desired,
                                      const Vector3f& q_current,
                                      const Vector3f& qd_current,
                                      const Vector3f& K,
                                      const Vector3f& eta) const;

    /**
     * @brief 力矩限幅
     * @param torque 输入力矩
     * @param limits 力矩限制 [min1, max1, min2, max2, min3, max3]
     * @return 限幅后的力矩
     */
    Vector3f clampTorque(const Vector3f& torque, const std::vector<float>& limits) const;

private:
    RobotParams params_;  // 机器人参数

    // 辅助函数
    Matrix3f computeTransformMatrix(int i, const Vector3f& q) const;
    Vector3f computeLinkVelocity(int i, const Vector3f& q, const Vector3f& qd) const;
    Vector3f computeLinkAcceleration(int i, const Vector3f& q, const Vector3f& qd, const Vector3f& qdd) const;

    // C#代码中的辅助函数
    float computeM1_Q1(float q2, float q3) const;
    float computeM2_Q2(float q3) const;
    float computeM2_Q3(float q3) const;
    float computeM3_Q2(float q3) const;
    float computeM3_Q3() const;
    float computeC1_Q1Q2(float q2, float q3) const;
    float computeC1_Q1Q3(float q2, float q3) const;
    float computeC2_Q1Q1(float q2, float q3) const;
    float computeC2_Q2Q2(float q3) const;
    float computeC2_Q2Q3(float q3) const;
    float computeC2_Q3Q3(float q3) const;
    float computeC3_Q1Q1(float q2, float q3) const;
    float computeC3_Q2Q2(float q3) const;
    float computeG2(float q2, float tau3) const;
    float computeG3(float q2, float q3) const;
};

#endif // ROBOT_MODEL_H