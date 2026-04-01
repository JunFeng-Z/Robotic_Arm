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

        // 逆速度计算（直接传入雅可比矩阵）
    bool inverseVelocity(const Matrix3f& J, const Vector3f& end_vel, Vector3f& qd) const;
    
    // 使用QR分解的逆速度计算
    bool inverseVelocityQR(const Matrix3f& J, const Vector3f& end_vel, Vector3f& qd) const;
    
    // 使用阻尼最小二乘法的逆速度计算
    bool inverseVelocityDamped(const Matrix3f& J, const Vector3f& end_vel, Vector3f& qd, float lambda = 0.01f) const;
    
    Vector3f forwardVelocity(const Vector3f& q, const Vector3f& qd) const;

        // 逆加速度计算
    bool inverseAcceleration(const Vector3f& q, const Vector3f& qd, 
                            const Vector3f& end_acc, Vector3f& qdd) const;
    
    bool inverseAccelerationQR(const Vector3f& q, const Vector3f& qd, 
                              const Vector3f& end_acc, Vector3f& qdd) const;



private:
    RobotParams params_;  // 机器人参数

    Matrix4d zero_config_pose_M_;  // 零位姿的齐次变换矩阵
    // 计算零位姿的齐次变换矩阵
    void calculateZeroConfigPoseM();

    Matrix4d dh_transform(double a, double alpha, double d, double theta);

    Eigen::Matrix3d skew(const Eigen::Vector3d& w) const;
    Eigen::Matrix4d expm_screw(const Eigen::VectorXd& S, double theta) const;
    Matrix6d adjoint(const Eigen::Matrix4d& T) const;
    Eigen::Matrix<double, 6, 6> ad(const Eigen::VectorXd& A) const;//李括号运算

    void CalculateSList();


    Eigen::VectorXd S1 = Eigen::VectorXd(6);
    Eigen::VectorXd S2 = Eigen::VectorXd(6);
    Eigen::VectorXd S3 = Eigen::VectorXd(6);


    Eigen::Matrix4d zero_pose_M1_;  // 对应M1
    Eigen::Matrix4d zero_pose_M2_;  // 对应M2
    Eigen::Matrix4d zero_pose_M3_;  // 对应M3


};

#endif // ROBOT_MODEL_H