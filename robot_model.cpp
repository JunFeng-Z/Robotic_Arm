#include "robot_model.h"
#include <cmath>

RobotModel::RobotModel(const RobotParams& params)
    : params_(params)
{
}

void RobotModel::setParameters(const RobotParams& params)
{
    params_ = params;
}

// ==================== 运动学计算 ====================

bool RobotModel::forwardKinematics(const Vector3f& q, Vector3f& position) const
{
    // 简化的正运动学，根据C#代码中的模型
    // 假设为平面3关节机器人，关节1绕Z轴，关节2和3在XY平面
    const float a2 = params_.dh[1].a;  // 0.12
    const float a3 = params_.dh[2].a;  // 0.12

    float q1 = q[0], q2 = q[1], q3 = q[2];

    position[0] = a2 * std::cos(q1) * std::cos(q2) + a3 * std::cos(q1) * std::cos(q2 + q3);
    position[1] = a2 * std::sin(q1) * std::cos(q2) + a3 * std::sin(q1) * std::cos(q2 + q3);
    position[2] = a2 * std::sin(q2) + a3 * std::sin(q2 + q3);

    return true;
}

bool RobotModel::inverseKinematics(const Vector3f& position, Vector3f& q, int elbow) const
{
    const float a2 = params_.dh[1].a;  // 0.12
    const float a3 = params_.dh[2].a;  // 0.12
    const float x = position[0], y = position[1], z = position[2];
    const float PI = static_cast<float>(M_PI);

    if (std::abs(z) < 1e-6f) {
        // z == 0 的情况
        float r = x * x + y * y + z * z;
        float sqrt_r = std::sqrt(r);
        float denominator = a2 + a3;
        if (denominator == 0.0f) {
            return false;
        }
        q[1] = std::acos(sqrt_r / denominator);
        q[2] = -q[1] - std::acos(sqrt_r / denominator);
        q[0] = std::atan2(y, x);
        return true;
    } else {
        // z != 0 的情况
        int FuY = (y < 0) ? -1 : 1;
        float r = x * x + y * y + z * z;
        float denominator = 2.0f * a2 * a3;
        if (denominator == 0.0f) {
            return false;
        }

        float cos_q3 = (r - (a2 * a2 + a3 * a3)) / denominator;
        if (cos_q3 < -1.0f || cos_q3 > 1.0f) {
            return false;
        }

        q[2] = elbow * std::acos(cos_q3);
        q[0] = std::atan2(y, x);

        // 计算q2
        float b = -a3 * std::cos(q[0]) * std::sin(q[2]);
        float a = a2 * std::cos(q[0]) + a3 * std::cos(q[0]) * std::cos(q[2]);
        float c = x;

        float q21, q22;
        if (std::abs(a) < 1e-6f) {
            q21 = PI / 2.0f;
        } else {
            q21 = std::atan2(b, a);
        }

        float sqrt_term = a * a + b * b - c * c;
        if (sqrt_term < 0.0f) {
            sqrt_term = 0.0f;
        }

        if (std::abs(c) < 1e-6f) {
            q22 = FuY * PI / 2.0f;
        } else {
            q22 = std::atan2(std::sqrt(sqrt_term), c);
        }

        q[1] = q21 + q22;
        return true;
    }
}

Matrix3f RobotModel::computeJacobian(const Vector3f& q) const
{
    // 简化的雅可比矩阵计算
    // TODO: 实现完整的雅可比矩阵计算
    Matrix3f J;
    J.setZero();
    return J;
}

Matrix3f RobotModel::computeJacobianDerivative(const Vector3f& q, const Vector3f& qd) const
{
    // TODO: 实现雅可比矩阵导数计算
    Matrix3f dJ;
    dJ.setZero();
    return dJ;
}

// ==================== 动力学计算 ====================

Matrix3f RobotModel::computeMassMatrix(const Vector3f& q) const
{
    // TODO: 实现质量矩阵计算
    Matrix3f M;
    M.setIdentity();
    return M;
}

Matrix3f RobotModel::computeCoriolisMatrix(const Vector3f& q, const Vector3f& qd) const
{
    // TODO: 实现科里奥利矩阵计算
    Matrix3f C;
    C.setZero();
    return C;
}

Vector3f RobotModel::computeGravityVector(const Vector3f& q) const
{
    // TODO: 实现重力向量计算
    return computeGravityCompensation(q);
}

Vector3f RobotModel::computeInverseDynamics(const Vector3f& q, const Vector3f& qd, const Vector3f& qdd) const
{
    // 使用质量矩阵-科里奥利-重力模型
    Matrix3f M = computeMassMatrix(q);
    Matrix3f C = computeCoriolisMatrix(q, qd);
    Vector3f G = computeGravityVector(q);

    return M * qdd + C * qd + G;
}

Vector3f RobotModel::computeForwardDynamics(const Vector3f& q, const Vector3f& qd, const Vector3f& tau) const
{
    // τ = M(q)qdd + C(q,qd)qd + G(q)
    // => qdd = M(q)^{-1}(τ - C(q,qd)qd - G(q))
    Matrix3f M = computeMassMatrix(q);
    Matrix3f C = computeCoriolisMatrix(q, qd);
    Vector3f G = computeGravityVector(q);

    return M.inverse() * (tau - C * qd - G);
}

Vector3f RobotModel::computeTorqueDecoupled(const std::vector<float>& jointstate) const
{
    // 从C#代码移植的Closed_Arm_Modle_decoup函数
    if (jointstate.size() < 9) {
        return Vector3f::Zero();
    }

    float q1 = jointstate[0], qd1 = jointstate[3], qdd1 = jointstate[6];
    float q2 = jointstate[1], qd2 = jointstate[4], qdd2 = jointstate[7];
    float q3 = jointstate[2], qd3 = jointstate[5], qdd3 = jointstate[8];

    const float a2 = params_.dh[1].a;  // 0.12
    const float a3 = params_.dh[2].a;  // 0.12
    const float m2 = params_.m[1];     // 0.35
    const float m3 = params_.m[2];     // 0.01

    // 从C#代码移植的常数
    const float Izz3 = 0.00107f;
    const float Izz2 = 0.00437f;
    const float Iyy3 = 0.00108f;
    const float Iyy2 = 0.00437f;

    // 计算G3
    float G3 = a3 * m3 * std::cos(q2 + q3) * 9.81f;

    // 计算C3_Q1Q1, C3_Q2Q2
    float C3_Q1Q1 = a3 * m3 * ((0.5f * a2 * std::sin(q3) + 0.5f * a3 * std::sin(2.0f * (q2 + q3)) + 0.5f * a2 * std::sin(2.0f * q2 + q3))) + 0.5f * Iyy3 * std::sin(2.0f * (q2 + q3));
    float C3_Q2Q2 = a3 * m3 * a2 * std::sin(q3);

    // 计算M3_Q3, M3_Q2
    float M3_Q3 = (a3 * a3 * m3 + Izz3);
    float M3_Q2 = (a3 * m3 * (a2 * std::cos(q3) + a3) + Izz3);

    // 计算tau3
    float tao3 = M3_Q2 * qdd2 + M3_Q3 * qdd3 + C3_Q1Q1 * qd1 * qd1 + C3_Q2Q2 * qd2 * qd2 + G3;

    // 计算G2
    float G2 = tao3 + a2 * std::cos(q2) * (9.81f * m2 + 9.81f * m3);

    // 计算C2相关项
    float C2_Q1Q1 = a2 * (a2 * std::sin(2.0f * q2) * 0.5f * (m2 + m3) + a3 * m3 * (-0.5f * std::sin(q3) + 0.5f * std::sin(2.0f * q2 + q3))) + 0.5f * std::sin(2.0f * q2) * Iyy2;
    float C2_Q2Q2 = -a2 * a3 * m3 * std::sin(q3);
    float C2_Q2Q3 = -2.0f * a2 * a3 * m3 * std::sin(q3);
    float C2_Q3Q3 = -a2 * a3 * m3 * std::sin(q3);
    float M2_Q3 = a2 * a3 * m3 * std::cos(q3);
    float M2_Q2 = (a2 * (a2 * m2 + a2 * m3 + a3 * m3 * std::cos(q3)) + Izz2);

    // 计算tau2
    float tao2 = M2_Q2 * qdd2 + M2_Q3 * qdd3 + C2_Q1Q1 * qd1 * qd1 + C2_Q2Q2 * qd2 * qd2 + C2_Q2Q3 * qd2 * qd3 + C2_Q3Q3 * qd3 * qd3 + G2;

    // 计算M1_Q1, C1相关项
    float M1_Q1 = 0.500f * (a3 * a3 * m3 + a2 * a2 * (m2 + m3) + a2 * a2 * (m2 + m3) * std::cos(2.0f * q2) + a3 * m3 * (4.0f * a2 * std::cos(q2) * std::cos(q2 + q3) + a3 * std::cos(2.0f * (q2 + q3)))
          + 2.0f * std::sin(q2) * std::sin(q2) * Iyy2 + 2.0f * std::sin(q2 + q3) * std::sin(q2 + q3) * Iyy3 + 2.0f * Iyy2 + 2.0f * std::cos(q2) * std::cos(q2) * Iyy2 + 2.0f * std::cos(q2 + q3) * std::cos(q2 + q3) * Iyy3);

    float C1_Q1Q3 = -(2.0f * std::sin(q2 + q3) * (a3 * m3 * (a2 * std::cos(q2) + a3 * std::cos(q2 + q3)) + std::cos(q2 + q3) * (-Iyy3 + Iyy3)));
    float C1_Q1Q2 = -(a2 * a2 * (m2 + m3) * std::sin(2.0f * q2) + a3 * m3 * (a3 * std::sin(2.0f * (q2 + q3)) + 2.0f * a2 * std::sin(2.0f * q2 + q3)) + std::sin(2.0f * q2) * (-Iyy2 + Iyy2) + std::sin(2.0f * (q2 + q3)) * (-Iyy3 + Iyy3));

    // 计算tau1
    float tao1 = M1_Q1 * qdd1 + C1_Q1Q3 * qd1 * qd3 + C1_Q1Q2 * qd1 * qd2;

    return Vector3f(tao1, tao2, tao3);
}

Vector3f RobotModel::computeGravityCompensation(const Vector3f& q) const
{
    // 简化的重力补偿计算
    const float a2 = params_.dh[1].a;
    const float a3 = params_.dh[2].a;
    const float m2 = params_.m[1];
    const float m3 = params_.m[2];

    float q2 = q[1], q3 = q[2];

    float G3 = a3 * m3 * std::cos(q2 + q3) * 9.81f;
    float G2 = G3 + a2 * std::cos(q2) * (9.81f * m2 + 9.81f * m3);
    float G1 = 0.0f;  // 关节1不受重力影响（假设）

    return Vector3f(G1, G2, G3);
}

// ==================== 控制算法 ====================

Vector3f RobotModel::computePDControl(const Vector3f& q_desired,
                                     const Vector3f& qd_desired,
                                     const Vector3f& q_current,
                                     const Vector3f& qd_current,
                                     const Matrix3f& Kp,
                                     const Matrix3f& Kd) const
{
    Vector3f error = q_desired - q_current;
    Vector3f error_dot = qd_desired - qd_current;

    return Kp * error + Kd * error_dot;
}

Vector3f RobotModel::computeTorqueControl(const Vector3f& q_desired,
                                         const Vector3f& qd_desired,
                                         const Vector3f& qdd_desired,
                                         const Vector3f& q_current,
                                         const Vector3f& qd_current,
                                         const Matrix3f& Kp,
                                         const Matrix3f& Kd) const
{
    // 计算力矩控制: τ = M(q)(qdd_desired + Kp(q_desired-q) + Kd(qd_desired-qd)) + C(q,qd)qd + G(q)
    Vector3f error = q_desired - q_current;
    Vector3f error_dot = qd_desired - qd_current;

    Vector3f qdd_command = qdd_desired + Kp * error + Kd * error_dot;

    return computeInverseDynamics(q_current, qd_current, qdd_command);
}

Vector3f RobotModel::computeSlidingModeControl(const Vector3f& q_desired,
                                              const Vector3f& qd_desired,
                                              const Vector3f& q_current,
                                              const Vector3f& qd_current,
                                              const Vector3f& K,
                                              const Vector3f& eta) const
{
    // 从C#代码移植的滑模控制
    Vector3f error = q_desired - q_current;
    Vector3f error_dot = qd_desired - qd_current;

    Vector3f s = error_dot + K.cwiseProduct(error);

    // 使用tanh函数代替符号函数，更平滑
    Vector3f Yd;
    Yd[0] = (54.0f + eta[0]) * std::tanh(s[0] / 3.0f);
    Yd[1] = (50.0f + eta[1]) * std::tanh(s[1]);
    Yd[2] = (100.0f + eta[2]) * (s[2] > 0 ? 1.0f : -1.0f);  // sign函数

    Vector3f V = qd_desired + K.cwiseProduct(error_dot) + Yd;

    // 这里应该计算力矩，但暂时返回V作为占位符
    return V;
}

Vector3f RobotModel::clampTorque(const Vector3f& torque, const std::vector<float>& limits) const
{
    Vector3f clamped = torque;

    if (limits.size() >= 6) {
        // 关节1限制
        if (clamped[0] > limits[1]) clamped[0] = limits[1];
        if (clamped[0] < limits[0]) clamped[0] = limits[0];

        // 关节2限制 (根据C#代码: -1.5 ~ 2.0)
        if (clamped[1] > limits[3]) clamped[1] = limits[3];
        if (clamped[1] < limits[2]) clamped[1] = limits[2];

        // 关节3限制
        if (clamped[2] > limits[5]) clamped[2] = limits[5];
        if (clamped[2] < limits[4]) clamped[2] = limits[4];
    }

    return clamped;
}

// ==================== 私有辅助函数 ====================

// 这里实现各种辅助函数...
// 由于时间关系，暂时留空，后续根据需要实现