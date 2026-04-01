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

void RobotModel::CalculateSList()
{
   // 关节1的z轴在基座的z轴，通过原点
     Vector3d omega1({0, 0, 1});
     Vector3d q_point1({0, 0, 0}); // 关节1的z轴通过原点
     Vector3d v1 = -omega1.cross(q_point1); // v = -omega x q_point
    
    S1.segment(0, 3) = omega1;
    S1.segment(3, 3) = v1;

    // 关节2的z轴在关节1的x-y平面内，随q1旋转
    // R0_1_zero 是q1=0时的旋转矩阵，即I
    // R0_1_zero * [0; 0; 1] = [0; 0; 1]
    // 但MATLAB代码中是绕-Y轴，所以omega2 = [0; -1; 0]
     Vector3d omega2({0, -1, 0}); // 绕-Y轴
     Vector3d q_point2({0, 0, 0}); // 关节2的z轴通过原点 (q1=0时)
     Vector3d v2 = -omega2.cross(q_point2);
  
    S2.segment(0, 3) = omega2;
    S2.segment(3, 3) = v2;

    // 关节3的z轴也绕-Y轴，但作用点在关节2的X方向上，距离为l2
     Vector3d omega3({0, -1, 0}); // 绕-Y轴
    double l2 = params_.dh[1].a; // 从参数中获取
     Vector3d q_point3({l2, 0, 0}); // 关节3的z轴通过(l2, 0, 0)点 (q1=q2=0时)
     Vector3d v3 = -omega3.cross(q_point3); // v = -omega x q_point = -[-y, x, 0] = [y, -x, 0] = [0, l2, 0]

    S3.segment(0, 3) = omega3;
    S3.segment(3, 3) = v3;
}
// ==================== 运动学计算 ====================

bool RobotModel::forwardKinematics(const Vector3f& q, Vector3f& position) const
{
   // 将输入的关节角转换为double
    VectorXd q_d = q.cast<double>();
    double q1 = q_d(0);
    double q2 = q_d(1);
    double q3 = q_d(2);

    // --- 计算指数映射 ---
     Matrix4d T1 = expm_screw(S1, q1);
     Matrix4d T2 = expm_screw(S2, q2);
     Matrix4d T3 = expm_screw(S3, q3);

    // --- 使用预计算的零位位姿 ---
     Matrix4d T_total = T1 * T2 * T3 * zero_config_pose_M_;

    // --- 提取位置 ---
    position = T_total.block<3, 1>(0, 3).cast<float>();

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
    // 将输入关节角转换为double
     Vector3d q_d = q.cast<double>();
    double q1 = q_d(0);
    double q2 = q_d(1);
    double q3 = q_d(2);
    
    // 计算指数映射
     Matrix4d T1 = expm_screw(S1, q1);
     Matrix4d T2 = expm_screw(S2, q2);
     Matrix4d T3 = expm_screw(S3, q3);
    
    // 计算末端位姿
     Matrix4d T_total = T1 * T2 * T3 * zero_config_pose_M_;
    
    // 提取末端位置p3_exp
     Vector3d p3_exp = T_total.block<3, 1>(0, 3);
    
    // 计算空间雅可比Js
    Eigen::Matrix<double, 6, 3> J_s;
    
    // 第一列: Js1 = S1
    J_s.col(0) = S1;
    
    // 第二列: Js2 = adjoint(T1) * S2
    // 需要先实现adjoint函数
    Matrix6d Ad_T1 = adjoint(T1);
    J_s.col(1) = Ad_T1 * S2;
    
    // 第三列: Js3 = adjoint(T1*T2) * S3
     Matrix4d T12 = T1 * T2;
    Matrix6d Ad_T12 = adjoint(T12);
    J_s.col(2) = Ad_T12 * S3;
    
    // 分离角速度和线速度部分
     Matrix3d J_s_angular = J_s.block<3, 3>(0, 0);  // 角速度部分
     Matrix3d J_s_linear = J_s.block<3, 3>(3, 0);   // 线速度部分
    
    // 计算几何雅可比: jacobian = -skew(p3_exp) * J_s(1:3,:) + J_s(4:6,:)
     Matrix3d p_hat = skew(p3_exp);
     Matrix3d J_geometric = J_s_linear - p_hat * J_s_angular;
    
    // 转换为float返回
    return J_geometric.cast<float>();
}

//=========================实现逆速度计算========================
//自行实现的基于史密斯正交化的QR分解方法，适用于3x3雅可比矩阵
bool RobotModel::inverseVelocity(const Matrix3f& J, const Vector3f& end_vel, Vector3f& qd) const
{
    // 检查雅可比矩阵是否有效
    if (J.hasNaN() || J.maxCoeff() > 1e6f || J.minCoeff() < -1e6f) {
        return false;
    }
    
    // 对雅可比矩阵进行Gram-Schmidt正交化（QR分解的手工实现）
    // 提取雅可比的列向量
    Eigen::Vector3f J1 = J.col(0);
    Eigen::Vector3f J2 = J.col(1);
    Eigen::Vector3f J3 = J.col(2);
    
    // 计算正交基e1, e2, e3
    Eigen::Vector3f e1, e2, e3;
    
    // 计算e1
    float norm_J1 = J1.norm();
    if (norm_J1 < 1e-6f) {
        return false; // 雅可比矩阵奇异
    }
    e1 = J1 / norm_J1;
    
    // 计算e2
    Eigen::Vector3f e2_raw = J2 - (J2.dot(e1)) * e1;
    float norm_e2_raw = e2_raw.norm();
    if (norm_e2_raw < 1e-6f) {
        return false; // 雅可比矩阵奇异
    }
    e2 = e2_raw / norm_e2_raw;
    
    // 计算e3
    Eigen::Vector3f e3_raw = J3 - (J3.dot(e1)) * e1 - (J3.dot(e2)) * e2;
    float norm_e3_raw = e3_raw.norm();
    if (norm_e3_raw < 1e-6f) {
        return false; // 雅可比矩阵奇异
    }
    e3 = e3_raw / norm_e3_raw;
    
    // 构造正交矩阵Q
    Eigen::Matrix3f Q_M;
    Q_M.col(0) = e1;
    Q_M.col(1) = e2;
    Q_M.col(2) = e3;
    
    // 计算上三角矩阵R = Q^T * J
    Eigen::Matrix3f R_M = Q_M.transpose() * J;
    
    // 检查R_M的对角元素（避免除零）
    if (std::abs(R_M(0, 0)) < 1e-6f || 
        std::abs(R_M(1, 1)) < 1e-6f || 
        std::abs(R_M(2, 2)) < 1e-6f) {
        return false;
    }
    
    // 解上三角线性方程组 R * qd = Q^T * v
    Eigen::Vector3f y = Q_M.transpose() * end_vel;
    
    // 回代求解
    qd[2] = y[2] / R_M(2, 2);
    qd[1] = (y[1] - R_M(1, 2) * qd[2]) / R_M(1, 1);
    qd[0] = (y[0] - R_M(0, 1) * qd[1] - R_M(0, 2) * qd[2]) / R_M(0, 0);
    
    return true;
}
// 使用Eigen内置的QR分解方法，适用于3x3雅可比矩阵
bool RobotModel::inverseVelocityQR(const Matrix3f& J, const Vector3f& end_vel, Vector3f& qd) const
{
    // 检查雅可比矩阵是否有效
    if (J.hasNaN() || J.maxCoeff() > 1e6f || J.minCoeff() < -1e6f) {
        return false;
    }
    
    // 使用Eigen内置的ColPivHouseholderQR分解
    // 这种分解可以处理奇异矩阵，并提供最小二乘解
    Eigen::ColPivHouseholderQR<Matrix3f> qr(J);
    
    if (!qr.isInvertible()) {
        // 雅可比矩阵奇异，无法求解
        return false;
    }
    
    // 求解 J * qd = end_vel
    qd = qr.solve(end_vel);
    
    return true;
}
// 使用阻尼最小二乘法的逆速度计算，适用于接近奇异的雅可比矩阵
bool RobotModel::inverseVelocityDamped(const Matrix3f& J, const Vector3f& end_vel, Vector3f& qd, float lambda) const
{
    // 检查雅可比矩阵是否有效
    if (J.hasNaN() || J.maxCoeff() > 1e6f || J.minCoeff() < -1e6f) {
        return false;
    }
    
    // 使用阻尼最小二乘法 (DLS)
    // 解: qd = (J^T * J + λ^2 * I)^(-1) * J^T * v
    Matrix3f I = Matrix3f::Identity();
    Matrix3f A = J.transpose() * J + lambda * lambda * I;
    
    // 检查A是否可逆
    float det = A.determinant();
    if (std::abs(det) < 1e-12f) {
        return false;
    }
    
    qd = A.inverse() * J.transpose() * end_vel;
    
    return true;
}

//正向计算连杆末端速度，输入关节角和关节速度，输出末端线速度
Vector3f RobotModel::forwardVelocity(const Vector3f& q, const Vector3f& qd) const
{
    // 将关节角转换为double
    Eigen::Vector3d q_d = q.cast<double>();
    Eigen::Vector3d qd_d = qd.cast<double>();
    
    double q1 = q_d[0], q2 = q_d[1], q3 = q_d[2];
    double qd1 = qd_d[0], qd2 = qd_d[1], qd3 = qd_d[2];
    
    const double l2 = params_.dh[1].a;
    const double l3 = params_.dh[2].a;
    
    // 1. 根据DH参数构建变换矩阵
    Eigen::Matrix4d T0_1, T1_2, T2_3;
    
    // T0_1
    double c1 = cos(q1), s1 = sin(q1);
    T0_1 << c1,  0,  s1,  0,
            s1,  0, -c1,  0,
            0,   1,  0,   0,
            0,   0,  0,   1;
    
    // T1_2
    double c2 = cos(q2), s2 = sin(q2);
    T1_2 << c2, -s2,  0,  l2 * c2,
            s2,  c2,  0,  l2 * s2,
            0,   0,   1,  0,
            0,   0,   0,  1;
    
    // T2_3
    double c3 = cos(q3), s3 = sin(q3);
    T2_3 << c3, -s3,  0,  l3 * c3,
            s3,  c3,  0,  l3 * s3,
            0,   0,   1,  0,
            0,   0,   0,  1;
    
    // 2. 计算逆变换矩阵
    Eigen::Matrix4d T0_1_inv = T0_1.inverse();
    Eigen::Matrix4d T1_2_inv = T1_2.inverse();
    Eigen::Matrix4d T2_3_inv = T2_3.inverse();
    
    // 3. 计算伴随变换 adjoint(inv_transform(T))
    Eigen::Matrix<double, 6, 6> Ad_T0_1_inv = adjoint(T0_1_inv);
    Eigen::Matrix<double, 6, 6> Ad_T1_2_inv = adjoint(T1_2_inv);
    Eigen::Matrix<double, 6, 6> Ad_T2_3_inv = adjoint(T2_3_inv);
    
    // 4. 计算零位变换矩阵（根据你提供的MATLAB代码）
    Eigen::Matrix4d M1, M2, M3;
    
    // M1 = T0_1_zero
    M1 << 1, 0, 0, 0,
          0, 0, -1, 0,
          0, 1, 0, 0,
          0, 0, 0, 1;
    
    // M2 = T0_1_zero * T1_2_zero
    Eigen::Matrix4d T1_2_zero;
    T1_2_zero << 1, 0, 0, l2,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
    M2 = M1 * T1_2_zero;
    
    // M3 = T0_1_zero * T1_2_zero * T2_3_zero
    Eigen::Matrix4d T2_3_zero;
    T2_3_zero << 1, 0, 0, l3,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
    M3 = M2 * T2_3_zero;
    
    // 5. 计算空间旋量 A_i = adjoint(inv(M_i)) * S_i
    Eigen::Matrix<double, 6, 6> Ad_M1_inv = adjoint(M1.inverse());
    Eigen::Matrix<double, 6, 6> Ad_M2_inv = adjoint(M2.inverse());
    Eigen::Matrix<double, 6, 6> Ad_M3_inv = adjoint(M3.inverse());
    
    Eigen::VectorXd A1 = Ad_M1_inv * S1;
    Eigen::VectorXd A2 = Ad_M2_inv * S2;
    Eigen::VectorXd A3 = Ad_M3_inv * S3;
    
    // 6. 速度递推
    Eigen::VectorXd V0 = Eigen::VectorXd::Zero(6);  // 基座速度为零
    
    // V1 = A1 * qd1 + adjoint(inv(T0_1)) * V0
    Eigen::VectorXd V1 = A1 * qd1 + Ad_T0_1_inv * V0;
    
    // V2 = A2 * qd2 + adjoint(inv(T1_2)) * V1
    Eigen::VectorXd V2 = A2 * qd2 + Ad_T1_2_inv * V1;
    
    // V3 = A3 * qd3 + adjoint(inv(T2_3)) * V2
    Eigen::VectorXd V3 = A3 * qd3 + Ad_T2_3_inv * V2;
    
    // 7. 提取末端线速度（旋量的后3个元素）
    Eigen::Vector3d end_vel = V3.segment(3, 3);
    
    return end_vel.cast<float>();
}


Matrix3f RobotModel::computeJacobianDerivative(const Vector3f& q, const Vector3f& qd) const
{
// 转换为double精度
    Eigen::Vector3d q_d = q.cast<double>();
    Eigen::Vector3d qd_d = qd.cast<double>();
    
    double q1 = q_d[0], q2 = q_d[1], q3 = q_d[2];
    double qd1 = qd_d[0], qd2 = qd_d[1], qd3 = qd_d[2];
    
    // 1. 计算变换矩阵
    Eigen::Matrix4d T1 = expm_screw(S1, q1);
    Eigen::Matrix4d T2 = expm_screw(S2, q2);
    
    // 2. 计算空间雅可比 J_s
    Eigen::Matrix<double, 6, 3> J_s;
    J_s.col(0) = S1;
    J_s.col(1) = adjoint(T1) * S2;
    J_s.col(2) = adjoint(T1 * T2) * S3;
    
    // 3. 计算空间雅可比导数 dJ_s
    Eigen::Matrix<double, 6, 3> dJ_s = Eigen::Matrix<double, 6, 3>::Zero();
    
    // 计算空间雅可比导数（李括号法）
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < i; ++j) {
            Eigen::VectorXd J_s_j = J_s.col(j);
            Eigen::VectorXd J_s_i = J_s.col(i);
            
            // dJ_s(:,i) = dJ_s(:,i) + ad(J_s(:,j)) * J_s(:,i) * qd(j)
            Eigen::Matrix<double, 6, 6> ad_J_s_j = ad(J_s_j);
            dJ_s.col(i) += ad_J_s_j * J_s_i * qd_d[j];
        }
    }
    
    // 4. 计算末端位置
    Eigen::Matrix4d T3 = expm_screw(S3, q3);
    Eigen::Matrix4d T_total = T1 * T2 * T3 * zero_config_pose_M_;
    Eigen::Vector3d p_tcp = T_total.block<3, 1>(0, 3);
    
    // 5. 计算末端速度 v_tcp = J_geo * qd
    Eigen::Matrix3d J_s_angular = J_s.block<3, 3>(0, 0);
    Eigen::Matrix3d J_s_linear = J_s.block<3, 3>(3, 0);
    
    // 几何雅可比: J_geo = J_s_v - p_hat * J_s_omega
    Eigen::Matrix3d p_hat = skew(p_tcp);
    Eigen::Matrix3d J_geo = J_s_linear - p_hat * J_s_angular;
    
    Eigen::Vector3d v_tcp = J_geo * qd_d;
    
    // 6. 分离dJ_s的角速度和线速度部分
    Eigen::Matrix3d dJ_s_angular = dJ_s.block<3, 3>(0, 0);
    Eigen::Matrix3d dJ_s_linear = dJ_s.block<3, 3>(3, 0);
    
    // 7. 计算几何雅可比导数
    // dJ_geo = dJ_s_v - skew(v_tcp) * J_s_omega - skew(p_tcp) * dJ_s_omega
    Eigen::Matrix3d v_hat = skew(v_tcp);
    
    Eigen::Matrix3d dJ_geo = dJ_s_linear
                           - v_hat * J_s_angular
                           - p_hat * dJ_s_angular;
    
    return dJ_geo.cast<float>();
}


bool RobotModel::inverseAcceleration(const Vector3f& q, const Vector3f& qd, 
                                     const Vector3f& end_acc, Vector3f& qdd) const
{
    // 1. 计算当前关节角下的雅可比矩阵
    Matrix3f J = computeJacobian(q);
    
    // 2. 计算雅可比导数
    Matrix3f dJ = computeJacobianDerivative(q, qd);
    
    // 3. 对雅可比矩阵进行QR分解
    // 提取雅可比的列向量
    Eigen::Vector3f J1 = J.col(0);
    Eigen::Vector3f J2 = J.col(1);
    Eigen::Vector3f J3 = J.col(2);
    
    // 计算正交基e1, e2, e3
    Eigen::Vector3f e1, e2, e3;
    
    // 计算e1
    float norm_J1 = J1.norm();
    if (norm_J1 < 1e-6f) {
        return false; // 雅可比矩阵奇异
    }
    e1 = J1 / norm_J1;
    
    // 计算e2
    Eigen::Vector3f e2_raw = J2 - (J2.dot(e1)) * e1;
    float norm_e2_raw = e2_raw.norm();
    if (norm_e2_raw < 1e-6f) {
        return false; // 雅可比矩阵奇异
    }
    e2 = e2_raw / norm_e2_raw;
    
    // 计算e3
    Eigen::Vector3f e3_raw = J3 - (J3.dot(e1)) * e1 - (J3.dot(e2)) * e2;
    float norm_e3_raw = e3_raw.norm();
    if (norm_e3_raw < 1e-6f) {
        return false; // 雅可比矩阵奇异
    }
    e3 = e3_raw / norm_e3_raw;
    
    // 4. 构造正交矩阵Q
    Matrix3f Q_M;
    Q_M.col(0) = e1;
    Q_M.col(1) = e2;
    Q_M.col(2) = e3;
    
    // 5. 计算上三角矩阵R = Q^T * J
    Matrix3f R_M = Q_M.transpose() * J;
    
    // 检查R_M是否奇异
    if (std::abs(R_M(0, 0)) < 1e-6f || 
        std::abs(R_M(1, 1)) < 1e-6f || 
        std::abs(R_M(2, 2)) < 1e-6f) {
        return false;
    }
    
    // 6. 计算右端项: xdd = end_acc - dJ * qd
    Vector3f xdd = end_acc - dJ * qd;
    
    // 7. 变换右端项: y_acc = Q^T * xdd
    Vector3f y_acc = Q_M.transpose() * xdd;
    
    // 8. 回代求解上三角方程组 R * qdd = y_acc
    qdd[2] = y_acc[2] / R_M(2, 2);
    qdd[1] = (y_acc[1] - R_M(1, 2) * qdd[2]) / R_M(1, 1);
    qdd[0] = (y_acc[0] - R_M(0, 1) * qdd[1] - R_M(0, 2) * qdd[2]) / R_M(0, 0);
    
    return true;
}

bool RobotModel::inverseAccelerationQR(const Vector3f& q, const Vector3f& qd, 
                                       const Vector3f& end_acc, Vector3f& qdd) const
{
    // 1. 计算雅可比和雅可比导数
    Matrix3f J = computeJacobian(q);
    Matrix3f dJ = computeJacobianDerivative(q, qd);
    
    // 2. 使用Eigen的QR分解
    Eigen::ColPivHouseholderQR<Matrix3f> qr(J);
    
    if (!qr.isInvertible()) {
        return false; // 雅可比矩阵奇异
    }
    
    // 3. 计算右端项: xdd = end_acc - dJ * qd
    Vector3f xdd = end_acc - dJ * qd;
    
    // 4. 求解线性方程组 J * qdd = xdd
    qdd = qr.solve(xdd);
    
    return true;
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


Matrix4d RobotModel::dh_transform(double a, double alpha, double d, double theta) {
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);

    Matrix4d T;
    T << ct, -st * ca, st * sa, a * ct,
         st, ct * ca, -ct * sa, a * st,
         0, sa, ca, d,
         0, 0, 0, 1;
    return T;
}

void RobotModel::calculateZeroConfigPoseM()
{
    // 从params_获取DH参数，并设置关节角为0
    double a1 = params_.dh[0].a; double alpha1 = params_.dh[0].alpha; double d1 = params_.dh[0].d; double theta1 = 0;
    double a2 = params_.dh[1].a; double alpha2 = params_.dh[1].alpha; double d2 = params_.dh[1].d; double theta2 = 0;
    double a3 = params_.dh[2].a; double alpha3 = params_.dh[2].alpha; double d3 = params_.dh[2].d; double theta3 = 0;

    // 计算零位时的变换矩阵
     Matrix4d T0_1 = dh_transform(a1, alpha1, d1, theta1);
     Matrix4d T1_2 = dh_transform(a2, alpha2, d2, theta2);
     Matrix4d T2_3 = dh_transform(a3, alpha3, d3, theta3);

    // 零位位姿 = T0_1 * T1_2 * T2_3
    zero_config_pose_M_ = T0_1 * T1_2 * T2_3;
}

 Matrix3d RobotModel::skew(const  Vector3d& w) const {
     Matrix3d wx;
    wx << 0, -w(2), w(1),
          w(2), 0, -w(0),
          -w(1), w(0), 0;
    return wx;
}

// 辅助函数：计算螺旋轴的指数映射
 Matrix4d RobotModel::expm_screw(const  VectorXd& S, double theta) const {
    // 分离角速度和线速度部分
     Vector3d w = S.segment(0, 3);
     Vector3d v = S.segment(3, 3);

     Matrix4d T;
    T.setIdentity(); // 初始化为单位矩阵

    double w_norm = w.norm();

    if (w_norm < 1e-10) {
        // 特殊情况：纯平移 (w接近零向量)
        T.topRightCorner(3, 1) = v * theta;
    } else {
        // 一般情况：旋转和平移
         Matrix3d wx = skew(w);
         Matrix3d I =  Matrix3d::Identity();
        
        // 计算旋转矩阵 R
         Matrix3d R = I + std::sin(theta) * wx + (1 - std::cos(theta)) * wx * wx;
        
        // 计算平移向量 p
         Matrix3d term_in_parentheses = I * theta + (1 - std::cos(theta)) * wx + (theta - std::sin(theta)) * wx * wx;
         Vector3d p = term_in_parentheses * v;

        T.topLeftCorner(3, 3) = R;
        T.topRightCorner(3, 1) = p;
    }
    return T;
}


Matrix6d RobotModel::adjoint(const  Matrix4d& T) const
{
    Matrix6d AdT;
    
    // 提取旋转矩阵R和平移向量p
     Matrix3d R = T.block<3, 3>(0, 0);
     Vector3d p = T.block<3, 1>(0, 3);
    
    // 计算p的斜对称矩阵
     Matrix3d p_hat = skew(p);
    
    // 构建伴随变换矩阵
    // 左上角: R
    AdT.block<3, 3>(0, 0) = R;
    // 右上角: 0
    AdT.block<3, 3>(0, 3) =  Matrix3d::Zero();
    // 左下角: p_hat * R
    AdT.block<3, 3>(3, 0) = p_hat * R;
    // 右下角: R
    AdT.block<3, 3>(3, 3) = R;
    
    return AdT;
}


Eigen::Matrix<double, 6, 6> RobotModel::ad(const Eigen::VectorXd& A) const
{
    // 检查输入维度
    if (A.size() != 6) {
        throw std::invalid_argument("ad: 输入向量必须是6维");
    }
    
    // 提取角速度和线速度部分
    Eigen::Vector3d w = A.segment(0, 3);
    Eigen::Vector3d v = A.segment(3, 3);
    
    // 计算斜对称矩阵
    Eigen::Matrix3d w_hat = skew(w);
    Eigen::Matrix3d v_hat = skew(v);
    
    // 构建伴随矩阵
    Eigen::Matrix<double, 6, 6> adA;
    adA.block<3, 3>(0, 0) = w_hat;
    adA.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    adA.block<3, 3>(3, 0) = v_hat;
    adA.block<3, 3>(3, 3) = w_hat;
    
    return adA;
}