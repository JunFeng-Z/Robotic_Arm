#include "robot_model.h"
#include <cmath>

RobotModel::RobotModel(const RobotParams& params)
    : params_(params),
      S_list_(3, Eigen::VectorXd(6)),
      A_list_(3, Eigen::VectorXd(6)),
      V_list_(4, Eigen::VectorXd::Zero(6)),
      Vd_list_(4, Eigen::VectorXd::Zero(6)),
      F_list_(4, Eigen::VectorXd::Zero(6)),
      G_list_(3, Matrix6d::Zero()),
      tau_(Eigen::Vector3f::Zero()),
      S1(S_list_[0]),S2(S_list_[1]),S3(S_list_[2]),
      A1(A_list_[0]),A2(A_list_[1]),A3(A_list_[2]),
      V0(V_list_[0]),V1(V_list_[1]),V2(V_list_[2]),V3(V_list_[3]),
      Vd0(Vd_list_[0]),Vd1(Vd_list_[1]),Vd2(Vd_list_[2]),Vd3(Vd_list_[3]),
      F1(F_list_[0]),F2(F_list_[1]),F3(F_list_[2]),F4(F_list_[3]),
      T0_1_(Eigen::Matrix4d::Identity()),T1_2_(Eigen::Matrix4d::Identity()),T2_3_(Eigen::Matrix4d::Identity()),T3_4_(Eigen::Matrix4d::Identity()),
      Ad_T0_1_inv_(Matrix6d::Zero()),Ad_T1_2_inv_(Matrix6d::Zero()),Ad_T2_3_inv_(Matrix6d::Zero()),Ad_T3_4_inv_(Matrix6d::Zero()),
      M1_(Eigen::Matrix4d::Identity()),M2_(Eigen::Matrix4d::Identity()),M3_(Eigen::Matrix4d::Identity()),
      M1_inv_(Eigen::Matrix4d::Identity()),M2_inv_(Eigen::Matrix4d::Identity()),M3_inv_(Eigen::Matrix4d::Identity()),
      Ad_M1_inv_(Matrix6d::Zero()),Ad_M2_inv_(Matrix6d::Zero()),Ad_M3_inv_(Matrix6d::Zero()) 
{
    Initialize();
}
void RobotModel::Initialize() {
    // 确保所有向量正确初始化
    for (auto& v : S_list_) {
        v = Eigen::VectorXd(6);
    }
    
    for (auto& a : A_list_) {
        a = Eigen::VectorXd(6);
    }
    
    for (auto& v : V_list_) {
        v = Eigen::VectorXd::Zero(6);
    }
    for (auto& vd : Vd_list_) 
    {
        vd = Eigen::VectorXd::Zero(6);
    }
    
    // 计算初始的S_list_
    CalculateSList();
    calculateZeroConfigPoseM();
    computeSpatialInertias();
    Vd0.setZero();
    Vd0(5) = 9.81;
}

void RobotModel::computeTransforms(const Vector3f& q) {
    // 使用DH参数计算变换矩阵
    // 注意：DH参数中的theta是关节角度，需要加上当前关节角度q
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    // T0_1: 从基座到关节1
    T0_1_ = dh_transform(
        params_.dh[0].a,
        params_.dh[0].alpha,
        params_.dh[0].d,
        params_.dh[0].theta + q1
    );
    
    // T1_2: 从关节1到关节2
    T1_2_ = dh_transform(
        params_.dh[1].a,
        params_.dh[1].alpha,
        params_.dh[1].d,
        params_.dh[1].theta + q2
    );
    
    // T2_3: 从关节2到关节3
    T2_3_ = dh_transform(
        params_.dh[2].a,
        params_.dh[2].alpha,
        params_.dh[2].d,
        params_.dh[2].theta + q3
    );
    
    // T3_4: 从关节3到末端执行器（单位矩阵）
    T3_4_.setIdentity();
    
    // 更新伴随变换缓存
    updateAdjointTransforms();
    computeZeroTransforms();

}

void RobotModel::updateAdjointTransforms() {
    // 计算逆变换的伴随矩阵并缓存
    
    // T0_1的逆伴随
    Eigen::Matrix4d T0_1_inv = T0_1_.inverse();
    Ad_T0_1_inv_ = adjoint(T0_1_inv);
    
    // T1_2的逆伴随
    Eigen::Matrix4d T1_2_inv = T1_2_.inverse();
    Ad_T1_2_inv_ = adjoint(T1_2_inv);
    
    // T2_3的逆伴随
    Eigen::Matrix4d T2_3_inv = T2_3_.inverse();
    Ad_T2_3_inv_ = adjoint(T2_3_inv);
    
    // T3_4的逆伴随（单位矩阵的逆还是单位矩阵）
    // 实际上不需要计算，但为了完整性保留
    Ad_T3_4_inv_ = adjoint(T3_4_.inverse());  // 或者直接设置为单位矩阵
}


void RobotModel::computeZeroTransforms() 
{
    // 方法1：使用DH函数计算（推荐）
    // 注意：关节在零位时，关节角度q=0，所以使用DH参数中的theta
    
    // 关节1在零位的变换
    M1_ = dh_transform(params_.dh[0].a, params_.dh[0].alpha, 
                       params_.dh[0].d, params_.dh[0].theta);
    
    // 关节2在零位的变换
    Eigen::Matrix4d T1_2_zero = dh_transform(params_.dh[1].a, params_.dh[1].alpha, 
                                            params_.dh[1].d, params_.dh[1].theta);
    
    // 关节3在零位的变换
    Eigen::Matrix4d T2_3_zero = dh_transform(params_.dh[2].a, params_.dh[2].alpha, 
                                            params_.dh[2].d, params_.dh[2].theta);
    
    // 累积变换
    M2_ = M1_ * T1_2_zero;
    M3_ = M2_ * T2_3_zero;


        // 在 computeTransforms 之后
    // std::cout << "M2 X-axis: " << M2_.block<3,1>(0,0).transpose() << std::endl;
    // std::cout << "M2 origin: " << M2_.block<3,1>(0,3).transpose() << std::endl;
    

    
    // 计算逆变换和伴随变换缓存
    M1_inv_ = M1_.inverse();
    M2_inv_ = M2_.inverse();
    M3_inv_ = M3_.inverse();
    
    Ad_M1_inv_ = adjoint(M1_inv_);
    Ad_M2_inv_ = adjoint(M2_inv_);
    Ad_M3_inv_ = adjoint(M3_inv_);
}
void RobotModel::setParameters(const RobotParams& params)
{
    params_ = params;
    CalculateSList();
    calculateZeroConfigPoseM();
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

bool RobotModel::forwardKinematics(const Vector3f& q, Vector3f& position)
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

bool RobotModel::inverseKinematics(const Vector3f& position, Vector3f& q, int elbow)
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

Matrix3f RobotModel::computeJacobian(const Vector3f& q)
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
bool RobotModel::inverseVelocity(const Matrix3f& J, const Vector3f& end_vel, Vector3f& qd) 
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
bool RobotModel::inverseVelocityQR(const Matrix3f& J, const Vector3f& end_vel, Vector3f& qd) 
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
bool RobotModel::inverseVelocityDamped(const Matrix3f& J, const Vector3f& end_vel, Vector3f& qd, float lambda) 
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
Vector3f RobotModel::forwardVelocity(const Vector3f& q, const Vector3f& qd) 
{
    this->computeTransforms(q); // 计算当前关节角对应的变换矩阵
    // 5. 计算空间旋量 A_i = adjoint(inv(M_i)) * S_i
    A1 = Ad_M1_inv_ * S1;
    A2 = Ad_M2_inv_ * S2;
    A3 = Ad_M3_inv_ * S3;
    
    // 6. 速度递推
    V0 = Eigen::VectorXd::Zero(6);  // 基座速度为零
    
    // V1 = A1 * qd1 + adjoint(inv(T0_1)) * V0
    V1 = A1 * qd(0) + Ad_T0_1_inv_ * V0;
    
    // V2 = A2 * qd2 + adjoint(inv(T1_2)) * V1
    V2 = A2 * qd(1) + Ad_T1_2_inv_ * V1;
    
    // V3 = A3 * qd3 + adjoint(inv(T2_3)) * V2
    V3 = A3 * qd(2) + Ad_T2_3_inv_ * V2;
    
    // 7. 提取末端线速度（旋量的后3个元素）
    Eigen::Vector3d end_vel = V3.segment(3, 3);
    if(flag_){
        std::cout<<"V0: "<<V0.transpose()<<std::endl;
        std::cout<<"V1: "<<V1.transpose()<<std::endl;
        std::cout<<"V2: "<<V2.transpose()<<std::endl;
        std::cout<<"V3: "<<V3.transpose()<<std::endl;
    }
    
    return end_vel.cast<float>();
}


Matrix3f RobotModel::computeJacobianDerivative(const Vector3f& q, const Vector3f& qd) 
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
                                     const Vector3f& end_acc, Vector3f& qdd) 
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
                                       const Vector3f& end_acc, Vector3f& qdd) 
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




void RobotModel::computeSpatialAccelerations(const Vector3f& q, const Vector3f& qd, const Vector3f& qdd) {
    // 检查是否已经计算了速度和变换矩阵
    if (V1.size() == 0 || A1.size() == 0) {
        throw std::runtime_error("Must call forwardKinematics and computeSpatialVelocities first!");
    }
    // 1. 计算基座加速度 Vd0（已在Initialize中设置）

    
    // 2. 计算关节1的加速度
    // Vd1 = adjoint(inv_transform(T0_1)) * Vd0 + ad(V1) * A1 * qd1 + A1 * qdd1

    Matrix6d ad_V1 = ad(V1);
    Vd1 = Ad_T0_1_inv_ * Vd0 + ad_V1 * A1 * qd(0) + A1 * qdd(0);
    
    // 3. 计算关节2的加速度
    // Vd2 = adjoint(inv_transform(T1_2)) * Vd1 + ad(V2) * A2 * qd2 + A2 * qdd2
    Matrix6d ad_V2 = ad(V2);
    Vd2 = Ad_T1_2_inv_ * Vd1 + ad_V2 * A2 * qd(1) + A2 * qdd(1);
    
    // 4. 计算关节3的加速度
    // Vd3 = adjoint(inv_transform(T2_3)) * Vd2 + ad(V3) * A3 * qd3 + A3 * qdd3

    Matrix6d ad_V3 = ad(V3);
    Vd3 = Ad_T2_3_inv_ * Vd2 + ad_V3 * A3 * qd(2) + A3 * qdd(2);
    if(flag_){
        std::cout<<"Vd0: " << Vd0.transpose() << std::endl;
        std::cout<<"Vd1: " << Vd1.transpose() << std::endl;
        std::cout<<"Vd2: " << Vd2.transpose() << std::endl;
        std::cout<<"Vd3: " << Vd3.transpose() << std::endl;
    }
}


// ==================== 动力学计算 ====================

Vector3f RobotModel::computeTorqueDecoupled(const std::vector<float>& jointstate) 
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

void RobotModel::computeSpatialInertias() {
    for (int i = 0; i < 3; i++) {
        G_list_[i] = computeSpatialInertia(i);
    }
}
// Matrix6d RobotModel::computeSpatialInertia(int link_index) {
//     if (link_index < 0 || link_index >= 3) {
//         throw std::out_of_range("Link index out of range");
//     }
    
//     // 根据MATLAB代码：
//     // G = [Ic, zeros(3); zeros(3), m*eye(3)]
//     // 这是在连杆坐标系原点的空间惯量矩阵，此时默认的是连杆质心和坐标系原点重合，所以没有平行轴定理的修正项
    
//     double m = params_.m[link_index];  // 连杆质量
//     Eigen::Matrix3f Ic_f = params_.Ic[link_index];  // 连杆在质心坐标系的惯量矩阵
    
//     // 转换为double
//     Eigen::Matrix3d Ic = Ic_f.cast<double>();
    
//     Matrix6d G = Matrix6d::Zero();
    
//     // 左上角: Ic (连杆在质心坐标系的惯量矩阵)
//     G.block<3, 3>(0, 0) = Ic;
    
//     // 右上角: zeros(3)
//     G.block<3, 3>(0, 3) = Matrix3d::Zero();
    
//     // 左下角: zeros(3)
//     G.block<3, 3>(3, 0) = Matrix3d::Zero();
    
//     // 右下角: m * eye(3)
//     G.block<3, 3>(3, 3) = m * Matrix3d::Identity();
    
//     return G;
// }
//这里很奇怪，质心如果设置在连杆2的中心坐标应该为（-0.06,0,0）但此时计算出的关节2的力矩反而比设置在连杆末端要更大
//设置为-0.06才正常，问题暂未解决，先和拉格朗日统一把质心设置在连杆末端
Matrix6d RobotModel::computeSpatialInertia(int link_index)  {
    if (link_index < 0 || link_index >= 3) {
        throw std::out_of_range("Link index out of range");
    }
    
    // 空间惯量矩阵公式：
    // G = [ I    m·[rc]×^T
    //       m·[rc]×   m·I ]
    // 其中 [rc]× 是质心位置向量的反对称矩阵
    
    double m = params_.m[link_index];  // 连杆质量
    Eigen::Vector3f rc_f = params_.rc[link_index];  // 质心位置
    Eigen::Matrix3f Ic_f = params_.Ic[link_index];  // 惯量矩阵
    
    // 转换为double
    Eigen::Vector3d rc = rc_f.cast<double>();
    Eigen::Matrix3d Ic = Ic_f.cast<double>();
    
    // 计算反对称矩阵
    Eigen::Matrix3d rc_skew = skew(rc);
    
    Matrix6d G = Matrix6d::Zero();
    
    // 左上角: I = Ic + m·[rc]×^T·[rc]×
    // 根据平行轴定理: I = I_c + m([rc]×^T[rc]×)
    Eigen::Matrix3d I = Ic + m * rc_skew.transpose() * rc_skew;
    G.block<3, 3>(0, 0) = I;
    
    // 右上角: m·[rc]×^T
    G.block<3, 3>(0, 3) = m * rc_skew.transpose();
    
    // 左下角: m·[rc]×
    G.block<3, 3>(3, 0) = m * rc_skew;
    
    // 右下角: m·I
    G.block<3, 3>(3, 3) = m * Eigen::Matrix3d::Identity();
    
    return G;
}



Eigen::Vector3f RobotModel::inverseDynamics(const Eigen::Vector3f& q,
                                           const Eigen::Vector3f& qd,
                                           const Eigen::Vector3f& qdd) {
    // 1. 计算正向运动学
   // computeTransforms(q);
    // 2. 计算空间速度V
    forwardVelocity(q,qd);
    // 3. 计算空间加速度
    // 注意：需要先计算加速度才能进行逆向动力学
    // 这里假设已经有计算加速度的函数
    computeSpatialAccelerations(q,qd,qdd);
    
    // 4. 计算力旋量
    computeWrenches();
    if(flag_){
        std::cout<<"F1: " << F1.transpose() << std::endl;
        std::cout<<"F2: " << F2.transpose() << std::endl;
        std::cout<<"F3: " << F3.transpose() << std::endl;
    }
    // 5. 计算关节力矩
    // τ_i = F_i^T * A_i
    tau_[0] = F1.dot(A1);
    tau_[1] = F2.dot(A2);
    tau_[2] = F3.dot(A3);
    
    return tau_;
}

// Eigen::Vector3f RobotModel::inverseDynamicsL(const Eigen::Vector3f& q,
//                                              const Eigen::Vector3f& qd,
//                                              const Eigen::Vector3f& qdd) {
//     using Eigen::Matrix3f;
//     using Eigen::Vector3f;

//     Vector3f tau = Vector3f::Zero();

//     // 提取关节变量
//     const float q1   = q[0];
//     const float q2   = q[1];
//     const float q3   = q[2];
//     const float qd1  = qd[0];
//     const float qd2  = qd[1];
//     const float qd3  = qd[2];
//     const float qdd1 = qdd[0];
//     const float qdd2 = qdd[1];
//     const float qdd3 = qdd[2];

//     (void)q1; // 当前模型中 M/C/G 不显含 q1，但保留变量便于以后扩展

//     // 三角函数
//     const float c2   = std::cos(q2);
//     const float s2   = std::sin(q2);
//     const float c3   = std::cos(q3);
//     const float s3   = std::sin(q3);
//     const float c23  = std::cos(q2 + q3);
//     const float s23  = std::sin(q2 + q3);
//     const float s2_2 = std::sin(2.0f * q2);
//     const float s2_23 = std::sin(2.0f * (q2 + q3));

//     // 几何参数
//     const float l1  = params_.dh[1].a;      // 连杆2长度
//     const float l2  = params_.dh[2].a;      // 连杆3长度
//     const float lc1 = params_.rc[1].x();    // 连杆2质心到关节2距离
//     const float lc2 = params_.rc[2].x();    // 连杆3质心到关节3距离

//     // 质量参数
//     const float m1 = params_.m[1];          // 连杆2质量
//     const float m2 = params_.m[2];          // 连杆3质量

//     // 惯量参数
//     // 与前面推导一致：第1关节用 link1 的 Iyy；link2/link3 用各自主轴惯量
//     const float Iyy1 = params_.Ic[0](1, 1);

//     const float Ixx2 = params_.Ic[1](0, 0);
//     const float Iyy2 = params_.Ic[1](1, 1);
//     const float Izz2 = params_.Ic[1](2, 2);

//     const float Ixx3 = params_.Ic[2](0, 0);
//     const float Iyy3 = params_.Ic[2](1, 1);
//     const float Izz3 = params_.Ic[2](2, 2);

//     // 重力大小
//     // 默认 gravity = (0, 0, -9.81)，这里取其模长并按前面推导使用“向下重力大小 g > 0”
//     const float g = params_.gravity.norm();

//     // =========================
//     // 1) 质量矩阵 M(q)
//     // =========================
//     Matrix3f M = Matrix3f::Zero();

//     // M11
//     const float rho = lc1 * c2 + (l1 * c2 + lc2 * c23 - lc1 * c2);
//     // 上式等价于 l1*c2 + lc2*c23；这样写只是提醒物理含义
//     const float r3h = l1 * c2 + lc2 * c23;

//     const float M11 =
//         m1 * lc1 * lc1 * c2 * c2 +
//         m2 * r3h * r3h +
//         Iyy1 +
//         (Ixx2 * s2 * s2 + Iyy2 * c2 * c2) +
//         (Ixx3 * s23 * s23 + Iyy3 * c23 * c23);

//     // M22, M23, M33
//     const float M22 =
//         m1 * lc1 * lc1 +
//         m2 * (l1 * l1 + lc2 * lc2 + 2.0f * l1 * lc2 * c3) +
//         Izz2 + Izz3;

//     const float M23 =
//         m2 * (lc2 * lc2 + l1 * lc2 * c3) +
//         Izz3;

//     const float M33 =
//         m2 * lc2 * lc2 +
//         Izz3;

//     M(0, 0) = M11;
//     M(1, 1) = M22;
//     M(1, 2) = M23;
//     M(2, 1) = M23;
//     M(2, 2) = M33;

//     // =========================
//     // 2) 重力项 G(q)
//     // =========================
//     Vector3f G = Vector3f::Zero();
//     G[0] = 0.0f;
//     G[1] = (m1 * lc1 + m2 * l1) * g * c2 + m2 * lc2 * g * c23;
//     G[2] = m2 * lc2 * g * c23;

//     // =========================
//     // 3) 科氏/离心项 h(q,qd)=C(q,qd)*qd
//     //    这里直接算 h，而不是先显式拼完整 C 矩阵
//     // =========================

//     // M11 对 q2, q3 的偏导
//     const float dM11_dq2 =
//         -m1 * lc1 * lc1 * s2_2
//         -m2 * (l1 * l1 * s2_2 + 2.0f * l1 * lc2 * std::sin(2.0f * q2 + q3)
//                + lc2 * lc2 * s2_23)
//         + (Ixx2 - Iyy2) * s2_2
//         + (Ixx3 - Iyy3) * s2_23;

//     const float dM11_dq3 =
//         -2.0f * m2 * lc2 * s23 * (l1 * c2 + lc2 * c23)
//         + (Ixx3 - Iyy3) * s2_23;

//     Vector3f h = Vector3f::Zero();

//     // 第1行
//     h[0] = dM11_dq2 * qd1 * qd2 + dM11_dq3 * qd1 * qd3;

//     // 第2行
//     h[1] =
//         -0.5f * dM11_dq2 * qd1 * qd1
//         -m2 * l1 * lc2 * s3 * (2.0f * qd2 * qd3 + qd3 * qd3);

//     // 第3行
//     h[2] =
//         -0.5f * dM11_dq3 * qd1 * qd1
//         +m2 * l1 * lc2 * s3 * qd2 * qd2;

//     // =========================
//     // 4) 合成力矩 tau = M*qdd + h + G
//     // =========================
//     const Vector3f qdd_vec(qdd1, qdd2, qdd3);
//     tau = M * qdd_vec + h + G;

//     tau_ = tau;
//     return tau_;
// }

Eigen::Vector3f RobotModel::inverseDynamicsL(const Eigen::Vector3f& q,
                                             const Eigen::Vector3f& qd,
                                             const Eigen::Vector3f& qdd) {
    using Eigen::Matrix3f;
    using Eigen::Vector3f;

    Vector3f tau = Vector3f::Zero();

    // =========================
    // 1) 提取关节变量
    // =========================
    const float q1   = q[0];
    const float q2   = q[1];
    const float q3   = q[2];

    const float qd1  = qd[0];
    const float qd2  = qd[1];
    const float qd3  = qd[2];

    const float qdd1 = qdd[0];
    const float qdd2 = qdd[1];
    const float qdd3 = qdd[2];

    (void)q1; // 当前 M,C,G 不显含 q1，但保留以便后续扩展

    // =========================
    // 2) 参数读取
    // =========================
    // 几何参数
    const float l1  = params_.dh[1].a;      // 连杆2长度
    const float l2  = params_.dh[2].a;      // 连杆3长度（本公式里不直接用到，仅用于结构一致性）
    const float lc1 = params_.rc[1].x()+l1/1.0;    // 连杆2质心距关节2
    const float lc2 = params_.rc[2].x()+l2/1.0;    // 连杆3质心距关节3

    (void)l2;

    // 质量参数
    const float m1 = params_.m[1];          // 连杆2质量
    const float m2 = params_.m[2];          // 连杆3质量

    // 惯量参数
    const float Iyy1 = params_.Ic[0](1, 1); // 第1轴等效惯量（按你现有参数结构）

    const float Ixx2 = params_.Ic[1](0, 0);
    const float Iyy2 = params_.Ic[1](1, 1);
    const float Izz2 = params_.Ic[1](2, 2);

    const float Ixx3 = params_.Ic[2](0, 0);
    const float Iyy3 = params_.Ic[2](1, 1);
    const float Izz3 = params_.Ic[2](2, 2);

    // 重力大小
    // 默认 gravity=(0,0,-9.81)，这里取模长作为 g>0 使用
    const float g = params_.gravity.norm();

    // =========================
    // 3) 三角函数
    // =========================
    const float c2   = std::cos(q2);
    const float s2   = std::sin(q2);
    const float c3   = std::cos(q3);
    const float s3   = std::sin(q3);
    const float c23  = std::cos(q2 + q3);
    const float s23  = std::sin(q2 + q3);

    const float sin2q2   = std::sin(2.0f * q2);
    const float sin2q23  = std::sin(2.0f * (q2 + q3));
    const float sin2q2q3 = std::sin(2.0f * q2 + q3);

    // =========================
    // 4) 质量矩阵 M(q)
    // =========================
    Matrix3f M = Matrix3f::Zero();

    // M11
    const float r3h = l1 * c2 + lc2 * c23;  // 连杆3质心的水平投影半径

    const float M11 =
        m1 * lc1 * lc1 * c2 * c2
        + m2 * r3h * r3h
        + Iyy1
        + (Ixx2 * s2 * s2 + Iyy2 * c2 * c2)
        + (Ixx3 * s23 * s23 + Iyy3 * c23 * c23);

    // M22
    const float M22 =
        m1 * lc1 * lc1
        + m2 * (l1 * l1 + lc2 * lc2 + 2.0f * l1 * lc2 * c3)
        + Izz2 + Izz3;

    // M23 = M32
    const float M23 =
        m2 * (lc2 * lc2 + l1 * lc2 * c3)
        + Izz3;

    // M33
    const float M33 =
        m2 * lc2 * lc2
        + Izz3;

    M(0, 0) = M11;
    M(0, 1) = 0.0f;
    M(0, 2) = 0.0f;

    M(1, 0) = 0.0f;
    M(1, 1) = M22;
    M(1, 2) = M23;

    M(2, 0) = 0.0f;
    M(2, 1) = M23;
    M(2, 2) = M33;

    // =========================
    // 5) 重力向量 G(q)
    // =========================
    Vector3f G = Vector3f::Zero();

    G[0] = 0.0f;
    G[1] = (m1 * lc1 + m2 * l1) * g * c2 + m2 * lc2 * g * c23;
    G[2] = m2 * lc2 * g * c23;

    // =========================
    // 6) 速度项 h(q,qd) = C(q,qd) * qd
    //    直接按推导公式计算，不单独显式构造 C 矩阵
    // =========================

    // dM11/dq2
    const float dM11_dq2 =
        -m1 * lc1 * lc1 * sin2q2
        -m2 * (
            l1 * l1 * sin2q2
            + 2.0f * l1 * lc2 * sin2q2q3
            + lc2 * lc2 * sin2q23
        )
        + (Ixx2 - Iyy2) * sin2q2
        + (Ixx3 - Iyy3) * sin2q23;

    // dM11/dq3
    const float dM11_dq3 =
        -2.0f * m2 * lc2 * s23 * (l1 * c2 + lc2 * c23)
        + (Ixx3 - Iyy3) * sin2q23;

    Vector3f h = Vector3f::Zero();

    // h1
    h[0] = dM11_dq2 * qd1 * qd2
         + dM11_dq3 * qd1 * qd3;

    // h2
    h[1] = -0.5f * dM11_dq2 * qd1 * qd1
         - m2 * l1 * lc2 * s3 * (2.0f * qd2 * qd3 + qd3 * qd3);

    // h3
    h[2] = -0.5f * dM11_dq3 * qd1 * qd1
         + m2 * l1 * lc2 * s3 * qd2 * qd2;

    // =========================
    // 7) 合成力矩 tau = M*qdd + h + G
    // =========================
    const Vector3f qdd_vec(qdd1, qdd2, qdd3);
    tau = M * qdd_vec + h + G;

    tau_ = tau;
    return tau_;
}


Eigen::Vector3f RobotModel::gravityCompensation(const Eigen::Vector3f& q) const
{
    using Eigen::Vector3f;

    Vector3f G = Vector3f::Zero();

    // 1) 提取关节角
    const float q2 = q[1];
    const float q3 = q[2];

    // 2) 参数读取
    const float l1 = params_.dh[1].a;   // 连杆2长度
    const float l2 = params_.dh[2].a;   // 连杆3长度

    // 这里请确认 rc 的定义！
    // 若 rc[i] 已经是“本关节到质心”的向量，就不要再加 l1/l2
    const float lc1 = params_.rc[1].x() + l1/1.0;
    const float lc2 = params_.rc[2].x() + l2/1.0;

    (void)l2;

    const float m1 = params_.m[1];      // 连杆2质量
    const float m2 = params_.m[2];      // 连杆3质量

    // 重力大小
    const float g = params_.gravity.norm();

    // 3) 三角函数
    const float c2  = std::cos(q2);
    const float c23 = std::cos(q2 + q3);

    // std::cout << "Gravity Compensation Debug Info:" << std::endl;
    // std::cout << "q2: " << q2 << ", q3: " << q3 << std::endl;
    // std::cout << "lc1: " << lc1 << ", lc2: " << lc2 << std::endl;
    // std::cout << "m1: " << m1 << ", m2: " << m2 << std::endl;
    // std::cout << "g: " << g << std::endl;
    // 4) 重力向量
    // 若第1轴是竖直转轴，则 G[0] = 0
    G[0] = 0.0f;
    G[1] = (m1 * lc1 + m2 * l1) * g * c2 + m2 * lc2 * g * c23;
    G[2] = m2 * lc2 * g * c23;

    return G;
}


void RobotModel::computeWrenches() {
    // 检查是否已计算速度和加速度
    if (V1.size() == 0 || Vd1.size() == 0) {
        throw std::runtime_error("Must compute velocities and accelerations first");
    }
    
    // 末端力旋量为0（无外力）或使用setEndEffectorWrench设置的值
    // F4 已经在构造函数中初始化为0
    
    // 逆向迭代计算力旋量
    
    // 关节3: F3 = adjoint(inv_transform(T3_4))' * F4 + G3*Vd3 - ad(V3)'*(G3*V3)
    F3 = Ad_T3_4_inv_.transpose() * F4 + G_list_[2] * Vd3 - ad(V3).transpose() * (G_list_[2] * V3);
    
    // 关节2: F2 = adjoint(inv_transform(T2_3))' * F3 + G2*Vd2 - ad(V2)'*(G2*V2)
    F2 = Ad_T2_3_inv_.transpose() * F3 + G_list_[1] * Vd2 - ad(V2).transpose() * (G_list_[1] * V2);
    
    // 关节1: F1 = adjoint(inv_transform(T1_2))' * F2 + G1*Vd1 - ad(V1)'*(G1*V1)
    F1 = Ad_T1_2_inv_.transpose() * F2 + G_list_[0] * Vd1 - ad(V1).transpose() * (G_list_[0] * V1);
}

void RobotModel::setEndEffectorWrench(const Eigen::VectorXd& F_tip) {
    if (F_tip.size() != 6) {
        throw std::invalid_argument("End effector wrench must be 6D vector");
    }
    F4 = F_tip;
}

Vector3f RobotModel::computeGravityCompensation(const Vector3f& q) 
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

 Matrix3d RobotModel::skew(const  Vector3d& w)  {
     Matrix3d wx;
    wx << 0, -w(2), w(1),
          w(2), 0, -w(0),
          -w(1), w(0), 0;
    return wx;
}

// 辅助函数：计算螺旋轴的指数映射
 Matrix4d RobotModel::expm_screw(const  VectorXd& S, double theta)  {
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


Matrix6d RobotModel::adjoint(const  Matrix4d& T) 
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


Eigen::Matrix<double, 6, 6> RobotModel::ad(const Eigen::VectorXd& A) 
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