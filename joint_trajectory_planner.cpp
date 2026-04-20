#include "joint_trajectory_planner.h"
#include <cmath>
#include <stdexcept>
#include <iomanip>
#include <filesystem>

JointTrajectoryPlanner::JointTrajectoryPlanner()
    : startAngles_(Eigen::Vector3f::Zero()),
      targetAngles_(Eigen::Vector3f::Zero()),
      startVelocities_(Eigen::Vector3f::Zero()),
      targetVelocities_(Eigen::Vector3f::Zero()),
      duration_(0.0f),
      interpolationType_(InterpolationType::Quintic),
      cubicCoeffs_(Eigen::Matrix<float, 3, 4>::Zero()),
      quinticCoeffs_(Eigen::Matrix<float, 3, 6>::Zero()),
      trajectorySet_(false),
      trajectoryLogFileName_("")
{
}

void JointTrajectoryPlanner::setInterpolationType(InterpolationType type)
{
    interpolationType_ = type;
    // 如果轨迹已设置，重新计算系数
    if (trajectorySet_) {
        computeCoefficients();
    }
}

void JointTrajectoryPlanner::setTrajectory(const Eigen::Vector3f& startAngles,
                                           const Eigen::Vector3f& targetAngles,
                                           float duration,
                                           const Eigen::Vector3f& startVelocities,
                                           const Eigen::Vector3f& targetVelocities)
{
    if (duration <= 0.0f) {
        throw std::invalid_argument("Trajectory duration must be positive");
    }

    //std::cout << "Setting trajectory: " << startAngles.transpose() << " -> " << targetAngles.transpose() << ", duration: " << duration << std::endl;
    startAngles_ = startAngles;
    targetAngles_ = targetAngles;
    startVelocities_ = startVelocities;
    targetVelocities_ = targetVelocities;
    duration_ = duration;

    computeCoefficients();
    trajectorySet_ = true;
}

void JointTrajectoryPlanner::computeCoefficients()
{
    if (interpolationType_ == InterpolationType::Cubic) {
        // 三次多项式系数计算
        float T = duration_;
        float T2 = T * T;
        float T3 = T2 * T;

        // 构建矩阵 A = [[T2, T3], [2*T, 3*T2]]
        float det = T2 * (3.0f * T2) - T3 * (2.0f * T);

        if (std::abs(det) < 1e-10f) {
            // 如果持续时间太小，使用简化解（直线插值）
            cubicCoeffs_.setZero();
            // 设置线性插值系数: q(t) = q0 + (qf - q0)*t/T
            for (int i = 0; i < 3; ++i) {
                cubicCoeffs_(i, 0) = startAngles_[i];  // a0 = q0
                cubicCoeffs_(i, 1) = (targetAngles_[i] - startAngles_[i]) / T;  // a1 = (qf - q0)/T
                cubicCoeffs_(i, 2) = 0.0f;  // a2 = 0
                cubicCoeffs_(i, 3) = 0.0f;  // a3 = 0
            }
            return;
        }

        float inv_det = 1.0f / det;
        float A_inv_00 = inv_det * (3.0f * T2);
        float A_inv_01 = inv_det * (-T3);
        float A_inv_10 = inv_det * (-2.0f * T);
        float A_inv_11 = inv_det * T2;

        for (int i = 0; i < 3; ++i) {
            float q0 = startAngles_[i];
            float qf = targetAngles_[i];
            float v0 = startVelocities_[i];
            float vf = targetVelocities_[i];

            // 右侧向量 b = [qf - q0 - v0*T, vf - v0]^T
            float b0 = qf - q0 - v0 * T;
            float b1 = vf - v0;

            // 解方程: [a2, a3]^T = A_inv * b
            float a2 = A_inv_00 * b0 + A_inv_01 * b1;
            float a3 = A_inv_10 * b0 + A_inv_11 * b1;

            // 存储系数到矩阵
            cubicCoeffs_(i, 0) = q0;  // a0
            cubicCoeffs_(i, 1) = v0;  // a1
            cubicCoeffs_(i, 2) = a2;  // a2
            cubicCoeffs_(i, 3) = a3;  // a3
        }
    } else if (interpolationType_ == InterpolationType::Quintic) {
        // 五次多项式系数计算
        float T = duration_;

        for (int i = 0; i < 3; ++i) {
            float q0 = startAngles_[i];
            float q1 = targetAngles_[i];
            float v0 = startVelocities_[i];
            float v1 = targetVelocities_[i];

            // 计算五次多项式系数（基于归一化时间 τ = t/T）
            // q(τ) = c0 + c1*τ + c2*τ² + c3*τ³ + c4*τ⁴ + c5*τ⁵
            // 系数公式来自 quinticSplineInterpolation 函数
            float c0 = q0;                                   // F
            float c1 = v0 * T;                              // E
            float c2 = 0.0f;                                // D (假设起始加速度为0)
            float c3 = 10.0f*(q1 - q0) - (6.0f*v0 + 4.0f*v1)*T;  // C
            float c4 = -15.0f*(q1 - q0) + (8.0f*v0 + 7.0f*v1)*T; // B
            float c5 = 6.0f*(q1 - q0) - 3.0f*(v1 + v0)*T;        // A

            // 存储系数到矩阵
            quinticCoeffs_(i, 0) = c0;
            quinticCoeffs_(i, 1) = c1;
            quinticCoeffs_(i, 2) = c2;
            quinticCoeffs_(i, 3) = c3;
            quinticCoeffs_(i, 4) = c4;
            quinticCoeffs_(i, 5) = c5;
        }
    }
}

void JointTrajectoryPlanner::generatePoint(float t, Eigen::Vector3f& q, Eigen::Vector3f& qd, Eigen::Vector3f& qdd) const
{
    if (!trajectorySet_) {
        q.setZero();
        qd.setZero();
        qdd.setZero();
        return;
    }

    // 限制时间在 [0, duration_] 范围内
    if (t < 0.0f) t = 0.0f;
    if (t > duration_) t = duration_;

    if (interpolationType_ == InterpolationType::Cubic) {
        // 三次多项式: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
        // 速度: qd(t) = a1 + 2*a2*t + 3*a3*t^2
        // 加速度: qdd(t) = 2*a2 + 6*a3*t
        float t2 = t * t;
        float t3 = t2 * t;

        // 对每个关节分别计算
        for (int i = 0; i < 3; ++i) {
            float a0 = cubicCoeffs_(i, 0);
            float a1 = cubicCoeffs_(i, 1);
            float a2 = cubicCoeffs_(i, 2);
            float a3 = cubicCoeffs_(i, 3);

            q[i] = a0 + a1 * t + a2 * t2 + a3 * t3;
            qd[i] = a1 + 2.0f * a2 * t + 3.0f * a3 * t2;
            qdd[i] = 2.0f * a2 + 6.0f * a3 * t;
        }
    } else if (interpolationType_ == InterpolationType::Quintic) {
        // 五次多项式: q(τ) = c0 + c1*τ + c2*τ² + c3*τ³ + c4*τ⁴ + c5*τ⁵
        // 其中 τ = t/T ∈ [0,1]
        // 速度: qd(τ) = (c1 + 2*c2*τ + 3*c3*τ² + 4*c4*τ³ + 5*c5*τ⁴) / T
        // 加速度: qdd(τ) = (2*c2 + 6*c3*τ + 12*c4*τ² + 20*c5*τ³) / T²
        float T = duration_;
        float tau = (T > 0.0f) ? t / T : 0.0f;
        float tau2 = tau * tau;
        float tau3 = tau2 * tau;
        float tau4 = tau3 * tau;
        float tau5 = tau4 * tau;

        // 对每个关节分别计算
        for (int i = 0; i < 3; ++i) {
            float c0 = quinticCoeffs_(i, 0);
            float c1 = quinticCoeffs_(i, 1);
            float c2 = quinticCoeffs_(i, 2);
            float c3 = quinticCoeffs_(i, 3);
            float c4 = quinticCoeffs_(i, 4);
            float c5 = quinticCoeffs_(i, 5);

            // 位置
            q[i] = c0 + c1*tau + c2*tau2 + c3*tau3 + c4*tau4 + c5*tau5;

            // 速度 (对τ求导，然后除以T得到对t的导数)
            qd[i] = (c1 + 2.0f*c2*tau + 3.0f*c3*tau2 + 4.0f*c4*tau3 + 5.0f*c5*tau4) / T;

            // 加速度
            qdd[i] = (2.0f*c2 + 6.0f*c3*tau + 12.0f*c4*tau2 + 20.0f*c5*tau3) / (T * T);
        }
    }
}

std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f>>
JointTrajectoryPlanner::generateTrajectory(float dt) const
{
    std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f>> trajectory;

    if (!trajectorySet_ || dt <= 0.0f) {
        return trajectory;
    }

    // 初始化轨迹记录文件
    initTrajectoryLogFile();

    int numPoints = static_cast<int>(duration_ / dt) + 1;
    trajectory.reserve(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = i * dt;
        Eigen::Vector3f q, qd, qdd;
        generatePoint(t, q, qd, qdd);
        trajectory.emplace_back(q, qd, qdd);

        // 记录轨迹点
        if (trajectoryLogFile_.is_open()) {
            trajectoryLogFile_ << std::fixed << std::setprecision(6)
                               << t << ","
                               << q[0] << "," << q[1] << "," << q[2] << ","
                               << qd[0] << "," << qd[1] << "," << qd[2] << ","
                               << qdd[0] << "," << qdd[1] << "," << qdd[2] << std::endl;
        }
    }

    return trajectory;
}

bool JointTrajectoryPlanner::isTrajectoryFinished(float t) const
{
    return t >= duration_;
}

void JointTrajectoryPlanner::clearTrajectory()
{
    startAngles_.setZero();
    targetAngles_.setZero();
    startVelocities_.setZero();
    targetVelocities_.setZero();
    duration_ = 0.0f;
    cubicCoeffs_.setZero();
    quinticCoeffs_.setZero();
    trajectorySet_ = false;
}

void JointTrajectoryPlanner::initTrajectoryLogFile() const
{
    // 固定文件名
    const_cast<std::string&>(trajectoryLogFileName_) = "joint_trajectory_log.txt";

    // 检查文件是否存在，存在则删除
    if (std::filesystem::exists(trajectoryLogFileName_)) {
        std::filesystem::remove(trajectoryLogFileName_);
    }

    // 打开文件进行写入
    const_cast<std::ofstream&>(trajectoryLogFile_).open(trajectoryLogFileName_, std::ios::out);
    if (!trajectoryLogFile_.is_open()) {
        return;
    }

    // 写入标题行
    trajectoryLogFile_ << "time,"
                       << "q1,q2,q3," << "qd1,qd2,qd3," << "qdd1,qdd2,qdd3" << std::endl;
}