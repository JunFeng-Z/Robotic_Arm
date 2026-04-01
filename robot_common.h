#ifndef ROBOT_COMMON_H
#define ROBOT_COMMON_H

#include <eigen3/Eigen/Dense>
#include <vector>

/**
 * @brief 机器人通用数据类型定义
 */

// 使用Eigen别名简化代码
using Vector3f = Eigen::Vector3f;
using Vector3d = Eigen::Vector3d;
using Matrix3f = Eigen::Matrix3f;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using VectorXd = Eigen::VectorXd;

/**
 * @brief 关节状态数据结构
 */
struct JointState
{
    int jointIndex;
    float position;      // 关节位置 (rad)
    float velocity;      // 关节速度 (rad/s)
    float timestamp;     // 时间戳 (s)

    JointState() : jointIndex(0), position(0), velocity(0), timestamp(0) {}
    JointState(int idx, float pos, float vel, float t = 0)
        : jointIndex(idx), position(pos), velocity(vel), timestamp(t) {}
};

/**
 * @brief DH参数结构体
 */
struct DHParameters
{
    float alpha;    // 连杆扭角 (rad)
    float a;        // 连杆长度 (m)
    float d;        // 连杆偏移 (m)
    float theta;    // 关节角度 (rad)

    DHParameters() : alpha(0), a(0), d(0), theta(0) {}
    DHParameters(float alpha_, float a_, float d_, float theta_)
        : alpha(alpha_), a(a_), d(d_), theta(theta_) {}
};

/**
 * @brief 机器人参数结构体 (仅包含参数，不包含方法)
 */
struct RobotParams
{
    // DH参数 (3关节)
    DHParameters dh[3];

    // 连杆质量 (kg)
    float m[3];  // m[0]为关节1质量，通常为0或基座质量

    // 连杆质心位置 (相对于连杆坐标系)
    Vector3f rc[3];

    // 连杆惯量矩阵 (在质心坐标系中)
    Matrix3f Ic[3];

    // 重力向量 (默认Z轴负方向)
    Vector3f gravity;

    // 默认构造函数 (根据C#代码设置默认值)
    RobotParams()
    {
        // 默认重力
        gravity = Vector3f(0, 0, -9.81f);

        // 设置默认DH参数 (根据C#代码中的a2=0.12, a3=0.12)
        // 假设为平面3关节机器人
        dh[0] = DHParameters(M_PI/2.0, 0, 0, 0);      // 关节1: 绕Z轴旋转
        dh[1] = DHParameters(0, 0.12f, 0, 0);  // 关节2: 连杆长度0.12m
        dh[2] = DHParameters(0, 0.12f, 0, 0);  // 关节3: 连杆长度0.12m

        // 质量 (根据C#代码: m2=0.35, m3=0.01)
        m[0] = 0.0f;    // 关节1质量 (通常为0)
        m[1] = 0.35f;   // 关节2质量
        m[2] = 0.01f;   // 关节3质量

        // 质心位置 (假设在连杆中心)
        rc[0] = Vector3f(0, 0, 0);
        rc[1] = Vector3f(0.06f, 0, 0);  // 在连杆中间
        rc[2] = Vector3f(0.06f, 0, 0);

        // 惯量矩阵 (根据C#代码中的值)
        // C#代码: Izz1=0.0015, Iyy2=0.00437, Izz2=0.00437, Iyy3=0.00108, Izz3=0.00107
        Ic[0] << 0.0015f, 0, 0,
                 0, 0.0015f, 0,
                 0, 0, 0.0015f;

        Ic[1] << 0, 0, 0,
                 0, 0.00437f, 0,
                 0, 0, 0.00437f;

        Ic[2] << 0, 0, 0,
                 0, 0.00108f, 0,
                 0, 0, 0.00107f;
    }
};

/**
 * @brief 轨迹类型枚举
 */
enum class TrajectoryType
{
    Sine,   // 正弦轨迹
    Spiral  // 螺旋线轨迹
};

/**
 * @brief 轨迹参数结构体
 */
struct TrajectoryParams
{
    TrajectoryType type = TrajectoryType::Spiral;

    // 通用参数
    float duration = 6.0f;          // 轨迹总时长 (秒)

    // 螺旋线轨迹参数
    float spiralAmplitude = 0.025f;    // 螺旋线幅度 (m)
    float spiralRate = 3.0f;           // 螺旋线角频率 (rad/s)
    float spiralX0 = 0.19f;            // 螺旋线中心X坐标 (m)
    float spiralY0 = 0.0f;             // 螺旋线中心Y坐标 (m)
    float spiralZ0 = 0.0f;             // 螺旋线起始Z坐标 (m)
    float spiralZRiseRate = 0.003f;    // Z轴上升速率 (m/s)

    // 正弦轨迹参数
    float sineAmplitude1 = 1.0f;   // 关节1幅度 (rad)
    float sineAmplitude2 = 1.0f;   // 关节2幅度 (rad)
    float sineAmplitude3 = 1.0f;   // 关节3幅度 (rad)
    float sineFrequency = 1.0f;    // 正弦频率 (Hz)
};

/**
 * @brief 轨迹点结构体 (包含位置、速度、加速度)
 */
struct TrajectoryPoint
{
    Vector3f position;      // 位置 (x, y, z) in meters
    Vector3f velocity;      // 速度 (dx, dy, dz) in m/s
    Vector3f acceleration;  // 加速度 (ddx, ddy, ddz) in m/s²

    TrajectoryPoint() : position(Vector3f::Zero()), velocity(Vector3f::Zero()), acceleration(Vector3f::Zero()) {}
    TrajectoryPoint(const Vector3f& pos, const Vector3f& vel, const Vector3f& acc)
        : position(pos), velocity(vel), acceleration(acc) {}
};

/**
 * @brief 控制参数结构体
 */
struct ControlParams
{
    // 控制器增益
    float k1 = 0.5f;  // 位置增益
    float k2 = 0.1f;  // 速度增益
    float k3 = 1.0f;  // 前馈增益

    // 控制周期（毫秒）
    int controlPeriod = 1;  // 默认1ms = 1000Hz

    // 轨迹参数
    TrajectoryParams trajectory;

    // 机器人参数
    RobotParams robotParams;
};

#endif // ROBOT_COMMON_H