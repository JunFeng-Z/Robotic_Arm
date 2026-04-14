#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QAtomicInt>
#include <QVector>
#include <array>
#include <memory>
#include <atomic>

#include "robot_common.h"
#include "robot_model.h"
#include "trajectory_generator.h"
#include "damiao.h"

class SerialPort;


/**
 * @brief 控制工作线程（独立线程执行控制算法）
 */
class ControlWorker : public QObject
{
    Q_OBJECT

public:
    explicit ControlWorker(QObject *parent = nullptr);

public slots:
    void start();
    void stop();
    void updateJointState(const JointState &state);
    void setControlParams(const ControlParams &params);
    void initTrajectory();
    void clearMoveIndex();

signals:
    void controlCommandSent(int jointIndex, float targetPos, float targetVel);
    void torqueCommandSent(int jointIndex, float torque);
    void controlStatusChanged(bool running);
    void logMessage(const QString &message);

private:
    void controlLoop();

    bool computeJointStatesFromTrajectory(float t,Eigen::Vector3f& q,Eigen::Vector3f& qd,Eigen::Vector3f& qdd);

        /**
     * @brief 根据末端状态计算关节状态（角度、速度、加速度）
     * @param position 末端位置 [x, y, z] (m)
     * @param velocity 末端速度 [dx, dy, dz] (m/s)
     * @param acceleration 末端加速度 [ddx, ddy, ddz] (m/s²)
     * @param q 输出关节角度 [q1, q2, q3] (rad)
     * @param qd 输出关节速度 [qd1, qd2, qd3] (rad/s)
     * @param qdd 输出关节加速度 [qdd1, qdd2, qdd3] (rad/s²)
     * @param elbow 肘部配置 (+1 或 -1)，默认-1
     * @return 是否成功计算
     */
    bool computeInverseKinematicsFull(const Eigen::Vector3f& position,
                                      const Eigen::Vector3f& velocity,
                                      const Eigen::Vector3f& acceleration,
                                      Eigen::Vector3f& q,
                                      Eigen::Vector3f& qd,
                                      Eigen::Vector3f& qdd,
                                      int elbow = -1);

    Eigen::Vector3f computeTorques(const Eigen::Vector3f& q,
            const Eigen::Vector3f& qd,
            const Eigen::Vector3f& qdd) ;
    void GetReferenceTorques() ;
    float getCurrentTime() const;
    void updateModelFromParams();  // 根据params_更新模型和生成器
    bool Slidingmode_controller(float t);

private:
    std::array<JointState, 4> jointStates_;  // 索引1-3对应关节1-3
    mutable QMutex stateMutex_;  // 保护关节状态
    ControlParams params_;
    mutable QMutex paramsMutex_;  // 保护参数

    // 机器人模型和轨迹生成器
    std::unique_ptr<RobotModel> robotModel_;
    std::unique_ptr<TrajectoryGenerator> trajectoryGenerator_;
    mutable QMutex modelMutex_;  // 保护模型和生成器

    std::atomic_bool running_{false};
    std::atomic_bool trajectoryInitialized_{false};
    float startTime_ = 0.0f;
    int controlPeriodMs_ = 1;//单位毫秒

    std::atomic_int moveIndex_{0};  // 预定义轨迹点的起始索引
    std::vector<Eigen::Vector3f> torque_d;
    std::vector<Eigen::Vector3f> q_d;
    std::vector<Eigen::Vector3f> qd_d;
    std::vector<Eigen::Vector3f> qdd_d;
};

/**
 * @brief 机器人控制器主类
 *
 * 负责协调控制线程、与MainWindow交互
 */
class RobotController : public QObject
{
    Q_OBJECT

public:
    explicit RobotController(QObject *parent = nullptr);
    ~RobotController();

    /**
     * @brief 设置串口对象（用于发送控制命令）
     */
    void setSerialPort(SerialPort *serialPort);

    /**
     * @brief 更新关节状态（由CAN解析线程调用）
     */
    void updateJointState(int jointIndex, float position, float velocity);

    /**
     * @brief 启动控制循环
     */
    void startControl();

    /**
    * @brief 清除控制数据（由MainWindow调用）
    */
    void clearMoveIndex();

    /**
     * @brief 停止控制循环
     */
    void stopControl();

    /**
     * @brief 设置控制参数
     */
    void setControlParams(const ControlParams &params);

    /**
     * @brief 获取当前控制参数
     */
    ControlParams getControlParams() const;

    /**
     * @brief 初始化轨迹跟踪
     */
    void initTrajectoryTracking();

    /**
     * @brief 使能电机
     */
    void enableMotors();

    /**
     * @brief 失能电机
     */
    void disableMotors();

    /**
     * @brief 获取指定关节的当前状态
     */
    JointState getJointState(int jointIndex) const;

    /**
     * @brief 是否正在运行控制算法
     */
    bool isControlRunning() const { return controlRunning_; }

signals:
    /**
     * @brief 关节状态更新信号
     */
    void jointStateChanged(const JointState &state);

    /**
     * @brief 控制命令发送信号（用于调试/显示）
     */
    void controlCommandSent(int jointIndex, float targetPos, float targetVel);

    /**
     * @brief 扭矩命令发送信号
     */
    void torqueCommandSent(int jointIndex, float torque);

    /**
     * @brief 控制循环状态变化
     */
    void controlStatusChanged(bool running);

    /**
     * @brief 日志消息
     */
    void logMessage(const QString &message);

public slots:
    /**
     * @brief 内部槽：处理控制命令发送
     */
    void onControlCommandSent(int jointIndex, float targetPos, float targetVel);

    /**
     * @brief 内部槽：处理扭矩命令发送
     */
    void onTorqueCommandSent(int jointIndex, float torque);

private:
    /**
     * @brief 获取当前时间（秒）
     */
    float getCurrentTime() const;

    /**
     * @brief 将浮点数转换为定点整数表示
     * @param x 输入浮点数
     * @param x_min 最小值
     * @param x_max 最大值
     * @param bits 位数
     * @return 定点整数
     */
    uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) const;

    /**
     * @brief 将浮点数转换为2字节数组（仿C# Float2Byte）
     * @param x 输入浮点数
     * @param x_min 最小值
     * @param x_max 最大值
     * @param bits 位数
     * @return 2字节数组
     */
    std::array<uint8_t, 2> floatToBytes(float x, float x_min, float x_max, uint8_t bits) const;

    /**
     * @brief 使用can_send_frame结构体填充扭矩命令帧
     * @param frame 输出的can_send_frame结构体
     * @param motorId 电机ID
     * @param torque 扭矩值
     */
    void fillTorqueFrame(damiao::can_send_frame& frame, uint8_t motorId, float torque) const;

    /**
     * @brief 发送扭矩命令到指定电机（仿C# InputMotorTor）
     * @param motorId 电机ID (1-3)
     * @param torque 扭矩值（Nm）
     */
    void sendTorqueCommand(uint8_t motorId, float torque);

    /**
     * @brief 构建30字节CAN命令帧
     * @param motorId 电机ID
     * @param data 9字节数据负载
     * @return 30字节完整帧
     */
    std::array<uint8_t, 30> buildCommandFrame(uint8_t motorId, const std::array<uint8_t, 9>& data) const;

private:
    SerialPort *serialPort_ = nullptr;
    ControlWorker *worker_ = nullptr;
    QThread *controlThread_ = nullptr;
    std::atomic_bool controlRunning_{false};
};

#endif // ROBOTCONTROLLER_H
