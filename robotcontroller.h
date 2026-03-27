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

class SerialPort;

/**
 * @brief 关节状态数据结构
 */
struct JointState
{
    int jointIndex;
    float position;
    float velocity;
    float timestamp;  // 时间戳（秒）

    JointState() : jointIndex(0), position(0), velocity(0), timestamp(0) {}
    JointState(int idx, float pos, float vel, float t = 0)
        : jointIndex(idx), position(pos), velocity(vel), timestamp(t) {}
};

/**
 * @brief 控制参数
 */
struct ControlParams
{
    // 轨迹参数
    float d1 = 1.0f;  // 轨迹参数1
    float d2 = 1.0f;  // 轨迹参数2
    float d3 = 1.0f;  // 轨迹参数3

    // 控制器增益
    float k1 = 0.5f;  // 位置增益
    float k2 = 0.1f;  // 速度增益
    float k3 = 1.0f;  // 前馈增益

    // 控制周期（毫秒）
    int controlPeriod = 10;  // 默认10ms = 100Hz

    // 轨迹总时长（秒）
    float trajectoryDuration = 6.0f;
};

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

signals:
    void controlCommandSent(int jointIndex, float targetPos, float targetVel);
    void controlStatusChanged(bool running);
    void logMessage(const QString &message);

private:
    void controlLoop();
    QVector<float> computeDesiredTrajectory(float t) const;
    std::pair<float, float> computeControl(int jointIndex,
                                          const JointState &state,
                                          const QVector<float> &desired) const;
    void sendMotorCommand(int canId, float position, float velocity);
    float getCurrentTime() const;

private:
    std::array<JointState, 4> jointStates_;  // 索引1-3对应关节1-3
    mutable QMutex stateMutex_;  // 保护关节状态
    ControlParams params_;
    mutable QMutex paramsMutex_;  // 保护参数
    std::atomic_bool running_{false};
    std::atomic_bool trajectoryInitialized_{false};
    float startTime_ = 0.0f;
    int controlPeriodMs_ = 10;
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

private:
    /**
     * @brief 获取当前时间（秒）
     */
    float getCurrentTime() const;

private:
    SerialPort *serialPort_ = nullptr;
    ControlWorker *worker_ = nullptr;
    QThread *controlThread_ = nullptr;
    std::atomic_bool controlRunning_{false};
};

#endif // ROBOTCONTROLLER_H
