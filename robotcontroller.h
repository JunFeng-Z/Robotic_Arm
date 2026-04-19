#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <QObject>
#include <QVector>
#include <array>
#include <memory>
#include <atomic>

#include "robot_common.h"
#include "robot_model.h"
#include "trajectory_generator.h"
#include "damiao.h"

class SerialPort;


#include "control_worker.h"

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

    /**
     * @brief 切换控制算法
     */
    void SwitchControlAlgorithm(ControlAlgorithm algorithm);

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
    std::unique_ptr<ControlWorker> worker_;
    std::atomic_bool controlRunning_{false};
    ControlAlgorithm currentAlgorithm_{ControlAlgorithm::GravityCompensation};
};

#endif // ROBOTCONTROLLER_H
