#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include "robot_common.h"
#include <memory>

/**
 * @brief 轨迹生成器类 - 生成各种类型的工作空间轨迹
 */
class TrajectoryGenerator
{
public:
    /**
     * @brief 构造函数
     * @param params 轨迹参数
     */
    explicit TrajectoryGenerator(const TrajectoryParams& params = TrajectoryParams());

    /**
     * @brief 设置轨迹参数
     */
    void setParameters(const TrajectoryParams& params);

    /**
     * @brief 获取轨迹参数
     */
    const TrajectoryParams& getParameters() const { return params_; }

    /**
     * @brief 设置轨迹类型
     */
    void setTrajectoryType(TrajectoryType type);

    /**
     * @brief 获取轨迹类型
     */
    TrajectoryType getTrajectoryType() const { return params_.type; }

    /**
     * @brief 生成轨迹点
     * @param t 时间 (秒)
     * @return 轨迹点 (位置、速度、加速度)
     */
    TrajectoryPoint generatePoint(float t) const;

    /**
     * @brief 生成螺旋线轨迹点
     * @param t 时间 (秒)
     * @return 螺旋线轨迹点
     */
    TrajectoryPoint generateSpiralPoint(float t) const;

    /**
     * @brief 生成正弦轨迹点 (关节空间)
     * @param t 时间 (秒)
     * @return 关节角度轨迹 [q1, q2, q3]
     */
    Eigen::Vector3f generateSineJointTrajectory(float t) const;

    /**
     * @brief 生成完整轨迹 (预计算)
     * @param dt 时间步长 (秒)
     * @return 轨迹点序列
     */
    std::vector<TrajectoryPoint> generateTrajectory(float dt = 0.001f) const;

    /**
     * @brief 检查轨迹是否完成
     * @param t 当前时间 (秒)
     * @return true 如果轨迹已完成
     */
    bool isTrajectoryFinished(float t) const;

    /**
     * @brief 获取轨迹持续时间
     */
    float getDuration() const { return params_.duration; }

    /**
     * @brief 设置轨迹持续时间
     */
    void setDuration(float duration) { params_.duration = duration; }

    // ==================== 螺旋线轨迹参数设置 ====================

    void setSpiralAmplitude(float amplitude) { params_.spiralAmplitude = amplitude; }
    void setSpiralRate(float rate) { params_.spiralRate = rate; }
    void setSpiralCenter(float x0, float y0, float z0) {
        params_.spiralX0 = x0;
        params_.spiralY0 = y0;
        params_.spiralZ0 = z0;
    }
    void setSpiralZRiseRate(float rate) { params_.spiralZRiseRate = rate; }

    // ==================== 正弦轨迹参数设置 ====================

    void setSineAmplitudes(float amp1, float amp2, float amp3) {
        params_.sineAmplitude1 = amp1;
        params_.sineAmplitude2 = amp2;
        params_.sineAmplitude3 = amp3;
    }
    void setSineFrequency(float freq) { params_.sineFrequency = freq; }

private:
    TrajectoryParams params_;
};

#endif // TRAJECTORY_GENERATOR_H