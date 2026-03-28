#include "trajectory_generator.h"
#include <cmath>

TrajectoryGenerator::TrajectoryGenerator(const TrajectoryParams& params)
    : params_(params)
{
}

void TrajectoryGenerator::setParameters(const TrajectoryParams& params)
{
    params_ = params;
}

void TrajectoryGenerator::setTrajectoryType(TrajectoryType type)
{
    params_.type = type;
}

TrajectoryPoint TrajectoryGenerator::generatePoint(float t) const
{
    switch (params_.type) {
    case TrajectoryType::Spiral:
        return generateSpiralPoint(t);
    case TrajectoryType::Sine:
        // 正弦轨迹在关节空间生成
        return TrajectoryPoint();
    default:
        return TrajectoryPoint();
    }
}

TrajectoryPoint TrajectoryGenerator::generateSpiralPoint(float t) const
{
    TrajectoryPoint point;

    // 从C#代码移植的螺旋线轨迹
    float rate = params_.spiralRate;
    float amplitude = params_.spiralAmplitude;
    float x0 = params_.spiralX0;
    float y0 = params_.spiralY0;
    float z0 = params_.spiralZ0;
    float zRiseRate = params_.spiralZRiseRate;

    // 位置
    point.position[0] = x0 + amplitude * std::cos(rate * t);
    point.position[1] = y0 + amplitude * std::sin(rate * t);
    point.position[2] = z0 + zRiseRate * t;

    // 速度 (与C#代码一致)
    point.velocity[0] = -amplitude * rate * std::sin(rate * t);
    point.velocity[1] = amplitude * rate * std::cos(rate * t);
    point.velocity[2] = zRiseRate;

    // 加速度
    point.acceleration[0] = -amplitude * rate * rate * std::cos(rate * t);
    point.acceleration[1] = -amplitude * rate * rate * std::sin(rate * t);
    point.acceleration[2] = 0.0f;

    return point;
}

Eigen::Vector3f TrajectoryGenerator::generateSineJointTrajectory(float t) const
{
    Eigen::Vector3f q;

    float omega = 2.0f * M_PI * params_.sineFrequency;

    q[0] = params_.sineAmplitude1 * std::sin(omega * t);
    q[1] = params_.sineAmplitude2 * std::sin(omega * t + M_PI / 4.0f);
    q[2] = -params_.sineAmplitude3 * std::sin(omega * t + M_PI / 2.0f);

    return q;
}

std::vector<TrajectoryPoint> TrajectoryGenerator::generateTrajectory(float dt) const
{
    std::vector<TrajectoryPoint> trajectory;
    int numPoints = static_cast<int>(params_.duration / dt) + 1;

    for (int i = 0; i < numPoints; ++i) {
        float t = i * dt;
        trajectory.push_back(generatePoint(t));
    }

    return trajectory;
}

bool TrajectoryGenerator::isTrajectoryFinished(float t) const
{
    return t >= params_.duration;
}