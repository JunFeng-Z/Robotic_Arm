#include "plotwidget.h"

#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QResizeEvent>
#include <QDateTime>
#include <cmath>
#include <algorithm>

// 默认颜色定义
const QVector<QColor> PlotWidget::defaultColors_ = {
    QColor(65, 105, 225),   // 蓝色 - 通道1
    QColor(220, 20, 60),    // 红色 - 通道2
    QColor(34, 139, 34)     // 绿色 - 通道3
};

PlotWidget::PlotWidget(QWidget *parent)
    : QWidget(parent),
      plotType_(JointPosition),
      dataChannels_(ChannelAll),
      timeWindow_(10.0f),    // 默认10秒时间窗口
      currentTime_(0.0f),
      timeStep_(0.01f),      // 默认10ms时间步长
      yMin_(-10.0f),
      yMax_(10.0f),
      autoYRange_(true),
      backgroundColor_(QColor("#f8f9fa")),
      gridColor_(QColor("#dee2e6")),
      axisColor_(QColor("#495057")),
      textColor_(QColor("#212529")),
      isPaused_(false)
{
    setMinimumSize(320, 240);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // 初始化数据系列（3个通道）
    for (int i = 0; i < 3; ++i) {
        DataSeries series;
        series.color = defaultColors_[i];
        series.label = QString("通道 %1").arg(i + 1);
        series.visible = true;
        dataSeries_.append(series);
    }

    // 根据绘图类型设置默认标签
    setPlotType(plotType_);
}

void PlotWidget::setPlotType(PlotType type)
{
    plotType_ = type;

    // 根据绘图类型设置默认标题和Y轴标签
    switch (type) {
    case JointPosition:
        plotTitle_ = "关节角度";
        yAxisLabel_ = "角度 (rad)";
        break;
    case JointVelocity:
        plotTitle_ = "关节速度";
        yAxisLabel_ = "速度 (rad/s)";
        break;
    case JointTorque:
        plotTitle_ = "关节扭矩";
        yAxisLabel_ = "扭矩 (N·m)";
        break;
    case EndEffectorPos:
        plotTitle_ = "末端位置";
        yAxisLabel_ = "位置 (m)";
        break;
    case EndEffectorVel:
        plotTitle_ = "末端速度";
        yAxisLabel_ = "速度 (m/s)";
        break;
    case EndEffectorAcc:
        plotTitle_ = "末端加速度";
        yAxisLabel_ = "加速度 (m/s²)";
        break;
    case Custom:
        plotTitle_ = "自定义数据";
        yAxisLabel_ = "数值";
        break;
    }

    // 设置通道标签
    if (type == JointPosition || type == JointVelocity || type == JointTorque) {
        for (int i = 0; i < 3; ++i) {
            dataSeries_[i].label = QString("关节 %1").arg(i + 1);
        }
    } else if (type == EndEffectorPos || type == EndEffectorVel || type == EndEffectorAcc) {
        dataSeries_[0].label = "X";
        dataSeries_[1].label = "Y";
        dataSeries_[2].label = "Z";
    }

    update();
}

void PlotWidget::setDataChannels(DataChannel channels)
{
    dataChannels_ = channels;

    // 设置通道可见性
    switch (channels) {
    case Channel1:
        dataSeries_[0].visible = true;
        dataSeries_[1].visible = false;
        dataSeries_[2].visible = false;
        break;
    case Channel2:
        dataSeries_[0].visible = false;
        dataSeries_[1].visible = true;
        dataSeries_[2].visible = false;
        break;
    case Channel3:
        dataSeries_[0].visible = false;
        dataSeries_[1].visible = false;
        dataSeries_[2].visible = true;
        break;
    case ChannelAll:
        dataSeries_[0].visible = true;
        dataSeries_[1].visible = true;
        dataSeries_[2].visible = true;
        break;
    }

    update();
}

void PlotWidget::setPlotTitle(const QString &title)
{
    plotTitle_ = title;
    update();
}

void PlotWidget::setYAxisLabel(const QString &label)
{
    yAxisLabel_ = label;
    update();
}

void PlotWidget::setTimeWindow(float seconds)
{
    if (seconds > 0) {
        timeWindow_ = seconds;

        // 清理超出时间窗口的数据
        for (auto &series : dataSeries_) {
            while (!series.points.isEmpty() &&
                   series.points.first().first < currentTime_ - timeWindow_) {
                series.points.removeFirst();
            }
        }

        update();
    }
}

void PlotWidget::updateData(int channel, float value)
{
    if (isPaused_) return;
    if (channel >= 0 && channel < 3) {
        addDataPoint(channel, value);

        // 更新当前时间（如果这是第一个数据点，使用0作为起始时间）
        if (dataSeries_[channel].points.isEmpty()) {
            currentTime_ = 0.0f;
        } else {
            currentTime_ += timeStep_;
        }

        // 清理超出时间窗口的数据
        while (!dataSeries_[channel].points.isEmpty() &&
               dataSeries_[channel].points.first().first < currentTime_ - timeWindow_) {
            dataSeries_[channel].points.removeFirst();
        }

        // 更新Y轴范围
        if (autoYRange_) {
            calculateYRange();
        }

        update();  // 触发重绘
    }
}

void PlotWidget::updateData(const QVector<float> &values)
{
    if (isPaused_) return;
    if (values.size() >= 3) {
        for (int i = 0; i < 3; ++i) {
            addDataPoint(i, values[i]);
        }

        // 更新当前时间
        currentTime_ += timeStep_;

        // 清理超出时间窗口的数据
        for (auto &series : dataSeries_) {
            while (!series.points.isEmpty() &&
                   series.points.first().first < currentTime_ - timeWindow_) {
                series.points.removeFirst();
            }
        }

        // 更新Y轴范围
        if (autoYRange_) {
            calculateYRange();
        }

        update();  // 触发重绘
    }
}

void PlotWidget::clearData()
{
    for (auto &series : dataSeries_) {
        series.points.clear();
    }
    currentTime_ = 0.0f;

    // 重置所有状态
    isPaused_ = false;

    update();
}

void PlotWidget::setChannelColor(int channel, const QColor &color)
{
    if (channel >= 0 && channel < 3) {
        dataSeries_[channel].color = color;
        update();
    }
}

void PlotWidget::setChannelLabel(int channel, const QString &label)
{
    if (channel >= 0 && channel < 3) {
        dataSeries_[channel].label = label;
        update();
    }
}

void PlotWidget::paintEvent(QPaintEvent *event)
{
    QWidget::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    // 绘制背景
    painter.fillRect(rect(), backgroundColor_);

    // 计算绘图区域
    const int topMargin = 30;
    const int bottomMargin = 40;
    const int leftMargin = 60;
    const int rightMargin = 20;

    plotArea_ = QRectF(leftMargin, topMargin,
                      width() - leftMargin - rightMargin,
                      height() - topMargin - bottomMargin);

    // 绘制网格
    drawGrid(painter);

    // 绘制坐标轴
    drawAxes(painter);

    // 绘制数据曲线
    drawDataCurves(painter);

    // 绘制图例
    drawLegend(painter);

    // 绘制标题
    painter.setPen(textColor_);
    painter.setFont(QFont("Arial", 10, QFont::Bold));
    painter.drawText(QRectF(0, 5, width(), 20), Qt::AlignCenter, plotTitle_);

    event->accept();
}

void PlotWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    update();
}

void PlotWidget::calculateYRange()
{
    if (dataSeries_.isEmpty()) {
        yMin_ = -1.0f;
        yMax_ = 1.0f;
        return;
    }

    float minVal = std::numeric_limits<float>::max();
    float maxVal = std::numeric_limits<float>::lowest();
    bool hasData = false;

    for (const auto &series : dataSeries_) {
        if (!series.visible || series.points.isEmpty()) continue;

        for (const auto &point : series.points) {
            minVal = std::min(minVal, point.second);
            maxVal = std::max(maxVal, point.second);
            hasData = true;
        }
    }

    if (!hasData) {
        yMin_ = -1.0f;
        yMax_ = 1.0f;
        return;
    }

    // 添加10%的边距
    float range = maxVal - minVal;
    if (range < 0.1f) range = 0.1f;  // 最小范围

    yMin_ = minVal - range * 0.1f;
    yMax_ = maxVal + range * 0.1f;
}

void PlotWidget::drawAxes(QPainter &painter)
{
    painter.setPen(QPen(axisColor_, 2));

    // 绘制X轴和Y轴
    painter.drawLine(QPointF(plotArea_.left(), plotArea_.bottom()),
                     QPointF(plotArea_.right(), plotArea_.bottom()));
    painter.drawLine(QPointF(plotArea_.left(), plotArea_.bottom()),
                     QPointF(plotArea_.left(), plotArea_.top()));

    // 绘制X轴刻度
    const int xTicks = 6;
    float displayStart = std::max(0.0f, currentTime_ - timeWindow_);
    painter.setFont(QFont("Arial", 8));

    for (int i = 0; i <= xTicks; ++i) {
        qreal x = plotArea_.left() + (plotArea_.width() * i / xTicks);
        qreal y = plotArea_.bottom();

        // 绘制刻度线
        painter.drawLine(QPointF(x, y), QPointF(x, y + 5));

        // 绘制刻度标签（显示绝对时间）
        float absoluteTime = displayStart + (timeWindow_ * i / xTicks);
        QString label;
        if (qAbs(absoluteTime) < 0.05f) {
            label = "0";
        } else {
            label = QString("%1").arg(absoluteTime, 0, 'f', 1);
        }

        QRectF labelRect(x - 30, y + 8, 60, 20);
        painter.drawText(labelRect, Qt::AlignCenter, label);
    }

    // 绘制Y轴刻度
    const int yTicks = 5;
    for (int i = 0; i <= yTicks; ++i) {
        float value = yMin_ + (yMax_ - yMin_) * i / yTicks;
        qreal y = plotArea_.bottom() - (plotArea_.height() * i / yTicks);

        // 绘制刻度线
        painter.drawLine(QPointF(plotArea_.left() - 5, y),
                        QPointF(plotArea_.left(), y));

        // 绘制刻度标签
        QString label = QString::number(value, 'f', 2);
        QRectF labelRect(plotArea_.left() - 55, y - 10, 50, 20);
        painter.drawText(labelRect, Qt::AlignRight | Qt::AlignVCenter, label);
    }

    // 绘制Y轴标签
    painter.save();
    painter.translate(10, plotArea_.center().y());
    painter.rotate(-90);
    painter.drawText(QRectF(-100, -100, 200, 20), Qt::AlignCenter, yAxisLabel_);
    painter.restore();
}

void PlotWidget::drawGrid(QPainter &painter)
{
    painter.setPen(QPen(gridColor_, 1, Qt::DotLine));

    // 绘制垂直网格线
    const int xGridLines = 6;
    for (int i = 1; i < xGridLines; ++i) {
        qreal x = plotArea_.left() + (plotArea_.width() * i / xGridLines);
        painter.drawLine(QPointF(x, plotArea_.top()),
                        QPointF(x, plotArea_.bottom()));
    }

    // 绘制水平网格线
    const int yGridLines = 5;
    for (int i = 1; i < yGridLines; ++i) {
        qreal y = plotArea_.bottom() - (plotArea_.height() * i / yGridLines);
        painter.drawLine(QPointF(plotArea_.left(), y),
                        QPointF(plotArea_.right(), y));
    }
}

void PlotWidget::drawDataCurves(QPainter &painter)
{
    if (plotArea_.isEmpty()) return;
    float displayStart = std::max(0.0f, currentTime_ - timeWindow_);

    for (int ch = 0; ch < 3; ++ch) {
        const auto &series = dataSeries_[ch];
        if (!series.visible || series.points.size() < 2) continue;

        QPainterPath path;
        bool firstPoint = true;

        for (const auto &point : series.points) {
            float time = point.first;
            float value = point.second;

            // 计算在绘图区域中的位置
            // time是数据点的绝对时间，displayStart是显示窗口的起始时间
            qreal x = plotArea_.left() +
                     ((time - displayStart) / timeWindow_) * plotArea_.width();
            qreal y = plotArea_.bottom() -
                     ((value - yMin_) / (yMax_ - yMin_)) * plotArea_.height();

            // 确保点在绘图区域内
            if (x < plotArea_.left()) x = plotArea_.left();
            if (x > plotArea_.right()) x = plotArea_.right();
            if (y < plotArea_.top()) y = plotArea_.top();
            if (y > plotArea_.bottom()) y = plotArea_.bottom();

            if (firstPoint) {
                path.moveTo(x, y);
                firstPoint = false;
            } else {
                path.lineTo(x, y);
            }
        }

        // 绘制曲线
        painter.setPen(QPen(series.color, 2));
        painter.drawPath(path);
    }
}

void PlotWidget::drawLegend(QPainter &painter)
{
    int visibleCount = 0;
    for (const auto &series : dataSeries_) {
        if (series.visible) visibleCount++;
    }

    if (visibleCount == 0) return;

    // 计算图例位置（右上角）
    int legendWidth = 100;
    int legendHeight = visibleCount * 25 + 10;
    QRectF legendRect(plotArea_.right() - legendWidth - 10,
                     plotArea_.top() + 10,
                     legendWidth, legendHeight);

    // 绘制图例背景
    painter.setBrush(QColor(255, 255, 255, 200));
    painter.setPen(QPen(axisColor_, 1));
    painter.drawRect(legendRect);

    // 绘制图例项
    painter.setFont(QFont("Arial", 9));
    int yOffset = 5;

    for (int i = 0; i < 3; ++i) {
        const auto &series = dataSeries_[i];
        if (!series.visible) continue;

        // 绘制颜色标记
        QRectF colorRect(legendRect.left() + 5, legendRect.top() + yOffset + 5, 15, 3);
        painter.setBrush(series.color);
        painter.setPen(QPen(series.color, 2));
        painter.drawRect(colorRect);

        // 绘制标签
        QRectF labelRect(legendRect.left() + 25, legendRect.top() + yOffset,
                         legendWidth - 30, 20);
        painter.setPen(textColor_);
        painter.drawText(labelRect, Qt::AlignLeft | Qt::AlignVCenter, series.label);

        yOffset += 25;
    }
}

void PlotWidget::addDataPoint(int channel, float value)
{
    if (channel >= 0 && channel < 3) {
        dataSeries_[channel].points.append(qMakePair(currentTime_, value));

        // 限制数据点数量，防止内存无限增长
        const int maxPoints = 10000;  // 最多存储10000个点
        if (dataSeries_[channel].points.size() > maxPoints) {
            dataSeries_[channel].points.removeFirst();
        }
    }
}

void PlotWidget::pause()
{
    isPaused_ = true;
    update(); // 触发重绘，可能更新显示状态
}

void PlotWidget::resume()
{
    isPaused_ = false;
    update();
}

