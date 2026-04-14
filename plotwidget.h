#ifndef PLOTWIDGET_H
#define PLOTWIDGET_H

#include <QWidget>
#include <QVector>
#include <QPair>
#include <QColor>
#include <QString>

class PlotWidget : public QWidget
{
    Q_OBJECT

public:
    // 绘图类型枚举
    enum PlotType {
        JointPosition,      // 关节角度
        JointVelocity,      // 关节速度
        JointTorque,        // 关节扭矩
        EndEffectorPos,     // 末端位置
        EndEffectorVel,     // 末端速度
        EndEffectorAcc,     // 末端加速度
        Custom              // 自定义数据
    };

    // 数据通道（最多支持3个数据通道，对应3个关节）
    enum DataChannel {
        Channel1 = 0,       // 关节1/通道1
        Channel2 = 1,       // 关节2/通道2
        Channel3 = 2,       // 关节3/通道3
        ChannelAll = 3      // 所有通道（用于多线绘图）
    };

    explicit PlotWidget(QWidget *parent = nullptr);

    // 设置绘图类型
    void setPlotType(PlotType type);

    // 设置数据通道（显示哪些通道的数据）
    void setDataChannels(DataChannel channels);

    // 设置窗口标题
    void setPlotTitle(const QString &title);

    // 设置Y轴标签
    void setYAxisLabel(const QString &label);

    // 设置时间窗口（秒）
    void setTimeWindow(float seconds);

    // 更新数据（单个通道）
    void updateData(int channel, float value);

    // 批量更新数据（多个通道）
    void updateData(const QVector<float> &values);

    // 清空数据
    void clearData();

    // 暂停/恢复数据更新
    void pause();
    void resume();
    bool isPaused() const { return isPaused_; }

    // 设置通道颜色
    void setChannelColor(int channel, const QColor &color);

    // 设置通道标签
    void setChannelLabel(int channel, const QString &label);

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    // 计算数据的Y轴范围
    void calculateYRange();

    // 绘制坐标轴
    void drawAxes(QPainter &painter);

    // 绘制网格
    void drawGrid(QPainter &painter);

    // 绘制数据曲线
    void drawDataCurves(QPainter &painter);

    // 绘制图例
    void drawLegend(QPainter &painter);

    // 添加新数据点
    void addDataPoint(int channel, float value);

    // 内部数据结构
    struct DataSeries {
        QVector<QPair<float, float>> points;  // (时间, 值) 对
        QColor color;
        QString label;
        bool visible;
    };

    PlotType plotType_;
    DataChannel dataChannels_;
    QString plotTitle_;
    QString yAxisLabel_;

    // 数据存储
    QVector<DataSeries> dataSeries_;

    // 时间管理
    float timeWindow_;      // 时间窗口（秒）
    float currentTime_;     // 当前时间（秒）
    float timeStep_;        // 时间步长（秒）
    bool isScrolling_;      // 是否已开始滚动（当currentTime_ > timeWindow_时）

    // 绘图范围
    float yMin_;
    float yMax_;
    bool autoYRange_;

    // 绘图控制
    bool isPaused_;         // 是否暂停数据更新

    // 外观设置
    QColor backgroundColor_;
    QColor gridColor_;
    QColor axisColor_;
    QColor textColor_;

    // 绘图区域
    QRectF plotArea_;

    // 默认颜色
    static const QVector<QColor> defaultColors_;
};

#endif // PLOTWIDGET_H
