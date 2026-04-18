#include "mainwindow.h"

#include "SerialPort.h"
#include "plotwidget.h"
#include "serialcanworkers.h"
#include "robotcontroller.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <QCloseEvent>
#include <QComboBox>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTextEdit>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QSerialPortInfo>
#include <QDateTime>
#include <QDir>

namespace {
speed_t baudFromText(const QString &baudText, bool *fallback)
{
    if (fallback) {
        *fallback = false;
    }

    if (baudText == "9600") {
        return B9600;
    }
    if (baudText == "57600") {
        return B57600;
    }
    if (baudText == "115200") {
        return B115200;
    }
    if (baudText == "921600") {
#ifdef B921600
        return B921600;
#else
        if (fallback) {
            *fallback = true;
        }
        return B115200;
#endif
    }

    if (fallback) {
        *fallback = true;
    }
    return B115200;
}
} // namespace

QGroupBox *MainWindow::buildConnectionPanel()
{
    auto *group = new QGroupBox(QStringLiteral("连接设备"));
    auto *layout = new QGridLayout(group);
    layout->setHorizontalSpacing(12);
    layout->setVerticalSpacing(8);
    layout->setRowStretch(6, 1);
    layout->setAlignment(Qt::AlignTop);

    auto *portLabel = new QLabel(QStringLiteral("串口号:"));

    portCombo_ = new QComboBox;
    //portCombo_->addItems({"COM1", "COM2", "COM3", "/dev/ttyUSB0", "/dev/ttyUSB1"});

    baudCombo_ = new QComboBox;
    baudCombo_->addItems({ "921600","9600", "57600", "115200"});

    parityCombo_ = new QComboBox;
    parityCombo_->addItems({QStringLiteral("None"), QStringLiteral("Odd"), QStringLiteral("Even")});

    dataBitsCombo_ = new QComboBox;
    dataBitsCombo_->addItems({"8", "7"});

    timeoutCombo_ = new QComboBox;
    timeoutCombo_->addItems({"2", "10", "50", "100"});

    openBtn_ = new QPushButton(QStringLiteral("打开串口"));
    refreshBtn_ = new QPushButton(QStringLiteral("刷新设备"));
    enableBtn_ = new QPushButton(QStringLiteral("使能电机"));
    disableBtn_ = new QPushButton(QStringLiteral("失能电机"));
    cleartextBtn_ = new QPushButton(QStringLiteral("清除日志"));

    layout->addWidget(portLabel, 0, 0);
    layout->addWidget(portCombo_, 1, 0);
    layout->addWidget(openBtn_, 1, 1);

    layout->addWidget(baudCombo_, 2, 0);
    layout->addWidget(refreshBtn_, 2, 1);

    layout->addWidget(parityCombo_, 3, 0);
    layout->addWidget(enableBtn_, 3, 1);

    layout->addWidget(dataBitsCombo_, 4, 0);
    layout->addWidget(disableBtn_, 4, 1);

    layout->addWidget(timeoutCombo_, 5, 0);
    layout->addWidget(cleartextBtn_, 5, 1);

    return group;
}

QGroupBox *MainWindow::buildRunPanel()
{
    auto *group = new QGroupBox(QStringLiteral("运行设置"));
    auto *layout = new QVBoxLayout(group);
    layout->setSpacing(10);
    layout->setAlignment(Qt::AlignTop);

    auto *row1 = new QHBoxLayout;
    initTrackBtn_ = new QPushButton(QStringLiteral("轨迹跟踪初始化"));
    teachBtn_ = new QPushButton(QStringLiteral("示教"));
    row1->addWidget(initTrackBtn_);
    row1->addWidget(teachBtn_);
    layout->addLayout(row1);

    clearDataBtn_ = new QPushButton(QStringLiteral("清除图表"));
    pausePlotBtn_ = new QPushButton(QStringLiteral("暂停图表"));
    resumePlotBtn_ = new QPushButton(QStringLiteral("重启图表"));
    setDurationBtn_ = new QPushButton(QStringLiteral("设置时长"));
    runAlgoBtn_ = new QPushButton(QStringLiteral("运行算法"));

    layout->addWidget(clearDataBtn_);
    auto *plotControlRow = new QHBoxLayout;
    plotControlRow->addWidget(pausePlotBtn_);
    plotControlRow->addWidget(resumePlotBtn_);
    layout->addLayout(plotControlRow);
    layout->addWidget(setDurationBtn_);
    layout->addWidget(runAlgoBtn_);

    auto *durationRow = new QHBoxLayout;
    durationRow->setContentsMargins(0, 0, 0, 0);
    durationRow->setSpacing(8);
    durationRow->addWidget(new QLabel(QStringLiteral("总时长")));

    secondsBox_ = new QSpinBox;
    secondsBox_->setRange(1, 120);
    secondsBox_->setValue(6);
    secondsBox_->setFixedWidth(72);
    durationRow->addWidget(secondsBox_);
    durationRow->addWidget(new QLabel(QStringLiteral("秒")));
    durationRow->addStretch();

    layout->addSpacing(8);
    layout->addLayout(durationRow);

    return group;
}

QGroupBox *MainWindow::buildTunePanel()
{
    auto *group = new QGroupBox(QStringLiteral("参数调试"));
    auto *layout = new QGridLayout(group);

    const QStringList names = {"d1", "d2", "d3", "k1", "k2", "k3"};
    const QStringList plus = {QStringLiteral("A1+"), QStringLiteral("A2+"), QStringLiteral("B1+"), QStringLiteral("B2+"), QStringLiteral("C1+"), QStringLiteral("C2+")};
    const QStringList minus = {QStringLiteral("A1−"), QStringLiteral("A2−"), QStringLiteral("B1−"), QStringLiteral("B2−"), QStringLiteral("C1−"), QStringLiteral("C2−")};

    for (int i = 0; i < names.size(); ++i) {
        auto *label = new QLabel(names[i]);
        auto *plusBtn = new QPushButton(plus[i]);
        auto *minusBtn = new QPushButton(minus[i]);
        auto *combo = new QComboBox;

        combo->addItems({"1", "2", "5", "10"});

        // 存入成员容器
        tunePlusBtns_.append(plusBtn);
        tuneMinusBtns_.append(minusBtn);
        tuneStepBoxes_.append(combo);

        // 连接加号按钮
        connect(plusBtn, &QPushButton::clicked, this, [this, i]() {
            float step = tuneStepBoxes_[i]->currentText().toFloat();
            adjustParameter(i, step);
        });

        // 连接减号按钮
        connect(minusBtn, &QPushButton::clicked, this, [this, i]() {
            float step = tuneStepBoxes_[i]->currentText().toFloat();
            adjustParameter(i, -step);
        });

        layout->addWidget(label, i, 0);
        layout->addWidget(plusBtn, i, 1);
        layout->addWidget(minusBtn, i, 2);
        layout->addWidget(combo, i, 3);
    }

    layout->addWidget(new QPushButton(QStringLiteral("手动刷新显示")), names.size(), 1, 1, 2);
    layout->addWidget(new QPushButton(QStringLiteral("测试键")), names.size(), 3);

    return group;
}
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("轻擎机械臂运动控制台"));
    resize(1600, 900);

    // 创建机器人控制器
    robotController_ = new RobotController(this);

    // 初始化控制参数（使用默认值）
    controlParams_ = ControlParams();
    robotController_->setControlParams(controlParams_);

    auto *central = new QWidget;
    auto *root = new QHBoxLayout(central);
    root->setSpacing(12);

    auto *leftPane = new QWidget;
    auto *leftLayout = new QVBoxLayout(leftPane);
    auto *plotGrid = new QGridLayout;
    plotGrid->setSpacing(10);

    // 创建4个绘图窗口，分别显示不同的数据
    plotWidgets_[0] = new PlotWidget;
    plotWidgets_[0]->setPlotType(PlotWidget::JointPosition);
    plotWidgets_[0]->setPlotTitle("关节角度");
    plotWidgets_[0]->setYAxisLabel("角度 (rad)");
    plotWidgets_[0]->setTimeWindow(10.0f);  // 10秒时间窗口
    plotWidgets_[0]->setDataChannels(PlotWidget::ChannelAll);
    plotGrid->addWidget(plotWidgets_[0], 0, 0);

    plotWidgets_[1] = new PlotWidget;
    plotWidgets_[1]->setPlotType(PlotWidget::JointVelocity);
    plotWidgets_[1]->setPlotTitle("关节速度");
    plotWidgets_[1]->setYAxisLabel("速度 (rad/s)");
    plotWidgets_[1]->setTimeWindow(10.0f);
    plotWidgets_[1]->setDataChannels(PlotWidget::ChannelAll);
    plotGrid->addWidget(plotWidgets_[1], 0, 1);

    plotWidgets_[2] = new PlotWidget;
    plotWidgets_[2]->setPlotType(PlotWidget::JointTorque);
    plotWidgets_[2]->setPlotTitle("关节扭矩");
    plotWidgets_[2]->setYAxisLabel("扭矩 (N·m)");
    plotWidgets_[2]->setTimeWindow(10.0f);
    plotWidgets_[2]->setDataChannels(PlotWidget::ChannelAll);
    plotGrid->addWidget(plotWidgets_[2], 1, 0);

    plotWidgets_[3] = new PlotWidget;
    plotWidgets_[3]->setPlotType(PlotWidget::EndEffectorPos);
    plotWidgets_[3]->setPlotTitle("末端位置");
    plotWidgets_[3]->setYAxisLabel("位置 (m)");
    plotWidgets_[3]->setTimeWindow(10.0f);
    plotWidgets_[3]->setDataChannels(PlotWidget::ChannelAll);
    plotGrid->addWidget(plotWidgets_[3], 1, 1);

    auto *imageFrame = new QFrame;
    imageFrame->setFrameShape(QFrame::StyledPanel);
    imageFrame->setMinimumHeight(90);
    auto *imageLayout = new QHBoxLayout(imageFrame);
    imageLayout->addStretch();
    imageLayout->addWidget(new QLabel(QStringLiteral("显 示 图 像")));
    imageLayout->addStretch();

    leftLayout->addLayout(plotGrid, 1);
    leftLayout->addWidget(imageFrame);

    auto *rightPane = new QWidget;
    auto *rightLayout = new QVBoxLayout(rightPane);

    auto *topRight = new QHBoxLayout;
    auto *logGroup = new QGroupBox(QStringLiteral("提示"));
    auto *logLayout = new QVBoxLayout(logGroup);

    logText_ = new QTextEdit;
    logText_->setReadOnly(true);
    logText_->setPlainText(
        QStringLiteral("嗨连接串口并使能电机\n\n如果想跟踪期望轨迹请先初始化，并等待初始化完成\n......"));
    logLayout->addWidget(logText_);

    auto *connectionPanel = buildConnectionPanel();
    auto *runPanel = buildRunPanel();
    auto *tunePanel = buildTunePanel();

    topRight->addWidget(logGroup, 3);
    topRight->addWidget(connectionPanel, 2);

    auto *bottomRight = new QHBoxLayout;
    bottomRight->addWidget(runPanel, 1);
    bottomRight->addWidget(tunePanel, 1);

    rightLayout->addLayout(topRight, 1);
    rightLayout->addLayout(bottomRight, 1);

    root->addWidget(leftPane, 3);
    root->addWidget(rightPane, 2);

    setCentralWidget(central);

    connect(openBtn_, &QPushButton::clicked, this, &MainWindow::onOpenSerialClicked);
    connect(refreshBtn_, &QPushButton::clicked, this, &MainWindow::onRefreshDevicesClicked);
    connect(setDurationBtn_, &QPushButton::clicked, this, &MainWindow::onSetDurationClicked);
    connect(enableBtn_, &QPushButton::clicked, this, &MainWindow::onEnableClicked);
    connect(disableBtn_, &QPushButton::clicked, this, &MainWindow::onDisableClicked);
    connect(cleartextBtn_, &QPushButton::clicked, this, &MainWindow::onCleartextDataClicked);
    connect(initTrackBtn_, &QPushButton::clicked, this, &MainWindow::onInitTrackClicked);
    connect(teachBtn_, &QPushButton::clicked, this, &MainWindow::onTeachClicked);
    connect(runAlgoBtn_, &QPushButton::clicked, this, &MainWindow::onRunAlgoClicked);
    connect(clearDataBtn_, &QPushButton::clicked, this, &MainWindow::onClearDataClicked);
    connect(pausePlotBtn_, &QPushButton::clicked, this, &MainWindow::onPausePlotClicked);
    connect(resumePlotBtn_, &QPushButton::clicked, this, &MainWindow::onResumePlotClicked);

    // 连接RobotController的信号
    connect(robotController_, &RobotController::jointStateChanged, this, &MainWindow::onJointStateUpdated);
    connect(robotController_, &RobotController::controlCommandSent, this, &MainWindow::onControlCommandSent);
    connect(robotController_, &RobotController::torqueCommandSent, this, &MainWindow::onTorqueCommandSent);
    connect(robotController_, &RobotController::controlStatusChanged, this, &MainWindow::onControlStatusChanged);
    connect(robotController_, &RobotController::logMessage, this, &MainWindow::appendLog);

    // 创建模拟数据定时器（用于测试绘图功能）
    simulationTimer_ = new QTimer(this);
    simulationTimer_->setInterval(50);  // 50ms更新间隔，约20Hz
    connect(simulationTimer_, &QTimer::timeout, this, &MainWindow::onSimulationTimer);

    // 自动开始数据记录（实时写入文件，防止系统崩溃）
    startDataRecording();

    simulationTimer_->start();

    this->onRefreshDevicesClicked();
}

MainWindow::~MainWindow()
{
    // 停止所有数据记录
    for (int i = 0; i < 4; ++i) {
        if (plotWidgets_[i]) {
            plotWidgets_[i]->stopRecording();
        }
    }

    stopWorkers();
    if (robotController_) {
        robotController_->stopControl();
    }
    delete serialPort_;
    serialPort_ = nullptr;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    stopWorkers();
    QMainWindow::closeEvent(event);
}

void MainWindow::appendLog(const QString &text)
{
    if (logText_) {
        logText_->append(text);
    }
}

void MainWindow::onEnableClicked()
{
    this->EnableMotor();
}

void MainWindow::onDisableClicked()
{
    this->DisableMotor();
}

void MainWindow::onOpenSerialClicked()
{
    if (serialPort_) {
        appendLog(QStringLiteral("串口线程已在运行，无需重复打开"));
        return;
    }

    const QString port = portCombo_ ? portCombo_->currentText() : QStringLiteral("/dev/ttyACM0");
    const QString baudText = baudCombo_ ? baudCombo_->currentText() : QStringLiteral("921600");
    const int timeoutMs = timeoutCombo_ ? timeoutCombo_->currentText().toInt() : 10;

    bool fallback = false;
    const speed_t baudrate = baudFromText(baudText, &fallback);

    if (fallback && baudText == "921600") {
        appendLog(QStringLiteral("当前平台不支持B921600，已回退到115200"));
    }

    appendLog(QStringLiteral("打开串口：%1, 波特率：%2, 超时：%3ms")
                  .arg(port, baudText)
                  .arg(timeoutMs));

    serialPort_ = new SerialPort(port.toStdString(), baudrate, timeoutMs);
    startWorkers();
}

void MainWindow::onRefreshDevicesClicked()
{
    if (!portCombo_) return;

    portCombo_->clear();

    const auto ports = QSerialPortInfo::availablePorts();

    if (ports.isEmpty()) {
        appendLog(QStringLiteral("未发现任何串口设备"));
        return;
    }

    appendLog(QStringLiteral("扫描到 %1 个串口设备").arg(ports.size()));

    for (const QSerialPortInfo &info : ports) {

        QString name = info.portName();        // 例: ttyUSB0 / COM3
        QString desc = info.description();     // 设备描述
        QString manu = info.manufacturer();    // 厂商
        QString system = info.systemLocation();// /dev/ttyUSB0

        portCombo_->addItem(system);

        appendLog(QStringLiteral("发现串口: %1 (%2)")
                      .arg(system, desc));
    }
}

void MainWindow::onSetDurationClicked()
{
    const int sec = secondsBox_ ? secondsBox_->value() : 0;
    controlParams_.trajectory.duration = sec;

    if (robotController_) {
        robotController_->setControlParams(controlParams_);
    }

    appendLog(QStringLiteral("总时长设置为 %1 秒").arg(sec));
}

void MainWindow::onJointStateUpdated(const JointState &state)
{
    // 更新日志
    appendLog(QStringLiteral("关节%1: Pos=%2 Vel=%3")
                  .arg(state.jointIndex)
                  .arg(state.position, 0, 'f', 3)
                  .arg(state.velocity, 0, 'f', 3));

    // 更新绘图
    if (plotWidgets_[0] && state.jointIndex >= 1 && state.jointIndex <= 3) {
        // 关节角度绘图（索引0）
        plotWidgets_[0]->updateData(state.jointIndex - 1, state.position);

        // 关节速度绘图（索引1）
        plotWidgets_[1]->updateData(state.jointIndex - 1, state.velocity);
    }
}

void MainWindow::onTorqueCommandSent(int jointIndex, float torque)
{
    // 更新日志
    // appendLog(QStringLiteral("关节%1 扭矩: %2 N·m")
    //               .arg(jointIndex)
    //               .arg(torque, 0, 'f', 3));

    // 更新扭矩绘图
    if (plotWidgets_[2] && jointIndex >= 1 && jointIndex <= 3) {
        plotWidgets_[2]->updateData(jointIndex - 1, torque);
    }
}

void MainWindow::onSimulationTimer()
{
    return; // 启用模拟数据用于测试
    // 生成模拟数据（正弦波）用于测试绘图
    static float time = 0.0f;
    const float dt = 0.05f;  // 50ms间隔
    time += dt;

    // 三个关节的模拟数据（不同频率和幅度的正弦波）
    for (int joint = 1; joint <= 3; ++joint) {
        // 关节位置：正弦波，不同频率和幅度
        float position = 0.5f * sin(2.0f * M_PI * 0.2f * time + joint * 0.5f);

        // 关节速度：位置的导数（余弦波）
        float velocity = 0.5f * 2.0f * M_PI * 0.2f * cos(2.0f * M_PI * 0.2f * time + joint * 0.5f);

        // 关节扭矩：另一个频率的正弦波
        float torque = 0.3f * sin(2.0f * M_PI * 0.3f * time + joint * 1.0f);

        // 更新绘图数据（直接调用绘图窗口，绕过RobotController）
        if (plotWidgets_[0]) {
            plotWidgets_[0]->updateData(joint - 1, position);
        }

        if (plotWidgets_[1]) {
            plotWidgets_[1]->updateData(joint - 1, velocity);
        }

        if (plotWidgets_[2]) {
            plotWidgets_[2]->updateData(joint - 1, torque);
        }

        // 更新末端位置绘图（索引3）- 使用一些模拟的XYZ数据
        if (plotWidgets_[3]) {
            // 模拟末端位置：螺旋线轨迹
            float radius = 0.02f;
            float x = radius * cos(2.0f * M_PI * 0.1f * time);
            float y = radius * sin(2.0f * M_PI * 0.1f * time);
            float z = 0.01f * time;

            QVector<float> endEffectorPos = {x, y, z};
            plotWidgets_[3]->updateData(endEffectorPos);
        }
    }

    // 可选：更新日志显示当前模拟时间
    // appendLog(QStringLiteral("模拟时间: %1秒").arg(time, 0, 'f', 1));
}

void MainWindow::startWorkers()
{
    if (!serialPort_ || serialThread_ || canParserThread_) {
        return;
    }

    serialThread_ = new QThread(this);
    canParserThread_ = new QThread(this);

    serialRxWorker_ = new SerialRxWorker(serialPort_, &canRxFifo_, &uartRxFifo_,
                                         &canRxMutex_, &uartRxMutex_, &canDataReady_);
    canParserWorker_ = new CanParserWorker(&canRxFifo_, &canRxMutex_, &canDataReady_);
    //决定这个 QObject 的槽函数将在哪个线程执行,此处表示将serialRxWorker_中的所有槽函数的执行权转移到serialThread_线程中
    serialRxWorker_->moveToThread(serialThread_);
    canParserWorker_->moveToThread(canParserThread_);

    //设置两个worker线程启动的开关
    connect(serialThread_, &QThread::started, serialRxWorker_, &SerialRxWorker::start);
    connect(canParserThread_, &QThread::started, canParserWorker_, &CanParserWorker::start);

    connect(serialRxWorker_, &SerialRxWorker::logMessage, this, &MainWindow::appendLog);
    connect(canParserWorker_, &CanParserWorker::logMessage, this, &MainWindow::appendLog);
    connect(canParserWorker_, &CanParserWorker::jointStateUpdated,
            robotController_, &RobotController::updateJointState);

            
    connect(serialThread_, &QThread::finished, serialRxWorker_, &QObject::deleteLater);
    connect(canParserThread_, &QThread::finished, canParserWorker_, &QObject::deleteLater);

    serialThread_->start();
    canParserThread_->start();

    // 设置串口到RobotController
    if (robotController_) {
        robotController_->setSerialPort(serialPort_);
    }

    appendLog(QStringLiteral("后台线程启动完成"));
}

void MainWindow::stopWorkers()
{
    this->DisableMotor();
    QThread::msleep(100);    // 先通知停止
    if (serialRxWorker_) {
        serialRxWorker_->stop();
    }
    if (canParserWorker_) {
        canParserWorker_->stop();
    }

    // 唤醒可能阻塞在条件变量上的消费者线程
    canDataReady_.wakeAll();

    if (serialThread_) {
        serialThread_->quit();
        serialThread_->wait();
        serialThread_->deleteLater();
        serialThread_ = nullptr;
    }

    if (canParserThread_) {
        canParserThread_->quit();
        canParserThread_->wait();
        canParserThread_->deleteLater();
        canParserThread_ = nullptr;
    }

    serialRxWorker_ = nullptr;
    canParserWorker_ = nullptr;
}


void MainWindow::EnableMotor()
{
    if(robotController_) {
        robotController_->enableMotors();
    }
}


void MainWindow::DisableMotor()
{
    if(robotController_) {
        robotController_->disableMotors();
    }
}

void MainWindow::onInitTrackClicked()
{
    if(robotController_) {
        robotController_->initTrajectoryTracking();
    }
}

void MainWindow::onTeachClicked()
{
    appendLog(QStringLiteral("示教模式（待实现）"));
}

void MainWindow::onRunAlgoClicked()
{
    if(robotController_) {
        robotController_->startControl();
        appendLog(QStringLiteral("控制算法已启动"));
    }
}

void MainWindow::onCleartextDataClicked()
{
    if(logText_) {
        logText_->clear();
    }
    appendLog(QStringLiteral("日志已清除"));
}

void MainWindow::onClearDataClicked()
{
    //清除UI上所有的图表数据
    for (int i = 0; i < 4; ++i) {
        if (plotWidgets_[i]) {
            plotWidgets_[i]->clearData();
        }
    }
    //将轨迹跟踪中的索引设置回初始状态0
    if(robotController_) {
        robotController_->clearMoveIndex();
    }
    appendLog(QStringLiteral("图表数据已清除"));
}

void MainWindow::onPausePlotClicked()
{
    for (int i = 0; i < 4; ++i) {
        if (plotWidgets_[i]) {
            plotWidgets_[i]->pause();
        }
    }
    appendLog(QStringLiteral("图表已暂停"));
}

void MainWindow::onResumePlotClicked()
{
    for (int i = 0; i < 4; ++i) {
        if (plotWidgets_[i]) {
            plotWidgets_[i]->resume();
        }
    }
    appendLog(QStringLiteral("图表已重启"));
}

void MainWindow::onControlCommandSent(int jointIndex, float targetPos, float targetVel)
{
    // 可以在这里更新图表或显示信息
    // appendLog(QStringLiteral("关节%1控制: Pos=%2 Vel=%3")
    //               .arg(jointIndex)
    //               .arg(targetPos, 0, 'f', 3)
    //               .arg(targetVel, 0, 'f', 3));
}

void MainWindow::onControlStatusChanged(bool running)
{
    QString status = running ? QStringLiteral("运行中") : QStringLiteral("已停止");
    appendLog(QStringLiteral("控制状态: %1").arg(status));
}

void MainWindow::adjustParameter(int index, float delta)
{
    // 索引对应: 0:d1, 1:d2, 2:d3, 3:k1, 4:k2, 5:k3
    QString paramName;
    float oldValue = 0.0f;
    float newValue = 0.0f;

    if (index < 3) {
        // d1, d2, d3: DH参数中的d值
        paramName = QString("d%1").arg(index + 1);
        oldValue = controlParams_.robotParams.dh[index].d;
        controlParams_.robotParams.dh[index].d += delta;
        newValue = controlParams_.robotParams.dh[index].d;
    } else if (index == 3) {
        paramName = "k1";
        oldValue = controlParams_.sliding_k1;
        controlParams_.sliding_k1 += delta;
        newValue = controlParams_.sliding_k1;
    } else if (index == 4) {
        paramName = "k2";
        oldValue = controlParams_.sliding_k2;
        controlParams_.sliding_k2 += delta;
        newValue = controlParams_.sliding_k2;
    } else if (index == 5) {
        paramName = "k3";
        oldValue = controlParams_.sliding_k3;
        controlParams_.sliding_k3 += delta;
        newValue = controlParams_.sliding_k3;
    }

    // 更新控制器参数
    if (robotController_) {
        robotController_->setControlParams(controlParams_);
    }

    appendLog(QStringLiteral("参数%1: %2 → %3 (Δ%4)")
                  .arg(paramName)
                  .arg(oldValue, 0, 'f', 3)
                  .arg(newValue, 0, 'f', 3)
                  .arg(delta, 0, 'f', 3));
}

void MainWindow::startDataRecording()
{
    // 创建logs目录（如果不存在）
    QDir logsDir("logs");
    if (!logsDir.exists()) {
        logsDir.mkpath(".");
        appendLog(QStringLiteral("创建日志目录: logs/"));
    }

    // 获取当前时间戳，用于文件名
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");

    // 为每个绘图窗口开始记录
    const char* plotNames[] = {
        "joint_position",
        "joint_velocity",
        "joint_torque",
        "end_effector_position"
    };

    for (int i = 0; i < 4; ++i) {
        if (plotWidgets_[i]) {
            QString filename = QString("logs/%1_%2.txt").arg(plotNames[i]).arg(timestamp);
            if (plotWidgets_[i]->startRecording(filename)) {
                appendLog(QStringLiteral("开始记录 %1 到文件: %2")
                              .arg(plotNames[i])
                              .arg(filename));
            } else {
                appendLog(QStringLiteral("无法开始记录 %1").arg(plotNames[i]));
            }
        }
    }
}
