#include "mainwindow.h"

#include "plotwidget.h"

#include <QComboBox>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTextEdit>
#include <QVBoxLayout>

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
    portCombo_->addItems({"COM1", "COM2", "COM3"});

    baudCombo_ = new QComboBox;
    baudCombo_->addItems({"9600", "57600", "115200"});

    parityCombo_ = new QComboBox;
    parityCombo_->addItems({QStringLiteral("None"), QStringLiteral("Odd"), QStringLiteral("Even")});

    dataBitsCombo_ = new QComboBox;
    dataBitsCombo_->addItems({"8", "7"});

    timeoutCombo_ = new QComboBox;
    timeoutCombo_->addItems({"500", "1000", "2000"});

    openBtn_ = new QPushButton(QStringLiteral("打开串口"));
    refreshBtn_ = new QPushButton(QStringLiteral("刷新设备"));
    enableBtn_ = new QPushButton(QStringLiteral("使能电机"));
    disableBtn_ = new QPushButton(QStringLiteral("失能电机"));

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

    clearDataBtn_ = new QPushButton(QStringLiteral("清除数据"));
    setDurationBtn_ = new QPushButton(QStringLiteral("设置时长"));
    runAlgoBtn_ = new QPushButton(QStringLiteral("运行算法"));

    layout->addWidget(clearDataBtn_);
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
        auto *combo = new QComboBox;
        combo->addItems({"1", "2", "5", "10"});
        layout->addWidget(new QLabel(names[i]), i, 0);
        layout->addWidget(new QPushButton(plus[i]), i, 1);
        layout->addWidget(new QPushButton(minus[i]), i, 2);
        layout->addWidget(combo, i, 3);
    }

    layout->addWidget(new QPushButton(QStringLiteral("手动刷新显示")), names.size(), 1, 1, 2);
    layout->addWidget(new QPushButton(QStringLiteral("测试键")), names.size(), 3);

    return group;
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("3D Camera 控制台"));
    resize(1600, 900);

    auto *central = new QWidget;
    auto *root = new QHBoxLayout(central);
    root->setSpacing(12);

    auto *leftPane = new QWidget;
    auto *leftLayout = new QVBoxLayout(leftPane);
    auto *plotGrid = new QGridLayout;
    plotGrid->setSpacing(10);

    for (int i = 0; i < 4; ++i) {
        plotGrid->addWidget(new PlotWidget, i / 2, i % 2);
    }

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

    topRight->addWidget(logGroup, 3);
    topRight->addWidget(buildConnectionPanel(), 2);

    auto *bottomRight = new QHBoxLayout;
    bottomRight->addWidget(buildRunPanel(), 1);
    bottomRight->addWidget(buildTunePanel(), 1);

    rightLayout->addLayout(topRight, 1);
    rightLayout->addLayout(bottomRight, 1);

    root->addWidget(leftPane, 3);
    root->addWidget(rightPane, 2);

    setCentralWidget(central);

    connect(openBtn_, &QPushButton::clicked, this, &MainWindow::onOpenSerialClicked);
    connect(refreshBtn_, &QPushButton::clicked, this, &MainWindow::onRefreshDevicesClicked);
    connect(setDurationBtn_, &QPushButton::clicked, this, &MainWindow::onSetDurationClicked);
}

void MainWindow::onOpenSerialClicked()
{
    logText_->append(QStringLiteral("打开串口：%1, 波特率：%2")
                         .arg(portCombo_->currentText(), baudCombo_->currentText()));
}

void MainWindow::onRefreshDevicesClicked()
{
    logText_->append(QStringLiteral("已刷新设备列表（示例逻辑）"));
}

void MainWindow::onSetDurationClicked()
{
    logText_->append(QStringLiteral("总时长设置为 %1 秒").arg(secondsBox_->value()));
}
