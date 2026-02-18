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

namespace {
QGroupBox *buildConnectionPanel()
{
    auto *group = new QGroupBox(QStringLiteral("连接设备"));
    auto *layout = new QGridLayout(group);
    layout->setHorizontalSpacing(12);
    layout->setVerticalSpacing(8);
    layout->setRowStretch(6, 1);

    auto *portLabel = new QLabel(QStringLiteral("串口号:"));
    auto *portCombo = new QComboBox;
    portCombo->addItems({"COM1", "COM2", "COM3"});

    auto *baudCombo = new QComboBox;
    baudCombo->addItems({"9600", "57600", "115200"});

    auto *parityCombo = new QComboBox;
    parityCombo->addItems({QStringLiteral("None"), QStringLiteral("Odd"), QStringLiteral("Even")});

    auto *dataBitsCombo = new QComboBox;
    dataBitsCombo->addItems({"8", "7"});

    auto *timeoutCombo = new QComboBox;
    timeoutCombo->addItems({"500", "1000", "2000"});

    auto *openBtn = new QPushButton(QStringLiteral("打开串口"));
    auto *refreshBtn = new QPushButton(QStringLiteral("刷新设备"));
    auto *enableBtn = new QPushButton(QStringLiteral("使能电机"));
    auto *disableBtn = new QPushButton(QStringLiteral("失能电机"));

    layout->addWidget(portLabel, 0, 0);
    layout->addWidget(portCombo, 1, 0);
    layout->addWidget(openBtn, 1, 1, 1, 1);

    layout->addWidget(baudCombo, 2, 0);
    layout->addWidget(refreshBtn, 2, 1);

    layout->addWidget(parityCombo, 3, 0);
    layout->addWidget(enableBtn, 3, 1);

    layout->addWidget(dataBitsCombo, 4, 0);
    layout->addWidget(disableBtn, 4, 1);

    layout->addWidget(timeoutCombo, 5, 0);
    layout->setAlignment(Qt::AlignTop);

    return group;
}

QGroupBox *buildRunPanel()
{
    auto *group = new QGroupBox(QStringLiteral("运行设置"));
    auto *layout = new QVBoxLayout(group);
    layout->setSpacing(10);

    auto *row1 = new QHBoxLayout;
    row1->addWidget(new QPushButton(QStringLiteral("轨迹跟踪初始化")));
    row1->addWidget(new QPushButton(QStringLiteral("示教")));
    layout->addLayout(row1);

    layout->addWidget(new QPushButton(QStringLiteral("清除数据")));
    layout->addWidget(new QPushButton(QStringLiteral("设置时长")));
    layout->addWidget(new QPushButton(QStringLiteral("运行算法")));

    auto *durationRow = new QHBoxLayout;
    durationRow->addStretch();
    durationRow->addWidget(new QLabel(QStringLiteral("总时长")));
    auto *secondsBox = new QSpinBox;
    secondsBox->setRange(1, 120);
    secondsBox->setValue(6);
    durationRow->addWidget(secondsBox);
    durationRow->addWidget(new QLabel(QStringLiteral("秒")));
    durationRow->addStretch();

    layout->addSpacing(8);
    layout->addLayout(durationRow);

    return group;
}

QGroupBox *buildTunePanel()
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
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("机械臂运动控制台"));
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
    auto *logText = new QTextEdit;
    logText->setReadOnly(true);
    logText->setPlainText(
        QStringLiteral("嗨连接串口并使能电机\n\n如果想跟踪期望轨迹请先初始化，并等待初始化完成\n......"));
    logLayout->addWidget(logText);

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

}
