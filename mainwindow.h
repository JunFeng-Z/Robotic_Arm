#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "FIFO.h"
#include "robot_common.h"

#include <QMainWindow>
#include <QMutex>
#include <QWaitCondition>

// 前向声明
class JointState;

class QCloseEvent;
class QComboBox;
class QGroupBox;
class QPushButton;
class QSpinBox;
class QTextEdit;
class QThread;
class QTimer;
class SerialPort;
class SerialRxWorker;
class CanParserWorker;
class RobotController;
class PlotWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
    void onOpenSerialClicked();
    void onRefreshDevicesClicked();
    void onSetDurationClicked();
    void onEnableClicked();
    void onDisableClicked();
    void onJointStateUpdated(const JointState &state);
    void onInitTrackClicked();
    void onTeachClicked();
    void onRunAlgoClicked();
    void onControlCommandSent(int jointIndex, float targetPos, float targetVel);
    void onTorqueCommandSent(int jointIndex, float torque);
    void onControlStatusChanged(bool running);
    void onClearDataClicked();
    void onCleartextDataClicked();
    void onPausePlotClicked();
    void onResumePlotClicked();

private:
    QGroupBox *buildConnectionPanel();
    QGroupBox *buildRunPanel();
    QGroupBox *buildTunePanel();

    void startWorkers();
    void stopWorkers();
    void appendLog(const QString &text);

    void EnableMotor();
    void DisableMotor();

    void adjustParameter(int index, float delta);
    void startDataRecording();


    QTextEdit *logText_ = nullptr;

    QVector<QComboBox*> tuneStepBoxes_;
    QVector<QPushButton*> tunePlusBtns_;
    QVector<QPushButton*> tuneMinusBtns_;

    QComboBox *portCombo_ = nullptr;
    QComboBox *baudCombo_ = nullptr;
    QComboBox *parityCombo_ = nullptr;
    QComboBox *dataBitsCombo_ = nullptr;
    QComboBox *timeoutCombo_ = nullptr;

    QPushButton *openBtn_ = nullptr;
    QPushButton *refreshBtn_ = nullptr;
    QPushButton *enableBtn_ = nullptr;
    QPushButton *disableBtn_ = nullptr;
    QPushButton *cleartextBtn_ = nullptr;

    QPushButton *initTrackBtn_ = nullptr;
    QPushButton *teachBtn_ = nullptr;
    QPushButton *clearDataBtn_ = nullptr;
    QPushButton *pausePlotBtn_ = nullptr;
    QPushButton *resumePlotBtn_ = nullptr;
    QPushButton *setDurationBtn_ = nullptr;
    QPushButton *runAlgoBtn_ = nullptr;
    QSpinBox *secondsBox_ = nullptr;

    FIFO canRxFifo_{4096};
    FIFO uartRxFifo_{4096};
    QMutex canRxMutex_;
    QMutex uartRxMutex_;
    QWaitCondition canDataReady_;

    SerialPort *serialPort_ = nullptr;
    QThread *serialThread_ = nullptr;
    QThread *canParserThread_ = nullptr;
    SerialRxWorker *serialRxWorker_ = nullptr;
    CanParserWorker *canParserWorker_ = nullptr;
    RobotController *robotController_ = nullptr;

    // 绘图窗口（4个象限）
    PlotWidget *plotWidgets_[4] = {nullptr, nullptr, nullptr, nullptr};

    ControlParams controlParams_;

private:
    // 模拟数据生成
    QTimer *simulationTimer_ = nullptr;
    float simulationTime_ = 0.0f;
    void onSimulationTimer();
};

#endif // MAINWINDOW_H
