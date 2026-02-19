#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "FIFO.h"

#include <QMainWindow>
#include <QMutex>

class QCloseEvent;
class QComboBox;
class QGroupBox;
class QPushButton;
class QSpinBox;
class QTextEdit;
class QThread;
class SerialPort;
class SerialRxWorker;
class CanParserWorker;

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
    void onJointStateUpdated(int jointIndex, float position, float velocity);

private:
    QGroupBox *buildConnectionPanel();
    QGroupBox *buildRunPanel();
    QGroupBox *buildTunePanel();

    void startWorkers();
    void stopWorkers();
    void appendLog(const QString &text);

    void EnableMotor();
    void DisableMotor();


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

    QPushButton *initTrackBtn_ = nullptr;
    QPushButton *teachBtn_ = nullptr;
    QPushButton *clearDataBtn_ = nullptr;
    QPushButton *setDurationBtn_ = nullptr;
    QPushButton *runAlgoBtn_ = nullptr;
    QSpinBox *secondsBox_ = nullptr;

    FIFO canRxFifo_{4096};
    FIFO uartRxFifo_{4096};
    QMutex canRxMutex_;
    QMutex uartRxMutex_;

    SerialPort *serialPort_ = nullptr;
    QThread *serialThread_ = nullptr;
    QThread *canParserThread_ = nullptr;
    SerialRxWorker *serialRxWorker_ = nullptr;
    CanParserWorker *canParserWorker_ = nullptr;
};

#endif // MAINWINDOW_H
