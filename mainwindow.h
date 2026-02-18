#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class QComboBox;
class QGroupBox;
class QPushButton;
class QSpinBox;
class QTextEdit;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

private slots:
    void onOpenSerialClicked();
    void onRefreshDevicesClicked();
    void onSetDurationClicked();

private:
    QGroupBox *buildConnectionPanel();
    QGroupBox *buildRunPanel();
    QGroupBox *buildTunePanel();

    QTextEdit *logText_ = nullptr;

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
};

#endif // MAINWINDOW_H
