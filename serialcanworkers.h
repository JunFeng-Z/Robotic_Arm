#ifndef SERIAL_CAN_WORKERS_H
#define SERIAL_CAN_WORKERS_H

#include "FIFO.h"
#include "SerialPort.h"

#include <QObject>
#include <QMutex>
#include <atomic>
#include <array>
#include <cstdint>


class SerialRxWorker : public QObject
{
    Q_OBJECT
public:
    SerialRxWorker(SerialPort *serial, FIFO *canRxFifo, FIFO *uartRxFifo,
                   QMutex *canMutex, QMutex *uartMutex,
                   QObject *parent = nullptr);
    enum class ExtractResult
    {
        GotFrame,
        NeedMoreData,
        NoFrame
    };


public slots:
    void start();
    void stop();

signals:
    void logMessage(const QString &message);

private:
    SerialRxWorker::ExtractResult  tryExtractCanFrame(std::vector<uint8_t> &streamBuf,
                            std::array<uint8_t, 16> &canFrame) const;

    SerialPort *serial_ = nullptr;
    FIFO *canRxFifo_ = nullptr;
    FIFO *uartRxFifo_ = nullptr;
    QMutex *canMutex_ = nullptr;
    QMutex *uartMutex_ = nullptr;
    std::atomic_bool running_{false};

};

class CanParserWorker : public QObject
{
    Q_OBJECT
public:
    CanParserWorker(FIFO *canRxFifo, QMutex *canMutex, QObject *parent = nullptr);

public slots:
    void start();
    void stop();

signals:
    void jointStateUpdated(int jointIndex, float position, float velocity);
    void logMessage(const QString &message);

private:
    enum class MotorField
    {
        Pos,
        Vel,
        Torque
    };

    float byt2Float(uint8_t high,uint8_t low,MotorField field) const;

    float uintToFloat(int x_int, float x_min, float x_max, int bits) const;
    FIFO *canRxFifo_ = nullptr;
    QMutex *canMutex_ = nullptr;
    std::atomic_bool running_{false};
};

#endif // SERIAL_CAN_WORKERS_H
