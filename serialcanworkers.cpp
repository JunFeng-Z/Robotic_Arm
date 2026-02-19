#include "serialcanworkers.h"

#include <QMutexLocker>
#include <QThread>

#include <algorithm>
//////////////////////////////////SerialRxWorker class Start////////////////////////////////////////
SerialRxWorker::SerialRxWorker(SerialPort *serial, FIFO *canRxFifo, FIFO *uartRxFifo,
                               QMutex *canMutex, QMutex *uartMutex,
                               QObject *parent)
    : QObject(parent)
    , serial_(serial)
    , canRxFifo_(canRxFifo)
    , uartRxFifo_(uartRxFifo)
    , canMutex_(canMutex)
    , uartMutex_(uartMutex)
{
}

void SerialRxWorker::start()
{
    if (!serial_ || !canRxFifo_ || !uartRxFifo_ || !canMutex_ || !uartMutex_) {
        emit logMessage(QStringLiteral("串口接收线程启动失败：依赖为空"));
        return;
    }

    running_.store(true);
    emit logMessage(QStringLiteral("串口接收线程已启动"));

    std::vector<uint8_t> streamBuf;
    streamBuf.reserve(512);

    uint8_t recvBuf[64];

    while (running_.load())
    {
        const ssize_t recvLen = serial_->recv(recvBuf, sizeof(recvBuf));

        if (recvLen <= 0) {
            QThread::msleep(1);
            continue;
        }

        streamBuf.insert(streamBuf.end(), recvBuf, recvBuf + recvLen);

        while (true)
        {
            std::array<uint8_t,16> canFrame{};
            auto result = tryExtractCanFrame(streamBuf, canFrame);

            if (result == ExtractResult::GotFrame)
            {
                uint8_t frameType = canFrame[1];

                // 只保留状态帧
                // AA 12 08 01 00 00 00 FF FF FF FF FF FF FF FD 55 ACK帧
                // AA 11 08 00 00 00 00 01 7F FF 7F F7 FF 1B 19 55 状态帧
                if (frameType != 0x11)
                {
                    // 如果想调试可以打印
                    emit logMessage("ACK filtered");
                    continue;
                }
                QMutexLocker canLock(canMutex_);
                canRxFifo_->WriteBuffer(canFrame.data(), 0, 16);
            }
            else if (result == ExtractResult::NeedMoreData)
            {
                // 半帧，绝不能动buffer
                break;
            }
            else // NoFrame
            {
                auto headIter = std::find(streamBuf.begin(), streamBuf.end(), 0xAA);

                if (headIter == streamBuf.end())
                {
                    if (!streamBuf.empty())
                    {
                        QMutexLocker uartLock(uartMutex_);
                        uartRxFifo_->WriteBuffer(streamBuf.data(),
                                                 0,
                                                 static_cast<int>(streamBuf.size()));
                        streamBuf.clear();
                    }
                }
                else if (headIter != streamBuf.begin())
                {
                    // AA前面的才是UART
                    size_t uartLen = std::distance(streamBuf.begin(), headIter);

                    QMutexLocker uartLock(uartMutex_);
                    uartRxFifo_->WriteBuffer(streamBuf.data(),
                                             0,
                                             static_cast<int>(uartLen));

                    streamBuf.erase(streamBuf.begin(), headIter);
                }

                break;
            }
        }
    }

    emit logMessage(QStringLiteral("串口接收线程已停止"));
}



void SerialRxWorker::stop()
{
    running_.store(false);
}
SerialRxWorker::ExtractResult SerialRxWorker::tryExtractCanFrame(std::vector<uint8_t> &streamBuf,
                                   std::array<uint8_t, 16> &canFrame) const
{
    constexpr uint8_t HEAD = 0xAA;
    constexpr uint8_t TAIL = 0x55;
    constexpr size_t FRAME_LEN = 16;

    auto headIter = std::find(streamBuf.begin(), streamBuf.end(), HEAD);

    // 没有帧头 → 全是UART数据
    if (headIter == streamBuf.end())
    {
        return ExtractResult::NoFrame;
    }

    size_t headPos = std::distance(streamBuf.begin(), headIter);

    // 帧不完整
    if (streamBuf.size() - headPos < FRAME_LEN)
    {
        return ExtractResult::NeedMoreData;
    }

    // 完整帧判断
    if (*(headIter + FRAME_LEN - 1) == TAIL)
    {
        std::copy(headIter,
                  headIter + FRAME_LEN,
                  canFrame.begin());

        streamBuf.erase(streamBuf.begin(), headIter + FRAME_LEN);
        return ExtractResult::GotFrame;
    }

    // 假帧头
    streamBuf.erase(streamBuf.begin(), headIter + 1);
    return ExtractResult::NoFrame;
}
//////////////////////////////////SerialRxWorker class end////////////////////////////////////////

//////////////////////////////////CanParserWorker class Satrt////////////////////////////////////////

CanParserWorker::CanParserWorker(FIFO *canRxFifo, QMutex *canMutex, QObject *parent)
    : QObject(parent)
    , canRxFifo_(canRxFifo)
    , canMutex_(canMutex)
{
}

void CanParserWorker::start()
{
    if (!canRxFifo_ || !canMutex_) {
        emit logMessage(QStringLiteral("CAN解析线程启动失败：依赖为空"));
        return;
    }

    running_.store(true);
    emit logMessage(QStringLiteral("CAN解析线程已启动"));

    std::array<unsigned char, 16> frame{};
    while (running_.load()) {
        bool hasFrame = false;
        {
            QMutexLocker locker(canMutex_);
            if (canRxFifo_->GetDataCount() >= 16) {
                canRxFifo_->ReadBuffer(frame.data(), 0, 16);
                canRxFifo_->Clear(16);
                hasFrame = true;
            }
        }

        if (!hasFrame) {
            QThread::msleep(1);
            continue;
        }

        QString frameStr = "CAN Frame: ";

        for (int i = 0; i < 16; i++)
        {
            frameStr += QString("%1 ").arg(frame[i], 2, 16, QLatin1Char('0')).toUpper();
        }

        emit logMessage(frameStr);

        const uint8_t motorId = frame[7];
        float pos = byt2Float(frame[8], frame[9],MotorField::Pos);
        float vel = byt2Float(frame[10], frame[11],MotorField::Vel);

        int jointIndex = -1;
        if (motorId == 17) {
            jointIndex = 1;
        } else if (motorId == 18) {
            jointIndex = 2;
        } else if (motorId == 19) {
            jointIndex = 3;
            pos = -pos;
            vel = -vel;
        }
        if (jointIndex > 0) {
            emit jointStateUpdated(jointIndex, pos, vel);
        } else {
            emit logMessage(QStringLiteral("CAN解析收到未知电机ID: %1").arg(motorId));
        }
    }

    emit logMessage(QStringLiteral("CAN解析线程已停止"));
}

float CanParserWorker::byt2Float(uint8_t high,
                                uint8_t low,
                                MotorField field) const
{
    int intValue = 0;

    switch (field)
    {
    case MotorField::Pos:
    {
        intValue = (static_cast<int>(high) << 8) | low;
        return uintToFloat(intValue, -12.5f, 12.5f, 16);
    }

    case MotorField::Vel:
    {
        intValue = (static_cast<int>(high) << 4) | (low >> 4);
        return uintToFloat(intValue, -30.0f, 30.0f, 12);
    }

    case MotorField::Torque:
    {
        intValue = ((high & 0x0F) << 8) | low;
        return uintToFloat(intValue, -10.0f, 10.0f, 12);
    }
    }

    throw std::invalid_argument("Unknown MotorField");
}

//将整数转化成浮点数
float CanParserWorker::uintToFloat(int x_int, float x_min, float x_max, int bits) const
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void CanParserWorker::stop()
{
    running_.store(false);
}


//////////////////////////////////CanParserWorker class end////////////////////////////////////////
