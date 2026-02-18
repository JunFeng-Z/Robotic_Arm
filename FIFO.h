#ifndef FIFO_H
#define FIFO_H

#include <iostream>
#include <cstring>
#include <stdexcept>
#include <vector>

class FIFO
{
public:
    // 属性
    std::vector<unsigned char> Buffer;  // 存放内存的数组
    int DataCount;                      // 写入数据大小
    int DataStart;                      // 数据起始索引
    int DataEnd;                        // 数据结束索引

    // 构造函数
    FIFO(int bufferSize)
    {
        DataCount = 0;
        DataStart = 0;
        DataEnd = 0;
        Buffer.resize(bufferSize);
    }

    // 索引器 - 通过重载[]操作符实现
    unsigned char operator[](int index) const
    {
        if (index >= DataCount)
            throw std::runtime_error("环形缓冲区异常，索引溢出");

        if (DataStart + index < static_cast<int>(Buffer.size()))
        {
            return Buffer[DataStart + index];
        }
        else
        {
            return Buffer[(DataStart + index) - static_cast<int>(Buffer.size())];
        }
    }

    // 获得当前写入的字节数
    int GetDataCount()
    {
        return DataCount;
    }

    // 获得剩余的字节数
    int GetReserveCount()
    {
        return static_cast<int>(Buffer.size()) - DataCount;
    }

    // 清空所有数据
    void Clear()
    {
        DataCount = 0;
        DataStart = 0;
        DataEnd = 0;
    }

    // 清空指定大小的数据
    void Clear(int count)
    {
        if (count >= DataCount) // 如果需要清理的数据大于现有数据大小，则全部清理
        {
            DataCount = 0;
            DataStart = 0;
            DataEnd = 0;
        }
        else
        {
            if (DataStart + count >= static_cast<int>(Buffer.size()))
            {
                DataStart = (DataStart + count) - static_cast<int>(Buffer.size());
            }
            else
            {
                DataStart += count;
            }
            DataCount -= count;
        }
    }

    // 写入缓冲区（带偏移和计数）
    void WriteBuffer(const unsigned char* buffer, int offset, int count)
    {
        int reserveCount = static_cast<int>(Buffer.size()) - DataCount;
        if (reserveCount >= count)             // 可用空间够使用
        {
            if (DataEnd + count < static_cast<int>(Buffer.size()))      // 数据没到结尾
            {
                std::memcpy(&Buffer[DataEnd], buffer + offset, count);
                DataEnd += count;
                DataCount += count;
            }
            else      // 数据结束索引超出结尾 循环到开始
            {
                std::cout << "缓存重新开始...." << std::endl;
                int overflowIndexLength = (DataEnd + count) - static_cast<int>(Buffer.size());   // 超出索引长度
                int endPushIndexLength = count - overflowIndexLength;       // 填充在末尾的数据长度

                std::memcpy(&Buffer[DataEnd], buffer + offset, endPushIndexLength);
                DataEnd = 0;
                offset += endPushIndexLength;
                DataCount += endPushIndexLength;

                if (overflowIndexLength != 0)
                {
                    std::memcpy(&Buffer[DataEnd], buffer + offset, overflowIndexLength);
                }

                DataEnd += overflowIndexLength;                   // 结束索引
                DataCount += overflowIndexLength;                  // 缓存大小
            }
        }
        else
        {
            // 缓存溢出，不处理
            // 可以添加日志或异常处理
        }
    }

    // 写入缓冲区（整个数组）
    void WriteBuffer(const unsigned char* buffer, int count)
    {
        WriteBuffer(buffer, 0, count);
    }

    // 写入缓冲区（使用std::vector）
    void WriteBuffer(const std::vector<unsigned char>& buffer)
    {
        WriteBuffer(buffer.data(), 0, static_cast<int>(buffer.size()));
    }

    // 读取缓冲区
    void ReadBuffer(unsigned char* targetBytes, int offset, int count)
    {
        if (count > DataCount)
            throw std::runtime_error("环形缓冲区异常，读取长度大于数据长度");

        if (DataStart + count < static_cast<int>(Buffer.size()))
        {
            std::memcpy(targetBytes + offset, &Buffer[DataStart], count);
        }
        else
        {
            int overflowIndexLength = (DataStart + count) - static_cast<int>(Buffer.size());  // 超出索引长度
            int endPushIndexLength = count - overflowIndexLength;       // 填充在末尾的数据长度

            std::memcpy(targetBytes + offset, &Buffer[DataStart], endPushIndexLength);
            offset += endPushIndexLength;

            if (overflowIndexLength != 0)
            {
                std::memcpy(targetBytes + offset, &Buffer[0], overflowIndexLength);
            }
        }
    }

    // 获取缓冲区大小
    int GetBufferSize() const
    {
        return static_cast<int>(Buffer.size());
    }

    // 获取数据起始位置的指针（用于直接访问）
    const unsigned char* GetDataStartPointer() const
    {
        return Buffer.data() + DataStart;
    }

    // 获取数据结束位置的指针
    const unsigned char* GetDataEndPointer() const
    {
        return Buffer.data() + DataEnd;
    }

    // 析构函数
    ~FIFO() = default;

private:
    // 禁用复制构造函数和赋值运算符
    FIFO(const FIFO&) = delete;
    FIFO& operator=(const FIFO&) = delete;
};

#endif // FIFO_H
