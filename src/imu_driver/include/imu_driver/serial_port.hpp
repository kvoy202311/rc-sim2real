#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace kvoy {

class SerialPort
{
public:
    enum class Mode
    {
        ReadOnly,
        ReadWrite,
    };

    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool is_open() const;
    void open(const std::string& path, int baud_rate, Mode mode);
    void close();
    int read_nonblocking(uint8_t* data, std::size_t size);

private:
    int fd_{-1};
};

} // namespace kvoy
