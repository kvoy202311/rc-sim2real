#include "serial_comm/serial_port.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace kvoy {
namespace {

std::runtime_error make_errno_error(const std::string& prefix)
{
    return std::runtime_error(prefix + ": " + std::strerror(errno));
}

speed_t baud_to_termios(int baud_rate)
{
    switch (baud_rate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 576000: return B576000;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 2500000: return B2500000;
        case 3000000: return B3000000;
        case 3500000: return B3500000;
        case 4000000: return B4000000;
        default:
            throw std::runtime_error("Unsupported baud rate: " + std::to_string(baud_rate));
    }
}

void configure_fd(int fd, int baud_rate)
{
    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        throw make_errno_error("tcgetattr failed");
    }

    cfmakeraw(&tty);
    const speed_t baud = baud_to_termios(baud_rate);
    if (cfsetispeed(&tty, baud) != 0) {
        throw make_errno_error("cfsetispeed failed");
    }
    if (cfsetospeed(&tty, baud) != 0) {
        throw make_errno_error("cfsetospeed failed");
    }

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        throw make_errno_error("tcsetattr failed");
    }

    if (tcflush(fd, TCIOFLUSH) != 0) {
        throw make_errno_error("tcflush failed");
    }
}

} // namespace

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::is_open() const
{
    return fd_ >= 0;
}

void SerialPort::open(const std::string& path, int baud_rate, Mode mode)
{
    if (is_open()) {
        return;
    }

    const int access = mode == Mode::ReadWrite ? O_RDWR : O_RDONLY;
    const int fd = ::open(path.c_str(), access | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        throw make_errno_error("Cannot open serial port " + path);
    }

    try {
        configure_fd(fd, baud_rate);
    } catch (...) {
        ::close(fd);
        throw;
    }

    fd_ = fd;
}

void SerialPort::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

int SerialPort::wait_readable(int timeout_ms) const
{
    if (fd_ < 0) {
        errno = EBADF;
        return -1;
    }

    struct pollfd pfd {};
    pfd.fd = fd_;
    pfd.events = POLLIN | POLLERR | POLLHUP | POLLNVAL;

    const int ret = ::poll(&pfd, 1, timeout_ms);
    if (ret <= 0) {
        return ret;
    }
    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        errno = EIO;
        return -1;
    }
    return (pfd.revents & POLLIN) != 0 ? 1 : 0;
}

int SerialPort::read_nonblocking(uint8_t* data, std::size_t size)
{
    const ssize_t ret = ::read(fd_, data, size);
    if (ret >= 0) {
        return static_cast<int>(ret);
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return 0;
    }
    return -1;
}

int SerialPort::write_all(const uint8_t* data, std::size_t size, int timeout_ms)
{
    using clock = std::chrono::steady_clock;

    if (fd_ < 0) {
        errno = EBADF;
        return -1;
    }

    const auto deadline =
        timeout_ms >= 0
            ? clock::now() + std::chrono::milliseconds(timeout_ms)
            : clock::time_point::max();
    std::size_t total_written = 0;
    while (total_written < size) {
        const ssize_t ret = ::write(fd_, data + total_written, size - total_written);
        if (ret > 0) {
            total_written += static_cast<std::size_t>(ret);
            continue;
        }

        if (ret < 0 && errno == EINTR) {
            continue;
        }

        if (ret == 0 || (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
            const auto now = clock::now();
            if (timeout_ms >= 0 && now >= deadline) {
                errno = ETIMEDOUT;
                return -1;
            }

            int wait_ms = timeout_ms < 0
                ? -1
                : static_cast<int>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
            if (timeout_ms >= 0 && wait_ms <= 0) {
                wait_ms = 1;
            }

            struct pollfd pfd {};
            pfd.fd = fd_;
            pfd.events = POLLOUT | POLLERR | POLLHUP | POLLNVAL;
            const int poll_ret = ::poll(&pfd, 1, wait_ms);
            if (poll_ret == 0) {
                errno = ETIMEDOUT;
                return -1;
            }
            if (poll_ret < 0) {
                if (errno == EINTR) {
                    continue;
                }
                return -1;
            }
            if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                errno = EIO;
                return -1;
            }
            continue;
        }

        return -1;
    }

    return static_cast<int>(total_written);
}

} // namespace kvoy
