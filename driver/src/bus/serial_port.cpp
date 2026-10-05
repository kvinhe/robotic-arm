#include "bus/serial_port.h"

#include <algorithm>
#include <cerrno>
#include <cstring>

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#endif

namespace {

constexpr int kFirstByteTimeoutMs = 50;   // a servo that is there answers within ~1 ms
// Silence that ends a read once a reply has started. Every transaction pays it, so it
// caps the bus rate; 3 ms is plenty for a reply that arrives in one USB frame.
constexpr int kInterByteTimeoutMs = 3;
constexpr int kWriteTimeoutMs = 100;

std::string errno_message(const std::string& prefix) {
    return prefix + ": " + std::strerror(errno);
}

bool wait_for(int fd, short events, int timeout_ms) {
    pollfd p{fd, events, 0};
    int ready;
    do {
        ready = poll(&p, 1, timeout_ms);
    } while (ready < 0 && errno == EINTR);
    return ready > 0;
}

}  // namespace

std::vector<std::string> SerialPort::list_adapters() {
    static const char* const kPrefixes[] = {"cu.usbmodem", "cu.usbserial", "cu.wchusbserial",
                                            "ttyUSB", "ttyACM"};
    std::vector<std::string> found;
    DIR* dev = opendir("/dev");
    if (!dev) return found;
    while (const dirent* entry = readdir(dev)) {
        const std::string name = entry->d_name;
        for (const char* prefix : kPrefixes) {
            if (name.rfind(prefix, 0) == 0) found.push_back("/dev/" + name);
        }
    }
    closedir(dev);
    std::sort(found.begin(), found.end());
    return found;
}

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& name, uint32_t baud_rate) {
    close();
    name_ = name;
    if (name_.empty() && !choose_adapter(list_adapters(), &name_, &last_error_)) return false;

    // Non-blocking, so open() does not wait for a carrier-detect line the adapter lacks
    // (the same reason to use /dev/cu.* rather than /dev/tty.* on macOS).
    const int fd = ::open(name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        last_error_ = errno_message("cannot open " + name_);
        return false;
    }
    auto fail = [&](const std::string& what) {
        last_error_ = errno_message(what);
        ::close(fd);
        return false;
    };

    if (ioctl(fd, TIOCEXCL) < 0) return fail("cannot get exclusive access to " + name_);

    termios tio{};
    if (tcgetattr(fd, &tio) < 0) return fail("tcgetattr failed");
    cfmakeraw(&tio);
    tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS | HUPCL);
    tio.c_cflag |= CS8 | CLOCAL | CREAD;   // 8N1, no flow control
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

#ifdef __APPLE__
    // macOS termios only takes the classic rates; 1 Mbaud is set with IOSSIOSPEED below.
    cfsetspeed(&tio, B9600);
    if (tcsetattr(fd, TCSANOW, &tio) < 0) return fail("tcsetattr failed");
    speed_t speed = baud_rate;
    if (ioctl(fd, IOSSIOSPEED, &speed) < 0) return fail("cannot set baud rate");
#else
    cfsetspeed(&tio, baud_rate == 1000000 ? B1000000 : B115200);
    if (tcsetattr(fd, TCSANOW, &tio) < 0) return fail("tcsetattr failed");
#endif

    int lines = TIOCM_DTR | TIOCM_RTS;   // hold DTR/RTS low
    ioctl(fd, TIOCMBIC, &lines);

    fd_ = fd;
    flush();
    last_error_.clear();
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
}

bool SerialPort::write(const uint8_t* data, size_t length) {
    size_t sent = 0;
    while (sent < length) {
        const ssize_t n = ::write(fd_, data + sent, length - sent);
        if (n > 0) {
            sent += static_cast<size_t>(n);
        } else if (n < 0 && errno != EAGAIN && errno != EINTR) {
            last_error_ = errno_message("write failed");
            return false;
        } else if (!wait_for(fd_, POLLOUT, kWriteTimeoutMs)) {
            last_error_ = "write timed out";
            return false;
        }
    }
    return true;
}

size_t SerialPort::read(uint8_t* buffer, size_t length) {
    size_t got = 0;
    int timeout_ms = kFirstByteTimeoutMs;
    while (got < length && wait_for(fd_, POLLIN, timeout_ms)) {
        const ssize_t n = ::read(fd_, buffer + got, length - got);
        if (n > 0) {
            got += static_cast<size_t>(n);
            timeout_ms = kInterByteTimeoutMs;
        } else if (n == 0 || (errno != EAGAIN && errno != EINTR)) {
            last_error_ = n == 0 ? "port closed" : errno_message("read failed");
            break;
        }
    }
    return got;
}

void SerialPort::flush() {
    if (fd_ >= 0) tcflush(fd_, TCIOFLUSH);
}
