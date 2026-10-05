#pragma once

#include "bus/transport.h"

#include <cstdint>
#include <string>
#include <vector>

// A USB serial port, on termios (macOS and Linux).
class SerialPort : public Transport {
public:
    SerialPort() = default;
    ~SerialPort() override;
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // An empty name means "the one USB serial adapter that is plugged in".
    bool open(const std::string& name, uint32_t baud_rate);
    void close();
    const std::string& name() const { return name_; }

    bool write(const uint8_t* data, size_t length) override;
    size_t read(uint8_t* buffer, size_t length) override;
    void flush() override;
    const std::string& last_error() const override { return last_error_; }

    // Ports that look like a USB serial adapter, sorted.
    static std::vector<std::string> list_adapters();

private:
    int fd_ = -1;
    std::string name_;
    std::string last_error_;
};

// True for "/dev/..." -- a port, not a command.
inline bool looks_like_port(const std::string& arg) { return arg.rfind("/dev/", 0) == 0; }

// Picks the only adapter found. Refuses to guess between several: writing servo
// packets to the wrong device is not harmless.
inline bool choose_adapter(const std::vector<std::string>& found, std::string* port,
                           std::string* error) {
    if (found.size() == 1) {
        *port = found[0];
        return true;
    }
    if (found.empty()) {
        *error = "no USB serial adapter found -- is the servo board plugged in?";
        return false;
    }
    *error = "found " + std::to_string(found.size()) + " serial adapters:";
    for (const std::string& name : found) *error += "\n  " + name;
    *error += "\nname the one to use first, e.g. servo_tool " + found[0] + " scan";
    return false;
}
