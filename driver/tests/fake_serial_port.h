#pragma once

#include "bus/transport.h"

#include <algorithm>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

using Bytes = std::vector<uint8_t>;

// Plays the servos in tests: queue one reply per packet the bus will send (an empty reply
// means silence); set `echo` to mimic adapters that loop transmitted bytes back.
class FakeSerialPort : public Transport {
public:
    bool echo = false;           // adapter echoes our own transmission back
    std::deque<Bytes> replies;   // consumed one per write(); empty = no reply
    std::vector<Bytes> writes;   // every packet the bus sent, in order
    bool fail_writes = false;    // make write() report a transport failure

    bool write(const uint8_t* data, size_t length) override {
        if (fail_writes) {
            last_error_ = "fake write failure";
            return false;
        }
        const Bytes sent(data, data + length);
        writes.push_back(sent);
        if (echo) {
            rx_.insert(rx_.end(), sent.begin(), sent.end());
        }
        if (!replies.empty()) {
            rx_.insert(rx_.end(), replies.front().begin(), replies.front().end());
            replies.pop_front();
        }
        return true;
    }

    size_t read(uint8_t* buffer, size_t length) override {
        const size_t n = std::min(length, rx_.size());
        std::copy(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(n), buffer);
        rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(n));
        return n;
    }

    void flush() override { rx_.clear(); }

    const std::string& last_error() const override { return last_error_; }

private:
    Bytes rx_;
    std::string last_error_;
};

// Frames built independently of sts3215.cpp, so tests check against the datasheet
// rather than against the code under test.
inline Bytes frame(uint8_t id, uint8_t first, const Bytes& rest) {
    Bytes out{0xFF, 0xFF, id, static_cast<uint8_t>(rest.size() + 2), first};
    out.insert(out.end(), rest.begin(), rest.end());
    unsigned sum = 0;
    for (size_t i = 2; i < out.size(); ++i) sum += out[i];
    out.push_back(static_cast<uint8_t>(~sum & 0xFF));
    return out;
}

inline Bytes packet(uint8_t id, uint8_t instruction, const Bytes& params) {
    return frame(id, instruction, params);
}

inline Bytes servo_reply(uint8_t id, uint8_t error, const Bytes& params) {
    return frame(id, error, params);
}
