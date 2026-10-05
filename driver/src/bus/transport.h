#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

// Bytes in, bytes out. Bus talks through this so tests can swap in a fake port.
class Transport {
public:
    virtual ~Transport() = default;
    virtual bool write(const uint8_t* data, size_t length) = 0;
    virtual size_t read(uint8_t* buffer, size_t length) = 0;   // returns bytes read
    virtual void flush() = 0;
    virtual const std::string& last_error() const = 0;
};
