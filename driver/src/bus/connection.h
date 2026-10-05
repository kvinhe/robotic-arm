#pragma once

#include "bus/transport.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

// An open link to the servos: the USB serial port, or a simulated bus if one is installed.
struct Connection {
    std::unique_ptr<Transport> port;
    std::string name;
};

// `requested` may be empty to auto-detect the USB adapter.
bool open_connection(const std::string& requested, uint32_t baud_rate, Connection* out,
                     std::string* error);

// From now on, open_connection() returns what `make` builds instead of a serial port.
void use_simulated_bus(std::function<std::unique_ptr<Transport>()> make);
