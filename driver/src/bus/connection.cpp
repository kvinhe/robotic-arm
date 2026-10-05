#include "bus/connection.h"

#include "bus/serial_port.h"

namespace {
std::function<std::unique_ptr<Transport>()> g_simulated;
}

void use_simulated_bus(std::function<std::unique_ptr<Transport>()> make) {
    g_simulated = std::move(make);
}

bool open_connection(const std::string& requested, uint32_t baud_rate, Connection* out,
                     std::string* error) {
    if (g_simulated) {
        out->port = g_simulated();
        out->name = "sim";
        return true;
    }
    auto port = std::make_unique<SerialPort>();
    if (!port->open(requested, baud_rate)) {
        *error = port->last_error();
        return false;
    }
    out->name = port->name();
    out->port = std::move(port);
    return true;
}
