#pragma once

#include <cstdint>
#include <string>
#include <vector>

class Transport;

namespace sts3215 {

// Packets: 0xFF 0xFF ID LEN INSTRUCTION|ERROR PARAM... CHECKSUM,
// with LEN = params + 2 and CHECKSUM = ~(ID + LEN + INSTRUCTION + params).
constexpr uint8_t kInstPing = 0x01;
constexpr uint8_t kInstRead = 0x02;
constexpr uint8_t kInstWrite = 0x03;

// A packet sent to this ID is obeyed by every servo on the bus, and nobody
// replies. Only ever safe when exactly one servo is connected.
constexpr uint8_t kBroadcastId = 0xFE;

// Control table. Below 40 is EEPROM (survives power-off); the rest is RAM.
constexpr uint8_t kRegId = 5;               // EEPROM: this servo's ID, 0-253
constexpr uint8_t kRegBaudRate = 6;         // EEPROM: baud index, 0 = 1 Mbaud
constexpr uint8_t kRegLock = 55;            // EEPROM write lock: 0 unlocked, 1 locked
constexpr uint8_t kRegTorqueEnable = 40;
constexpr uint8_t kRegAcceleration = 41;    // units of 100 steps/s^2
constexpr uint8_t kRegGoalPosition = 42;    // 2 bytes
constexpr uint8_t kRegGoalSpeed = 46;       // 2 bytes, steps/s
constexpr uint8_t kRegTorqueLimit = 48;     // 2 bytes, 0.1 %
constexpr uint8_t kRegPresentPosition = 56; // 2 bytes

constexpr uint32_t kDefaultBaudRate = 1000000;  // factory setting of a new STS3215

// One servo bus, over an already-open port (a SerialPort, or a fake in tests).
class Bus {
public:
    explicit Bus(Transport& port) : port_(port) {}

    bool ping(uint8_t id);

    // Register access; false if the servo did not reply. 2-byte values are low byte first.
    bool write_u8(uint8_t id, uint8_t reg, uint8_t value);
    bool read_u8(uint8_t id, uint8_t reg, uint8_t* out);
    bool read_u16(uint8_t id, uint8_t reg, uint16_t* out);
    bool write_u16(uint8_t id, uint8_t reg, uint16_t value);

    // Pings every ID from 0 to 253 and returns the ones that answered.
    std::vector<uint8_t> scan();

    // Unlock EEPROM -> write the new ID -> lock EEPROM again (addressing the
    // lock write to the NEW id, because the servo has already renamed itself).
    bool set_id(uint8_t current_id, uint8_t new_id);

    const std::string& last_error() const { return last_error_; }

private:
    // Sends one packet and waits for the reply. `params` may be empty.
    // On success `reply_params` holds whatever the servo sent back.
    bool transact(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params,
                  std::vector<uint8_t>* reply_params, size_t expected_param_count);

    Transport& port_;
    std::string last_error_;
};

}  // namespace sts3215
