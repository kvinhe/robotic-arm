#include "sts3215.h"

#include "transport.h"

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>

namespace sts3215 {
namespace {

uint8_t checksum(const uint8_t* bytes, size_t length) {
    uint32_t sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum += bytes[i];
    }
    return static_cast<uint8_t>(~sum & 0xFF);
}

// Builds 0xFF 0xFF ID LEN INST PARAM... CHECKSUM.
std::vector<uint8_t> build_packet(uint8_t id, uint8_t instruction,
                                  const std::vector<uint8_t>& params) {
    std::vector<uint8_t> packet;
    packet.reserve(params.size() + 6);
    packet.push_back(0xFF);
    packet.push_back(0xFF);
    packet.push_back(id);
    packet.push_back(static_cast<uint8_t>(params.size() + 2));  // LEN
    packet.push_back(instruction);
    packet.insert(packet.end(), params.begin(), params.end());
    // The checksum covers everything except the two header bytes.
    packet.push_back(checksum(packet.data() + 2, packet.size() - 2));
    return packet;
}

void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

}  // namespace

bool Bus::transact(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params,
                   std::vector<uint8_t>* reply_params, size_t expected_param_count) {
    last_error_.clear();

    const std::vector<uint8_t> packet = build_packet(id, instruction, params);

    port_.flush();
    if (!port_.write(packet.data(), packet.size())) {
        last_error_ = "serial write failed: " + port_.last_error();
        return false;
    }

    // The broadcast ID is a shout, not a question -- nobody answers it.
    if (id == kBroadcastId) {
        return true;
    }

    // Collect bytes until a whole reply frame is in hand or the port goes quiet.
    std::vector<uint8_t> buffer;
    uint8_t chunk[64];
    const size_t wanted = 6 + expected_param_count;

    for (int attempt = 0; attempt < 4; ++attempt) {
        const size_t got = port_.read(chunk, sizeof(chunk));
        if (got > 0) {
            buffer.insert(buffer.end(), chunk, chunk + got);
        }

        // Look for a valid frame. Some USB adapters echo our own transmission
        // back at us, so a frame identical to what we just sent is skipped.
        for (size_t i = 0; i + 4 <= buffer.size(); ++i) {
            if (buffer[i] != 0xFF || buffer[i + 1] != 0xFF) {
                continue;
            }
            const uint8_t len = buffer[i + 3];
            const size_t frame_size = 4 + static_cast<size_t>(len);
            if (len < 2 || i + frame_size > buffer.size()) {
                break;  // frame started but has not fully arrived yet
            }

            const uint8_t* frame = buffer.data() + i;
            if (frame_size == packet.size() &&
                std::equal(frame, frame + frame_size, packet.begin())) {
                continue;  // our own echo
            }
            if (checksum(frame + 2, frame_size - 3) != frame[frame_size - 1]) {
                continue;  // corrupt, keep looking
            }
            if (frame[2] != id) {
                continue;  // somebody else's reply
            }

            const uint8_t error_flags = frame[4];
            if (error_flags != 0) {
                last_error_ = "servo " + std::to_string(id) + " reported error flags 0x" +
                              [](uint8_t v) {
                                  const char* hex = "0123456789ABCDEF";
                                  return std::string{hex[v >> 4], hex[v & 0x0F]};
                              }(error_flags);
                return false;
            }

            if (reply_params) {
                reply_params->assign(frame + 5, frame + frame_size - 1);
            }
            return true;
        }

        if (got == 0 && buffer.size() >= wanted) {
            break;
        }
        if (got == 0 && attempt > 0) {
            break;
        }
    }

    last_error_ = buffer.empty() ? "no reply from servo " + std::to_string(id)
                                 : "malformed reply from servo " + std::to_string(id);
    return false;
}

bool Bus::ping(uint8_t id) {
    std::vector<uint8_t> reply;
    return transact(id, kInstPing, {}, &reply, 0);
}

bool Bus::write_u8(uint8_t id, uint8_t reg, uint8_t value) {
    std::vector<uint8_t> reply;
    return transact(id, kInstWrite, {reg, value}, &reply, 0);
}

bool Bus::read_u8(uint8_t id, uint8_t reg, uint8_t* out) {
    std::vector<uint8_t> reply;
    // For a read the two params are: start address, how many bytes to send back.
    if (!transact(id, kInstRead, {reg, 1}, &reply, 1) || reply.size() < 1) {
        return false;
    }
    *out = reply[0];
    return true;
}

bool Bus::read_u16(uint8_t id, uint8_t reg, uint16_t* out) {
    std::vector<uint8_t> reply;
    if (!transact(id, kInstRead, {reg, 2}, &reply, 2) || reply.size() < 2) {
        return false;
    }
    *out = static_cast<uint16_t>(reply[0] | (reply[1] << 8));  // low byte first
    return true;
}

bool Bus::write_u16(uint8_t id, uint8_t reg, uint16_t value) {
    std::vector<uint8_t> reply;
    // Low byte first, matching read_u16.
    const uint8_t low = static_cast<uint8_t>(value & 0xFF);
    const uint8_t high = static_cast<uint8_t>((value >> 8) & 0xFF);
    return transact(id, kInstWrite, {reg, low, high}, &reply, 0);
}

std::vector<uint8_t> Bus::scan() {
    std::vector<uint8_t> found;
    for (int id = 0; id <= 253; ++id) {
        if (ping(static_cast<uint8_t>(id))) {
            found.push_back(static_cast<uint8_t>(id));
        }
    }
    last_error_.clear();
    return found;
}

bool Bus::set_id(uint8_t current_id, uint8_t new_id) {
    if (new_id > 253) {
        last_error_ = "new ID must be 0-253";
        return false;
    }

    // The ID lives in EEPROM, which the servo refuses to change while locked.
    if (!write_u8(current_id, kRegLock, 0)) {
        last_error_ = "could not unlock EEPROM: " + last_error_;
        return false;
    }
    sleep_ms(20);

    // The moment this write lands the servo stops answering to current_id.
    if (!write_u8(current_id, kRegId, new_id)) {
        last_error_ = "could not write new ID: " + last_error_;
        return false;
    }
    sleep_ms(20);

    // Lock it again, now addressing the servo by its new name.
    if (!write_u8(new_id, kRegLock, 1)) {
        last_error_ = "ID was changed but EEPROM could not be re-locked: " + last_error_;
        return false;
    }
    sleep_ms(20);

    if (!ping(new_id)) {
        last_error_ = "servo did not answer on its new ID " + std::to_string(new_id);
        return false;
    }
    return true;
}

}  // namespace sts3215
