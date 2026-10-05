#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "fake_serial_port.h"
#include "bus/sts3215.h"

using sts3215::Bus;

// Runs the test once per adapter style: with and without echo of our own bytes.
#define FOR_EACH_ADAPTER(port)                                   \
    FakeSerialPort port;                                         \
    SUBCASE("adapter without echo") { port.echo = false; }       \
    SUBCASE("adapter that echoes") { port.echo = true; }

TEST_CASE("ping sends the datasheet packet and accepts the reply") {
    FOR_EACH_ADAPTER(port);
    port.replies.push_back(servo_reply(1, 0x00, {}));
    Bus bus(port);

    CHECK(bus.ping(1));
    REQUIRE(port.writes.size() == 1);
    CHECK(port.writes[0] == Bytes{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB});
}

TEST_CASE("ping fails when nobody answers") {
    FOR_EACH_ADAPTER(port);
    Bus bus(port);

    CHECK_FALSE(bus.ping(7));
}

TEST_CASE("read_u16 asks for two bytes and decodes them low byte first") {
    FOR_EACH_ADAPTER(port);
    port.replies.push_back(servo_reply(4, 0x00, {0x3B, 0x06}));  // 0x063B = 1595
    Bus bus(port);

    uint16_t position = 0;
    REQUIRE(bus.read_u16(4, sts3215::kRegPresentPosition, &position));
    CHECK(position == 1595);
    CHECK(port.writes[0] == packet(4, sts3215::kInstRead, {sts3215::kRegPresentPosition, 2}));
}

TEST_CASE("write_u16 sends the value low byte first") {
    FOR_EACH_ADAPTER(port);
    port.replies.push_back(servo_reply(3, 0x00, {}));
    Bus bus(port);

    CHECK(bus.write_u16(3, sts3215::kRegGoalPosition, 2048));  // 0x0800
    CHECK(port.writes[0] == packet(3, sts3215::kInstWrite, {sts3215::kRegGoalPosition, 0x00, 0x08}));
}

TEST_CASE("a reply with a bad checksum is rejected") {
    FOR_EACH_ADAPTER(port);
    Bytes reply = servo_reply(1, 0x00, {});
    reply.back() ^= 0xFF;
    port.replies.push_back(reply);
    Bus bus(port);

    CHECK_FALSE(bus.ping(1));
}

TEST_CASE("a reply from a different servo is not taken as the answer") {
    FOR_EACH_ADAPTER(port);
    port.replies.push_back(servo_reply(2, 0x00, {}));
    Bus bus(port);

    CHECK_FALSE(bus.ping(1));
}

TEST_CASE("error flags from the servo are reported as a failure") {
    FOR_EACH_ADAPTER(port);
    port.replies.push_back(servo_reply(2, 0x20, {}));  // 0x20 = overload
    Bus bus(port);

    CHECK_FALSE(bus.ping(2));
    CHECK(bus.last_error().find("error flags") != std::string::npos);
}

TEST_CASE("a servo fault is not retried") {
    // Passes today because nothing is retried at all. It is here for when retries land
    // (test_known_weaknesses.cpp): they must stop at a fault. An overheating servo does
    // not get better by being asked again.
    FakeSerialPort port;
    port.replies.push_back(servo_reply(1, 0x04, {}));    // overheat
    port.replies.push_back(servo_reply(1, 0x00, {}));
    Bus bus(port);

    CHECK_FALSE(bus.ping(1));
    CHECK(port.writes.size() == 1);
    CHECK(bus.ping(1));   // ...and the unused reply proves no second send happened.
}

TEST_CASE("a transport write failure is reported") {
    FakeSerialPort port;
    port.fail_writes = true;
    Bus bus(port);

    CHECK_FALSE(bus.ping(1));
    CHECK(bus.last_error().find("serial write failed") != std::string::npos);
}

TEST_CASE("set_id unlocks EEPROM, writes the ID, then re-locks under the new ID") {
    FOR_EACH_ADAPTER(port);
    port.replies.push_back(servo_reply(1, 0x00, {}));  // unlock
    port.replies.push_back(servo_reply(1, 0x00, {}));  // write ID
    port.replies.push_back(servo_reply(4, 0x00, {}));  // lock, now answering as 4
    port.replies.push_back(servo_reply(4, 0x00, {}));  // ping on the new ID
    Bus bus(port);

    REQUIRE(bus.set_id(1, 4));
    REQUIRE(port.writes.size() == 4);
    CHECK(port.writes[0] == packet(1, sts3215::kInstWrite, {sts3215::kRegLock, 0}));
    CHECK(port.writes[1] == packet(1, sts3215::kInstWrite, {sts3215::kRegId, 4}));
    CHECK(port.writes[2] == packet(4, sts3215::kInstWrite, {sts3215::kRegLock, 1}));
    CHECK(port.writes[3] == packet(4, sts3215::kInstPing, {}));
}

TEST_CASE("set_id refuses an out-of-range ID without touching the bus") {
    FakeSerialPort port;
    Bus bus(port);

    CHECK_FALSE(bus.set_id(1, 254));
    CHECK(port.writes.empty());
}

// ---- Known weaknesses ----------------------------------------------------------------
// Behaviour the driver should have and does not yet. `should_fail` makes them pass while
// they fail; when a fix makes one pass the run goes red -- delete the marker then.


TEST_CASE("retry re-sends the packet when the servo never heard it" * doctest::should_fail()) {
    // The first transmission is lost on the wire, so the servo stays silent. Re-reading the
    // port (what the retry loop does today) can never recover that; re-sending can.
    FakeSerialPort port;
    port.replies.push_back({});                          // first packet: silence
    port.replies.push_back(servo_reply(1, 0x00, {}));    // second packet: answered
    Bus bus(port);

    CHECK(bus.ping(1));
    CHECK(port.writes.size() == 2);
}

TEST_CASE("set_id re-locks EEPROM when writing the new ID fails" * doctest::should_fail()) {
    // Today an ID-write failure returns early and leaves the servo's EEPROM unlocked.
    // Design question for the fix: with no reply you cannot tell whether the ID write
    // landed (servo is now 4, reply lost) or not (still 1). Which ID do you re-lock?
    // This test accepts either, so the choice stays yours.
    FakeSerialPort port;
    port.replies.push_back(servo_reply(1, 0x00, {}));    // unlock: ok
    port.replies.push_back({});                          // ID write: no reply
    port.replies.push_back(servo_reply(1, 0x00, {}));    // re-lock: answered
    port.replies.push_back(servo_reply(4, 0x00, {}));
    Bus bus(port);

    CHECK_FALSE(bus.set_id(1, 4));
    bool relocked = false;
    for (size_t i = 2; i < port.writes.size(); ++i) {
        relocked = relocked ||
                   port.writes[i] == packet(1, sts3215::kInstWrite, {sts3215::kRegLock, 1}) ||
                   port.writes[i] == packet(4, sts3215::kInstWrite, {sts3215::kRegLock, 1});
    }
    CHECK(relocked);
}

TEST_CASE("an echoing adapter still reports silence as \"no reply\"" * doctest::should_fail()) {
    // Found by the harness. With echo on, the buffer holds our own packet even when the
    // servo says nothing, so the error comes out as "malformed reply" -- which reads like
    // a corrupted frame and sends you debugging the wrong thing. A typed error fixes this
    // properly; the message check below is the stand-in until then.
    FakeSerialPort port;
    port.echo = true;
    Bus bus(port);

    CHECK_FALSE(bus.ping(7));
    CHECK(bus.last_error().find("no reply") != std::string::npos);
}

TEST_CASE("a stray 0xFF before the reply header does not lose the reply" * doctest::should_fail()) {
    // Found by the harness. Bytes FF FF FF 01 02 00 FC: the scan locks onto the first
    // FF FF, reads the third FF as an ID and 0x01 as a length, decides the frame has "not
    // fully arrived", and `break`s out of the search -- so the real frame one byte later
    // is never examined. Happens with or without echo.
    FakeSerialPort port;
    Bytes noisy{0x00, 0x13, 0xFF};
    const Bytes reply = servo_reply(1, 0x00, {});
    noisy.insert(noisy.end(), reply.begin(), reply.end());
    port.replies.push_back(noisy);
    Bus bus(port);

    CHECK(bus.ping(1));
}

// ---- Placeholders: design the API, then write the test ------------------------------

TEST_CASE("encode/decode are testable without a port" * doctest::skip()) {
    // Split Bus::transact: a pure encoder, a pure decoder (echo, noise, partial frames),
    // and a thin transaction layer. Then test the decoder directly on byte vectors.
}

TEST_CASE("a 2-byte register cannot be read as 1 byte" * doctest::skip()) {
    // struct Register { address; size; ... }. Ideally this is a compile error rather than
    // a runtime check -- in which case the "test" is that the bad call does not compile.
}

TEST_CASE("no-reply and servo-fault are distinguishable errors" * doctest::skip()) {
    // Replace bool + last_error() with a typed error; assert on the error kind and flags.
}

TEST_CASE("scan pings only the expected IDs by default" * doctest::skip()) {
    // Count port.writes after a default scan on a four-servo arm.
}

TEST_CASE("sync_read returns all four positions from one packet" * doctest::skip()) {
    // SYNC_READ (0x82): one packet out, one reply per servo back. Check both halves.
}

TEST_CASE("sync_write commands all four goals in one packet" * doctest::skip()) {
    // SYNC_WRITE (0x83): assert exactly one packet, with the datasheet layout.
}
