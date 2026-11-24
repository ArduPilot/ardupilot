#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <deque>
#include <string>

struct FakeSerial {
    std::deque<char> input;
    std::string output;
    void begin(unsigned long) {}
    int available() const { return int(input.size()); }
    int read() { char c = input.front(); input.pop_front(); return c; }
    void printf(const char *format, ...) {
        char buffer[128];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        output += buffer;
    }
    void feed(const std::string &text) { for (char c : text) input.push_back(c); }
} Serial;

static unsigned long test_ms;
unsigned long millis() { return test_ms; }
unsigned long esp_random() {
    static const unsigned long sessions[] = {0x11111111, 0x1234ABCD, 0x87654321};
    static unsigned index;
    return sessions[index++];
}

#define ATAL_HOST_TEST
#include "esp32_guarded/esp32_guarded.ino"

static std::string frame(const std::string &body) {
    char crc[8];
    snprintf(crc, sizeof(crc), ",%04X", ActuatorGuard::crc16(body.data(), body.size()));
    return body + crc + "\n";
}

int main() {
    setup();
    assert(Serial.output.empty());
    Serial.feed("A1,HELLO,89ABCDEF\n");
    loop();
    assert(Serial.output == "A1,SESSION,1234ABCD,89ABCDEF\n");
    Serial.feed(frame("A1,ARM,1234ABCD,1"));
    loop();
    assert(guard.state() == ActuatorGuard::State::Armed);
    Serial.feed(frame("A1,PWM,1234ABCD,2,1300,1400,1500,1600,1700,1500,1500,1500"));
    loop();
    assert(guard.outputs()[0] == 1300);
    Serial.feed("A1,HELLO,76543210\n");
    loop();
    assert(guard.state() == ActuatorGuard::State::Disarmed);
    assert(guard.outputs()[0] == 1500);
    assert(Serial.output.find("A1,SESSION,87654321,76543210\n") != std::string::npos);
    Serial.feed(frame("A1,ARM,87654321,1"));
    loop();
    assert(guard.state() == ActuatorGuard::State::Armed);
    Serial.feed(std::string(200, 'X') + "\n");
    loop();
    assert(guard.state() == ActuatorGuard::State::Fault);
    assert(guard.outputs()[0] == 1500);
    assert(Serial.output.find("A1,FAULT\n") != std::string::npos);
    Serial.feed(frame("A1,RESET,87654321,2"));
    loop();
    assert(guard.state() == ActuatorGuard::State::Disarmed);
    Serial.feed(frame("A1,ARM,87654321,3"));
    loop();
    test_ms = 251;
    loop();
    assert(guard.state() == ActuatorGuard::State::Fault);
    puts("ESP32 sketch parser transitions passed");
}
