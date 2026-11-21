#include "actuator_guard.h"

#include <assert.h>
#include <stdio.h>
#include <string>

static std::string frame(const std::string &body) {
    char crc[8];
    snprintf(crc, sizeof(crc), ",%04X", ActuatorGuard::crc16(body.data(), body.size()));
    return body + crc;
}

static void neutral(const ActuatorGuard &guard) {
    for (unsigned i = 0; i < ActuatorGuard::MOTOR_COUNT; ++i) {
        assert(guard.outputs()[i] == ActuatorGuard::NEUTRAL_US);
    }
}

int main() {
    assert(ActuatorGuard::crc16("A1,ARM,1234ABCD,3", 17) == 0xA9C1);
    ActuatorGuard guard(0x1234ABCD);
    neutral(guard);
    assert(guard.state() == ActuatorGuard::State::Disarmed);
    assert(!guard.accept(frame("A1,PWM,1234ABCD,1,1500,1500,1500,1500,1500,1500,1500,1500").c_str(), 10));
    assert(guard.state() == ActuatorGuard::State::Fault);
    assert(guard.accept(frame("A1,RESET,1234ABCD,2").c_str(), 11));
    assert(guard.accept(frame("A1,ARM,1234ABCD,3").c_str(), 12));
    assert(guard.state() == ActuatorGuard::State::Armed);
    const auto command = frame("A1,PWM,1234ABCD,4,1300,1400,1500,1600,1700,1500,1501,1499");
    assert(guard.accept(command.c_str(), 20));
    assert(guard.outputs()[0] == 1300 && guard.outputs()[4] == 1700);
    assert(!guard.accept(command.c_str(), 21)); // replay latches fault
    neutral(guard);
    assert(!guard.accept(frame("A1,ARM,1234ABCD,5").c_str(), 22));
    assert(guard.accept(frame("A1,RESET,1234ABCD,6").c_str(), 23));
    assert(guard.accept(frame("A1,ARM,1234ABCD,7").c_str(), 24));
    guard.tick(275);
    assert(guard.state() == ActuatorGuard::State::Fault);
    neutral(guard);
    assert(guard.accept(frame("A1,RESET,1234ABCD,8").c_str(), 276));
    assert(guard.accept(frame("A1,ARM,1234ABCD,9").c_str(), 277));
    assert(!guard.accept(frame("A1,PWM,1234ABCD,10,1300,1400,1500,1600,1701,1500,1501,1499").c_str(), 278));
    neutral(guard);
    assert(guard.accept(frame("A1,RESET,1234ABCD,11").c_str(), 279));
    assert(guard.accept(frame("A1,ARM,1234ABCD,12").c_str(), 280));
    assert(!guard.accept(frame("A1,PWM,1234ABCD,13,1300,1400,1500,1600,1700,1500,1501,").c_str(), 281));
    neutral(guard);
    assert(guard.accept(frame("A1,RESET,1234ABCD,14").c_str(), 282));
    assert(!guard.accept(frame("A1,ARM,00000000,15").c_str(), 283));
    neutral(guard);
    assert(guard.accept(frame("A1,RESET,1234ABCD,16").c_str(), 284));
    assert(guard.accept(frame("A1,ARM,1234ABCD,17").c_str(), 285));
    assert(guard.accept(frame("A1,DISARM,1234ABCD,18").c_str(), 286));
    neutral(guard);
    assert(guard.state() == ActuatorGuard::State::Disarmed);
    puts("actuator guard transitions passed");
}
