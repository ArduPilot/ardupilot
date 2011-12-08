/*
 * AP_ArmingMechanism.h
 *
 */

#ifndef AP_ARMINGMECHANISM_H_
#define AP_ARMINGMECHANISM_H_

#include <inttypes.h>
#include <wiring.h>

namespace apo {

class AP_Board;
class AP_Controller;

class AP_ArmingMechanism {

public:
    /**
     * Constructor
     *
     * @param ch1: typically throttle channel
     * @param ch2: typically yaw channel
     * @param ch1Min: disarms/arms belows this
     * @param ch2Min: disarms below this
     * @param ch2Max: arms above this
     */
    AP_ArmingMechanism(AP_Board * board, AP_Controller * controller,
                       uint8_t ch1, uint8_t ch2, float ch1Min, float ch2Min,
                       float ch2Max) : _armingClock(0), _board(board), _controller(controller), 
        _ch1(ch1), _ch2(ch2), _ch1Min(ch1Min), _ch2Min(ch2Min), _ch2Max(ch2Max) {
    }

    /**
     * update
     *
     * arming:
     *
     * to arm: put stick to bottom right for 100 controller cycles
     * (max yaw, min throttle)
     *
     * didn't use clock here in case of millis() roll over
     * for long runs
     *
     * disarming:
     *
     * to disarm: put stick to bottom left for 100 controller cycles
     * (min yaw, min throttle)
     */
    void update(const float dt);

private:

    AP_Board * _board;
    AP_Controller * _controller;
    int8_t _armingClock;
    uint8_t _ch1; /// typically throttle channel
    uint8_t _ch2; /// typically yaw channel
    float _ch1Min; /// arms/disarms below this on ch1
    float _ch2Min; /// disarms below this on ch2
    float _ch2Max; /// arms above this on ch2
};

} // namespace apo

#endif /* AP_ARMINGMECHANISM */

// vim:ts=4:sw=4:expandtab
