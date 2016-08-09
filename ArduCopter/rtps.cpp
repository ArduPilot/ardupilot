/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Copter.h"

#if defined(HAVE_FASTRTPS) && defined(HAVE_FASTCDR) && \
    HAVE_FASTRTPS == 1 && HAVE_FASTCDR == 1

#include <functional>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

#include "AP_HAL/AP_HAL.h"
#include "AP_HAL/Scheduler.h"
#include "AP_HAL/Semaphores.h"
#include "AP_RTPS/AHRS.h"
#include "AP_RTPS/RTPSPublisher.h"

extern const AP_HAL::HAL& hal;

namespace {

static const uint32_t MAX_WAIT_UPDATE_MS = 2;
static const uint32_t MAX_WAIT_PUBLISH_MS = 2;

class RTPSPublisherItem {
    // Class used only for the std::vector in PublisherHelper below.
public:
    virtual void update() = 0;
    virtual void publish() = 0;
};

template <class State>
class RTPSPublisher : public RTPSPublisherItem {
public:
    void init(const std::string& topic,
        std::function<State(void)> obtain_state_func);

    virtual void update() override;
    virtual void publish() override;

private:
    AP_HAL::Semaphore *semaphore;

    std::function<State(void)> obtain_state_func;

    RTPSIsolatedPublisher<State> publisher;
    State state;

    bool changed{false};
    bool initialized{false};
};

template <class State>
void RTPSPublisher<State>::init(const std::string& topic,
    std::function<State(void)> obtain_state_func_)
{
    std::string publisher_name{SKETCHNAME "_"};

    if (initialized) {
        return;
    }

    semaphore = hal.util->new_semaphore();
    if (!semaphore) {
        AP_HAL::panic("Failed to create semaphore for %s RTPS publisher",
            topic.c_str());
        return;
    }

    publisher_name.append(topic);
    if (!publisher.init(publisher_name, topic)) {
        AP_HAL::panic("Failed to initialize RTPS publisher: %s",
            publisher_name.c_str());
        return;
    }

    obtain_state_func = obtain_state_func_;

    initialized = true;
}

template <class State>
void RTPSPublisher<State>::publish()
{
    if (!initialized) {
        return;
    }

    if (semaphore->take(MAX_WAIT_PUBLISH_MS)) {
        if (!changed) {
            semaphore->give();
            return;
        }

        auto copy = state;
        changed = false;
        semaphore->give();

        publisher.iter(copy);
    }
}

template <class State>
void RTPSPublisher<State>::update()
{
    if (!initialized) {
        return;
    }

    auto new_state = obtain_state_func();
    if (semaphore->take(MAX_WAIT_UPDATE_MS)) {
        state = std::move(new_state);
        changed = true;

        semaphore->give();
    }
}

class PublisherHelper {
private:
    std::vector<RTPSPublisherItem *> publishers;
public:
    PublisherHelper(std::initializer_list<RTPSPublisherItem *> items)
        : publishers{std::move(items)} {}

    void publish();
    void update();
};

void PublisherHelper::publish()
{
    for (auto pub : publishers) {
        pub->publish();
    }
}

void PublisherHelper::update()
{
    for (auto pub : publishers) {
        pub->update();
    }
}

RTPSPublisher<AHRS> ahrs_publisher;
PublisherHelper helper{&ahrs_publisher};

}

void Copter::init_rtps()
{
    ahrs_publisher.init("AHRS", [this]() -> AHRS {
        AHRS new_state;

        new_state.roll(ahrs.roll);
        new_state.pitch(ahrs.pitch);
        new_state.yaw(ahrs.yaw);
        new_state.roll_pitch_error(ahrs.get_error_rp());
        new_state.yaw_error(ahrs.get_error_yaw());
        const auto gyro = ahrs.get_gyro();
        new_state.gyro({gyro.x, gyro.y, gyro.z});

        return new_state;
    });

    hal.scheduler->register_io_process(
        FUNCTOR_BIND(&helper, &PublisherHelper::publish, void));
}

void Copter::rtps_update_task()
{
    helper.update();
}
#else
void Copter::init_rtps()
{
}

void Copter::rtps_update_task()
{
}
#endif
