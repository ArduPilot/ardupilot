/*
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
 *
 * Author: Dmitry Ponomarev
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_CYPHAL_DRIVERS

#include <SRV_Channel/SRV_Channel.h>

#include "reg/udral/service/actuator/common/sp/Scalar_0_1.h"
#include "reg/udral/service/actuator/common/sp/Vector4_0_1.h"
#include "reg/udral/service/common/Readiness_0_1.h"
#include "reg/udral/physics/electricity/PowerTs_0_1.h"
#include "reg/udral/service/actuator/common/Feedback_0_1.h"
#include "reg/udral/service/actuator/common/Status_0_1.h"
#include "reg/udral/physics/dynamics/rotation/PlanarTs_0_1.h"

#include "AP_CYPHAL_publisher.h"
#include "AP_CYPHAL_subscriber.h"
#include "AP_CYPHAL_registers.h"


#ifndef CYPHAL_SRV_NUMBER
#define CYPHAL_SRV_NUMBER            4
#endif


class CyphalPowerSubscriber;
class CyphalSetpointPublisher;
class CyphalReadinessPublisher;
class CyphalDynamicsSubscriber;
class CyphalStatusSubscriber;


struct SrvConfig {
    uint16_t pulse;
    bool esc_pending;
    bool servo_pending;
};


/**
 * Publishes the following CYPHAL messages:
 *   reg.udral.service.actuator.common.sp.Vector4.0.1
 *   reg.udral.service.actuator.common.Readiness.0.1
 *
 * Subscribes to the following CYPHAL messages:
 *   reg.udral.physics.electricity.PowerTs_0_1
 *   reg.udral.service.actuator.common.Feedback_0_1
 *   reg.udral.service.actuator.common.Status_0_1
 *   reg.udral.physics.dynamics.rotation.PlanarTs_0_1
 */
class CyphalEscController
{
public:
    CyphalEscController(CyphalRegisters& uavcan_registers): _registers(uavcan_registers) {}

    bool init(CyphalSubscriberManager &sub_manager,
              CyphalPublisherManager &pub_manager,
              CanardInstance &ins,
              CanardTxQueue& tx_queue);
    void SRV_push_servos(void);

private:
    SrvConfig _SRV_conf[CYPHAL_SRV_NUMBER];

    uint8_t _esc_state;
    uint32_t _SRV_last_send_us;
    HAL_Semaphore SRV_sem;

    CyphalPowerSubscriber *_sub_power[4];
    CyphalDynamicsSubscriber *_sub_dynamics[4];
    CyphalStatusSubscriber *_sub_status[4];
    CyphalSetpointPublisher *_pub_setpoint;
    CyphalReadinessPublisher *_pub_readiness;

    bool _is_inited{false};

    CyphalRegisters& _registers;
};


/**
 * @note reg.udral.physics.dynamics.rotation.PlanarTs_0_1
 */
class CyphalDynamicsSubscriber: public CyphalBaseSubscriber, AP_ESC_Telem_Backend
{
public:
    CyphalDynamicsSubscriber(CanardInstance &ins, CanardTxQueue& tx_queue, int16_t port_id, uint8_t esc_idx) :
        CyphalBaseSubscriber(ins, tx_queue, port_id), _esc_idx(esc_idx) { };

    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;

private:
    uint8_t _esc_idx;
};


/**
 * @note reg.udral.physics.electricity.PowerTs_0_1
 */
class CyphalPowerSubscriber: public CyphalBaseSubscriber, AP_ESC_Telem_Backend
{
public:
    CyphalPowerSubscriber(CanardInstance &ins, CanardTxQueue& tx_queue, int16_t port_id, uint8_t esc_idx) :
        CyphalBaseSubscriber(ins, tx_queue, port_id), _esc_idx(esc_idx) { };

    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;

private:
    uint8_t _esc_idx;
};


/**
 * @note reg.udral.service.actuator.common.Status_0_1
 */
class CyphalStatusSubscriber: public CyphalBaseSubscriber, AP_ESC_Telem_Backend
{
public:
    CyphalStatusSubscriber(CanardInstance &ins, CanardTxQueue& tx_queue, int16_t port_id, uint8_t esc_idx) :
        CyphalBaseSubscriber(ins, tx_queue, port_id), _esc_idx(esc_idx) { };

    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;

private:
    uint8_t _esc_idx;
};


/**
 * @note reg.udral.service.actuator.common.sp.*
 */
class CyphalSetpointPublisher : public CyphalBasePublisher
{
public:
    CyphalSetpointPublisher(CanardInstance &ins, CanardTxQueue& tx_queue, CanardPortID port_id);
    virtual void update() override;
    void set_setpoint(SrvConfig *src_config);

private:
    static constexpr uint32_t publish_period_ms = 5;
    uint32_t next_publish_time_ms{publish_period_ms};

    reg_udral_service_actuator_common_sp_Vector4_0_1 _vector4_sp;

    void publish();
};


/**
 * @note reg.udral.service.common.Readiness.0.1
 */
class CyphalReadinessPublisher : public CyphalBasePublisher
{
public:
    CyphalReadinessPublisher(CanardInstance &ins, CanardTxQueue& tx_queue, CanardPortID port_id);
    virtual void update() override;
    void update_readiness();

private:
    static constexpr uint32_t publish_period_ms = 50;
    uint32_t next_publish_time_ms{publish_period_ms};

    reg_udral_service_common_Readiness_0_1 _readiness;

    void publish();
};

#endif // HAL_ENABLE_CYPHAL_DRIVERS
