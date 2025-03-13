#pragma once

#include <AP_HAL/AP_HAL.h>



#include "cyphal/canard.h"


#include "uavcan/node/Heartbeat_1_0.h"
#include "zubax/primitive/real16/Vector31_1_0.h"
#include "zubax/service/Readiness_1_0.h"


class CyphalBasePublisher;


class CyphalPublisherManager
{
public:
    CyphalPublisherManager() {};
    void init(CypInstance &ins, CypTxQueue& tx_queue);

    // return true in sucess, otherwise false
    bool add_publisher(CyphalBasePublisher *publisher);

    void process_all();
private:
    static constexpr uint8_t max_number_of_publishers = 4;
    uint8_t number_of_publishers = 0;
    CyphalBasePublisher *publishers[max_number_of_publishers];
};


class CyphalBasePublisher
{
public:
    CyphalBasePublisher(CypInstance &ins, CypTxQueue& tx_queue, CanardPortID port_id) :
        _canard(ins), _tx_queue(tx_queue), _port_id(port_id) {};
    uint16_t get_port_id()
    {
        return _port_id;
    }
    virtual void update() = 0;

protected:
    void push(size_t buf_size, uint8_t* buf);

    CypInstance &_canard;
    CypTxQueue &_tx_queue;
    CanardPortID _port_id;
    CanardTransferMetadata _transfer_metadata;
};


/**
 * @note uavcan.node.Heartbeat_1_0
 */
class CyphalHeartbeatPublisher : public CyphalBasePublisher
{
public:
    CyphalHeartbeatPublisher(CypInstance &ins, CypTxQueue& tx_queue);
    virtual void update() override;

private:
    static constexpr uint32_t publish_period_ms = 1000;
    uint32_t next_publish_time_ms{publish_period_ms};
    uavcan_node_Heartbeat_1_0 msg;

    void publish();
};

/**
 * @note zubax.setpoint
 */
class CyphalSetpointPublisher : public CyphalBasePublisher
{
    public: 
        CyphalSetpointPublisher( CypInstance &ins, CypTxQueue &tx_queue);
        virtual void update() override;
    private:
        static constexpr uint32_t publish_period_ms = 2;
        uint32_t next_publish_time_ms{publish_period_ms};
        zubax_primitive_real16_Vector31_1_0 msg;

        void publish();
};

/**
 * @note zubax.service.Readiness_1_0
 */ 
class CyphalReadinessPublisherFront: public CyphalBasePublisher
{
    public:
        CyphalReadinessPublisherFront(CypInstance &ins, CypTxQueue& tx_queue);
        virtual void update() override;
    private:
        static constexpr uint32_t publish_period_ms = 100;
        uint32_t next_publish_time_ms{publish_period_ms};
        zubax_service_Readiness_1_0 msg;

        void publish();

};

/**
 * @note zubax.service.Readiness_1_0
 */
class CyphalReadinessPublisherBack: public CyphalBasePublisher
{
    public:
        CyphalReadinessPublisherBack(CypInstance &ins, CypTxQueue& tx_queue);
        virtual void update() override;
    private:
        static constexpr uint32_t publish_period_ms = 100;
        uint32_t next_publish_time_ms{publish_period_ms};
        zubax_service_Readiness_1_0 msg;

        void publish();

};
