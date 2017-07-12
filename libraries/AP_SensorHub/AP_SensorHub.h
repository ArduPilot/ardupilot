#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#if HAL_SENSORHUB_ENABLED

#if !defined(SENSORHUB_MODE)
    #error not defined
#endif

class DataFlash_Class;

#include "AP_SensorHub_Debug.h"
#include "Protocol.h"
#include "AP_SensorHub_IO.h"
#include "AP_SensorHub_Handler.h"

#include <AP_Common/AP_Common.h>

using namespace SensorHub;

#define SENSORHUB_MAX_PORTS 2

class AP_SensorHub {
public:
    static const uint16_t UPDATE_RATE_HZ = 1000;

    /* Handler Registration */
    void registerHandler(AP_SensorHub_Handler<BaroMessage> *handler)
    {
        _baroHandler = handler;
    }

    void registerHandler(AP_SensorHub_Handler<GyroMessage> *handler)
    {
        _gyroHandler = handler;
    }

    void registerHandler(AP_SensorHub_Handler<AccelMessage> *handler)
    {
        _accelHandler = handler;
    }

    void registerHandler(AP_SensorHub_Handler<CompassMessage> *handler)
    {
        _compassHandler = handler;
    }

    void registerHandler(AP_SensorHub_Handler<GPSMessage> *handler)
    {
        _gpsHandler = handler;
    }

    void registerIO(AP_SensorHub_IO *io);

    /* Singleton methods */
    static AP_SensorHub *init_instance();
    static AP_SensorHub *get_instance()
    {
        if (_initialized) {
            return &_instance;
        }
        return nullptr;
    }

    bool init(AP_SerialManager &serial_manager);

    /*
     * Given a valid packet, determine its Message type & call appropriate
     * handler.
     */
    bool handlePacket(Packet::packet_t *packet);

    /*
     * Given a buffer decode packet & call handler.
     */
    int process_packet(uint8_t *buf, size_t len, size_t &p_len);

    /*
     * Call IO port to process raw data.
     */
    void read();

    /*
     * Forward packet to IO port.
     */
    void write(Packet::packet_t *packet);

    void setSourceMode() {
        _sourceMode = true;
    }

    void setSinkMode() {
        _sourceMode = false;
    }

    bool isSource() {
        return _sourceMode;
    }

    /*
     * Explicitly set DataFlash logging. This can be useful for debugging.
     */
    void setDataFlash(DataFlash_Class *dataflash) {
        _dataflash = dataflash;
    }

    void start() {
        _vehicle_ready = true;
    }

    bool isReady() {
        return _vehicle_ready;
    }


private:
    static bool _initialized;
    static AP_SensorHub _instance;
    DataFlash_Class *_dataflash;

    bool _sourceMode;

    bool _vehicle_ready;

    AP_SensorHub_IO *_port[SENSORHUB_MAX_PORTS];
    int _io_count;

    /* Message Handlers */

    class UnknownMessageHandler : public AP_SensorHub_Handler<UnknownMessage> {
    public:
        virtual void handle(UnknownMessage::data_t *data) {}
        virtual bool isValid(UnknownMessage::data_t *data) { return true; }
    };

    AP_SensorHub_Handler<BaroMessage> *_baroHandler;
    AP_SensorHub_Handler<GyroMessage> *_gyroHandler;
    AP_SensorHub_Handler<AccelMessage> *_accelHandler;
    AP_SensorHub_Handler<CompassMessage> *_compassHandler;
    AP_SensorHub_Handler<GPSMessage> *_gpsHandler;
    UnknownMessageHandler _defaultHandler;

    /* Utils */
    AP_HAL::Util::perf_counter_t _perf_read;
    AP_HAL::Util::perf_counter_t _perf_write;

    /* Helper Methods */
    template <class MessageType, class MessageHandler>
    void _handlePacketHelper(Packet::packet_t *packet, MessageHandler *handler)
    {
        if (!Message::verify<MessageType>(packet)) {
            return;
        }
        typename MessageType::data_t data{};
        Message::decode<MessageType>(packet, &data);
        if (!handler) {
            return;
        }
        handler->handle(&data);
    }

    bool _notFirstPacket;
    Packet::seq_t _readSeq;
    uint32_t _packetLoss;
    uint32_t _lastPacketTime;
    float _simPacketLoss;

};
#endif
