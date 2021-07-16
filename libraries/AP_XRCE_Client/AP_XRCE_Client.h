#if AP_XRCE_ENABLED

#include "uxr/client/client.h"
#include "ucdr/microcdr.h"

#include "Num.h"
#include "AP_INS.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Scheduler.h>
#include <AP_HAL/Semaphores.h>
#include <AP_AHRS/AP_AHRS.h>

#include "fcntl.h"

#define STREAM_HISTORY 8
#define BUFFER_SIZE_SERIAL UXR_CONFIG_SERIAL_TRANSPORT_MTU * STREAM_HISTORY

extern const AP_HAL::HAL& hal;

class AP_XRCE_Client {
 
private:

    // Initialize
    uint32_t max_topics; // Maximum number of topics the client can use
    
    // Serial Device 
    uint8_t fd;
    uint8_t relativeSerialAgentAddr;
    uint8_t relativeSerialClientAddr;
    
    uxrSerialTransport serial_transport; // client uxr serial transport
    uxrSession session; //Session

    // Input Stream

    uint8_t input_reliable_stream[BUFFER_SIZE_SERIAL];
    uxrStreamId reliable_in;
    
    // Output Stream

    uint8_t output_reliable_stream[BUFFER_SIZE_SERIAL];
    uxrStreamId reliable_out;
    
    // Create
    // Participant

    uxrObjectId participant_id; 
    uint16_t participant_req;
    
    // Topic
    
    uxrObjectId topic_id; 
    uint16_t topic_req;
    
    // Publisher
    
    uxrObjectId pub_id; 
    uint16_t pub_req;
    
    // DataWriter
    
    uxrObjectId dwriter_id;
    uint16_t dwriter_req;
    
    //Status requests
    uint8_t status[4];

    // Write
    Num topic;  // simple int topic for testing
    AP_INS ins_topic; // INS topics

    // Semaphore
    AP_HAL::Semaphore *_csem;

    // connection parametrics
    bool connected;

public:
    // Constructor (takes maximum number of topics as argument,by default it is 1)
    AP_XRCE_Client(uint32_t maxtopics=1) {
        
        this->max_topics = maxtopics;
        
        relativeSerialClientAddr=0;
        relativeSerialAgentAddr=1;
        
        connected=true;
        
        //this->topic.num=0;
        
        this->ins_topic.accel_count=0;
        this->ins_topic.gyro_count=0;
        for(uint8_t i=0;i<3;i++){
            this->ins_topic.accel_scale[i]=0.0;
            this->ins_topic.accel_offsets[i]=0.0;
            this->ins_topic.gyro_offsets[i]=0.0;
        }
    }

    bool init();
    bool create();
    void write();
    void updateINSTopic(AP_InertialSensor& ins); 
    AP_HAL::Semaphore* get_clientsemaphore();
};

#endif // AP_XRCE_ENABLED


