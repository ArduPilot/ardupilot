#include <gtest/gtest.h>
#include <uxr/client/client.h>

#include <functional>
#include <thread>

#define BUFFER_SIZE             1024
#define HISTORY_SIZE            8
#define SAMPLE_IDENTITY_SIZE    24

inline bool operator ==(
        const uxrObjectId& lhs,
        const uxrObjectId& rhs)
{
    return lhs.id == rhs.id && lhs.type == rhs.type;
}

enum XRCECreationMode
{
    XRCE_XML_CREATION,
    XRCE_BIN_CREATION,
    XRCE_REF_CREATION
};

class SharedMemoryTest : public ::testing::TestWithParam<XRCECreationMode>
{
public:

    SharedMemoryTest()
    {
        creation_mode = GetParam();

        comm.send_msg = send_msg_mock;
        comm.recv_msg = recv_msg_mock;

        uxr_init_session(&session, &comm, 0xAAAABBBB);
        uxr_set_topic_callback(&session, on_topic_dispatcher, this);
        uxr_set_request_callback(&session, on_request_dispatcher, this);
        uxr_set_reply_callback(&session, on_reply_dispatcher, this);

        output_besteffort = uxr_create_output_best_effort_stream(&session, output_besteffort_buffer, BUFFER_SIZE);
        input_besteffort = uxr_create_input_best_effort_stream(&session);

        output_reliable =
                uxr_create_output_reliable_stream(&session, output_reliable_buffer, sizeof(output_reliable_buffer),
                        HISTORY_SIZE);
        input_reliable = uxr_create_input_reliable_stream(&session, input_reliable_buffer,
                        sizeof(input_reliable_buffer), HISTORY_SIZE);

        participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    }

    void TearDown() override
    {
        ASSERT_NO_FATAL_FAILURE(uxr_delete_session(&session));
    }

protected:

    uxrQoS_t qos = {UXR_DURABILITY_TRANSIENT_LOCAL, UXR_RELIABILITY_RELIABLE, UXR_HISTORY_KEEP_LAST, 0};

    uxrObjectId create_datawriter(
            std::string topic,
            std::string type)
    {
        uxrObjectId topic_id = uxr_object_id(topic_no++, UXR_TOPIC_ID);
        uxrObjectId publisher_id = uxr_object_id(publishers_no++, UXR_PUBLISHER_ID);
        uxrObjectId datawriter_id = uxr_object_id(datawriters_no++, UXR_DATAWRITER_ID);

        std::ostringstream stringStream;

        switch (creation_mode)
        {
            case XRCE_XML_CREATION:
                stringStream << "<dds>"
                    "<data_writer>"
                    "<topic>"
                    "<kind>NO_KEY</kind>"
                    "<name>" << topic << "</name>"
                    "<dataType>" << type << "</dataType>"
                    "</topic>"
                    "</data_writer>"
                    "</dds>";
                uxr_buffer_create_datawriter_xml(&session, output_besteffort, datawriter_id, publisher_id,
                        stringStream.str().c_str(), UXR_REPLACE);
                break;

            case XRCE_BIN_CREATION:
                uxr_buffer_create_topic_bin(&session, output_besteffort, topic_id, participant_id,
                        topic.c_str(), type.c_str(), UXR_REPLACE);
                uxr_buffer_create_datawriter_bin(&session, output_besteffort, datawriter_id, publisher_id, topic_id,
                        qos, UXR_REPLACE);
                break;
            default:
                // ASSERT_TRUE(0);
                break;
        }
        return datawriter_id;
    }

    uxrObjectId create_datareaded(
            std::string topic,
            std::string type)
    {
        uxrObjectId topic_id = uxr_object_id(topic_no++, UXR_TOPIC_ID);
        uxrObjectId subscriber_id = uxr_object_id(subscribers_no++, UXR_SUBSCRIBER_ID);
        uxrObjectId datareader_id = uxr_object_id(datareaders_no++, UXR_DATAREADER_ID);

        std::ostringstream stringStream;

        switch (creation_mode)
        {
            case XRCE_XML_CREATION:
                stringStream << "<dds>"
                    "<data_reader>"
                    "<topic>"
                    "<kind>NO_KEY</kind>"
                    "<name>" << topic << "</name>"
                    "<dataType>" << type << "</dataType>"
                    "</topic>"
                    "</data_reader>"
                    "</dds>";

                uxr_buffer_create_datareader_xml(&session, output_besteffort, datareader_id, subscriber_id,
                        stringStream.str().c_str(), UXR_REPLACE);
                break;

            case XRCE_BIN_CREATION:
                uxr_buffer_create_topic_bin(&session, output_besteffort, topic_id, participant_id,
                        topic.c_str(), type.c_str(), UXR_REPLACE);
                uxr_buffer_create_datareader_bin(&session, output_besteffort, datareader_id, subscriber_id, topic_id,
                        qos, UXR_REPLACE);
                break;
            default:
                // ASSERT_TRUE(0);
                break;
        }
        return datareader_id;
    }

    uxrObjectId create_requester(
            std::string name,
            std::string request_type,
            std::string reply_type)
    {
        uxrObjectId requester_id = uxr_object_id(requesters_no++, UXR_REQUESTER_ID);

        std::ostringstream stringStream;

        switch (creation_mode)
        {
            case XRCE_XML_CREATION:
                stringStream << "<dds>"
                    "<requester profile_name=\"my_requester\""
                    "service_name=\"" << name << "\""
                    "request_type=\"" << request_type << "\""
                    "reply_type=\"" << reply_type << "\">"
                    "</requester>"
                    "</dds>";

                uxr_buffer_create_requester_xml(&session, output_besteffort, requester_id, participant_id,
                        stringStream.str().c_str(), UXR_REPLACE);
                break;
            case XRCE_BIN_CREATION:
                uxr_buffer_create_requester_bin(&session, output_besteffort, requester_id, participant_id,
                        name.c_str(), request_type.c_str(), reply_type.c_str(), "", "", qos, UXR_REPLACE);
                break;
            default:
                // ASSERT_TRUE(0);
                break;
        }
        return requester_id;
    }

    uxrObjectId create_replier(
            std::string name,
            std::string request_type,
            std::string reply_type)
    {
        uxrObjectId replier_id = uxr_object_id(requesters_no++, UXR_REPLIER_ID);

        std::ostringstream stringStream;

        switch (creation_mode)
        {
            case XRCE_XML_CREATION:
                stringStream << "<dds>"
                    "<replier profile_name=\"my_requester\""
                    "service_name=\"" << name << "\""
                    "request_type=\"" << request_type << "\""
                    "reply_type=\"" << reply_type << "\">"
                    "</replier>"
                    "</dds>";

                uxr_buffer_create_replier_xml(&session, output_besteffort, replier_id, participant_id,
                        stringStream.str().c_str(), UXR_REPLACE);
                break;

            case XRCE_BIN_CREATION:
                uxr_buffer_create_replier_bin(&session, output_besteffort, replier_id, participant_id,
                        name.c_str(), request_type.c_str(), reply_type.c_str(), "", "", qos, UXR_REPLACE);
                break;
            default:
                // ASSERT_TRUE(0);
                break;
        }
        return replier_id;
    }

    static void on_request_dispatcher(
            struct uxrSession* session,
            uxrObjectId object_id,
            uint16_t request_id,
            SampleIdentity* sample_id,
            struct ucdrBuffer* ub,
            uint16_t length,
            void* args)
    {
        static_cast<SharedMemoryTest*>(args)->on_request(session, object_id, request_id, sample_id, ub, length, args);
    }

    static void on_reply_dispatcher(
            struct uxrSession* session,
            uxrObjectId object_id,
            uint16_t request_id,
            uint16_t reply_id,
            struct ucdrBuffer* ub,
            uint16_t length,
            void* args)
    {
        static_cast<SharedMemoryTest*>(args)->on_reply(session, object_id, request_id, reply_id, ub, length, args);
    }

    static void on_topic_dispatcher(
            uxrSession* session_,
            uxrObjectId object_id,
            uint16_t request_id,
            uxrStreamId stream_id,
            struct ucdrBuffer* serialization,
            uint16_t length,
            void* args)
    {
        static_cast<SharedMemoryTest*>(args)->on_topic(session_, object_id, request_id, stream_id, serialization,
                length);
    }

    static bool send_msg_mock(
            void* /*instance*/,
            const uint8_t* /*buf*/,
            size_t /*len*/)
    {
        return true;
    }

    static bool recv_msg_mock(
            void* /*instance*/,
            uint8_t** /*buf*/,
            size_t* /*len*/,
            int /*timeout*/)
    {
        return false;
    }

    void test_services(
            std::string requester_name,
            std::string requester_request_type,
            std::string requester_reply_type,
            std::string replier_name,
            std::string replier_request_type,
            std::string replier_reply_type,
            bool expected_matching)
    {
        uxrObjectId requester_id = create_requester(requester_name, requester_request_type, requester_reply_type);
        uxrObjectId replier_id = create_replier(replier_name, replier_request_type, replier_reply_type);

        const size_t data_lenght = sizeof(uint32_t);

        size_t req_count = 0;
        size_t res_count = 0;

        on_request = [&](
            struct uxrSession* session_internal,
            uxrObjectId /* object_id */,
            uint16_t /*request_id*/,
            SampleIdentity* sample_id,
            struct ucdrBuffer* /* ub */,
            uint16_t /* length */,
            void* /*args*/) -> void
                {
                    req_count++;
                    ucdrBuffer replay_ub;
                    uxr_prepare_output_stream(session_internal, output_besteffort, replier_id, &replay_ub,
                            data_lenght + SAMPLE_IDENTITY_SIZE);
                    uxr_serialize_SampleIdentity(&replay_ub, sample_id);
                    ucdr_serialize_uint32_t(&replay_ub, 12);
                    uxr_run_session_time(&session, 1000);
                };

        on_reply = [&](
            struct uxrSession* /*session*/,
            uxrObjectId /* object_id */,
            uint16_t /*request_id*/,
            uint16_t /* reply_id */,
            struct ucdrBuffer* /* ub */,
            uint16_t /* length */,
            void* /*args*/) -> void
                {
                    res_count++;
                };

        ucdrBuffer ub;
        uxr_prepare_output_stream(&session, output_besteffort, requester_id, &ub, data_lenght);
        ucdr_serialize_uint32_t(&ub, 12);
        uxr_run_session_time(&session, 1000);

        size_t expected_count = (expected_matching) ? 1 : 0;
        ASSERT_EQ(req_count, expected_count);
        ASSERT_EQ(res_count, expected_count);
    }

protected:

    XRCECreationMode creation_mode;
    uxrSession session;
    uxrCommunication comm;
    uint8_t output_besteffort_buffer[BUFFER_SIZE];
    uint8_t output_reliable_buffer[BUFFER_SIZE * HISTORY_SIZE];
    uint8_t input_reliable_buffer[BUFFER_SIZE * HISTORY_SIZE];
    uxrObjectId participant_id;

    uint16_t topic_no = 1;
    uint16_t publishers_no = 1;
    uint16_t datawriters_no = 1;
    uint16_t subscribers_no = 1;
    uint16_t datareaders_no = 1;
    uint16_t requesters_no = 1;
    uint16_t repliers_no = 1;

    uxrStreamId output_besteffort;
    uxrStreamId input_besteffort;
    uxrStreamId output_reliable;
    uxrStreamId input_reliable;

    std::function<void(uxrSession*, uxrObjectId, uint16_t, uxrStreamId, struct ucdrBuffer*, uint16_t)> on_topic;
    std::function<void(struct uxrSession*, uxrObjectId, uint16_t, SampleIdentity*, struct ucdrBuffer*, uint16_t,
            void*)> on_request;
    std::function<void(struct uxrSession*, uxrObjectId, uint16_t, uint16_t, struct ucdrBuffer*, uint16_t,
            void*)> on_reply;
};

TEST_P(SharedMemoryTest, SharedMemoryPubSub)
{
    uxrObjectId datawriter_id = create_datawriter("shared_memory_topic", "shared_memory_type");
    uxrObjectId datareader_id = create_datareaded("shared_memory_topic", "shared_memory_type");

    const size_t data_lenght = sizeof(uint32_t);
    const uint32_t data = 42;

    int received_topics = 0;

    on_topic = [&](
        uxrSession* /*session*/,
        uxrObjectId object_id,
        uint16_t /*request_id*/,
        uxrStreamId stream_id,
        struct ucdrBuffer* serialization,
        uint16_t length) -> void
            {
                ASSERT_EQ(stream_id.type, UXR_SHARED_MEMORY_STREAM);
                ASSERT_EQ(length, data_lenght);
                ASSERT_EQ(object_id, datareader_id);

                uint32_t out;
                ASSERT_TRUE(ucdr_deserialize_uint32_t(serialization, &out));
                ASSERT_EQ(out, data);

                received_topics++;
            };

    ucdrBuffer ub;

    // Test serializing and running session: no limit
    for (size_t i = 0; i < UXR_CONFIG_SHARED_MEMORY_STATIC_MEM_SIZE * 2; i++)
    {
        uxr_prepare_output_stream(&session, output_besteffort, datawriter_id, &ub, data_lenght);
        ucdr_serialize_uint32_t(&ub, data);

        uxr_run_session_time(&session, 1000);
    }
    ASSERT_EQ(received_topics, UXR_CONFIG_SHARED_MEMORY_STATIC_MEM_SIZE * 2);

    received_topics = 0;
    // Test serializing and then running session: mempool limit
    for (size_t i = 0; i < UXR_CONFIG_SHARED_MEMORY_STATIC_MEM_SIZE; i++)
    {
        uxr_prepare_output_stream(&session, output_besteffort, datawriter_id, &ub, data_lenght);
        ucdr_serialize_uint32_t(&ub, data);
    }

    // Extra topic that should not be in shared memory
    uxr_prepare_output_stream(&session, output_besteffort, datawriter_id, &ub, data_lenght);
    ucdr_serialize_uint32_t(&ub, data);

    uxr_run_session_time(&session, 1000);

    ASSERT_EQ(received_topics, UXR_CONFIG_SHARED_MEMORY_STATIC_MEM_SIZE);
}

TEST_P(SharedMemoryTest, SharedMemoryPubSubSimpleAPI)
{
    uxrObjectId datawriter_id = create_datawriter("shared_memory_topic", "shared_memory_type");
    uxrObjectId datareader_id = create_datareaded("shared_memory_topic", "shared_memory_type");

    const size_t data_lenght = sizeof(uint32_t);
    const uint32_t data = 42;

    int received_topics = 0;

    on_topic = [&](
        uxrSession* /*session*/,
        uxrObjectId object_id,
        uint16_t /*request_id*/,
        uxrStreamId stream_id,
        struct ucdrBuffer* serialization,
        uint16_t length) -> void
            {
                ASSERT_EQ(stream_id.type, UXR_SHARED_MEMORY_STREAM);
                ASSERT_EQ(length, data_lenght);
                ASSERT_EQ(object_id, datareader_id);

                uint32_t out;
                ASSERT_TRUE(ucdr_deserialize_uint32_t(serialization, &out));
                ASSERT_EQ(out, data);

                received_topics++;
            };

    ucdrBuffer ub;
    uint8_t buffer[data_lenght];
    ucdr_init_buffer(&ub, buffer, data_lenght);
    ucdr_serialize_uint32_t(&ub, data);

    uxr_buffer_topic(&session, output_besteffort, datawriter_id, buffer, data_lenght);
    uxr_run_session_time(&session, 1000);

    ASSERT_EQ(received_topics, 1);
}

#ifdef UCLIENT_PROFILE_MULTITHREAD
TEST_P(SharedMemoryTest, SharedMemoryPubSubMultithread)
{
    uxrObjectId datawriter_id_1 = create_datawriter("shared_memory_topic", "shared_memory_type");
    uxrObjectId datawriter_id_2 = create_datawriter("shared_memory_topic", "shared_memory_type");
    uxrObjectId datareader_id = create_datareaded("shared_memory_topic", "shared_memory_type");

    const size_t data_lenght = sizeof(uint32_t);
    const uint32_t data = 42;

    int received_topics = 0;

    on_topic = [&](
        uxrSession* /*session*/,
        uxrObjectId object_id,
        uint16_t /*request_id*/,
        uxrStreamId stream_id,
        struct ucdrBuffer* ub,
        uint16_t length) -> void
            {
                ASSERT_EQ(stream_id.type, UXR_SHARED_MEMORY_STREAM);
                ASSERT_EQ(length, data_lenght);
                ASSERT_EQ(object_id, datareader_id);

                uint32_t out;
                ASSERT_TRUE(ucdr_deserialize_uint32_t(ub, &out));
                ASSERT_EQ(out, data);

                received_topics++;
            };

    auto publisher_thread = [&](uxrObjectId& datawriter)
            {
                return [&]()
                       {
                           ucdrBuffer ub;
                           for (size_t i = 0; i < 100; i++)
                           {
                               ASSERT_TRUE(uxr_prepare_output_stream(&session, output_besteffort, datawriter, &ub,
                                       data_lenght));
                               ASSERT_TRUE(ucdr_serialize_uint32_t(&ub, data));

                               uxr_run_session_time(&session, 10);

                               UXR_UNLOCK_STREAM_ID(&session, output_besteffort);
                           }

                       };
            };

    std::thread publisher_1(publisher_thread(datawriter_id_1));
    std::thread publisher_2(publisher_thread(datawriter_id_1));
    std::thread publisher_3(publisher_thread(datawriter_id_2));

    publisher_1.join();
    publisher_2.join();
    publisher_3.join();

    ASSERT_EQ(received_topics, 300);
}
#endif // ifdef UCLIENT_PROFILE_MULTITHREAD

TEST_P(SharedMemoryTest, SharedMemory1Pub3Sub)
{
    uxrObjectId datawriter_id = create_datawriter("shared_memory_topic", "shared_memory_type");
    create_datareaded("shared_memory_topic", "shared_memory_type");
    create_datareaded("shared_memory_topic", "shared_memory_type");
    create_datareaded("shared_memory_topic", "shared_memory_type");

    const size_t data_lenght = sizeof(uint32_t);
    const uint32_t data = 42;

    int received_topics = 0;

    on_topic = [&](
        uxrSession* /*session*/,
        uxrObjectId /*object_id*/,
        uint16_t /*request_id*/,
        uxrStreamId /*stream_id*/,
        struct ucdrBuffer* /*serialization*/,
        uint16_t /*length*/) -> void
            {
                received_topics++;
            };

    ucdrBuffer ub;
    uxr_prepare_output_stream(&session, output_besteffort, datawriter_id, &ub, data_lenght);
    ucdr_serialize_uint32_t(&ub, data);
    uxr_run_session_time(&session, 1000);

    ASSERT_EQ(received_topics, 3);
}

TEST_P(SharedMemoryTest, SharedMemory3Pub1Sub)
{
    uxrObjectId datawriter_id_1 = create_datawriter("shared_memory_topic", "shared_memory_type");
    uxrObjectId datawriter_id_2 = create_datawriter("shared_memory_topic", "shared_memory_type");
    uxrObjectId datawriter_id_3 = create_datawriter("shared_memory_topic", "shared_memory_type");
    create_datareaded("shared_memory_topic", "shared_memory_type");

    const size_t data_lenght = sizeof(uint32_t);
    const uint32_t data = 42;

    int received_topics = 0;

    on_topic = [&](
        uxrSession* /*session*/,
        uxrObjectId /*object_id*/,
        uint16_t /*request_id*/,
        uxrStreamId /*stream_id*/,
        struct ucdrBuffer* /*serialization*/,
        uint16_t /*length*/) -> void
            {
                received_topics++;
            };

    ucdrBuffer ub;
    uxr_prepare_output_stream(&session, output_besteffort, datawriter_id_1, &ub, data_lenght);
    ucdr_serialize_uint32_t(&ub, data);

    uxr_prepare_output_stream(&session, output_besteffort, datawriter_id_2, &ub, data_lenght);
    ucdr_serialize_uint32_t(&ub, data);

    uxr_prepare_output_stream(&session, output_besteffort, datawriter_id_3, &ub, data_lenght);
    ucdr_serialize_uint32_t(&ub, data);

    uxr_run_session_time(&session, 1000);

    ASSERT_EQ(received_topics, 3);
}

TEST_P(SharedMemoryTest, SharedMemoryNoMatch)
{
    static const std::string topic1 = "shared_memory_topic";
    static const std::string topic2 = "shared_memory_topic_123";

    uxrObjectId datawriter_id = create_datawriter(topic1, topic1);
    create_datareaded(topic2, topic2);

    const size_t data_lenght = sizeof(uint32_t);
    const uint32_t data = 42;

    int received_topics = 0;

    on_topic = [&](
        uxrSession* /*session*/,
        uxrObjectId /*object_id*/,
        uint16_t /*request_id*/,
        uxrStreamId /*stream_id*/,
        struct ucdrBuffer* /*serialization*/,
        uint16_t /*length*/) -> void
            {
                received_topics++;
            };

    ucdrBuffer ub;
    uxr_prepare_output_stream(&session, output_besteffort, datawriter_id, &ub, data_lenght);
    ucdr_serialize_uint32_t(&ub, data);

    uxr_run_session_time(&session, 1000);

    ASSERT_EQ(received_topics, 0);
}

TEST_P(SharedMemoryTest, SharedMemoryFragmentation)
{
    uxrObjectId datawriter_id = create_datawriter("shared_memory_topic", "shared_memory_type");
    create_datareaded("shared_memory_topic", "shared_memory_type");

    const size_t data_lenght = 2 * BUFFER_SIZE;
    char* data = static_cast<char*>(malloc(data_lenght));
    memset(data, 'a', data_lenght);

    int received_topics = 0;

    on_topic = [&](
        uxrSession* /*session*/,
        uxrObjectId /*object_id*/,
        uint16_t /*request_id*/,
        uxrStreamId /*stream_id*/,
        struct ucdrBuffer* serialization,
        uint16_t length) -> void
            {
                ASSERT_EQ(length, data_lenght);

                char* data_out = static_cast<char*>(malloc(data_lenght));
                memset(data_out, 'z', data_lenght);
                ASSERT_TRUE(ucdr_deserialize_array_char(serialization, data_out, data_lenght));

                bool array_check = true;
                for (size_t i = 0; i < data_lenght; i++)
                {
                    if (data_out[i] != 'a')
                    {
                        array_check = false;
                        break;
                    }
                }

                ASSERT_TRUE(array_check);
                free(data_out);

                received_topics++;
            };

    ucdrBuffer ub;
    uxr_prepare_output_stream(&session, output_reliable, datawriter_id, &ub, data_lenght);
    ucdr_serialize_array_char(&ub, data, data_lenght);

    uxr_run_session_time(&session, 1000);

    ASSERT_EQ(received_topics, 1);

    free(data);
}


TEST_P(SharedMemoryTest, SharedMemoryReqRes)
{
    uxrObjectId requester_id = create_requester("shared_memory_reqres", "req_type", "res_type");
    uxrObjectId replier_id = create_replier("shared_memory_reqres", "req_type", "res_type");

    const size_t data_lenght = sizeof(uint32_t);
    std::map<uint16_t, uint32_t> req_data;
    uint16_t req_ids[3];

    on_request = [&](
        struct uxrSession* session_internal,
        uxrObjectId object_id,
        uint16_t /*request_id*/,
        SampleIdentity* sample_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* /*args*/) -> void
            {
                ASSERT_EQ(object_id, replier_id);
                ASSERT_NE(sample_id, nullptr);
                ASSERT_EQ(length, data_lenght);

                uint32_t out;
                ASSERT_TRUE(ucdr_deserialize_uint32_t(ub, &out));
                ASSERT_EQ(out, req_data[static_cast<uint16_t>(sample_id->sequence_number.low)]);

                ucdrBuffer replay_ub;
                uxr_prepare_output_stream(session_internal, output_besteffort, replier_id, &replay_ub,
                        data_lenght + SAMPLE_IDENTITY_SIZE);
                uxr_serialize_SampleIdentity(&replay_ub, sample_id);
                ucdr_serialize_uint32_t(&replay_ub, out * 2);

                uxr_run_session_time(&session, 1000);
            };

    on_reply = [&](
        struct uxrSession* /*session*/,
        uxrObjectId object_id,
        uint16_t /*request_id*/,
        uint16_t reply_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* /*args*/) -> void
            {
                ASSERT_EQ(length, data_lenght);
                ASSERT_EQ(object_id, requester_id);
                ASSERT_EQ(length, data_lenght);

                uint32_t out;
                ASSERT_TRUE(ucdr_deserialize_uint32_t(ub, &out));
                ASSERT_EQ(out, req_data[reply_id] * 2);
            };

    for (size_t i = 0; i < sizeof(req_ids) / sizeof(req_ids[0]); i++)
    {
        ucdrBuffer ub;
        req_ids[i] = uxr_prepare_output_stream(&session, output_besteffort, requester_id, &ub, data_lenght);
        req_data[req_ids[i]] = static_cast<uint32_t>(i * 10);
        ucdr_serialize_uint32_t(&ub, req_data[req_ids[i]]);
    }

    uxr_run_session_time(&session, 1000);
}

TEST_P(SharedMemoryTest, SharedMemoryReqResMatching)
{
    test_services("serv_name", "req_type", "rep_type",
            "serv_name", "req_type", "rep_type",
            true);
    test_services("1_serv_name_1", "1_req_type", "1_rep_type",
            "1_serv_name",   "1_req_type", "1_rep_type",
            false);
    test_services("2_serv_name", "2_req_type_1", "2_rep_type",
            "2_serv_name", "2_req_type",   "2_rep_type",
            false);
    test_services("3_serv_name", "3_req_type", "3_rep_type_1",
            "3_serv_name", "3_req_type", "3_rep_type",
            false);
    test_services("4_serv_name",   "4_req_type", "4_rep_type",
            "4_serv_name_1", "4_req_type", "4_rep_type",
            false);
    test_services("5_serv_name", "5_req_type",   "5_rep_type",
            "5_serv_name", "5_req_type_1", "5_rep_type",
            false);
    test_services("6_serv_name", "6_req_type", "6_rep_type",
            "6_serv_name", "6_req_type", "6_rep_type_1",
            false);
}

TEST_P(SharedMemoryTest, SharedMemoryReqResSimpleAPI)
{
    uxrObjectId requester_id = create_requester("shared_memory_reqres", "req_type", "res_type");
    uxrObjectId replier_id = create_replier("shared_memory_reqres", "req_type", "res_type");

    const size_t data_lenght = sizeof(uint32_t);
    std::map<uint16_t, uint32_t> req_data;
    uint16_t req_ids[3];

    int received_req = 0;
    int received_res = 0;

    on_request = [&](
        struct uxrSession* session_internal,
        uxrObjectId /*object_id*/,
        uint16_t /*request_id*/,
        SampleIdentity* sample_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* /*args*/) -> void
            {
                uint32_t out;
                ASSERT_TRUE(ucdr_deserialize_uint32_t(ub, &out));
                ASSERT_EQ(length, data_lenght);

                uint8_t buffer[data_lenght];
                ucdrBuffer replay_ub;
                ucdr_init_buffer(&replay_ub, buffer, sizeof(buffer));
                ucdr_serialize_uint32_t(&replay_ub, out * 2);

                uxr_buffer_reply(session_internal, output_besteffort, replier_id, sample_id, buffer, data_lenght);

                uxr_run_session_time(&session, 1000);

                received_req++;
            };

    on_reply = [&](
        struct uxrSession* /*session*/,
        uxrObjectId /*object_id*/,
        uint16_t /*request_id*/,
        uint16_t reply_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* /*args*/) -> void
            {
                uint32_t out;
                ASSERT_TRUE(ucdr_deserialize_uint32_t(ub, &out));
                ASSERT_EQ(out, req_data[reply_id] * 2);
                ASSERT_EQ(length, data_lenght);
            };

    for (size_t i = 0; i < sizeof(req_ids) / sizeof(req_ids[0]); i++)
    {
        uint32_t data = static_cast<uint32_t>(i * 10);
        uint8_t buffer[data_lenght];
        ucdrBuffer replay_ub;
        ucdr_init_buffer(&replay_ub, buffer, sizeof(buffer));
        ucdr_serialize_uint32_t(&replay_ub, data);

        req_ids[i] = uxr_buffer_request(&session, output_besteffort, requester_id, buffer, data_lenght);
        req_data[req_ids[i]] = data;

        received_res++;
    }

    uxr_run_session_time(&session, 1000);
    ASSERT_EQ(received_req, 3);
    ASSERT_EQ(received_res, 3);
}

#ifdef UCLIENT_PROFILE_MULTITHREAD
TEST_P(SharedMemoryTest, SharedMemoryReqRepMultithread)
{
    uxrObjectId requester_id_1 = create_requester("shared_memory_reqres", "req_type", "res_type");
    uxrObjectId requester_id_2 = create_requester("shared_memory_reqres", "req_type", "res_type");
    uxrObjectId replier_id = create_replier("shared_memory_reqres", "req_type", "res_type");

    const size_t data_lenght = sizeof(uint32_t);
    std::map<uint16_t, std::tuple<uxrObjectId, uint32_t>> req_data;
    size_t received_request = 0;
    uint16_t req_ids[3];

    on_request = [&](
        struct uxrSession* session_internal,
        uxrObjectId object_id,
        uint16_t /*request_id*/,
        SampleIdentity* sample_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* /*args*/) -> void
            {
                ASSERT_EQ(object_id, replier_id);
                ASSERT_NE(sample_id, nullptr);
                ASSERT_EQ(length, data_lenght);

                uint32_t out;
                ASSERT_TRUE(ucdr_deserialize_uint32_t(ub, &out));
                ASSERT_EQ(out, std::get<1>(req_data[static_cast<uint16_t>(sample_id->sequence_number.low)]));

                ucdrBuffer replay_ub;
                ASSERT_TRUE(uxr_prepare_output_stream(session_internal, output_besteffort, replier_id, &replay_ub,
                        data_lenght + SAMPLE_IDENTITY_SIZE));
                uxr_serialize_SampleIdentity(&replay_ub, sample_id);
                ucdr_serialize_uint32_t(&replay_ub, out * 2);

                uxr_run_session_time(&session, 100);
                UXR_UNLOCK_STREAM_ID(&session, output_besteffort);
            };

    on_reply = [&](
        struct uxrSession* /*session*/,
        uxrObjectId object_id,
        uint16_t /*request_id*/,
        uint16_t reply_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* /*args*/) -> void
            {
                ASSERT_EQ(length, data_lenght);
                ASSERT_EQ(object_id, std::get<0>(req_data[reply_id]));

                uint32_t out;
                ASSERT_TRUE(ucdr_deserialize_uint32_t(ub, &out));
                ASSERT_EQ(out, std::get<1>(req_data[reply_id]) * 2);
                received_request++;
            };

    auto requester_thread = [&](uxrObjectId& replier)
            {
                return [&]()
                       {
                           ucdrBuffer ub;

                           for (size_t i = 0; i < sizeof(req_ids) / sizeof(req_ids[0]); i++)
                           {
                               req_ids[i] = uxr_prepare_output_stream(&session, output_besteffort, replier, &ub,
                                               data_lenght);
                               req_data[req_ids[i]] = std::make_tuple(replier, static_cast<uint32_t>(i * 10));
                               ucdr_serialize_uint32_t(&ub, std::get<1>(req_data[req_ids[i]]));
                               uxr_run_session_time(&session, 100);
                               UXR_UNLOCK_STREAM_ID(&session, output_besteffort);
                           }
                       };
            };

    std::thread requester_1(requester_thread(requester_id_1));
    std::thread requester_2(requester_thread(requester_id_1));
    std::thread requester_3(requester_thread(requester_id_2));

    requester_1.join();
    requester_2.join();
    requester_3.join();
    ASSERT_EQ(received_request, req_data.size());
}

TEST_P(SharedMemoryTest, SharedMemoryReqRepFragmentation)
{
    uxrObjectId requester_id = create_requester("shared_memory_reqres", "req_type", "res_type");
    create_replier("shared_memory_reqres", "req_type", "res_type");

    const size_t data_length = 2 * BUFFER_SIZE;
    char* data = static_cast<char*>(malloc(data_length));
    memset(data, 'a', data_length);

    int received_data = 0;

    on_request = [&](
        struct uxrSession* session_internal,
        uxrObjectId object_id,
        uint16_t /*request_id*/,
        SampleIdentity* sample_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* /*args*/) -> void
            {
                ASSERT_EQ(length, data_length);

                char* data_out = static_cast<char*>(malloc(length));
                memset(data_out, 'z', length);
                ASSERT_TRUE(ucdr_deserialize_array_char(ub, data_out, length));

                bool array_check = true;
                for (size_t i = 0; i < length; i++)
                {
                    if (data_out[i] != 'a')
                    {
                        array_check = false;
                        break;
                    }
                }

                ASSERT_TRUE(array_check);
                received_data++;

                ucdrBuffer replay_ub;
                uxr_prepare_output_stream(session_internal, output_reliable, object_id, &replay_ub,
                        data_length + SAMPLE_IDENTITY_SIZE);
                uxr_serialize_SampleIdentity(&replay_ub, sample_id);
                ucdr_serialize_array_char(&replay_ub, data_out, length);

                uxr_run_session_until_confirm_delivery(session_internal, 1000);
                free(data_out);
            };

    on_reply = [&](
        struct uxrSession* /*session*/,
        uxrObjectId /*object_id*/,
        uint16_t /*request_id*/,
        uint16_t /*reply_id*/,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* /*args*/) -> void
            {
                ASSERT_EQ(length, data_length);

                char* data_out = static_cast<char*>(malloc(length));
                memset(data_out, 'z', length);
                ASSERT_TRUE(ucdr_deserialize_array_char(ub, data_out, length));

                bool array_check = true;
                for (size_t i = 0; i < length; i++)
                {
                    if (data_out[i] != 'a')
                    {
                        array_check = false;
                        break;
                    }
                }

                ASSERT_TRUE(array_check);
                free(data_out);

                received_data++;
            };

    ucdrBuffer ub;
    uxr_prepare_output_stream(&session, output_reliable, requester_id, &ub, data_length);
    ucdr_serialize_array_char(&ub, data, data_length);

    uxr_run_session_until_confirm_delivery(&session, 1000);

    ASSERT_EQ(received_data, 2);
    free(data);
}
#endif // ifdef UCLIENT_PROFILE_MULTITHREAD

#ifdef INSTANTIATE_TEST_SUITE_P
#define GTEST_INSTANTIATE_TEST_MACRO(x, y, z, w) INSTANTIATE_TEST_SUITE_P(x, y, z, w)
#else
#define GTEST_INSTANTIATE_TEST_MACRO(x, y, z, w) INSTANTIATE_TEST_CASE_P(x, y, z, w)
#endif // ifdef INSTANTIATE_TEST_SUITE_P

GTEST_INSTANTIATE_TEST_MACRO(
    SharedMemoryTest,
    SharedMemoryTest,
    ::testing::Values(XRCECreationMode::XRCE_XML_CREATION, XRCECreationMode::XRCE_BIN_CREATION),
    ::testing::PrintToStringParamName()
    );

int main(
        int args,
        char** argv)
{
    ::testing::InitGoogleTest(&args, argv);
    return RUN_ALL_TESTS();
}