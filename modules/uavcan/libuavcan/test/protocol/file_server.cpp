/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/file_server.hpp>
#include "helpers.hpp"


class TestFileServerBackend : public uavcan::IFileServerBackend
{
public:
    static const std::string file_name;
    static const std::string file_data;

    virtual int16_t getInfo(const Path& path, uint64_t& out_size, EntryType& out_type)
    {
        if (path == file_name)
        {
            out_size = uint16_t(file_data.length());
            out_type.flags |= EntryType::FLAG_FILE;
            out_type.flags |= EntryType::FLAG_READABLE;
            return 0;
        }
        else
        {
            return Error::NOT_FOUND;
        }
    }

    virtual int16_t read(const Path& path, const uint64_t offset, uint8_t* out_buffer, uint16_t& inout_size)
    {
        if (path == file_name)
        {
            if (offset < file_data.length())
            {
                inout_size = uint16_t(file_data.length() - offset);
                std::memcpy(out_buffer, file_data.c_str() + offset, inout_size);
            }
            else
            {
                inout_size = 0;
            }
            return 0;
        }
        else
        {
            return Error::NOT_FOUND;
        }
    }
};

const std::string TestFileServerBackend::file_name = "test";
const std::string TestFileServerBackend::file_data = "123456789";

TEST(BasicFileServer, Basic)
{
    using namespace uavcan::protocol::file;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<GetInfo> _reg1;
    uavcan::DefaultDataTypeRegistrator<Read> _reg2;

    InterlinkedTestNodesWithSysClock nodes;

    TestFileServerBackend backend;

    uavcan::BasicFileServer serv(nodes.a, backend);
    std::cout << "sizeof(uavcan::BasicFileServer): " << sizeof(uavcan::BasicFileServer) << std::endl;

    ASSERT_LE(0, serv.start());

    /*
     * GetInfo, existing file
     */
    {
        ServiceClientWithCollector<GetInfo> get_info(nodes.b);

        GetInfo::Request get_info_req;
        get_info_req.path.path = "test";

        ASSERT_LE(0, get_info.call(1, get_info_req));
        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));

        ASSERT_TRUE(get_info.collector.result.get());
        ASSERT_TRUE(get_info.collector.result->isSuccessful());

        ASSERT_EQ(0, get_info.collector.result->getResponse().error.value);
        ASSERT_EQ(9, get_info.collector.result->getResponse().size);
        ASSERT_EQ(EntryType::FLAG_FILE | EntryType::FLAG_READABLE,
                  get_info.collector.result->getResponse().entry_type.flags);
    }

    /*
     * Read, existing file
     */
    {
        ServiceClientWithCollector<Read> read(nodes.b);

        Read::Request read_req;
        read_req.path.path = "test";

        ASSERT_LE(0, read.call(1, read_req));
        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));

        ASSERT_TRUE(read.collector.result.get());
        ASSERT_TRUE(read.collector.result->isSuccessful());

        ASSERT_EQ("123456789", read.collector.result->getResponse().data);
        ASSERT_EQ(0, read.collector.result->getResponse().error.value);
    }
}


TEST(FileServer, Basic)
{
    using namespace uavcan::protocol::file;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<GetInfo> _reg1;
    uavcan::DefaultDataTypeRegistrator<Read> _reg2;
    uavcan::DefaultDataTypeRegistrator<Write> _reg3;
    uavcan::DefaultDataTypeRegistrator<Delete> _reg4;
    uavcan::DefaultDataTypeRegistrator<GetDirectoryEntryInfo> _reg5;

    InterlinkedTestNodesWithSysClock nodes;

    TestFileServerBackend backend;

    uavcan::FileServer serv(nodes.a, backend);
    std::cout << "sizeof(uavcan::FileServer): " << sizeof(uavcan::FileServer) << std::endl;

    ASSERT_LE(0, serv.start());

    // TODO TEST
}
