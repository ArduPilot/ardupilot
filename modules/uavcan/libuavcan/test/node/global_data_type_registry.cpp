/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/node/global_data_type_registry.hpp>

namespace
{

struct DataTypeAMessage
{
    enum { DefaultDataTypeID = 0 };
    enum { DataTypeKind = uavcan::DataTypeKindMessage };
    static uavcan::DataTypeSignature getDataTypeSignature() { return uavcan::DataTypeSignature(123); }
    static const char* getDataTypeFullName() { return "my_namespace.DataTypeA"; }
};

struct DataTypeAService
{
    enum { DefaultDataTypeID = 0 };
    enum { DataTypeKind = uavcan::DataTypeKindService };
    static uavcan::DataTypeSignature getDataTypeSignature() { return uavcan::DataTypeSignature(789); }
    static const char* getDataTypeFullName() { return "my_namespace.DataTypeA"; }
};

struct DataTypeB
{
    enum { DefaultDataTypeID = 42 };
    enum { DataTypeKind = uavcan::DataTypeKindMessage };
    static uavcan::DataTypeSignature getDataTypeSignature() { return uavcan::DataTypeSignature(456); }
    static const char* getDataTypeFullName() { return "my_namespace.DataTypeB"; }
};

struct DataTypeC
{
    enum { DefaultDataTypeID = 1023 };
    enum { DataTypeKind = uavcan::DataTypeKindMessage };
    static uavcan::DataTypeSignature getDataTypeSignature() { return uavcan::DataTypeSignature(654); }
    static const char* getDataTypeFullName() { return "foo.DataTypeC"; }
};

struct DataTypeD
{
    enum { DefaultDataTypeID = 43 };
    enum { DataTypeKind = uavcan::DataTypeKindService };
    static uavcan::DataTypeSignature getDataTypeSignature() { return uavcan::DataTypeSignature(987); }
    static const char* getDataTypeFullName() { return "foo.DataTypeD"; }
};

template <typename Type>
uavcan::DataTypeDescriptor extractDescriptor(uint16_t dtid = Type::DefaultDataTypeID)
{
    return uavcan::DataTypeDescriptor(uavcan::DataTypeKind(Type::DataTypeKind), dtid,
                                      Type::getDataTypeSignature(), Type::getDataTypeFullName());
}

}


TEST(GlobalDataTypeRegistry, Basic)
{
    using uavcan::GlobalDataTypeRegistry;
    using uavcan::DataTypeSignature;

    GlobalDataTypeRegistry::instance().reset();
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().isFrozen());
    ASSERT_EQ(0, GlobalDataTypeRegistry::instance().getNumMessageTypes());
    ASSERT_EQ(0, GlobalDataTypeRegistry::instance().getNumServiceTypes());

    /*
     * Static registrations
     */
    uavcan::DefaultDataTypeRegistrator<DataTypeAMessage> reg_DataTypeAMessage;
    uavcan::DefaultDataTypeRegistrator<DataTypeB> reg_DataTypeB;

    ASSERT_EQ(2, GlobalDataTypeRegistry::instance().getNumMessageTypes());
    ASSERT_EQ(0, GlobalDataTypeRegistry::instance().getNumServiceTypes());

    /*
     * Runtime registrations
     */
    ASSERT_EQ(GlobalDataTypeRegistry::RegistrationResultOk,
              GlobalDataTypeRegistry::instance().registerDataType<DataTypeAService>(
                  DataTypeAService::DefaultDataTypeID));

    ASSERT_EQ(2, GlobalDataTypeRegistry::instance().getNumMessageTypes());
    ASSERT_EQ(1, GlobalDataTypeRegistry::instance().getNumServiceTypes());

    /*
     * Runtime re-registration
     */
    ASSERT_EQ(GlobalDataTypeRegistry::RegistrationResultOk,
              GlobalDataTypeRegistry::instance().registerDataType<DataTypeAService>(147));
    ASSERT_EQ(GlobalDataTypeRegistry::RegistrationResultOk,
              GlobalDataTypeRegistry::instance().registerDataType<DataTypeB>(741));

    ASSERT_EQ(2, GlobalDataTypeRegistry::instance().getNumMessageTypes());
    ASSERT_EQ(1, GlobalDataTypeRegistry::instance().getNumServiceTypes());

    /*
     * These types will be necessary for the aggregate signature test
     */
    ASSERT_EQ(GlobalDataTypeRegistry::RegistrationResultCollision,
              GlobalDataTypeRegistry::instance().registerDataType<DataTypeC>(741));                   // ID COLLISION

    ASSERT_EQ(GlobalDataTypeRegistry::RegistrationResultOk,
              GlobalDataTypeRegistry::instance().registerDataType<DataTypeC>(DataTypeC::DefaultDataTypeID));
    uavcan::DefaultDataTypeRegistrator<DataTypeD> reg_DataTypeD;

    /*
     * Frozen state
     */
    GlobalDataTypeRegistry::instance().freeze();

    ASSERT_EQ(GlobalDataTypeRegistry::RegistrationResultFrozen,
              GlobalDataTypeRegistry::instance().registerDataType<DataTypeAService>(555)); // Rejected

    ASSERT_EQ(GlobalDataTypeRegistry::RegistrationResultFrozen,
              GlobalDataTypeRegistry::instance().registerDataType<DataTypeAMessage>(999)); // Rejected

    ASSERT_EQ(GlobalDataTypeRegistry::RegistrationResultFrozen,
              GlobalDataTypeRegistry::instance().registerDataType<DataTypeB>(888));        // Rejected

    /*
     * Searching
     */
    const uavcan::DataTypeDescriptor* pdtd = UAVCAN_NULLPTR;
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindMessage, "Nonexistent"));
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().find("Nonexistent"));
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindMessage, 987));
    // Asking for service, but this is a message:
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindService, "my_namespace.DataTypeB"));
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindService, 42));

    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindMessage,
                                                                "my_namespace.DataTypeB")));
    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find("my_namespace.DataTypeB")));
    ASSERT_EQ(extractDescriptor<DataTypeB>(741), *pdtd);
    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindMessage, 741)));
    ASSERT_EQ(extractDescriptor<DataTypeB>(741), *pdtd);

    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindMessage,
                                                                "my_namespace.DataTypeA")));
    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find("my_namespace.DataTypeA")));
    ASSERT_EQ(extractDescriptor<DataTypeAMessage>(), *pdtd);
    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindMessage, uavcan::DataTypeID(0))));
    ASSERT_EQ(extractDescriptor<DataTypeAMessage>(), *pdtd);

    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindService,
                                                                "my_namespace.DataTypeA")));
    ASSERT_EQ(extractDescriptor<DataTypeAService>(147), *pdtd);
    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindService, 147)));
    ASSERT_EQ(extractDescriptor<DataTypeAService>(147), *pdtd);
}


TEST(GlobalDataTypeRegistry, Reset)
{
    using uavcan::GlobalDataTypeRegistry;

    /*
     * Since we're dealing with singleton, we need to reset it for other tests to use
     */
    ASSERT_TRUE(GlobalDataTypeRegistry::instance().isFrozen());
    GlobalDataTypeRegistry::instance().reset();
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().isFrozen());
}
