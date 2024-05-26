/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>,
 *                    Ilia  Sheremet <illia.sheremet@gmail.com>
 */

#ifndef UAVCAN_TRANSPORT_CAN_ACCEPTANCE_FILTER_CONFIGURATOR_HPP_INCLUDED
#define UAVCAN_TRANSPORT_CAN_ACCEPTANCE_FILTER_CONFIGURATOR_HPP_INCLUDED

#include <cassert>
#include <uavcan/data_type.hpp>
#include <uavcan/error.hpp>
#include <uavcan/transport/dispatcher.hpp>
#include <uavcan/node/abstract_node.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/util/multiset.hpp>

namespace uavcan
{
/**
 * This class configures hardware acceptance filters (if this feature is present on the particular CAN driver) to
 * preclude reception of irrelevant CAN frames on the hardware level.
 *
 * Configuration starts by creating an object of class @ref CanAcceptanceFilterConfigurator on the stack.
 * By means of computeConfiguration() method the class determines the number of available HW filters and the number
 * of listeners. In case if custom configuration required, it is possible to add it through addFilterConfig().
 * Subsequently obtained configurations are then loaded into the CAN driver by calling the applyConfiguration() method.
 * If the cumulative number of configurations obtained by computeConfiguration() and addFilterConfig() is higher than
 * the number of available HW filters, configurations will be merged automatically in the most efficient way.
 *
 * Note that if the application adds additional server or subscriber objects after the filters have been configured,
 * the configuration procedure will have to be performed again.
 *
 * The maximum number of CAN acceptance filters is predefined in uavcan/build_config.hpp through a constant
 * @ref MaxCanAcceptanceFilters. The algorithm doesn't allow to have higher number of HW filters configurations than
 * defined by MaxCanAcceptanceFilters. You can change this value according to the number specified in your CAN driver
 * datasheet.
 */
class CanAcceptanceFilterConfigurator
{
public:
    /**
     * These arguments defines whether acceptance filter configuration has anonymous messages or not
     */
    enum AnonymousMessages
    {
        AcceptAnonymousMessages,
        IgnoreAnonymousMessages
    };

private:
    /**
     * Below constants based on UAVCAN transport layer specification. Masks and ID's depends on message
     * TypeID, TransferID (RequestNotResponse - for service types, ServiceNotMessage - for all types of messages).
     * For more details refer to uavcan.org/CAN_bus_transport_layer_specification.
     * For clarity let's represent "i" as Data Type ID and "d" as Destination Node Id
     * DefaultFilterMsgMask = 00000 11111111 11111111 10000000
     * DefaultFilterMsgID   = 00000 iiiiiiii iiiiiiii 00000000, no need to explicitly define, since MsgID initialized
     * as 0.
     * DefaultFilterServiceMask = 00000 00000000 01111111 10000000
     * DefaultFilterServiceID   = 00000 00000000 0ddddddd 10000000, all Service Response Frames are accepted by
     * HW filters.
     * DefaultAnonMsgMask = 00000 00000000 00000000 11111111
     * DefaultAnonMsgID   = 00000 00000000 00000000 00000000, by default the config is added to accept all anonymous
     * frames. In case there are no anonymous messages, invoke computeConfiguration(IgnoreAnonymousMessages).
     */
    static const unsigned DefaultFilterMsgMask = 0xFFFF80;
    static const unsigned DefaultFilterServiceMask = 0x7F80;
    static const unsigned DefaultFilterServiceID = 0x80;
    static const unsigned DefaultAnonMsgMask = 0xFF;
    static const unsigned DefaultAnonMsgID = 0x0;

    typedef uavcan::Multiset<CanFilterConfig> MultisetConfigContainer;

    static CanFilterConfig mergeFilters(CanFilterConfig& a_, CanFilterConfig& b_);
    static uint8_t countBits(uint32_t n_);
    uint16_t getNumFilters() const;

    /**
     * Fills the multiset_configs_ to proceed it with mergeConfigurations()
     */
    int16_t loadInputConfiguration(AnonymousMessages load_mode);

    /**
     * This method merges several listeners's filter configurations by predetermined algorithm
     * if number of available hardware acceptance filters less than number of listeners
     */
    int16_t mergeConfigurations();

    INode& node_;               //< Node reference is needed for access to ICanDriver and Dispatcher
    MultisetConfigContainer multiset_configs_;
    uint16_t filters_number_;

public:
    /**
     * @param node              Libuavcan node whose subscribers/servers/etc will be used to configure the filters.
     *
     * @param filters_number    Allows to override the maximum number of hardware filters to use.
     *                          If set to zero (which is default), the class will obtain the number of available
     *                          filters from the CAN driver via @ref ICanIface::getNumFilters().
     */
    explicit CanAcceptanceFilterConfigurator(INode& node, uint16_t filters_number = 0)
        : node_(node)
        , multiset_configs_(node.getAllocator())
        , filters_number_(filters_number)
    { }

    /**
     * This method invokes loadInputConfiguration() and mergeConfigurations() consequently
     * in order to comute optimal filter configurations for the current hardware.
     *
     * It can only be invoked when all of the subscriber and server objects are initialized.
     * If new subscriber or server objects are added later, the filters will have to be reconfigured again.
     *
     * @param mode Either: AcceptAnonymousMessages - the filters will accept all anonymous messages (this is default)
     *                     IgnoreAnonymousMessages - anonymous messages will be ignored
     * @return 0 = success, negative for error.
     */
    int computeConfiguration(AnonymousMessages mode = AcceptAnonymousMessages);

    /**
     * Add an additional filter configuration.
     * This method must not be invoked after @ref computeConfiguration().
     * @return 0 = success, negative for error.
     */
    int addFilterConfig(const CanFilterConfig& config);

    /**
     * This method loads the configuration computed with mergeConfigurations() or explicitly added by addFilterConfig()
     * to the CAN driver. Must be called after computeConfiguration() and addFilterConfig().
     * @return 0 = success, negative for error.
     */
    int applyConfiguration();

    /**
     * Returns the configuration computed with mergeConfigurations() or added by addFilterConfig().
     * If mergeConfigurations() or addFilterConfig() have not been called yet, an empty configuration will be returned.
     */
    const MultisetConfigContainer& getConfiguration() const
    {
        return multiset_configs_;
    }
};

/**
 * This function is a shortcut for @ref CanAcceptanceFilterConfigurator.
 * It allows to compute filter configuration and then apply it in just one step.
 * It implements only the most common use case; if you have special requirements,
 * use @ref CanAcceptanceFilterConfigurator directly.
 *
 * @param node  Refer to @ref CanAcceptanceFilterConfigurator constructor for explanation.
 * @param mode  Refer to @ref CanAcceptanceFilterConfigurator::computeConfiguration() for explanation.
 * @return non-negative on success, negative error code on error.
 */
static inline int configureCanAcceptanceFilters(INode& node,
                                                CanAcceptanceFilterConfigurator::AnonymousMessages mode
                                                    = CanAcceptanceFilterConfigurator::AcceptAnonymousMessages)
{
    CanAcceptanceFilterConfigurator cfger(node);

    const int compute_res = cfger.computeConfiguration(mode);
    if (compute_res < 0)
    {
        return compute_res;
    }

    return cfger.applyConfiguration();
}

}

#endif
