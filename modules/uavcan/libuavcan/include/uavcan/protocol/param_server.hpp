/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_PARAM_SERVER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_PARAM_SERVER_HPP_INCLUDED

#include <uavcan/debug.hpp>
#include <uavcan/protocol/param/GetSet.hpp>
#include <uavcan/protocol/param/ExecuteOpcode.hpp>
#include <uavcan/node/service_server.hpp>
#include <uavcan/util/method_binder.hpp>
#include <cassert>
#include <cstdlib>

namespace uavcan
{
/**
 * Implement this interface in the application to support the standard remote reconfiguration services.
 * Refer to @ref ParamServer.
 */
class UAVCAN_EXPORT IParamManager
{
public:
    typedef typename StorageType<typename protocol::param::GetSet::Response::FieldTypes::name>::Type Name;
    typedef typename StorageType<typename protocol::param::GetSet::Request::FieldTypes::index>::Type Index;
    typedef protocol::param::Value Value;
    typedef protocol::param::NumericValue NumericValue;

    virtual ~IParamManager() { }

    /**
     * Copy the parameter name to @ref out_name if it exists, otherwise do nothing.
     */
    virtual void getParamNameByIndex(Index index, Name& out_name) const = 0;

    /**
     * Assign by name if exists.
     */
    virtual void assignParamValue(const Name& name, const Value& value) = 0;

    /**
     * Read by name if exists, otherwise do nothing.
     */
    virtual void readParamValue(const Name& name, Value& out_value) const = 0;

    /**
     * Read param's default/max/min if available.
     * Note that min/max are only applicable to numeric params.
     * Implementation is optional.
     */
    virtual void readParamDefaultMaxMin(const Name& name, Value& out_default,
                                        NumericValue& out_max, NumericValue& out_min) const
    {
        (void)name;
        (void)out_default;
        (void)out_max;
        (void)out_min;
    }

    /**
     * Save all params to non-volatile storage.
     * @return Negative if failed.
     */
    virtual int saveAllParams() = 0;

    /**
     * Clear the non-volatile storage.
     * @return Negative if failed.
     */
    virtual int eraseAllParams() = 0;
};

/**
 * Convenience class for supporting the standard configuration services.
 * Highly recommended to use.
 */
class UAVCAN_EXPORT ParamServer
{
    typedef MethodBinder<ParamServer*, void (ParamServer::*)(const protocol::param::GetSet::Request&,
                                                             protocol::param::GetSet::Response&)> GetSetCallback;

    typedef MethodBinder<ParamServer*,
                         void (ParamServer::*)(const protocol::param::ExecuteOpcode::Request&,
                                               protocol::param::ExecuteOpcode::Response&)> ExecuteOpcodeCallback;

    ServiceServer<protocol::param::GetSet, GetSetCallback> get_set_srv_;
    ServiceServer<protocol::param::ExecuteOpcode, ExecuteOpcodeCallback> save_erase_srv_;
    IParamManager* manager_;

    void handleGetSet(const protocol::param::GetSet::Request& in, protocol::param::GetSet::Response& out)
    {
        UAVCAN_ASSERT(manager_ != UAVCAN_NULLPTR);

        // Recover the name from index
        if (in.name.empty())
        {
            manager_->getParamNameByIndex(in.index, out.name);
            UAVCAN_TRACE("ParamServer", "GetSet: Index %i --> '%s'", int(in.index), out.name.c_str());
        }
        else
        {
            out.name = in.name;
        }

        if (out.name.empty())
        {
            UAVCAN_TRACE("ParamServer", "GetSet: Can't resolve parameter name, index=%i", int(in.index));
            return;
        }

        // Assign if needed, read back
        if (!in.value.is(protocol::param::Value::Tag::empty))
        {
            manager_->assignParamValue(out.name, in.value);
        }
        manager_->readParamValue(out.name, out.value);

        // Check if the value is OK, otherwise reset the name to indicate that we have no idea what is it all about
        if (!out.value.is(protocol::param::Value::Tag::empty))
        {
            manager_->readParamDefaultMaxMin(out.name, out.default_value, out.max_value, out.min_value);
        }
        else
        {
            UAVCAN_TRACE("ParamServer", "GetSet: Unknown param: index=%i name='%s'", int(in.index), out.name.c_str());
            out.name.clear();
        }
    }

    void handleExecuteOpcode(const protocol::param::ExecuteOpcode::Request& in,
                             protocol::param::ExecuteOpcode::Response& out)
    {
        UAVCAN_ASSERT(manager_ != UAVCAN_NULLPTR);

        if (in.opcode == protocol::param::ExecuteOpcode::Request::OPCODE_SAVE)
        {
            out.ok = manager_->saveAllParams() >= 0;
        }
        else if (in.opcode == protocol::param::ExecuteOpcode::Request::OPCODE_ERASE)
        {
            out.ok = manager_->eraseAllParams() >= 0;
        }
        else
        {
            UAVCAN_TRACE("ParamServer", "ExecuteOpcode: invalid opcode %i", int(in.opcode));
            out.ok = false;
        }
    }

public:
    explicit ParamServer(INode& node)
        : get_set_srv_(node)
        , save_erase_srv_(node)
        , manager_(UAVCAN_NULLPTR)
    { }

    /**
     * Starts the parameter server with given param manager instance.
     * Returns negative error code.
     */
    int start(IParamManager* manager)
    {
        if (manager == UAVCAN_NULLPTR)
        {
            return -ErrInvalidParam;
        }
        manager_ = manager;

        int res = get_set_srv_.start(GetSetCallback(this, &ParamServer::handleGetSet));
        if (res < 0)
        {
            return res;
        }

        res = save_erase_srv_.start(ExecuteOpcodeCallback(this, &ParamServer::handleExecuteOpcode));
        if (res < 0)
        {
            get_set_srv_.stop();
        }
        return res;
    }

    /**
     * @ref IParamManager
     */
    IParamManager* getParamManager() const { return manager_; }
};

}

#endif // UAVCAN_PROTOCOL_PARAM_SERVER_HPP_INCLUDED
