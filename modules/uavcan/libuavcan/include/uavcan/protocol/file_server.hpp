/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_FILE_SERVER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_FILE_SERVER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/node/service_server.hpp>
#include <uavcan/util/method_binder.hpp>
// UAVCAN types
#include <uavcan/protocol/file/GetInfo.hpp>
#include <uavcan/protocol/file/GetDirectoryEntryInfo.hpp>
#include <uavcan/protocol/file/Read.hpp>
#include <uavcan/protocol/file/Write.hpp>
#include <uavcan/protocol/file/Delete.hpp>

namespace uavcan
{
/**
 * The file server backend should implement this interface.
 * Note that error codes returned by these methods are defined in uavcan.protocol.file.Error; these are
 * not the same as libuavcan-internal error codes defined in uavcan.error.hpp.
 */
class UAVCAN_EXPORT IFileServerBackend
{

public:
    typedef protocol::file::Path::FieldTypes::path Path;
    typedef protocol::file::EntryType EntryType;
    typedef protocol::file::Error Error;

    IFileServerBackend::Path root_path_;
    IFileServerBackend::Path alt_root_path_;

    /**
     * All read operations must return this number of bytes, unless end of file is reached.
     */
    enum { ReadSize = protocol::file::Read::Response::FieldTypes::data::MaxSize };

    /**
     * Shortcut for uavcan.protocol.file.Path.SEPARATOR.
     */
    static char getPathSeparator() { return static_cast<char>(protocol::file::Path::SEPARATOR); }

    /**
     * Set a base path to the files.
     */
    void setRootPath(const char * path)
    {
      if (path)
      {
        root_path_.clear();
        root_path_ = path;
        if (root_path_.back() != getPathSeparator())
        {
            root_path_.push_back(getPathSeparator());
        }
      }
    }

    void setAltRootPath(const char * path)
    {
      if (path)
      {
          alt_root_path_.clear();
        alt_root_path_ = path;
        if (alt_root_path_.back() != getPathSeparator())
        {
            alt_root_path_.push_back(getPathSeparator());
        }
      }
    }

    /**
     * Get a base path to the files.
     */
    Path&  getRootPath()
    {
      return root_path_;
    }

    /**
     * Get a base path to the files.
     */
    Path&  getAltRootPath()
    {
      return alt_root_path_;
    }

    /**
     * Backend for uavcan.protocol.file.GetInfo.
     * Refer to uavcan.protocol.file.EntryType for the list of available bit flags.
     * Implementation of this method is required.
     * On success the method must return zero.
     */
    virtual int16_t getInfo(const Path& path, uint64_t& out_size, EntryType& out_type) = 0;

    /**
     * Backend for uavcan.protocol.file.Read.
     * Implementation of this method is required.
     * @ref inout_size is set to @ref ReadSize; read operation is required to return exactly this amount, except
     * if the end of file is reached.
     * On success the method must return zero.
     */
    virtual int16_t read(const Path& path, const uint64_t offset, uint8_t* out_buffer, uint16_t& inout_size) = 0;

    // Methods below are optional.

    /**
     * Backend for uavcan.protocol.file.Write.
     * Implementation of this method is NOT required; by default it returns uavcan.protocol.file.Error.NOT_IMPLEMENTED.
     * On success the method must return zero.
     */
    virtual int16_t write(const Path& path, const uint64_t offset, const uint8_t* buffer, const uint16_t size)
    {
        (void)path;
        (void)offset;
        (void)buffer;
        (void)size;
        return Error::NOT_IMPLEMENTED;
    }

    /**
     * Backend for uavcan.protocol.file.Delete. ('delete' is a C++ keyword, so 'remove' is used instead)
     * Implementation of this method is NOT required; by default it returns uavcan.protocol.file.Error.NOT_IMPLEMENTED.
     * On success the method must return zero.
     */
    virtual int16_t remove(const Path& path)
    {
        (void)path;
        return Error::NOT_IMPLEMENTED;
    }

    /**
     * Backend for uavcan.protocol.file.GetDirectoryEntryInfo.
     * Refer to uavcan.protocol.file.EntryType for the list of available bit flags.
     * Implementation of this method is NOT required; by default it returns uavcan.protocol.file.Error.NOT_IMPLEMENTED.
     * On success the method must return zero.
     */
    virtual int16_t getDirectoryEntryInfo(const Path& directory_path, const uint32_t entry_index,
                                          EntryType& out_type, Path& out_entry_full_path)
    {
        (void)directory_path;
        (void)entry_index;
        (void)out_type;
        (void)out_entry_full_path;
        return Error::NOT_IMPLEMENTED;
    }

    virtual ~IFileServerBackend() { }
};

/**
 * Basic file server implements only the following services:
 *      uavcan.protocol.file.GetInfo
 *      uavcan.protocol.file.Read
 * Also see @ref IFileServerBackend.
 */
class BasicFileServer
{
    typedef MethodBinder<BasicFileServer*,
        void (BasicFileServer::*)(const protocol::file::GetInfo::Request&, protocol::file::GetInfo::Response&)>
            GetInfoCallback;

    typedef MethodBinder<BasicFileServer*,
        void (BasicFileServer::*)(const protocol::file::Read::Request&, protocol::file::Read::Response&)>
            ReadCallback;

    ServiceServer<protocol::file::GetInfo, GetInfoCallback> get_info_srv_;
    ServiceServer<protocol::file::Read, ReadCallback> read_srv_;

    void handleGetInfo(const protocol::file::GetInfo::Request& req, protocol::file::GetInfo::Response& resp)
    {
        resp.error.value = backend_.getInfo(req.path.path, resp.size, resp.entry_type);
    }

    void handleRead(const protocol::file::Read::Request& req, protocol::file::Read::Response& resp)
    {
        uint16_t inout_size = resp.data.capacity();

        resp.data.resize(inout_size);

        resp.error.value = backend_.read(req.path.path, req.offset, resp.data.begin(), inout_size);

        if (resp.error.value != protocol::file::Error::OK)
        {
            inout_size = 0;
        }

        if (inout_size > resp.data.capacity())
        {
            UAVCAN_ASSERT(0);
            resp.error.value = protocol::file::Error::UNKNOWN_ERROR;
        }
        else
        {
            resp.data.resize(inout_size);
        }
    }

protected:
    IFileServerBackend& backend_;       ///< Derived types can use it

public:
    BasicFileServer(INode& node, IFileServerBackend& backend)
        : get_info_srv_(node)
        , read_srv_(node)
        , backend_(backend)
    { }

    int start(const char* server_root = UAVCAN_NULLPTR, const char* server_alt_root = UAVCAN_NULLPTR)
    {
      backend_.setRootPath(server_root);
      backend_.setAltRootPath(server_alt_root);

        int res = get_info_srv_.start(GetInfoCallback(this, &BasicFileServer::handleGetInfo));
        if (res < 0)
        {
            return res;
        }

        res = read_srv_.start(ReadCallback(this, &BasicFileServer::handleRead));
        if (res < 0)
        {
            return res;
        }

        return 0;
    }
};

/**
 * Full file server implements all file services:
 *      uavcan.protocol.file.GetInfo
 *      uavcan.protocol.file.Read
 *      uavcan.protocol.file.Write
 *      uavcan.protocol.file.Delete
 *      uavcan.protocol.file.GetDirectoryEntryInfo
 * Also see @ref IFileServerBackend.
 */
class FileServer : protected BasicFileServer
{
    typedef MethodBinder<FileServer*,
        void (FileServer::*)(const protocol::file::Write::Request&, protocol::file::Write::Response&)>
            WriteCallback;

    typedef MethodBinder<FileServer*,
        void (FileServer::*)(const protocol::file::Delete::Request&, protocol::file::Delete::Response&)>
            DeleteCallback;

    typedef MethodBinder<FileServer*,
        void (FileServer::*)(const protocol::file::GetDirectoryEntryInfo::Request&,
                             protocol::file::GetDirectoryEntryInfo::Response&)>
            GetDirectoryEntryInfoCallback;

    ServiceServer<protocol::file::Write, WriteCallback> write_srv_;
    ServiceServer<protocol::file::Delete, DeleteCallback> delete_srv_;
    ServiceServer<protocol::file::GetDirectoryEntryInfo, GetDirectoryEntryInfoCallback> get_directory_entry_info_srv_;

    void handleWrite(const protocol::file::Write::Request& req, protocol::file::Write::Response& resp)
    {
        resp.error.value = backend_.write(req.path.path, req.offset, req.data.begin(), req.data.size());
    }

    void handleDelete(const protocol::file::Delete::Request& req, protocol::file::Delete::Response& resp)
    {
        resp.error.value = backend_.remove(req.path.path);
    }

    void handleGetDirectoryEntryInfo(const protocol::file::GetDirectoryEntryInfo::Request& req,
                                     protocol::file::GetDirectoryEntryInfo::Response& resp)
    {
        resp.error.value = backend_.getDirectoryEntryInfo(req.directory_path.path, req.entry_index,
                                                          resp.entry_type, resp.entry_full_path.path);
    }


public:
    FileServer(INode& node, IFileServerBackend& backend)
        : BasicFileServer(node, backend)
        , write_srv_(node)
        , delete_srv_(node)
        , get_directory_entry_info_srv_(node)
    { }

    int start()
    {
        int res = BasicFileServer::start();
        if (res < 0)
        {
            return res;
        }

        res = write_srv_.start(WriteCallback(this, &FileServer::handleWrite));
        if (res < 0)
        {
            return res;
        }

        res = delete_srv_.start(DeleteCallback(this, &FileServer::handleDelete));
        if (res < 0)
        {
            return res;
        }

        res = get_directory_entry_info_srv_.start(
            GetDirectoryEntryInfoCallback(this, &FileServer::handleGetDirectoryEntryInfo));
        if (res < 0)
        {
            return res;
        }

        return 0;
    }
};

}

#endif // Include guard
