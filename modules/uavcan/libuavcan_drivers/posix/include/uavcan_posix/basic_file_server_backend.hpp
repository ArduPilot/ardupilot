/****************************************************************************
*
*   Copyright (c) 2015, 2021 PX4 Development Team. All rights reserved.
*      Author: Pavel Kirienko <pavel.kirienko@gmail.com>
*              David Sidrane <david_s5@usa.net>
*
****************************************************************************/

#ifndef UAVCAN_POSIX_BASIC_FILE_SERVER_BACKEND_HPP_INCLUDED
#define UAVCAN_POSIX_BASIC_FILE_SERVER_BACKEND_HPP_INCLUDED

#include <sys/stat.h>
#include <cstdio>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <ctime>
#include <unistd.h>
#include <fcntl.h>

#include <uavcan/node/timer.hpp>
#include <uavcan/data_type.hpp>
#include <uavcan/protocol/file/Error.hpp>
#include <uavcan/protocol/file/EntryType.hpp>
#include <uavcan/protocol/file/Read.hpp>
#include <uavcan/protocol/file_server.hpp>
#include <uavcan/data_type.hpp>

namespace uavcan_posix
{
/**
 * This interface implements a POSIX compliant IFileServerBackend interface
 */
class BasicFileServerBackend : public uavcan::IFileServerBackend
{
    enum { FilePermissions = 438 };   ///< 0o666

protected:

    class FDCacheBase
    {
    public:
        FDCacheBase() { }
        virtual ~FDCacheBase() { }

        virtual int open(const char* path, int oflags)
        {
            using namespace std;

            return ::open(path, oflags);
        }

        virtual int close(int fd, bool done = true)
        {
            (void)done;
            using namespace std;

            return ::close(fd);
        }

        virtual void init() { }
    };

    FDCacheBase fallback_;

    class FDCache : public FDCacheBase, protected uavcan::TimerBase
    {
        /// Age in Seconds an entry will stay in the cache if not accessed.
        enum { MaxAgeSeconds = 7 };

        /// Rate in Seconds that the cache will be flushed of stale entries.
        enum { GarbageCollectionSeconds = 60 };

        IFileServerBackend::Path& alt_root_path_;
        IFileServerBackend::Path& root_path_;

        class FDCacheItem : uavcan::Noncopyable
        {
            friend FDCache;

            FDCacheItem* next_;
            std::time_t last_access_;
            const int fd_;
            const int oflags_;
            const char* const path_;

        public:
            enum { InvalidFD = -1 };

            FDCacheItem() :
                next_(UAVCAN_NULLPTR),
                last_access_(0),
                fd_(InvalidFD),
                oflags_(0),
                path_(UAVCAN_NULLPTR)
            { }

            FDCacheItem(int fd, const char* path, int oflags) :
                next_(UAVCAN_NULLPTR),
                last_access_(0),
                fd_(fd),
                oflags_(oflags),
                path_(::strndup(path, uavcan::protocol::file::Path::FieldTypes::path::MaxSize))
            { }

            ~FDCacheItem()
            {
                using namespace std;
                if (valid())
                {
                    ::free(const_cast<char*>(path_));
                }
            }

            bool valid() const
            {
                return path_ != UAVCAN_NULLPTR;
            }

            int getFD() const
            {
                return fd_;
            }

            std::time_t getAccess() const
            {
                return last_access_;
            }

            std::time_t acessed()
            {
                using namespace std;
                last_access_ = time(UAVCAN_NULLPTR);
                return getAccess();
            }

            void expire()
            {
                last_access_ = 0;
            }

            bool expired() const
            {
                using namespace std;
                return 0 == last_access_ || (time(UAVCAN_NULLPTR) - last_access_) > MaxAgeSeconds;
            }

            bool equals(const char* path, int oflags) const
            {
                using namespace std;
                return oflags_ == oflags && 0 == ::strcmp(path, path_);
            }

            bool equals(int fd) const
            {
                return fd_ == fd;
            }
        };

        FDCacheItem* head_;

        FDCacheItem* find(const char* path, int oflags)
        {
            for (FDCacheItem* pi = head_; pi; pi = pi->next_)
            {
                if (pi->equals(path, oflags))
                {
                    return pi;
                }
            }
            return UAVCAN_NULLPTR;
        }

        FDCacheItem* find(int fd)
        {
            for (FDCacheItem* pi = head_; pi; pi = pi->next_)
            {
                if (pi->equals(fd))
                {
                    return pi;
                }
            }
            return UAVCAN_NULLPTR;
        }

        FDCacheItem* add(FDCacheItem* pi)
        {
            pi->next_ = head_;
            head_ = pi;
            pi->acessed();
            return pi;
        }

        void removeExpired(FDCacheItem** pi)
        {
            while (*pi)
            {
                if ((*pi)->expired())
                {
                    FDCacheItem* next = (*pi)->next_;
                    (void)FDCacheBase::close((*pi)->fd_);
                    delete (*pi);
                    *pi = next;
                    continue;
                }
                pi = &(*pi)->next_;
            }
        }

        void remove(FDCacheItem* pi, bool done)
        {
            if (done)
            {
                pi->expire();
            }
            removeExpired(&head_);
        }

        void clear()
        {
            FDCacheItem* tmp;
            for (FDCacheItem* pi = head_; pi; pi = tmp)
            {
                tmp = pi->next_;
                (void)FDCacheBase::close(pi->fd_);
                delete pi;
            }
        }

        /* Removed stale entries. In the normal case a node will read the
         * complete contents of a file and the read of the last block will
         * cause the method remove() to be invoked with done true. Thereby
         * flushing the entry from the cache. But if the node does not
         * stay the course of the read, it may leave a dangling entry.
         * This call back handles the garbage collection.
         */
        virtual void handleTimerEvent(const uavcan::TimerEvent&)
        {
            removeExpired(&head_);
        }

    public:
        FDCache(uavcan::INode& node, IFileServerBackend::Path& root_path, IFileServerBackend::Path& alt_root_path) :
            TimerBase(node),
            alt_root_path_(alt_root_path),
            root_path_(root_path),
            head_(UAVCAN_NULLPTR)
        { }

        virtual ~FDCache()
        {
            stop();
            clear();
        }

        virtual void init()
        {
            startPeriodic(uavcan::MonotonicDuration::fromMSec(GarbageCollectionSeconds * 1000));
        }

        virtual int open(const char* path, int oflags)
        {
            int fd = FDCacheItem::InvalidFD;

            FDCacheItem* pi = find(path, oflags);

            if (pi != UAVCAN_NULLPTR)
            {
                pi->acessed();
            }
            else
            {
                Path vpath = root_path_.c_str();
                vpath += path;

                fd = FDCacheBase::open(vpath.c_str(), oflags);

                if (fd < 0)
                {
                    vpath = alt_root_path_.c_str();
                    vpath += path;
                    fd = FDCacheBase::open(vpath.c_str(), oflags);
                }

                if (fd < 0)
                {
                    return fd;
                }

                /* Allocate and clone path */

                pi = new FDCacheItem(fd, path, oflags);

                /* Allocation worked but check clone */

                if (pi && !pi->valid())
                {
                    /* Allocation worked but clone or path failed */
                    delete pi;
                    pi = UAVCAN_NULLPTR;
                }

                if (pi == UAVCAN_NULLPTR)
                {
                    /*
                     * If allocation fails no harm just can not cache it
                     * return open fd
                     */
                    return fd;
                }
                /* add new */
                add(pi);
            }
            return pi->getFD();
        }

        virtual int close(int fd, bool done)
        {
            FDCacheItem* pi = find(fd);
            if (pi == UAVCAN_NULLPTR)
            {
                /*
                 * If not found just close it
                 */
                return FDCacheBase::close(fd);
            }
            remove(pi, done);
            return 0;
        }
    };

    FDCacheBase* fdcache_;
    uavcan::INode& node_;

    FDCacheBase& getFDCache()
    {
        if (fdcache_ == UAVCAN_NULLPTR)
        {
            fdcache_ = new FDCache(node_, getRootPath(), getAltRootPath());

            if (fdcache_ == UAVCAN_NULLPTR)
            {
                fdcache_ = &fallback_;
            }

            fdcache_->init();
        }
        return *fdcache_;
    }

    /**
     * Back-end for uavcan.protocol.file.GetInfo.
     * Implementation of this method is required.
     * On success the method must return zero.
     */
    virtual uavcan::int16_t getInfo(const Path& path, uavcan::uint64_t& out_size, EntryType& out_type)
    {
        int rv = uavcan::protocol::file::Error::INVALID_VALUE;

        if (path.size() > 0)
        {

          using namespace std;

          struct stat sb;

          Path vpath = getRootPath().c_str();
          vpath += path;

          rv = stat(vpath.c_str(), &sb);
          if (rv < 0)
          {
              vpath = getAltRootPath().c_str();
              vpath += path;
              rv = stat(vpath.c_str(), &sb);
          }

          if (rv < 0)
          {
              rv = errno;
          }
          else
          {
              rv = 0;
              out_size = sb.st_size;
              out_type.flags = uavcan::protocol::file::EntryType::FLAG_READABLE;
              if (S_ISDIR(sb.st_mode))
              {
                  out_type.flags |= uavcan::protocol::file::EntryType::FLAG_DIRECTORY;
              }
              else if (S_ISREG(sb.st_mode))
              {
                  out_type.flags |= uavcan::protocol::file::EntryType::FLAG_FILE;
              }
              // TODO Using fixed flag FLAG_READABLE until we add file permission checks to return actual value.
          }
        }
        return rv;
    }

    /**
     * Back-end for uavcan.protocol.file.Read.
     * Implementation of this method is required.
     * @ref inout_size is set to @ref ReadSize; read operation is required to return exactly this amount, except
     * if the end of file is reached.
     * On success the method must return zero.
     */
    virtual uavcan::int16_t read(const Path& path, const uavcan::uint64_t offset, uavcan::uint8_t* out_buffer,
                                 uavcan::uint16_t& inout_size)
    {
        int rv = uavcan::protocol::file::Error::INVALID_VALUE;

        if (path.size() > 0 && inout_size != 0)
        {
            using namespace std;

            FDCacheBase& cache = getFDCache();

            int fd = cache.open(path.c_str(), O_RDONLY);

            if (fd < 0)
            {
                rv = -errno;
            }
            else
            {
                ssize_t total_read = 0;

                rv = ::lseek(fd, offset, SEEK_SET);

                if (rv < 0)
                {
                    rv = -errno;
                }
                else
                {
                    rv = 0;
                    ssize_t remaining = inout_size;
                    ssize_t nread = 0;
                    do
                    {
                        nread = ::read(fd, &out_buffer[total_read], remaining);
                        if (nread < 0)
                        {
                            rv = errno;
                        }
                        else
                        {
                            remaining -= nread,
                            total_read += nread;
                        }
                    }
                    while (nread > 0 && remaining > 0);
                }

                (void)cache.close(fd, rv != 0 || total_read != inout_size);
                inout_size = total_read;
            }
        }
        return rv;
    }

public:
    BasicFileServerBackend(uavcan::INode& node) :
        fdcache_(UAVCAN_NULLPTR),
        node_(node)
    { }

    ~BasicFileServerBackend()
    {
        if (fdcache_ != &fallback_)
        {
            delete fdcache_;
            fdcache_ = UAVCAN_NULLPTR;
        }
    }
};

#if __GNUC__
/// Typo fix in a backwards-compatible way (only for GCC projects). Will be removed someday.
typedef BasicFileServerBackend
        BasicFileSeverBackend           // Missing 'r'
        __attribute__((deprecated));
#endif

}

#endif // Include guard
