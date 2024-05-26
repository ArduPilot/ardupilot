/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_TIME_HPP_INCLUDED
#define UAVCAN_TIME_HPP_INCLUDED

#include <cstdio>
#include <uavcan/std.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/util/templates.hpp>
#include <uavcan/Timestamp.hpp>


namespace uavcan
{

template <typename D>
class DurationBase
{
    int64_t usec_;

protected:
    ~DurationBase() { }

    DurationBase()
        : usec_(0)
    {
        StaticAssert<(sizeof(D) == 8)>::check();
    }

public:
    static D getInfinite() { return fromUSec(NumericTraits<int64_t>::max()); }

    static D fromUSec(int64_t us)
    {
        D d;
        d.usec_ = us;
        return d;
    }
    static D fromMSec(int64_t ms) { return fromUSec(ms * 1000); }

    int64_t toUSec() const { return usec_; }
    int64_t toMSec() const { return usec_ / 1000; }

    D getAbs() const { return D::fromUSec((usec_ < 0) ? (-usec_) : usec_); }

    bool isPositive() const { return usec_ > 0; }
    bool isNegative() const { return usec_ < 0; }
    bool isZero() const { return usec_ == 0; }

    bool operator==(const D& r) const { return usec_ == r.usec_; }
    bool operator!=(const D& r) const { return !operator==(r); }

    bool operator<(const D& r) const { return usec_ < r.usec_; }
    bool operator>(const D& r) const { return usec_ > r.usec_; }
    bool operator<=(const D& r) const { return usec_ <= r.usec_; }
    bool operator>=(const D& r) const { return usec_ >= r.usec_; }

    D operator+(const D& r) const { return fromUSec(usec_ + r.usec_); } // TODO: overflow check
    D operator-(const D& r) const { return fromUSec(usec_ - r.usec_); } // ditto

    D operator-() const { return fromUSec(-usec_); }

    D& operator+=(const D& r)
    {
        *this = *this + r;
        return *static_cast<D*>(this);
    }
    D& operator-=(const D& r)
    {
        *this = *this - r;
        return *static_cast<D*>(this);
    }

    template <typename Scale>
    D operator*(Scale scale)   const { return fromUSec(usec_ * scale); }

    template <typename Scale>
    D& operator*=(Scale scale)
    {
        *this = *this * scale;
        return *static_cast<D*>(this);
    }

    static const unsigned StringBufSize = 32;
    void toString(char buf[StringBufSize]) const; ///< Prints time in seconds with microsecond resolution
#if UAVCAN_TOSTRING
    std::string toString() const;                 ///< Prints time in seconds with microsecond resolution
#endif
};


template <typename T, typename D>
class TimeBase
{
    uint64_t usec_;

protected:
    ~TimeBase() { }

    TimeBase()
        : usec_(0)
    {
        StaticAssert<(sizeof(T) == 8)>::check();
        StaticAssert<(sizeof(D) == 8)>::check();
    }

public:
    static T getMax() { return fromUSec(NumericTraits<uint64_t>::max()); }

    static T fromUSec(uint64_t us)
    {
        T d;
        d.usec_ = us;
        return d;
    }
    static T fromMSec(uint64_t ms) { return fromUSec(ms * 1000); }

    uint64_t toUSec() const { return usec_; }
    uint64_t toMSec() const { return usec_ / 1000; }

    bool isZero() const { return usec_ == 0; }

    bool operator==(const T& r) const { return usec_ == r.usec_; }
    bool operator!=(const T& r) const { return !operator==(r); }

    bool operator<(const T& r) const { return usec_ < r.usec_; }
    bool operator>(const T& r) const { return usec_ > r.usec_; }
    bool operator<=(const T& r) const { return usec_ <= r.usec_; }
    bool operator>=(const T& r) const { return usec_ >= r.usec_; }

    T operator+(const D& r) const
    {
        if (r.isNegative())
        {
            if (uint64_t(r.getAbs().toUSec()) > usec_)
            {
                return fromUSec(0);
            }
        }
        else
        {
            if (uint64_t(int64_t(usec_) + r.toUSec()) < usec_)
            {
                return fromUSec(NumericTraits<uint64_t>::max());
            }
        }
        return fromUSec(uint64_t(int64_t(usec_) + r.toUSec()));
    }

    T operator-(const D& r) const
    {
        return *static_cast<const T*>(this) + (-r);
    }
    D operator-(const T& r) const
    {
        return D::fromUSec((usec_ > r.usec_) ? int64_t(usec_ - r.usec_) : -int64_t(r.usec_ - usec_));
    }

    T& operator+=(const D& r)
    {
        *this = *this + r;
        return *static_cast<T*>(this);
    }
    T& operator-=(const D& r)
    {
        *this = *this - r;
        return *static_cast<T*>(this);
    }

    static const unsigned StringBufSize = 32;
    void toString(char buf[StringBufSize]) const; ///< Prints time in seconds with microsecond resolution
#if UAVCAN_TOSTRING
    std::string toString() const;                 ///< Prints time in seconds with microsecond resolution
#endif
};

/*
 * Monotonic
 */
class UAVCAN_EXPORT MonotonicDuration : public DurationBase<MonotonicDuration> { };

class UAVCAN_EXPORT MonotonicTime : public TimeBase<MonotonicTime, MonotonicDuration> { };

/*
 * UTC
 */
class UAVCAN_EXPORT UtcDuration : public DurationBase<UtcDuration> { };

class UAVCAN_EXPORT UtcTime : public TimeBase<UtcTime, UtcDuration> /// Implicitly convertible to/from uavcan.Timestamp
{
public:
    UtcTime() { }

    UtcTime(const Timestamp& ts)  // Implicit
    {
        operator=(ts);
    }

    UtcTime& operator=(const Timestamp& ts)
    {
        *this = fromUSec(ts.usec);
        return *this;
    }

    operator Timestamp() const
    {
        Timestamp ts;
        ts.usec = toUSec();
        return ts;
    }
};

// ----------------------------------------------------------------------------

template <typename D>
const unsigned DurationBase<D>::StringBufSize;

template <typename T, typename D>
const unsigned TimeBase<T, D>::StringBufSize;

template <typename D>
void DurationBase<D>::toString(char buf[StringBufSize]) const
{
    char* ptr = buf;
    if (isNegative())
    {
        *ptr++ = '-';
    }
    (void)snprintf(ptr, StringBufSize - 1, "%llu.%06lu",
                   static_cast<unsigned long long>(getAbs().toUSec() / 1000000L),
                   static_cast<unsigned long>(getAbs().toUSec() % 1000000L));
}


template <typename T, typename D>
void TimeBase<T, D>::toString(char buf[StringBufSize]) const
{
    (void)snprintf(buf, StringBufSize, "%llu.%06lu",
                   static_cast<unsigned long long>(toUSec() / 1000000UL),
                   static_cast<unsigned long>(toUSec() % 1000000UL));
}


#if UAVCAN_TOSTRING

template <typename D>
std::string DurationBase<D>::toString() const
{
    char buf[StringBufSize];
    toString(buf);
    return std::string(buf);
}

template <typename T, typename D>
std::string TimeBase<T, D>::toString() const
{
    char buf[StringBufSize];
    toString(buf);
    return std::string(buf);
}

#endif


template <typename Stream, typename D>
UAVCAN_EXPORT
Stream& operator<<(Stream& s, const DurationBase<D>& d)
{
    char buf[DurationBase<D>::StringBufSize];
    d.toString(buf);
    s << buf;
    return s;
}

template <typename Stream, typename T, typename D>
UAVCAN_EXPORT
Stream& operator<<(Stream& s, const TimeBase<T, D>& t)
{
    char buf[TimeBase<T, D>::StringBufSize];
    t.toString(buf);
    s << buf;
    return s;
}

}

#endif // UAVCAN_TIME_HPP_INCLUDED
