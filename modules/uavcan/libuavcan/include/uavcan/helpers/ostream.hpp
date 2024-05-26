/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_HELPERS_OSTREAM_HPP_INCLUDED
#define UAVCAN_HELPERS_OSTREAM_HPP_INCLUDED

#include <uavcan/util/templates.hpp>
#include <cstdio>

namespace uavcan
{
/**
 * Compact replacement for std::ostream for use on embedded systems.
 * Can be used for printing UAVCAN messages to stdout.
 *
 * Relevant discussion: https://groups.google.com/forum/#!topic/px4users/6c1CLNutN90
 *
 * Usage:
 *      OStream::instance() << "Hello world!" << OStream::endl;
 */
class UAVCAN_EXPORT OStream : uavcan::Noncopyable
{
    OStream() { }

public:
    static OStream& instance()
    {
        static OStream s;
        return s;
    }

    static OStream& endl(OStream& stream)
    {
        std::printf("\n");
        return stream;
    }
};

inline OStream& operator<<(OStream& s, long long x)          { std::printf("%lld", x);  return s; }
inline OStream& operator<<(OStream& s, unsigned long long x) { std::printf("%llu", x); return s; }

inline OStream& operator<<(OStream& s, long x)           { std::printf("%ld", x); return s; }
inline OStream& operator<<(OStream& s, unsigned long x)  { std::printf("%lu", x); return s; }

inline OStream& operator<<(OStream& s, int x)            { std::printf("%d", x);  return s; }
inline OStream& operator<<(OStream& s, unsigned int x)   { std::printf("%u", x);  return s; }

inline OStream& operator<<(OStream& s, short x)          { return operator<<(s, static_cast<int>(x)); }
inline OStream& operator<<(OStream& s, unsigned short x) { return operator<<(s, static_cast<unsigned>(x)); }

inline OStream& operator<<(OStream& s, long double x) { std::printf("%Lg", x); return s; }
inline OStream& operator<<(OStream& s, double x)      { std::printf("%g", x);  return s; }
inline OStream& operator<<(OStream& s, float x)       { return operator<<(s, static_cast<double>(x)); }

inline OStream& operator<<(OStream& s, char x)        { std::printf("%c", x); return s; }
inline OStream& operator<<(OStream& s, const char* x) { std::printf("%s", x); return s; }

inline OStream& operator<<(OStream& s, OStream&(*manip)(OStream&)) { return manip(s); }

}

#endif // UAVCAN_HELPERS_OSTREAM_HPP_INCLUDED
