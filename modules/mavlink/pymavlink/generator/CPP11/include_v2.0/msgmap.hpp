
#pragma once

#include <algorithm>
#ifdef FREEBSD
#include <sys/endian.h>
#elif __APPLE__
#include <machine/endian.h>
#else
#include <endian.h>
#endif
#include <type_traits>

namespace mavlink {

/**
 * Serialization helper wrapper for mavlink_message_t
 */
class MsgMap {
public:

	explicit MsgMap(mavlink_message_t *p) :
		msg(p), cmsg(p), pos(0)
	{ }

	explicit MsgMap(mavlink_message_t &p) :
		msg(&p), cmsg(&p), pos(0)
	{ }

	explicit MsgMap(const mavlink_message_t *p) :
		msg(nullptr), cmsg(p), pos(0)
	{ }

	inline void reset()
	{
		pos = 0;
	}

	inline void reset(uint32_t msgid, uint8_t len)
	{
		assert(msg);

		msg->msgid = msgid;	// necessary for finalize
		msg->len = len;		// needed only for deserialization w/o finalize
		pos = 0;
	}

	template<typename _T>
	void operator<< (const _T data);

	template<class _T, size_t _Size>
	void operator<< (const std::array<_T, _Size> &data);

	template<typename _T>
	void operator>> (_T &data);

	template<class _T, size_t _Size>
	void operator>> (std::array<_T, _Size> &data);

private:
	mavlink_message_t *msg;		// for serialization
	const mavlink_message_t *cmsg;	// for deserialization
	size_t pos;
};

namespace impl {

template<size_t _N>
struct UintBufferHelper;

template<>
struct UintBufferHelper<1>
{
    typedef uint8_t Type;
};

template<>
struct UintBufferHelper<2>
{
    typedef uint16_t Type;
};

template<>
struct UintBufferHelper<4>
{
    typedef uint32_t Type;
};

template<>
struct UintBufferHelper<8>
{
    typedef uint64_t Type;
};

template<typename _T>
struct UintBuffer
{
    typedef typename UintBufferHelper<sizeof(_T)>::Type Type;
};


template<typename _T>
_T to_little_endian_internal(_T);

template<>
inline uint8_t to_little_endian_internal<uint8_t>(uint8_t data)
{
    return data;
}

template<>
inline uint16_t to_little_endian_internal<uint16_t>(uint16_t data)
{
    return htole16(data);
}

template<>
inline uint32_t to_little_endian_internal<uint32_t>(uint32_t data)
{
    return htole32(data);
}

template<>
inline uint64_t to_little_endian_internal<uint64_t>(uint64_t data)
{
    return htole64(data);
}

template<typename _T>
typename std::enable_if<std::is_floating_point<_T>::value, typename UintBuffer<_T>::Type>::type to_little_endian(_T data)
{
    typedef typename UintBuffer<_T>::Type ReturnType;
    ReturnType buf;
    memcpy(&buf, &data, sizeof(ReturnType));
    return to_little_endian_internal<ReturnType>(buf);
}

template<typename _T>
typename std::enable_if<std::is_integral<_T>::value, typename UintBuffer<_T>::Type>::type to_little_endian(_T data)
{
    return to_little_endian_internal<typename UintBuffer<_T>::Type>(data);
}

template<typename _T>
_T to_host_from_little_endian_internal(_T);

template<>
inline uint8_t to_host_from_little_endian_internal<uint8_t>(uint8_t data)
{
    return data;
}

template<>
inline uint16_t to_host_from_little_endian_internal<uint16_t>(uint16_t data)
{
    return le16toh(data);
}

template<>
inline uint32_t to_host_from_little_endian_internal<uint32_t>(uint32_t data)
{
    return le32toh(data);
}

template<>
inline uint64_t to_host_from_little_endian_internal<uint64_t>(uint64_t data)
{
    return le64toh(data);
}

template<typename _TOutput, typename _TInput,
         class = typename std::enable_if<std::is_unsigned<_TInput>::value>::type>
typename std::enable_if<std::is_floating_point<_TOutput>::value, _TOutput>::type to_host_from_little_endian(_TInput data)
{
    static_assert(sizeof(_TInput) == sizeof(_TOutput), "Size of input and output must match");
    data = to_host_from_little_endian_internal(data);

    _TOutput buf;
    memcpy(&buf, &data, sizeof(_TOutput));
    return buf;
}

template<typename _TOutput, typename _TInput,
         class = typename std::enable_if<std::is_unsigned<_TInput>::value>::type>
typename std::enable_if<std::is_integral<_TOutput>::value, _TOutput>::type to_host_from_little_endian(_TInput data)
{
    static_assert(sizeof(_TInput) == sizeof(_TOutput), "Size of input and output must match");
    return to_host_from_little_endian_internal(data);
}

} // namespace impl
} // namespace mavlink

// implementation

template<typename _T>
void mavlink::MsgMap::operator<< (const _T data)
{
    assert(msg);
    assert(pos + sizeof(_T) <= MAVLINK_MAX_PAYLOAD_LEN);

    auto data_le = mavlink::impl::to_little_endian<_T>(data);
    memcpy(&_MAV_PAYLOAD_NON_CONST(msg)[pos], &data_le, sizeof(data_le));
    pos += sizeof(_T);
}

template<class _T, size_t _Size>
void mavlink::MsgMap::operator<< (const std::array<_T, _Size> &data)
{
	for (auto &v : data) {
		*this << v;
	}
}

template<typename _T>
void mavlink::MsgMap::operator>> (_T &data)
{
    assert(cmsg);
    assert(pos + sizeof(_T) <= MAVLINK_MAX_PAYLOAD_LEN);

    ssize_t remaining_non_zero_data = cmsg->len - pos;
    typename mavlink::impl::UintBuffer<_T>::Type buf;

    if (static_cast<ssize_t>(sizeof(_T)) <= remaining_non_zero_data) { // field is not truncated
        memcpy(&buf, &_MAV_PAYLOAD(cmsg)[pos], sizeof(_T));
    } else if (remaining_non_zero_data <= 0) {
        // there is no non-zero data left, so just fill with 0
        buf = 0;
    } else { // field is trimmed - pad with zeroes
        // here remaining_non_zero_data < sizeof(_T) holds
        size_t non_zero_count = std::max<decltype(remaining_non_zero_data)>(remaining_non_zero_data, 0);
        size_t pad_zero_count = sizeof(_T) - non_zero_count;

        std::array<char, sizeof(_T)> raw_buf;
        memcpy(raw_buf.data(), &_MAV_PAYLOAD(cmsg)[pos], non_zero_count);
        memset(raw_buf.data() + non_zero_count, 0, pad_zero_count);

        memcpy(&buf, raw_buf.data(), raw_buf.size());
    }

    data = mavlink::impl::to_host_from_little_endian<_T>(buf);
    pos += sizeof(_T);
}

template<class _T, size_t _Size>
void mavlink::MsgMap::operator>> (std::array<_T, _Size> &data)
{
	for (auto &v : data) {
		*this >> v;
	}
}
