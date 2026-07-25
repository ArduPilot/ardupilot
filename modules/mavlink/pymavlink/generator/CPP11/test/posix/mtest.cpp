/*
 * MAVLink C++11 test runner.
 *
 * Run command (ubuntu):

 g++ -std=c++11 -I mavlink/common -I /usr/src/gtest -pthread mtest.cpp /usr/src/gtest/src/gtest-all.cc && ./a.out

 * NOTE:
 *  - mavlink is a directory with geterated C and C++ library.
 *  - by changing "common" to other dialect may select another messages
 */

#include <iostream>
#include <array>

//#define PRINT_MSG(m)	print_msg(m)
namespace mavlink {
struct __mavlink_message;
void print_msg(struct __mavlink_message &m);
}

// Since C++11 do not have things like std::to_array() which needs C++14 features
template<size_t _N>
std::array<char, _N> to_char_array(const char (&a)[_N])
{
	std::array<char, _N> out{};
	std::copy(a, a+_N, out.begin());
	return out;
}

#define TEST_INTEROP
#include "gtestsuite.hpp"

const mavlink::mavlink_msg_entry_t *mavlink::mavlink_get_msg_entry(uint32_t msgid)
{
	return nullptr;
}

void mavlink::print_msg(mavlink_message_t &m)
{
	std::cout << std::hex << std::setfill('0')
		<< "msgid: " << std::setw(6) << m.msgid
		<< " len: " << std::setw(2) << +m.len
		<< " crc: " << std::setw(4) << m.checksum
		<< " p:";
	for (size_t i = 0; i < MAVLINK_MAX_PAYLOAD_LEN; i++)
		std::cout << " " << std::hex << std::setw(2) << std::setfill('0') << +(_MAV_PAYLOAD(&m)[i]);

	std::cout << std::endl;
}

int main(int argc, char *argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
