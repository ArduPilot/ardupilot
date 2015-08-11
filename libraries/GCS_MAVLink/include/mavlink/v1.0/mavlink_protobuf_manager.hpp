#ifndef MAVLINKPROTOBUFMANAGER_HPP
#define MAVLINKPROTOBUFMANAGER_HPP

#include <deque>
#include <google/protobuf/message.h>
#include <iostream>
#include <tr1/memory>

#include "checksum.h"
#include "common/mavlink.h"
#include "mavlink_types.h"
#include <pixhawk/pixhawk.pb.h>

namespace mavlink
{

class ProtobufManager
{
public:
	ProtobufManager()
	 : mRegisteredTypeCount(0)
	 , mStreamID(0)
	 , mVerbose(false)
	 , kExtendedHeaderSize(MAVLINK_EXTENDED_HEADER_LEN)
	 , kExtendedPayloadMaxSize(MAVLINK_MAX_EXTENDED_PAYLOAD_LEN)
	{
		// register GLOverlay
		{
			std::tr1::shared_ptr<px::GLOverlay> msg(new px::GLOverlay);
			registerType(msg);
		}

		// register ObstacleList
		{
			std::tr1::shared_ptr<px::ObstacleList> msg(new px::ObstacleList);
			registerType(msg);
		}

		// register ObstacleMap
		{
			std::tr1::shared_ptr<px::ObstacleMap> msg(new px::ObstacleMap);
			registerType(msg);
		}

		// register Path
		{
			std::tr1::shared_ptr<px::Path> msg(new px::Path);
                        registerType(msg);
		}

		// register PointCloudXYZI
		{
			std::tr1::shared_ptr<px::PointCloudXYZI> msg(new px::PointCloudXYZI);
			registerType(msg);
		}

		// register PointCloudXYZRGB
		{
			std::tr1::shared_ptr<px::PointCloudXYZRGB> msg(new px::PointCloudXYZRGB);
			registerType(msg);
		}

		// register RGBDImage
		{
			std::tr1::shared_ptr<px::RGBDImage> msg(new px::RGBDImage);
			registerType(msg);
		}

		srand(time(NULL));
		mStreamID = rand() + 1;
	}

	bool fragmentMessage(uint8_t system_id, uint8_t component_id,
						 uint8_t target_system, uint8_t target_component,
						 const google::protobuf::Message& protobuf_msg,
						 std::vector<mavlink_extended_message_t>& fragments) const
	{
		TypeMap::const_iterator it = mTypeMap.find(protobuf_msg.GetTypeName());
		if (it == mTypeMap.end())
		{
			std::cout << "# WARNING: Protobuf message with type "
					  << protobuf_msg.GetTypeName() << " is not registered."
					  << std::endl;
			return false;
		}

		uint8_t typecode = it->second;

		std::string data = protobuf_msg.SerializeAsString();

		int fragmentCount = (protobuf_msg.ByteSize() + kExtendedPayloadMaxSize - 1) / kExtendedPayloadMaxSize;
		unsigned int offset = 0;

		for (int i = 0; i < fragmentCount; ++i)
		{
			mavlink_extended_message_t fragment;			

			// write extended header data
			uint8_t* payload = reinterpret_cast<uint8_t*>(fragment.base_msg.payload64);
			unsigned int length = 0;
			uint8_t flags = 0;

			if (i < fragmentCount - 1)
			{
				length = kExtendedPayloadMaxSize;
				flags |= 0x1;
			}
			else
			{
				length = protobuf_msg.ByteSize() - kExtendedPayloadMaxSize * (fragmentCount - 1);
			}

			memcpy(payload, &target_system, 1);
			memcpy(payload + 1, &target_component, 1);
			memcpy(payload + 2, &typecode, 1);
			memcpy(payload + 3, &length, 4);
			memcpy(payload + 7, &mStreamID, 2);
			memcpy(payload + 9, &offset, 4);
			memcpy(payload + 13, &flags, 1);

			fragment.base_msg.msgid = MAVLINK_MSG_ID_EXTENDED_MESSAGE;
			mavlink_finalize_message(&fragment.base_msg, system_id, component_id, kExtendedHeaderSize, 0);
			
			// write extended payload data
			fragment.extended_payload_len = length;
			memcpy(fragment.extended_payload, &data[offset], length);

			fragments.push_back(fragment);
			offset += length;
		}

		if (mVerbose)
		{
			std::cerr << "# INFO: Split extended message with size "
					  << protobuf_msg.ByteSize() << " into "
					  << fragmentCount << " fragments." << std::endl;
		}

		return true;
	}

	bool cacheFragment(mavlink_extended_message_t& msg)
	{
		if (!validFragment(msg))
		{
			if (mVerbose)
			{
				std::cerr << "# WARNING: Message is not a valid fragment. "
						  << "Dropping message..." << std::endl;
			}
			return false;
		}

		// read extended header
		uint8_t* payload = reinterpret_cast<uint8_t*>(msg.base_msg.payload64);
		uint8_t typecode = 0;
		unsigned int length = 0;
		unsigned short streamID = 0;
		unsigned int offset = 0;
		uint8_t flags = 0;

		memcpy(&typecode, payload + 2, 1);
		memcpy(&length, payload + 3, 4);
		memcpy(&streamID, payload + 7, 2);
		memcpy(&offset, payload + 9, 4);
		memcpy(&flags, payload + 13, 1);

		if (typecode >= mTypeMap.size())
		{
			std::cout << "# WARNING: Protobuf message with type code "
					  << static_cast<int>(typecode) << " is not registered." << std::endl;
			return false;
		}

		bool reassemble = false;

		FragmentQueue::iterator it = mFragmentQueue.find(streamID);
		if (it == mFragmentQueue.end())
		{
			if (offset == 0)
			{
				mFragmentQueue[streamID].push_back(msg);

				if ((flags & 0x1) != 0x1)
				{
					reassemble = true;
				}

				if (mVerbose)
				{
					std::cerr << "# INFO: Added fragment to new queue."
							  << std::endl;
				}
			}
			else
			{
				if (mVerbose)
				{
					std::cerr << "# WARNING: Message is not a valid fragment. "
							  << "Dropping message..." << std::endl;
				}
			}
		}
		else
		{
			std::deque<mavlink_extended_message_t>& queue = it->second;

			if (queue.empty())
			{
				if (offset == 0)
				{
					queue.push_back(msg);

					if ((flags & 0x1) != 0x1)
					{
						reassemble = true;
					}
				}
				else
				{
					if (mVerbose)
					{
						std::cerr << "# WARNING: Message is not a valid fragment. "
								  << "Dropping message..." << std::endl;
					}
				}
			}
			else
			{
				if (fragmentDataSize(queue.back()) + fragmentOffset(queue.back()) != offset)
				{
					if (mVerbose)
					{
						std::cerr << "# WARNING: Previous fragment(s) have been lost. "
								  << "Dropping message and clearing queue..." << std::endl;
					}
					queue.clear();
				}
				else
				{
					queue.push_back(msg);

					if ((flags & 0x1) != 0x1)
					{
						reassemble = true;
					}
				}
			}
		}

		if (reassemble)
		{
			std::deque<mavlink_extended_message_t>& queue = mFragmentQueue[streamID];

			std::string data;
			for (size_t i = 0; i < queue.size(); ++i)
			{
				mavlink_extended_message_t& mavlink_msg = queue.at(i);

				data.append(reinterpret_cast<char*>(&mavlink_msg.extended_payload[0]),
							static_cast<size_t>(mavlink_msg.extended_payload_len));
			}

			mMessages.at(typecode)->ParseFromString(data);

			mMessageAvailable.at(typecode) = true;

			queue.clear();

			if (mVerbose)
			{
				std::cerr << "# INFO: Reassembled fragments for message with typename "
						  << mMessages.at(typecode)->GetTypeName() << " and size "
						  << mMessages.at(typecode)->ByteSize()
						  << "." << std::endl;
			}
		}

		return true;
	}

	bool getMessage(std::tr1::shared_ptr<google::protobuf::Message>& msg)
	{
		for (size_t i = 0; i < mMessageAvailable.size(); ++i)
		{
			if (mMessageAvailable.at(i))
			{
				msg = mMessages.at(i);
				mMessageAvailable.at(i) = false;

				return true;
			}
		}

		return false;
	}

private:
	void registerType(const std::tr1::shared_ptr<google::protobuf::Message>& msg)
	{
		mTypeMap[msg->GetTypeName()] = mRegisteredTypeCount;
		++mRegisteredTypeCount;
		mMessages.push_back(msg);
		mMessageAvailable.push_back(false);
	}

	bool validFragment(const mavlink_extended_message_t& msg) const
	{
		if (msg.base_msg.magic != MAVLINK_STX ||
			msg.base_msg.len != kExtendedHeaderSize ||
			msg.base_msg.msgid != MAVLINK_MSG_ID_EXTENDED_MESSAGE)
		{
			return false;
		}

		uint16_t checksum;
		checksum = crc_calculate(reinterpret_cast<const uint8_t*>(&msg.base_msg.len), MAVLINK_CORE_HEADER_LEN);
		crc_accumulate_buffer(&checksum, reinterpret_cast<const char*>(&msg.base_msg.payload64), kExtendedHeaderSize);
#if MAVLINK_CRC_EXTRA
		static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;
		crc_accumulate(mavlink_message_crcs[msg.base_msg.msgid], &checksum);
#endif

		if (mavlink_ck_a(&(msg.base_msg)) != (uint8_t)(checksum & 0xFF) &&
		    mavlink_ck_b(&(msg.base_msg)) != (uint8_t)(checksum >> 8))
		{
			return false;
		}

		return true;
	}

	unsigned int fragmentDataSize(const mavlink_extended_message_t& msg) const
	{
		const uint8_t* payload = reinterpret_cast<const uint8_t*>(msg.base_msg.payload64);

		return *(reinterpret_cast<const unsigned int*>(payload + 3));
	}

	unsigned int fragmentOffset(const mavlink_extended_message_t& msg) const
	{
		const uint8_t* payload = reinterpret_cast<const uint8_t*>(msg.base_msg.payload64);

		return *(reinterpret_cast<const unsigned int*>(payload + 9));
	}

	int mRegisteredTypeCount;
	unsigned short mStreamID;
	bool mVerbose;

	typedef std::map<std::string, uint8_t> TypeMap;
	TypeMap mTypeMap;
	std::vector< std::tr1::shared_ptr<google::protobuf::Message> > mMessages;
	std::vector<bool> mMessageAvailable;

	typedef std::map<unsigned short, std::deque<mavlink_extended_message_t> > FragmentQueue;
	FragmentQueue mFragmentQueue;

	const int kExtendedHeaderSize;
	/**
	 * Extended header structure
	 * =========================
	 *   byte 0 - target_system
	 *   byte 1 - target_component
	 *   byte 2 - extended message id (type code)
	 *   bytes 3-6 - extended payload size in bytes
	 *   byte 7-8 - stream ID
	 *   byte 9-12 - fragment offset
	 *   byte 13 - fragment flags (bit 0 - 1=more fragments, 0=last fragment)
	 */

	const int kExtendedPayloadMaxSize;
};

}

#endif
