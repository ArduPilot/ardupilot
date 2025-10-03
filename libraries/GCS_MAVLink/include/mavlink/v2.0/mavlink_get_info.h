#pragma once

#ifdef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_HAVE_GET_MESSAGE_INFO

/*
  return the message_info struct for a message
*/
MAVLINK_HELPER const mavlink_message_info_t *mavlink_get_message_info_by_id(uint32_t msgid)
{
	static const mavlink_message_info_t mavlink_message_info[] = MAVLINK_MESSAGE_INFO;
	/*
	  use a bisection search to find the right entry. A perfect hash may be better
	  Note that this assumes the table is sorted with primary key msgid
	*/
	const uint32_t count = sizeof(mavlink_message_info)/sizeof(mavlink_message_info[0]);
	if (count == 0) {
		return NULL;
	}
	uint32_t low=0, high=count-1;
	while (low < high) {
		uint32_t mid = (low+high)/2;
		if (msgid < mavlink_message_info[mid].msgid) {
			high = mid;
			continue;
		}
		if (msgid > mavlink_message_info[mid].msgid) {
			low = mid+1;
			continue;
		}
		return &mavlink_message_info[mid];
	}
	if (mavlink_message_info[low].msgid == msgid) {
		return &mavlink_message_info[low];
	}
	return NULL;
}

/*
  return the message_info struct for a message
*/
MAVLINK_HELPER const mavlink_message_info_t *mavlink_get_message_info(const mavlink_message_t *msg)
{
    return mavlink_get_message_info_by_id(msg->msgid);
}

/*
  return the message_info struct for a message
*/
MAVLINK_HELPER const mavlink_message_info_t *mavlink_get_message_info_by_name(const char *name)
{
	static const struct { const char *name; uint32_t msgid; } mavlink_message_names[] = MAVLINK_MESSAGE_NAMES;
	/*
	  use a bisection search to find the right entry. A perfect hash may be better
	  Note that this assumes the table is sorted with primary key name
	*/
	const uint32_t count = sizeof(mavlink_message_names)/sizeof(mavlink_message_names[0]);
	if (count == 0) {
		return NULL;
	}
	uint32_t low=0, high=count-1;
	while (low < high) {
		uint32_t mid = (low+high)/2;
		int cmp = strcmp(mavlink_message_names[mid].name, name);
		if (cmp > 0) {
			high = mid;
			continue;
		}
		if (cmp < 0) {
			low = mid+1;
			continue;
		}
		low = mid;
		break;
	}
	if (strcmp(mavlink_message_names[low].name, name) == 0) {
		return mavlink_get_message_info_by_id(mavlink_message_names[low].msgid);
	}
	return NULL;
}
#endif // MAVLINK_USE_MESSAGE_INFO


