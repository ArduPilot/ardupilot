//
// THIS FILE WAS AUTOMATICALLY GENERATED - DO NOT EDIT
//
#pragma pack(1)

//////////////////////////////////////////////////////////////////////
/// @name MSG_ACKNOWLEDGE 
//@{

/// Structure describing the payload section of the MSG_ACKNOWLEDGE message
struct msg_acknowledge {
	uint8_t msgID;
	uint16_t msgSum;
};

/// Send a MSG_ACKNOWLEDGE message
inline void
BinComm::send_msg_acknowledge(
	const uint8_t msgID,
	const uint16_t msgSum)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, msgID);
	_pack(p, msgSum);
	_encodeBuf.header.length = 3;
	_encodeBuf.header.messageID = MSG_ACKNOWLEDGE;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_ACKNOWLEDGE message
inline void
BinComm::unpack_msg_acknowledge(
	uint8_t &msgID,
	uint16_t &msgSum)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, msgID);
	_unpack(p, msgSum);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_HEARTBEAT 
//@{

/// Structure describing the payload section of the MSG_HEARTBEAT message
struct msg_heartbeat {
	uint8_t flightMode;
	uint16_t timeStamp;
	uint16_t batteryVoltage;
	uint16_t commandIndex;
};

/// Send a MSG_HEARTBEAT message
inline void
BinComm::send_msg_heartbeat(
	const uint8_t flightMode,
	const uint16_t timeStamp,
	const uint16_t batteryVoltage,
	const uint16_t commandIndex)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, flightMode);
	_pack(p, timeStamp);
	_pack(p, batteryVoltage);
	_pack(p, commandIndex);
	_encodeBuf.header.length = 7;
	_encodeBuf.header.messageID = MSG_HEARTBEAT;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_HEARTBEAT message
inline void
BinComm::unpack_msg_heartbeat(
	uint8_t &flightMode,
	uint16_t &timeStamp,
	uint16_t &batteryVoltage,
	uint16_t &commandIndex)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, flightMode);
	_unpack(p, timeStamp);
	_unpack(p, batteryVoltage);
	_unpack(p, commandIndex);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_ATTITUDE 
//@{

/// Structure describing the payload section of the MSG_ATTITUDE message
struct msg_attitude {
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
};

/// Send a MSG_ATTITUDE message
inline void
BinComm::send_msg_attitude(
	const int16_t roll,
	const int16_t pitch,
	const int16_t yaw)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, roll);
	_pack(p, pitch);
	_pack(p, yaw);
	_encodeBuf.header.length = 6;
	_encodeBuf.header.messageID = MSG_ATTITUDE;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_ATTITUDE message
inline void
BinComm::unpack_msg_attitude(
	int16_t &roll,
	int16_t &pitch,
	int16_t &yaw)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, roll);
	_unpack(p, pitch);
	_unpack(p, yaw);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_LOCATION 
//@{

/// Structure describing the payload section of the MSG_LOCATION message
struct msg_location {
	int32_t latitude;
	int32_t longitude;
	int16_t altitude;
	int16_t groundSpeed;
	int16_t groundCourse;
	uint16_t timeOfWeek;
};

/// Send a MSG_LOCATION message
inline void
BinComm::send_msg_location(
	const int32_t latitude,
	const int32_t longitude,
	const int16_t altitude,
	const int16_t groundSpeed,
	const int16_t groundCourse,
	const uint16_t timeOfWeek)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, latitude);
	_pack(p, longitude);
	_pack(p, altitude);
	_pack(p, groundSpeed);
	_pack(p, groundCourse);
	_pack(p, timeOfWeek);
	_encodeBuf.header.length = 16;
	_encodeBuf.header.messageID = MSG_LOCATION;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_LOCATION message
inline void
BinComm::unpack_msg_location(
	int32_t &latitude,
	int32_t &longitude,
	int16_t &altitude,
	int16_t &groundSpeed,
	int16_t &groundCourse,
	uint16_t &timeOfWeek)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, latitude);
	_unpack(p, longitude);
	_unpack(p, altitude);
	_unpack(p, groundSpeed);
	_unpack(p, groundCourse);
	_unpack(p, timeOfWeek);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_PRESSURE 
//@{

/// Structure describing the payload section of the MSG_PRESSURE message
struct msg_pressure {
	uint16_t pressureAltitude;
	uint16_t airSpeed;
};

/// Send a MSG_PRESSURE message
inline void
BinComm::send_msg_pressure(
	const uint16_t pressureAltitude,
	const uint16_t airSpeed)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, pressureAltitude);
	_pack(p, airSpeed);
	_encodeBuf.header.length = 4;
	_encodeBuf.header.messageID = MSG_PRESSURE;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_PRESSURE message
inline void
BinComm::unpack_msg_pressure(
	uint16_t &pressureAltitude,
	uint16_t &airSpeed)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, pressureAltitude);
	_unpack(p, airSpeed);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_STATUS_TEXT 
//@{

/// Structure describing the payload section of the MSG_STATUS_TEXT message
struct msg_status_text {
	uint8_t severity;
	char text[50];
};

/// Send a MSG_STATUS_TEXT message
inline void
BinComm::send_msg_status_text(
	const uint8_t severity,
	const char (&text)[50])
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, severity);
	_pack(p, text, 50);
	_encodeBuf.header.length = 51;
	_encodeBuf.header.messageID = MSG_STATUS_TEXT;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_STATUS_TEXT message
inline void
BinComm::unpack_msg_status_text(
	uint8_t &severity,
	char (&text)[50])
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, severity);
	_unpack(p, text, 50);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_PERF_REPORT 
//@{

/// Structure describing the payload section of the MSG_PERF_REPORT message
struct msg_perf_report {
	uint32_t interval;
	uint16_t mainLoopCycles;
	uint8_t mainLoopTime;
	uint8_t gyroSaturationCount;
	uint8_t adcConstraintCount;
	uint16_t imuHealth;
	uint16_t gcsMessageCount;
};

/// Send a MSG_PERF_REPORT message
inline void
BinComm::send_msg_perf_report(
	const uint32_t interval,
	const uint16_t mainLoopCycles,
	const uint8_t mainLoopTime,
	const uint8_t gyroSaturationCount,
	const uint8_t adcConstraintCount,
	const uint16_t imuHealth,
	const uint16_t gcsMessageCount)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, interval);
	_pack(p, mainLoopCycles);
	_pack(p, mainLoopTime);
	_pack(p, gyroSaturationCount);
	_pack(p, adcConstraintCount);
	_pack(p, imuHealth);
	_pack(p, gcsMessageCount);
	_encodeBuf.header.length = 13;
	_encodeBuf.header.messageID = MSG_PERF_REPORT;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_PERF_REPORT message
inline void
BinComm::unpack_msg_perf_report(
	uint32_t &interval,
	uint16_t &mainLoopCycles,
	uint8_t &mainLoopTime,
	uint8_t &gyroSaturationCount,
	uint8_t &adcConstraintCount,
	uint16_t &imuHealth,
	uint16_t &gcsMessageCount)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, interval);
	_unpack(p, mainLoopCycles);
	_unpack(p, mainLoopTime);
	_unpack(p, gyroSaturationCount);
	_unpack(p, adcConstraintCount);
	_unpack(p, imuHealth);
	_unpack(p, gcsMessageCount);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_VERSION_REQUEST 
//@{

/// Structure describing the payload section of the MSG_VERSION_REQUEST message
struct msg_version_request {
	uint8_t systemType;
	uint8_t systemID;
};

/// Send a MSG_VERSION_REQUEST message
inline void
BinComm::send_msg_version_request(
	const uint8_t systemType,
	const uint8_t systemID)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, systemType);
	_pack(p, systemID);
	_encodeBuf.header.length = 2;
	_encodeBuf.header.messageID = MSG_VERSION_REQUEST;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_VERSION_REQUEST message
inline void
BinComm::unpack_msg_version_request(
	uint8_t &systemType,
	uint8_t &systemID)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, systemType);
	_unpack(p, systemID);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_VERSION 
//@{

/// Structure describing the payload section of the MSG_VERSION message
struct msg_version {
	uint8_t systemType;
	uint8_t systemID;
	uint8_t firmwareVersion[3];
};

/// Send a MSG_VERSION message
inline void
BinComm::send_msg_version(
	const uint8_t systemType,
	const uint8_t systemID,
	const uint8_t (&firmwareVersion)[3])
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, systemType);
	_pack(p, systemID);
	_pack(p, firmwareVersion, 3);
	_encodeBuf.header.length = 5;
	_encodeBuf.header.messageID = MSG_VERSION;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_VERSION message
inline void
BinComm::unpack_msg_version(
	uint8_t &systemType,
	uint8_t &systemID,
	uint8_t (&firmwareVersion)[3])
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, systemType);
	_unpack(p, systemID);
	_unpack(p, firmwareVersion, 3);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_COMMAND_REQUEST 
//@{

/// Structure describing the payload section of the MSG_COMMAND_REQUEST message
struct msg_command_request {
	uint16_t UNSPECIFIED;
};

/// Send a MSG_COMMAND_REQUEST message
inline void
BinComm::send_msg_command_request(
	const uint16_t UNSPECIFIED)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, UNSPECIFIED);
	_encodeBuf.header.length = 2;
	_encodeBuf.header.messageID = MSG_COMMAND_REQUEST;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_COMMAND_REQUEST message
inline void
BinComm::unpack_msg_command_request(
	uint16_t &UNSPECIFIED)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, UNSPECIFIED);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_COMMAND_UPLOAD 
//@{

/// Structure describing the payload section of the MSG_COMMAND_UPLOAD message
struct msg_command_upload {
	uint8_t action;
	uint16_t itemNumber;
	int listLength;
	uint8_t commandID;
	uint8_t p1;
	uint16_t p2;
	uint32_t p3;
	uint32_t p4;
};

/// Send a MSG_COMMAND_UPLOAD message
inline void
BinComm::send_msg_command_upload(
	const uint8_t action,
	const uint16_t itemNumber,
	const int listLength,
	const uint8_t commandID,
	const uint8_t p1,
	const uint16_t p2,
	const uint32_t p3,
	const uint32_t p4)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, action);
	_pack(p, itemNumber);
	_pack(p, listLength);
	_pack(p, commandID);
	_pack(p, p1);
	_pack(p, p2);
	_pack(p, p3);
	_pack(p, p4);
	_encodeBuf.header.length = 16;
	_encodeBuf.header.messageID = MSG_COMMAND_UPLOAD;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_COMMAND_UPLOAD message
inline void
BinComm::unpack_msg_command_upload(
	uint8_t &action,
	uint16_t &itemNumber,
	int &listLength,
	uint8_t &commandID,
	uint8_t &p1,
	uint16_t &p2,
	uint32_t &p3,
	uint32_t &p4)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, action);
	_unpack(p, itemNumber);
	_unpack(p, listLength);
	_unpack(p, commandID);
	_unpack(p, p1);
	_unpack(p, p2);
	_unpack(p, p3);
	_unpack(p, p4);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_COMMAND_LIST 
//@{

/// Structure describing the payload section of the MSG_COMMAND_LIST message
struct msg_command_list {
	int itemNumber;
	int listLength;
	uint8_t commandID;
	uint8_t p1;
	uint16_t p2;
	uint32_t p3;
	uint32_t p4;
};

/// Send a MSG_COMMAND_LIST message
inline void
BinComm::send_msg_command_list(
	const int itemNumber,
	const int listLength,
	const uint8_t commandID,
	const uint8_t p1,
	const uint16_t p2,
	const uint32_t p3,
	const uint32_t p4)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, itemNumber);
	_pack(p, listLength);
	_pack(p, commandID);
	_pack(p, p1);
	_pack(p, p2);
	_pack(p, p3);
	_pack(p, p4);
	_encodeBuf.header.length = 14;
	_encodeBuf.header.messageID = MSG_COMMAND_LIST;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_COMMAND_LIST message
inline void
BinComm::unpack_msg_command_list(
	int &itemNumber,
	int &listLength,
	uint8_t &commandID,
	uint8_t &p1,
	uint16_t &p2,
	uint32_t &p3,
	uint32_t &p4)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, itemNumber);
	_unpack(p, listLength);
	_unpack(p, commandID);
	_unpack(p, p1);
	_unpack(p, p2);
	_unpack(p, p3);
	_unpack(p, p4);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_COMMAND_MODE_CHANGE 
//@{

/// Structure describing the payload section of the MSG_COMMAND_MODE_CHANGE message
struct msg_command_mode_change {
	uint16_t UNSPECIFIED;
};

/// Send a MSG_COMMAND_MODE_CHANGE message
inline void
BinComm::send_msg_command_mode_change(
	const uint16_t UNSPECIFIED)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, UNSPECIFIED);
	_encodeBuf.header.length = 2;
	_encodeBuf.header.messageID = MSG_COMMAND_MODE_CHANGE;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_COMMAND_MODE_CHANGE message
inline void
BinComm::unpack_msg_command_mode_change(
	uint16_t &UNSPECIFIED)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, UNSPECIFIED);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_VALUE_REQUEST 
//@{

/// Structure describing the payload section of the MSG_VALUE_REQUEST message
struct msg_value_request {
	uint8_t valueID;
	uint8_t broadcast;
};

/// Send a MSG_VALUE_REQUEST message
inline void
BinComm::send_msg_value_request(
	const uint8_t valueID,
	const uint8_t broadcast)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, valueID);
	_pack(p, broadcast);
	_encodeBuf.header.length = 2;
	_encodeBuf.header.messageID = MSG_VALUE_REQUEST;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_VALUE_REQUEST message
inline void
BinComm::unpack_msg_value_request(
	uint8_t &valueID,
	uint8_t &broadcast)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, valueID);
	_unpack(p, broadcast);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_VALUE_SET 
//@{

/// Structure describing the payload section of the MSG_VALUE_SET message
struct msg_value_set {
	uint8_t valueID;
	uint32_t value;
};

/// Send a MSG_VALUE_SET message
inline void
BinComm::send_msg_value_set(
	const uint8_t valueID,
	const uint32_t value)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, valueID);
	_pack(p, value);
	_encodeBuf.header.length = 5;
	_encodeBuf.header.messageID = MSG_VALUE_SET;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_VALUE_SET message
inline void
BinComm::unpack_msg_value_set(
	uint8_t &valueID,
	uint32_t &value)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, valueID);
	_unpack(p, value);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_VALUE 
//@{

/// Structure describing the payload section of the MSG_VALUE message
struct msg_value {
	uint8_t valueID;
	uint32_t value;
};

/// Send a MSG_VALUE message
inline void
BinComm::send_msg_value(
	const uint8_t valueID,
	const uint32_t value)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, valueID);
	_pack(p, value);
	_encodeBuf.header.length = 5;
	_encodeBuf.header.messageID = MSG_VALUE;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_VALUE message
inline void
BinComm::unpack_msg_value(
	uint8_t &valueID,
	uint32_t &value)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, valueID);
	_unpack(p, value);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_PID_REQUEST 
//@{

/// Structure describing the payload section of the MSG_PID_REQUEST message
struct msg_pid_request {
	uint8_t pidSet;
};

/// Send a MSG_PID_REQUEST message
inline void
BinComm::send_msg_pid_request(
	const uint8_t pidSet)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, pidSet);
	_encodeBuf.header.length = 1;
	_encodeBuf.header.messageID = MSG_PID_REQUEST;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_PID_REQUEST message
inline void
BinComm::unpack_msg_pid_request(
	uint8_t &pidSet)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, pidSet);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_PID_SET 
//@{

/// Structure describing the payload section of the MSG_PID_SET message
struct msg_pid_set {
	uint8_t pidSet;
	int32_t p;
	int32_t i;
	int32_t d;
	int16_t integratorMax;
};

/// Send a MSG_PID_SET message
inline void
BinComm::send_msg_pid_set(
	const uint8_t pidSet,
	const int32_t p,
	const int32_t i,
	const int32_t d,
	const int16_t integratorMax)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, pidSet);
	_pack(p, p);
	_pack(p, i);
	_pack(p, d);
	_pack(p, integratorMax);
	_encodeBuf.header.length = 15;
	_encodeBuf.header.messageID = MSG_PID_SET;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_PID_SET message
inline void
BinComm::unpack_msg_pid_set(
	uint8_t &pidSet,
	int32_t &p,
	int32_t &i,
	int32_t &d,
	int16_t &integratorMax)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, pidSet);
	_unpack(p, p);
	_unpack(p, i);
	_unpack(p, d);
	_unpack(p, integratorMax);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_PID 
//@{

/// Structure describing the payload section of the MSG_PID message
struct msg_pid {
	uint8_t pidSet;
	int32_t p;
	int32_t i;
	int32_t d;
	int16_t integratorMax;
};

/// Send a MSG_PID message
inline void
BinComm::send_msg_pid(
	const uint8_t pidSet,
	const int32_t p,
	const int32_t i,
	const int32_t d,
	const int16_t integratorMax)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, pidSet);
	_pack(p, p);
	_pack(p, i);
	_pack(p, d);
	_pack(p, integratorMax);
	_encodeBuf.header.length = 15;
	_encodeBuf.header.messageID = MSG_PID;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_PID message
inline void
BinComm::unpack_msg_pid(
	uint8_t &pidSet,
	int32_t &p,
	int32_t &i,
	int32_t &d,
	int16_t &integratorMax)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, pidSet);
	_unpack(p, p);
	_unpack(p, i);
	_unpack(p, d);
	_unpack(p, integratorMax);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_TRIM_STARTUP 
//@{

/// Structure describing the payload section of the MSG_TRIM_STARTUP message
struct msg_trim_startup {
	uint16_t value[8];
};

/// Send a MSG_TRIM_STARTUP message
inline void
BinComm::send_msg_trim_startup(
	const uint16_t (&value)[8])
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, value, 8);
	_encodeBuf.header.length = 16;
	_encodeBuf.header.messageID = MSG_TRIM_STARTUP;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_TRIM_STARTUP message
inline void
BinComm::unpack_msg_trim_startup(
	uint16_t (&value)[8])
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, value, 8);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_TRIM_MIN 
//@{

/// Structure describing the payload section of the MSG_TRIM_MIN message
struct msg_trim_min {
	uint16_t value[8];
};

/// Send a MSG_TRIM_MIN message
inline void
BinComm::send_msg_trim_min(
	const uint16_t (&value)[8])
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, value, 8);
	_encodeBuf.header.length = 16;
	_encodeBuf.header.messageID = MSG_TRIM_MIN;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_TRIM_MIN message
inline void
BinComm::unpack_msg_trim_min(
	uint16_t (&value)[8])
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, value, 8);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_TRIM_MAX 
//@{

/// Structure describing the payload section of the MSG_TRIM_MAX message
struct msg_trim_max {
	uint16_t value[8];
};

/// Send a MSG_TRIM_MAX message
inline void
BinComm::send_msg_trim_max(
	const uint16_t (&value)[8])
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, value, 8);
	_encodeBuf.header.length = 16;
	_encodeBuf.header.messageID = MSG_TRIM_MAX;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_TRIM_MAX message
inline void
BinComm::unpack_msg_trim_max(
	uint16_t (&value)[8])
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, value, 8);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_SENSOR 
//@{

/// Structure describing the payload section of the MSG_SENSOR message
struct msg_sensor {
	uint16_t UNSPECIFIED;
};

/// Send a MSG_SENSOR message
inline void
BinComm::send_msg_sensor(
	const uint16_t UNSPECIFIED)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, UNSPECIFIED);
	_encodeBuf.header.length = 2;
	_encodeBuf.header.messageID = MSG_SENSOR;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_SENSOR message
inline void
BinComm::unpack_msg_sensor(
	uint16_t &UNSPECIFIED)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, UNSPECIFIED);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_SIM 
//@{

/// Structure describing the payload section of the MSG_SIM message
struct msg_sim {
	uint16_t UNSPECIFIED;
};

/// Send a MSG_SIM message
inline void
BinComm::send_msg_sim(
	const uint16_t UNSPECIFIED)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, UNSPECIFIED);
	_encodeBuf.header.length = 2;
	_encodeBuf.header.messageID = MSG_SIM;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_SIM message
inline void
BinComm::unpack_msg_sim(
	uint16_t &UNSPECIFIED)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, UNSPECIFIED);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_PIN_REQUEST 
//@{

/// Structure describing the payload section of the MSG_PIN_REQUEST message
struct msg_pin_request {
	uint16_t UNSPECIFIED;
};

/// Send a MSG_PIN_REQUEST message
inline void
BinComm::send_msg_pin_request(
	const uint16_t UNSPECIFIED)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, UNSPECIFIED);
	_encodeBuf.header.length = 2;
	_encodeBuf.header.messageID = MSG_PIN_REQUEST;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_PIN_REQUEST message
inline void
BinComm::unpack_msg_pin_request(
	uint16_t &UNSPECIFIED)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, UNSPECIFIED);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_PIN_SET 
//@{

/// Structure describing the payload section of the MSG_PIN_SET message
struct msg_pin_set {
	uint16_t UNSPECIFIED;
};

/// Send a MSG_PIN_SET message
inline void
BinComm::send_msg_pin_set(
	const uint16_t UNSPECIFIED)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, UNSPECIFIED);
	_encodeBuf.header.length = 2;
	_encodeBuf.header.messageID = MSG_PIN_SET;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_PIN_SET message
inline void
BinComm::unpack_msg_pin_set(
	uint16_t &UNSPECIFIED)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, UNSPECIFIED);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_DATAFLASH_REQUEST 
//@{

/// Structure describing the payload section of the MSG_DATAFLASH_REQUEST message
struct msg_dataflash_request {
	uint16_t UNSPECIFIED;
};

/// Send a MSG_DATAFLASH_REQUEST message
inline void
BinComm::send_msg_dataflash_request(
	const uint16_t UNSPECIFIED)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, UNSPECIFIED);
	_encodeBuf.header.length = 2;
	_encodeBuf.header.messageID = MSG_DATAFLASH_REQUEST;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_DATAFLASH_REQUEST message
inline void
BinComm::unpack_msg_dataflash_request(
	uint16_t &UNSPECIFIED)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, UNSPECIFIED);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_DATAFLASH_SET 
//@{

/// Structure describing the payload section of the MSG_DATAFLASH_SET message
struct msg_dataflash_set {
	uint16_t UNSPECIFIED;
};

/// Send a MSG_DATAFLASH_SET message
inline void
BinComm::send_msg_dataflash_set(
	const uint16_t UNSPECIFIED)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, UNSPECIFIED);
	_encodeBuf.header.length = 2;
	_encodeBuf.header.messageID = MSG_DATAFLASH_SET;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_DATAFLASH_SET message
inline void
BinComm::unpack_msg_dataflash_set(
	uint16_t &UNSPECIFIED)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, UNSPECIFIED);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_EEPROM_REQUEST 
//@{

/// Structure describing the payload section of the MSG_EEPROM_REQUEST message
struct msg_eeprom_request {
	uint16_t UNSPECIFIED;
};

/// Send a MSG_EEPROM_REQUEST message
inline void
BinComm::send_msg_eeprom_request(
	const uint16_t UNSPECIFIED)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, UNSPECIFIED);
	_encodeBuf.header.length = 2;
	_encodeBuf.header.messageID = MSG_EEPROM_REQUEST;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_EEPROM_REQUEST message
inline void
BinComm::unpack_msg_eeprom_request(
	uint16_t &UNSPECIFIED)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, UNSPECIFIED);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_EEPROM_SET 
//@{

/// Structure describing the payload section of the MSG_EEPROM_SET message
struct msg_eeprom_set {
	uint16_t UNSPECIFIED;
};

/// Send a MSG_EEPROM_SET message
inline void
BinComm::send_msg_eeprom_set(
	const uint16_t UNSPECIFIED)
{
	uint8_t *p = &_encodeBuf.payload;
	_pack(p, UNSPECIFIED);
	_encodeBuf.header.length = 2;
	_encodeBuf.header.messageID = MSG_EEPROM_SET;
	_encodeBuf.header.messageVersion = MSG_VERSION_1;
	_sendMessage();
};

/// Unpack a MSG_EEPROM_SET message
inline void
BinComm::unpack_msg_eeprom_set(
	uint16_t &UNSPECIFIED)
{
	uint8_t *p = &_decodeBuf.payload;
	_unpack(p, UNSPECIFIED);
};
//@}

//////////////////////////////////////////////////////////////////////
/// Message ID values
enum messageID {
	MSG_PID = 0x42,
	MSG_DATAFLASH_REQUEST = 0x90,
	MSG_DATAFLASH_SET = 0x91,
	MSG_SENSOR = 0x60,
	MSG_VALUE_REQUEST = 0x30,
	MSG_VALUE_SET = 0x31,
	MSG_VALUE = 0x32,
	MSG_PIN_REQUEST = 0x80,
	MSG_PIN_SET = 0x81,
	MSG_ACKNOWLEDGE = 0x0,
	MSG_HEARTBEAT = 0x1,
	MSG_ATTITUDE = 0x2,
	MSG_LOCATION = 0x3,
	MSG_PRESSURE = 0x4,
	MSG_TRIM_STARTUP = 0x50,
	MSG_STATUS_TEXT = 0x5,
	MSG_TRIM_MIN = 0x51,
	MSG_PERF_REPORT = 0x6,
	MSG_TRIM_MAX = 0x52,
	MSG_VERSION_REQUEST = 0x7,
	MSG_VERSION = 0x8,
	MSG_COMMAND_REQUEST = 0x20,
	MSG_COMMAND_UPLOAD = 0x21,
	MSG_COMMAND_LIST = 0x22,
	MSG_COMMAND_MODE_CHANGE = 0x23,
	MSG_SIM = 0x70,
	MSG_EEPROM_REQUEST = 0xa0,
	MSG_EEPROM_SET = 0xa1,
	MSG_PID_REQUEST = 0x40,
	MSG_PID_SET = 0x41,
	MSG_ANY = 0xfe,
	MSG_NULL = 0xff
};
#pragma pack(pop)
