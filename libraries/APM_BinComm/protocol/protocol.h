//
// THIS FILE WAS AUTOMATICALLY GENERATED - DO NOT EDIT
//
/// @file protocol.h
#pragma pack(push)
#pragma pack(1)

//////////////////////////////////////////////////////////////////////
/// @name MSG_ACKNOWLEDGE 
//@{

/// Structure describing the payload section of the MSG_ACKNOWLEDGE message
struct msg_acknowledge {
	uint8_t msgID;
	uint8_t sum1;
	uint8_t sum2;
};

/// Send a MSG_ACKNOWLEDGE message
inline void
send_msg_acknowledge(
	const uint8_t msgID,
	const uint8_t sum1,
	const uint8_t sum2)
{
	_startMessage(MSG_ACKNOWLEDGE,
		sizeof(msgID) +
		sizeof(sum1) +
		sizeof(sum2) + 0);
	_emit(msgID);
	_emit(sum1);
	_emit(sum2);
	_endMessage();
};

/// Unpack a MSG_ACKNOWLEDGE message
inline void
unpack_msg_acknowledge(
	uint8_t &msgID,
	uint8_t &sum1,
	uint8_t &sum2)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, msgID);
	_unpack(__p, sum1);
	_unpack(__p, sum2);
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
send_msg_status_text(
	const uint8_t severity,
	const char (&text)[50])
{
	_startMessage(MSG_STATUS_TEXT,
		sizeof(severity) +
		(sizeof(text[0]) * 50) + 0);
	_emit(severity);
	_emit(text, 50);
	_endMessage();
};

/// Unpack a MSG_STATUS_TEXT message
inline void
unpack_msg_status_text(
	uint8_t &severity,
	char (&text)[50])
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, severity);
	_unpack(__p, text, 50);
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
send_msg_heartbeat(
	const uint8_t flightMode,
	const uint16_t timeStamp,
	const uint16_t batteryVoltage,
	const uint16_t commandIndex)
{
	_startMessage(MSG_HEARTBEAT,
		sizeof(flightMode) +
		sizeof(timeStamp) +
		sizeof(batteryVoltage) +
		sizeof(commandIndex) + 0);
	_emit(flightMode);
	_emit(timeStamp);
	_emit(batteryVoltage);
	_emit(commandIndex);
	_endMessage();
};

/// Unpack a MSG_HEARTBEAT message
inline void
unpack_msg_heartbeat(
	uint8_t &flightMode,
	uint16_t &timeStamp,
	uint16_t &batteryVoltage,
	uint16_t &commandIndex)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, flightMode);
	_unpack(__p, timeStamp);
	_unpack(__p, batteryVoltage);
	_unpack(__p, commandIndex);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_ATTITUDE 
//@{

/// Structure describing the payload section of the MSG_ATTITUDE message
struct msg_attitude {
	int16_t roll;
	int16_t pitch;
	uint16_t yaw;
};

/// Send a MSG_ATTITUDE message
inline void
send_msg_attitude(
	const int16_t roll,
	const int16_t pitch,
	const uint16_t yaw)
{
	_startMessage(MSG_ATTITUDE,
		sizeof(roll) +
		sizeof(pitch) +
		sizeof(yaw) + 0);
	_emit(roll);
	_emit(pitch);
	_emit(yaw);
	_endMessage();
};

/// Unpack a MSG_ATTITUDE message
inline void
unpack_msg_attitude(
	int16_t &roll,
	int16_t &pitch,
	uint16_t &yaw)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, roll);
	_unpack(__p, pitch);
	_unpack(__p, yaw);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_LOCATION 
//@{

/// Structure describing the payload section of the MSG_LOCATION message
struct msg_location {
	int32_t latitude;
	int32_t longitude;
	int32_t altitude;
	uint16_t groundSpeed;
	uint16_t groundCourse;
	uint32_t timeOfWeek;
};

/// Send a MSG_LOCATION message
inline void
send_msg_location(
	const int32_t latitude,
	const int32_t longitude,
	const int32_t altitude,
	const uint16_t groundSpeed,
	const uint16_t groundCourse,
	const uint32_t timeOfWeek)
{
	_startMessage(MSG_LOCATION,
		sizeof(latitude) +
		sizeof(longitude) +
		sizeof(altitude) +
		sizeof(groundSpeed) +
		sizeof(groundCourse) +
		sizeof(timeOfWeek) + 0);
	_emit(latitude);
	_emit(longitude);
	_emit(altitude);
	_emit(groundSpeed);
	_emit(groundCourse);
	_emit(timeOfWeek);
	_endMessage();
};

/// Unpack a MSG_LOCATION message
inline void
unpack_msg_location(
	int32_t &latitude,
	int32_t &longitude,
	int32_t &altitude,
	uint16_t &groundSpeed,
	uint16_t &groundCourse,
	uint32_t &timeOfWeek)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, latitude);
	_unpack(__p, longitude);
	_unpack(__p, altitude);
	_unpack(__p, groundSpeed);
	_unpack(__p, groundCourse);
	_unpack(__p, timeOfWeek);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_PRESSURE 
//@{

/// Structure describing the payload section of the MSG_PRESSURE message
struct msg_pressure {
	int32_t pressureAltitude;
	int16_t airSpeed;
};

/// Send a MSG_PRESSURE message
inline void
send_msg_pressure(
	const int32_t pressureAltitude,
	const int16_t airSpeed)
{
	_startMessage(MSG_PRESSURE,
		sizeof(pressureAltitude) +
		sizeof(airSpeed) + 0);
	_emit(pressureAltitude);
	_emit(airSpeed);
	_endMessage();
};

/// Unpack a MSG_PRESSURE message
inline void
unpack_msg_pressure(
	int32_t &pressureAltitude,
	int16_t &airSpeed)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, pressureAltitude);
	_unpack(__p, airSpeed);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_PERF_REPORT 
//@{

/// Structure describing the payload section of the MSG_PERF_REPORT message
struct msg_perf_report {
	uint32_t interval;
	uint16_t mainLoopCycles;
	uint8_t mainLoopCycleTime;
	uint8_t gyroSaturationCount;
	uint8_t adcConstraintCount;
	uint8_t renormSqrtCount;
	uint8_t renormBlowupCount;
	uint8_t gpsFixCount;
	uint16_t imuHealth;
	uint16_t gcsMessageCount;
};

/// Send a MSG_PERF_REPORT message
inline void
send_msg_perf_report(
	const uint32_t interval,
	const uint16_t mainLoopCycles,
	const uint8_t mainLoopCycleTime,
	const uint8_t gyroSaturationCount,
	const uint8_t adcConstraintCount,
	const uint8_t renormSqrtCount,
	const uint8_t renormBlowupCount,
	const uint8_t gpsFixCount,
	const uint16_t imuHealth,
	const uint16_t gcsMessageCount)
{
	_startMessage(MSG_PERF_REPORT,
		sizeof(interval) +
		sizeof(mainLoopCycles) +
		sizeof(mainLoopCycleTime) +
		sizeof(gyroSaturationCount) +
		sizeof(adcConstraintCount) +
		sizeof(renormSqrtCount) +
		sizeof(renormBlowupCount) +
		sizeof(gpsFixCount) +
		sizeof(imuHealth) +
		sizeof(gcsMessageCount) + 0);
	_emit(interval);
	_emit(mainLoopCycles);
	_emit(mainLoopCycleTime);
	_emit(gyroSaturationCount);
	_emit(adcConstraintCount);
	_emit(renormSqrtCount);
	_emit(renormBlowupCount);
	_emit(gpsFixCount);
	_emit(imuHealth);
	_emit(gcsMessageCount);
	_endMessage();
};

/// Unpack a MSG_PERF_REPORT message
inline void
unpack_msg_perf_report(
	uint32_t &interval,
	uint16_t &mainLoopCycles,
	uint8_t &mainLoopCycleTime,
	uint8_t &gyroSaturationCount,
	uint8_t &adcConstraintCount,
	uint8_t &renormSqrtCount,
	uint8_t &renormBlowupCount,
	uint8_t &gpsFixCount,
	uint16_t &imuHealth,
	uint16_t &gcsMessageCount)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, interval);
	_unpack(__p, mainLoopCycles);
	_unpack(__p, mainLoopCycleTime);
	_unpack(__p, gyroSaturationCount);
	_unpack(__p, adcConstraintCount);
	_unpack(__p, renormSqrtCount);
	_unpack(__p, renormBlowupCount);
	_unpack(__p, gpsFixCount);
	_unpack(__p, imuHealth);
	_unpack(__p, gcsMessageCount);
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
send_msg_version_request(
	const uint8_t systemType,
	const uint8_t systemID)
{
	_startMessage(MSG_VERSION_REQUEST,
		sizeof(systemType) +
		sizeof(systemID) + 0);
	_emit(systemType);
	_emit(systemID);
	_endMessage();
};

/// Unpack a MSG_VERSION_REQUEST message
inline void
unpack_msg_version_request(
	uint8_t &systemType,
	uint8_t &systemID)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, systemType);
	_unpack(__p, systemID);
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
send_msg_version(
	const uint8_t systemType,
	const uint8_t systemID,
	const uint8_t (&firmwareVersion)[3])
{
	_startMessage(MSG_VERSION,
		sizeof(systemType) +
		sizeof(systemID) +
		(sizeof(firmwareVersion[0]) * 3) + 0);
	_emit(systemType);
	_emit(systemID);
	_emit(firmwareVersion, 3);
	_endMessage();
};

/// Unpack a MSG_VERSION message
inline void
unpack_msg_version(
	uint8_t &systemType,
	uint8_t &systemID,
	uint8_t (&firmwareVersion)[3])
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, systemType);
	_unpack(__p, systemID);
	_unpack(__p, firmwareVersion, 3);
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
send_msg_command_request(
	const uint16_t UNSPECIFIED)
{
	_startMessage(MSG_COMMAND_REQUEST,
		sizeof(UNSPECIFIED) + 0);
	_emit(UNSPECIFIED);
	_endMessage();
};

/// Unpack a MSG_COMMAND_REQUEST message
inline void
unpack_msg_command_request(
	uint16_t &UNSPECIFIED)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, UNSPECIFIED);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_COMMAND_UPLOAD 
//@{

/// Structure describing the payload section of the MSG_COMMAND_UPLOAD message
struct msg_command_upload {
	uint8_t action;
	uint16_t itemNumber;
	uint16_t listLength;
	uint8_t commandID;
	uint8_t p1;
	int32_t p2;
	int32_t p3;
	int32_t p4;
};

/// Send a MSG_COMMAND_UPLOAD message
inline void
send_msg_command_upload(
	const uint8_t action,
	const uint16_t itemNumber,
	const uint16_t listLength,
	const uint8_t commandID,
	const uint8_t p1,
	const int32_t p2,
	const int32_t p3,
	const int32_t p4)
{
	_startMessage(MSG_COMMAND_UPLOAD,
		sizeof(action) +
		sizeof(itemNumber) +
		sizeof(listLength) +
		sizeof(commandID) +
		sizeof(p1) +
		sizeof(p2) +
		sizeof(p3) +
		sizeof(p4) + 0);
	_emit(action);
	_emit(itemNumber);
	_emit(listLength);
	_emit(commandID);
	_emit(p1);
	_emit(p2);
	_emit(p3);
	_emit(p4);
	_endMessage();
};

/// Unpack a MSG_COMMAND_UPLOAD message
inline void
unpack_msg_command_upload(
	uint8_t &action,
	uint16_t &itemNumber,
	uint16_t &listLength,
	uint8_t &commandID,
	uint8_t &p1,
	int32_t &p2,
	int32_t &p3,
	int32_t &p4)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, action);
	_unpack(__p, itemNumber);
	_unpack(__p, listLength);
	_unpack(__p, commandID);
	_unpack(__p, p1);
	_unpack(__p, p2);
	_unpack(__p, p3);
	_unpack(__p, p4);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_COMMAND_LIST 
//@{

/// Structure describing the payload section of the MSG_COMMAND_LIST message
struct msg_command_list {
	uint16_t itemNumber;
	uint16_t listLength;
	uint8_t commandID;
	uint8_t p1;
	int32_t p2;
	int32_t p3;
	int32_t p4;
};

/// Send a MSG_COMMAND_LIST message
inline void
send_msg_command_list(
	const uint16_t itemNumber,
	const uint16_t listLength,
	const uint8_t commandID,
	const uint8_t p1,
	const int32_t p2,
	const int32_t p3,
	const int32_t p4)
{
	_startMessage(MSG_COMMAND_LIST,
		sizeof(itemNumber) +
		sizeof(listLength) +
		sizeof(commandID) +
		sizeof(p1) +
		sizeof(p2) +
		sizeof(p3) +
		sizeof(p4) + 0);
	_emit(itemNumber);
	_emit(listLength);
	_emit(commandID);
	_emit(p1);
	_emit(p2);
	_emit(p3);
	_emit(p4);
	_endMessage();
};

/// Unpack a MSG_COMMAND_LIST message
inline void
unpack_msg_command_list(
	uint16_t &itemNumber,
	uint16_t &listLength,
	uint8_t &commandID,
	uint8_t &p1,
	int32_t &p2,
	int32_t &p3,
	int32_t &p4)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, itemNumber);
	_unpack(__p, listLength);
	_unpack(__p, commandID);
	_unpack(__p, p1);
	_unpack(__p, p2);
	_unpack(__p, p3);
	_unpack(__p, p4);
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
send_msg_command_mode_change(
	const uint16_t UNSPECIFIED)
{
	_startMessage(MSG_COMMAND_MODE_CHANGE,
		sizeof(UNSPECIFIED) + 0);
	_emit(UNSPECIFIED);
	_endMessage();
};

/// Unpack a MSG_COMMAND_MODE_CHANGE message
inline void
unpack_msg_command_mode_change(
	uint16_t &UNSPECIFIED)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, UNSPECIFIED);
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
send_msg_value_request(
	const uint8_t valueID,
	const uint8_t broadcast)
{
	_startMessage(MSG_VALUE_REQUEST,
		sizeof(valueID) +
		sizeof(broadcast) + 0);
	_emit(valueID);
	_emit(broadcast);
	_endMessage();
};

/// Unpack a MSG_VALUE_REQUEST message
inline void
unpack_msg_value_request(
	uint8_t &valueID,
	uint8_t &broadcast)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, valueID);
	_unpack(__p, broadcast);
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
send_msg_value_set(
	const uint8_t valueID,
	const uint32_t value)
{
	_startMessage(MSG_VALUE_SET,
		sizeof(valueID) +
		sizeof(value) + 0);
	_emit(valueID);
	_emit(value);
	_endMessage();
};

/// Unpack a MSG_VALUE_SET message
inline void
unpack_msg_value_set(
	uint8_t &valueID,
	uint32_t &value)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, valueID);
	_unpack(__p, value);
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
send_msg_value(
	const uint8_t valueID,
	const uint32_t value)
{
	_startMessage(MSG_VALUE,
		sizeof(valueID) +
		sizeof(value) + 0);
	_emit(valueID);
	_emit(value);
	_endMessage();
};

/// Unpack a MSG_VALUE message
inline void
unpack_msg_value(
	uint8_t &valueID,
	uint32_t &value)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, valueID);
	_unpack(__p, value);
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
send_msg_pid_request(
	const uint8_t pidSet)
{
	_startMessage(MSG_PID_REQUEST,
		sizeof(pidSet) + 0);
	_emit(pidSet);
	_endMessage();
};

/// Unpack a MSG_PID_REQUEST message
inline void
unpack_msg_pid_request(
	uint8_t &pidSet)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, pidSet);
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
send_msg_pid_set(
	const uint8_t pidSet,
	const int32_t p,
	const int32_t i,
	const int32_t d,
	const int16_t integratorMax)
{
	_startMessage(MSG_PID_SET,
		sizeof(pidSet) +
		sizeof(p) +
		sizeof(i) +
		sizeof(d) +
		sizeof(integratorMax) + 0);
	_emit(pidSet);
	_emit(p);
	_emit(i);
	_emit(d);
	_emit(integratorMax);
	_endMessage();
};

/// Unpack a MSG_PID_SET message
inline void
unpack_msg_pid_set(
	uint8_t &pidSet,
	int32_t &p,
	int32_t &i,
	int32_t &d,
	int16_t &integratorMax)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, pidSet);
	_unpack(__p, p);
	_unpack(__p, i);
	_unpack(__p, d);
	_unpack(__p, integratorMax);
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
send_msg_pid(
	const uint8_t pidSet,
	const int32_t p,
	const int32_t i,
	const int32_t d,
	const int16_t integratorMax)
{
	_startMessage(MSG_PID,
		sizeof(pidSet) +
		sizeof(p) +
		sizeof(i) +
		sizeof(d) +
		sizeof(integratorMax) + 0);
	_emit(pidSet);
	_emit(p);
	_emit(i);
	_emit(d);
	_emit(integratorMax);
	_endMessage();
};

/// Unpack a MSG_PID message
inline void
unpack_msg_pid(
	uint8_t &pidSet,
	int32_t &p,
	int32_t &i,
	int32_t &d,
	int16_t &integratorMax)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, pidSet);
	_unpack(__p, p);
	_unpack(__p, i);
	_unpack(__p, d);
	_unpack(__p, integratorMax);
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
send_msg_trim_startup(
	const uint16_t (&value)[8])
{
	_startMessage(MSG_TRIM_STARTUP,
		(sizeof(value[0]) * 8) + 0);
	_emit(value, 8);
	_endMessage();
};

/// Unpack a MSG_TRIM_STARTUP message
inline void
unpack_msg_trim_startup(
	uint16_t (&value)[8])
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, value, 8);
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
send_msg_trim_min(
	const uint16_t (&value)[8])
{
	_startMessage(MSG_TRIM_MIN,
		(sizeof(value[0]) * 8) + 0);
	_emit(value, 8);
	_endMessage();
};

/// Unpack a MSG_TRIM_MIN message
inline void
unpack_msg_trim_min(
	uint16_t (&value)[8])
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, value, 8);
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
send_msg_trim_max(
	const uint16_t (&value)[8])
{
	_startMessage(MSG_TRIM_MAX,
		(sizeof(value[0]) * 8) + 0);
	_emit(value, 8);
	_endMessage();
};

/// Unpack a MSG_TRIM_MAX message
inline void
unpack_msg_trim_max(
	uint16_t (&value)[8])
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, value, 8);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_RADIO_OUT 
//@{

/// Structure describing the payload section of the MSG_RADIO_OUT message
struct msg_radio_out {
	uint16_t value[8];
};

/// Send a MSG_RADIO_OUT message
inline void
send_msg_radio_out(
	const uint16_t (&value)[8])
{
	_startMessage(MSG_RADIO_OUT,
		(sizeof(value[0]) * 8) + 0);
	_emit(value, 8);
	_endMessage();
};

/// Unpack a MSG_RADIO_OUT message
inline void
unpack_msg_radio_out(
	uint16_t (&value)[8])
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, value, 8);
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
send_msg_sensor(
	const uint16_t UNSPECIFIED)
{
	_startMessage(MSG_SENSOR,
		sizeof(UNSPECIFIED) + 0);
	_emit(UNSPECIFIED);
	_endMessage();
};

/// Unpack a MSG_SENSOR message
inline void
unpack_msg_sensor(
	uint16_t &UNSPECIFIED)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, UNSPECIFIED);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_SERVO_OUT 
//@{

/// Structure describing the payload section of the MSG_SERVO_OUT message
struct msg_servo_out {
	int16_t value[8];
};

/// Send a MSG_SERVO_OUT message
inline void
send_msg_servo_out(
	const int16_t (&value)[8])
{
	_startMessage(MSG_SERVO_OUT,
		(sizeof(value[0]) * 8) + 0);
	_emit(value, 8);
	_endMessage();
};

/// Unpack a MSG_SERVO_OUT message
inline void
unpack_msg_servo_out(
	int16_t (&value)[8])
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, value, 8);
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
send_msg_pin_request(
	const uint16_t UNSPECIFIED)
{
	_startMessage(MSG_PIN_REQUEST,
		sizeof(UNSPECIFIED) + 0);
	_emit(UNSPECIFIED);
	_endMessage();
};

/// Unpack a MSG_PIN_REQUEST message
inline void
unpack_msg_pin_request(
	uint16_t &UNSPECIFIED)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, UNSPECIFIED);
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
send_msg_pin_set(
	const uint16_t UNSPECIFIED)
{
	_startMessage(MSG_PIN_SET,
		sizeof(UNSPECIFIED) + 0);
	_emit(UNSPECIFIED);
	_endMessage();
};

/// Unpack a MSG_PIN_SET message
inline void
unpack_msg_pin_set(
	uint16_t &UNSPECIFIED)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, UNSPECIFIED);
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
send_msg_dataflash_request(
	const uint16_t UNSPECIFIED)
{
	_startMessage(MSG_DATAFLASH_REQUEST,
		sizeof(UNSPECIFIED) + 0);
	_emit(UNSPECIFIED);
	_endMessage();
};

/// Unpack a MSG_DATAFLASH_REQUEST message
inline void
unpack_msg_dataflash_request(
	uint16_t &UNSPECIFIED)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, UNSPECIFIED);
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
send_msg_dataflash_set(
	const uint16_t UNSPECIFIED)
{
	_startMessage(MSG_DATAFLASH_SET,
		sizeof(UNSPECIFIED) + 0);
	_emit(UNSPECIFIED);
	_endMessage();
};

/// Unpack a MSG_DATAFLASH_SET message
inline void
unpack_msg_dataflash_set(
	uint16_t &UNSPECIFIED)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, UNSPECIFIED);
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
send_msg_eeprom_request(
	const uint16_t UNSPECIFIED)
{
	_startMessage(MSG_EEPROM_REQUEST,
		sizeof(UNSPECIFIED) + 0);
	_emit(UNSPECIFIED);
	_endMessage();
};

/// Unpack a MSG_EEPROM_REQUEST message
inline void
unpack_msg_eeprom_request(
	uint16_t &UNSPECIFIED)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, UNSPECIFIED);
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
send_msg_eeprom_set(
	const uint16_t UNSPECIFIED)
{
	_startMessage(MSG_EEPROM_SET,
		sizeof(UNSPECIFIED) + 0);
	_emit(UNSPECIFIED);
	_endMessage();
};

/// Unpack a MSG_EEPROM_SET message
inline void
unpack_msg_eeprom_set(
	uint16_t &UNSPECIFIED)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, UNSPECIFIED);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_POSITION_CORRECT 
//@{

/// Structure describing the payload section of the MSG_POSITION_CORRECT message
struct msg_position_correct {
	int16_t latError;
	int16_t lonError;
	int16_t altError;
	int16_t groundSpeedError;
};

/// Send a MSG_POSITION_CORRECT message
inline void
send_msg_position_correct(
	const int16_t latError,
	const int16_t lonError,
	const int16_t altError,
	const int16_t groundSpeedError)
{
	_startMessage(MSG_POSITION_CORRECT,
		sizeof(latError) +
		sizeof(lonError) +
		sizeof(altError) +
		sizeof(groundSpeedError) + 0);
	_emit(latError);
	_emit(lonError);
	_emit(altError);
	_emit(groundSpeedError);
	_endMessage();
};

/// Unpack a MSG_POSITION_CORRECT message
inline void
unpack_msg_position_correct(
	int16_t &latError,
	int16_t &lonError,
	int16_t &altError,
	int16_t &groundSpeedError)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, latError);
	_unpack(__p, lonError);
	_unpack(__p, altError);
	_unpack(__p, groundSpeedError);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_ATTITUDE_CORRECT 
//@{

/// Structure describing the payload section of the MSG_ATTITUDE_CORRECT message
struct msg_attitude_correct {
	int16_t rollError;
	int16_t pitchError;
	int16_t yawError;
};

/// Send a MSG_ATTITUDE_CORRECT message
inline void
send_msg_attitude_correct(
	const int16_t rollError,
	const int16_t pitchError,
	const int16_t yawError)
{
	_startMessage(MSG_ATTITUDE_CORRECT,
		sizeof(rollError) +
		sizeof(pitchError) +
		sizeof(yawError) + 0);
	_emit(rollError);
	_emit(pitchError);
	_emit(yawError);
	_endMessage();
};

/// Unpack a MSG_ATTITUDE_CORRECT message
inline void
unpack_msg_attitude_correct(
	int16_t &rollError,
	int16_t &pitchError,
	int16_t &yawError)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, rollError);
	_unpack(__p, pitchError);
	_unpack(__p, yawError);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_POSITION_SET 
//@{

/// Structure describing the payload section of the MSG_POSITION_SET message
struct msg_position_set {
	int32_t latitude;
	int32_t longitude;
	int32_t altitude;
	uint16_t heading;
};

/// Send a MSG_POSITION_SET message
inline void
send_msg_position_set(
	const int32_t latitude,
	const int32_t longitude,
	const int32_t altitude,
	const uint16_t heading)
{
	_startMessage(MSG_POSITION_SET,
		sizeof(latitude) +
		sizeof(longitude) +
		sizeof(altitude) +
		sizeof(heading) + 0);
	_emit(latitude);
	_emit(longitude);
	_emit(altitude);
	_emit(heading);
	_endMessage();
};

/// Unpack a MSG_POSITION_SET message
inline void
unpack_msg_position_set(
	int32_t &latitude,
	int32_t &longitude,
	int32_t &altitude,
	uint16_t &heading)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, latitude);
	_unpack(__p, longitude);
	_unpack(__p, altitude);
	_unpack(__p, heading);
};
//@}

//////////////////////////////////////////////////////////////////////
/// @name MSG_ATTITUDE_SET 
//@{

/// Structure describing the payload section of the MSG_ATTITUDE_SET message
struct msg_attitude_set {
	int16_t roll;
	int16_t pitch;
	uint16_t yaw;
};

/// Send a MSG_ATTITUDE_SET message
inline void
send_msg_attitude_set(
	const int16_t roll,
	const int16_t pitch,
	const uint16_t yaw)
{
	_startMessage(MSG_ATTITUDE_SET,
		sizeof(roll) +
		sizeof(pitch) +
		sizeof(yaw) + 0);
	_emit(roll);
	_emit(pitch);
	_emit(yaw);
	_endMessage();
};

/// Unpack a MSG_ATTITUDE_SET message
inline void
unpack_msg_attitude_set(
	int16_t &roll,
	int16_t &pitch,
	uint16_t &yaw)
{
	uint8_t *__p = &_decodeBuf.payload[0];
	_unpack(__p, roll);
	_unpack(__p, pitch);
	_unpack(__p, yaw);
};
//@}

//////////////////////////////////////////////////////////////////////
/// Message ID values
enum MessageID {
	MSG_PID = 0x42,
	MSG_DATAFLASH_REQUEST = 0x90,
	MSG_DATAFLASH_SET = 0x91,
	MSG_SENSOR = 0x60,
	MSG_VALUE_REQUEST = 0x30,
	MSG_VALUE_SET = 0x31,
	MSG_VALUE = 0x32,
	MSG_PIN_REQUEST = 0x80,
	MSG_PIN_SET = 0x81,
	MSG_POSITION_CORRECT = 0xb0,
	MSG_ACKNOWLEDGE = 0x0,
	MSG_ATTITUDE_CORRECT = 0xb1,
	MSG_HEARTBEAT = 0x1,
	MSG_POSITION_SET = 0xb2,
	MSG_ATTITUDE = 0x2,
	MSG_ATTITUDE_SET = 0xb3,
	MSG_LOCATION = 0x3,
	MSG_PRESSURE = 0x4,
	MSG_TRIM_STARTUP = 0x50,
	MSG_STATUS_TEXT = 0x5,
	MSG_TRIM_MIN = 0x51,
	MSG_PERF_REPORT = 0x6,
	MSG_TRIM_MAX = 0x52,
	MSG_VERSION_REQUEST = 0x7,
	MSG_RADIO_OUT = 0x53,
	MSG_VERSION = 0x8,
	MSG_COMMAND_REQUEST = 0x20,
	MSG_COMMAND_UPLOAD = 0x21,
	MSG_COMMAND_LIST = 0x22,
	MSG_COMMAND_MODE_CHANGE = 0x23,
	MSG_SERVO_OUT = 0x70,
	MSG_EEPROM_REQUEST = 0xa0,
	MSG_EEPROM_SET = 0xa1,
	MSG_PID_REQUEST = 0x40,
	MSG_PID_SET = 0x41,
	MSG_ANY = 0xfe,
	MSG_NULL = 0xff
};

//////////////////////////////////////////////////////////////////////
/// Message buffer sizing
union _binCommBufferSizer {
	struct msg_acknowledge msg_acknowledge;
	struct msg_status_text msg_status_text;
	struct msg_heartbeat msg_heartbeat;
	struct msg_attitude msg_attitude;
	struct msg_location msg_location;
	struct msg_pressure msg_pressure;
	struct msg_perf_report msg_perf_report;
	struct msg_version_request msg_version_request;
	struct msg_version msg_version;
	struct msg_command_request msg_command_request;
	struct msg_command_upload msg_command_upload;
	struct msg_command_list msg_command_list;
	struct msg_command_mode_change msg_command_mode_change;
	struct msg_value_request msg_value_request;
	struct msg_value_set msg_value_set;
	struct msg_value msg_value;
	struct msg_pid_request msg_pid_request;
	struct msg_pid_set msg_pid_set;
	struct msg_pid msg_pid;
	struct msg_trim_startup msg_trim_startup;
	struct msg_trim_min msg_trim_min;
	struct msg_trim_max msg_trim_max;
	struct msg_radio_out msg_radio_out;
	struct msg_sensor msg_sensor;
	struct msg_servo_out msg_servo_out;
	struct msg_pin_request msg_pin_request;
	struct msg_pin_set msg_pin_set;
	struct msg_dataflash_request msg_dataflash_request;
	struct msg_dataflash_set msg_dataflash_set;
	struct msg_eeprom_request msg_eeprom_request;
	struct msg_eeprom_set msg_eeprom_set;
	struct msg_position_correct msg_position_correct;
	struct msg_attitude_correct msg_attitude_correct;
	struct msg_position_set msg_position_set;
	struct msg_attitude_set msg_attitude_set;
};
#define BINCOMM_MAX_MESSAGE_SIZE sizeof(union _binCommBufferSizer)

#pragma pack(pop)
