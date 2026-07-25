from collections import OrderedDict

mavlink_message_lengths_dict = OrderedDict([
(                                    'NONE' ,   0),
(                               'HEARTBEAT' ,  17), # ID#000 The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying out the user interface based on the autopilot).
(                              'SYS_STATUS' ,  39), # ID#001 The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows whether the system is currently active or not and if an emergency occurred. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occurred it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
(                             'SYSTEM_TIME' ,  20), # ID#002 The system time is the time of the master clock, typically the computer clock of the main onboard computer.
(                                    'PING' ,  22), # ID#004 A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections.
(             'CHANGE_OPERATOR_CONTROL_ACK' ,  11), # ID#006 Accept / deny control of this MAV
(                             'PARAM_VALUE' ,  33), # ID#022 Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout.
(                             'GPS_RAW_INT' ,  38), # ID#024 The global position, as returned by the Global Positioning System (GPS). This is                NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
(                              'GPS_STATUS' , 109), # ID#025 The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
(                              'SCALED_IMU' ,  30), # ID#026 The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
(                                 'RAW_IMU' ,  34), # ID#027 The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw values without any scaling to allow data capture and system debugging.
(                            'RAW_PRESSURE' ,  24), # ID#028 The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
(                         'SCALED_PRESSURE' ,  22), # ID#029 The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.
(                                'ATTITUDE' ,  36), # ID#030 The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
(                     'ATTITUDE_QUATERNION' ,  40), # ID#031 The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
(                      'LOCAL_POSITION_NED' ,  36), # ID#032 The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
(                     'GLOBAL_POSITION_INT' ,  36), # ID#033 The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It               is designed as scaled integer message since the resolution of float is not sufficient.
(                      'RC_CHANNELS_SCALED' ,  30), # ID#034 The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to UINT16_MAX.
(                         'RC_CHANNELS_RAW' ,  30), # ID#035 The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
(                        'SERVO_OUTPUT_RAW' ,  29), # ID#036 The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
(                         'MISSION_CURRENT' ,  10), # ID#042 Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
(                           'MISSION_COUNT' ,  12), # ID#044 This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of MISSIONs.
(                    'MISSION_ITEM_REACHED' ,  10), # ID#046 A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next MISSION.
(                             'MISSION_ACK' ,  11), # ID#047 Ack message during MISSION handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
(                       'GPS_GLOBAL_ORIGIN' ,  20), # ID#049 Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) position
(                            'PARAM_MAP_RC' ,  45), # ID#050 Bind a RC channel to a parameter. The parameter should change according to the RC channel value.
(                     'MISSION_REQUEST_INT' ,  12), # ID#051 Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM_INT message. https://mavlink.io/en/services/mission.html
(                     'SAFETY_ALLOWED_AREA' ,  33), # ID#055 Read out the safety zone the MAV currently assumes.
(                 'ATTITUDE_QUATERNION_COV' ,  80), # ID#061 The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
(                   'NAV_CONTROLLER_OUTPUT' ,  34), # ID#062 The state of the fixed wing navigation and position controller.
(                 'GLOBAL_POSITION_INT_COV' , 189), # ID#063 The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset.
(                  'LOCAL_POSITION_NED_COV' , 233), # ID#064 The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
(                             'RC_CHANNELS' ,  50), # ID#065 The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
(                        'MISSION_ITEM_INT' ,  45), # ID#073 Message encoding a mission item. This message is emitted to announce                the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also https://mavlink.io/en/services/mission.html
(                                 'VFR_HUD' ,  28), # ID#074 Metrics typically displayed on a HUD for fixed wing aircraft
(                             'COMMAND_ACK' ,  11), # ID#077 Report status of a command. Includes feedback whether the command was executed.
(                         'ATTITUDE_TARGET' ,  45), # ID#083 Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way.
(               'POSITION_TARGET_LOCAL_NED' ,  59), # ID#085 Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled this way.
(              'POSITION_TARGET_GLOBAL_INT' ,  59), # ID#087 Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled this way.
( 'LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET' ,  36), # ID#089 The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
(                            'HIL_CONTROLS' ,  50), # ID#091 Sent from autopilot to simulation. Hardware in the loop control outputs
(                   'HIL_ACTUATOR_CONTROLS' ,  89), # ID#093 Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS)
(                            'OPTICAL_FLOW' ,  34), # ID#100 Optical flow from a flow sensor (e.g. optical mouse sensor)
(         'GLOBAL_VISION_POSITION_ESTIMATE' ,  40), # ID#101 
(                'VISION_POSITION_ESTIMATE' ,  40), # ID#102 
(                   'VISION_SPEED_ESTIMATE' ,  28), # ID#103 
(                 'VICON_POSITION_ESTIMATE' ,  40), # ID#104 
(                             'HIGHRES_IMU' ,  70), # ID#105 The IMU readings in SI units in NED body frame
(                        'OPTICAL_FLOW_RAD' ,  52), # ID#106 Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
(                            'RADIO_STATUS' ,  17), # ID#109 Status generated by radio and injected into MAVLink stream.
(                  'FILE_TRANSFER_PROTOCOL' , 262), # ID#110 File transfer message
(                                'TIMESYNC' ,  24), # ID#111 Time synchronization message.
(                          'CAMERA_TRIGGER' ,  20), # ID#112 Camera-IMU triggering and synchronisation message.
(                             'SCALED_IMU2' ,  30), # ID#116 The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to the described units
(                               'LOG_ENTRY' ,  22), # ID#118 Reply to LOG_REQUEST_LIST
(                                'LOG_DATA' , 105), # ID#120 Reply to LOG_REQUEST_DATA
(                                'GPS2_RAW' ,  43), # ID#124 Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).
(                            'POWER_STATUS' ,  14), # ID#125 Power supply status
(                                 'GPS_RTK' ,  43), # ID#127 RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
(                                'GPS2_RTK' ,  43), # ID#128 RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
(                             'SCALED_IMU3' ,  30), # ID#129 The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described units
(             'DATA_TRANSMISSION_HANDSHAKE' ,  21), # ID#130 
(                       'ENCAPSULATED_DATA' , 263), # ID#131 
(                         'DISTANCE_SENSOR' ,  22), # ID#132 
(                         'TERRAIN_REQUEST' ,  26), # ID#133 Request for terrain data and terrain status
(                          'TERRAIN_REPORT' ,  30), # ID#136 Response from a TERRAIN_CHECK request
(                        'SCALED_PRESSURE2' ,  22), # ID#137 Barometer readings for 2nd barometer
(                           'ATT_POS_MOCAP' ,  44), # ID#138 Motion capture attitude and position
(                                'ALTITUDE' ,  40), # ID#141 The current system altitude.
(                        'SCALED_PRESSURE3' ,  22), # ID#143 Barometer readings for 3rd barometer
(                           'FOLLOW_TARGET' , 101), # ID#144 current motion information from a designated system
(                    'CONTROL_SYSTEM_STATE' , 108), # ID#146 The smoothed, monotonic system state used to feed the control loops of the system.
(                          'BATTERY_STATUS' ,  44), # ID#147 Battery information
(                       'AUTOPILOT_VERSION' ,  68), # ID#148 Version and capability of autopilot software
(                          'LANDING_TARGET' ,  38), # ID#149 The location of a landing area captured from a downward facing camera
(                          'SENSOR_OFFSETS' ,  50), # ID#150 Offsets and calibrations values for hardware sensors. This makes it easier to debug the calibration process.
(                                 'MEMINFO' ,  12), # ID#152 state of APM memory
(                                  'AP_ADC' ,  20), # ID#153 raw ADC output
(                         'DIGICAM_CONTROL' ,  21), # ID#155 Control on-board Camera Control System to take shots.
(                            'MOUNT_STATUS' ,  22), # ID#158 Message with some status from APM to GCS about camera or antenna mount
(                             'FENCE_POINT' ,  20), # ID#160 A fence point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS
(                            'FENCE_STATUS' ,  16), # ID#162 Status of geo-fencing. Sent in extended status stream when fencing enabled
(                                    'AHRS' ,  36), # ID#163 Status of DCM attitude estimator
(                                'SIMSTATE' ,  52), # ID#164 Status of simulation environment, if used
(                                'HWSTATUS' ,  11), # ID#165 Status of key hardware
(                                   'RADIO' ,  17), # ID#166 Status generated by radio
(                           'LIMITS_STATUS' ,  30), # ID#167 Status of AP_Limits. Sent in extended status stream when AP_Limits is enabled
(                                    'WIND' ,  20), # ID#168 Wind estimation
(                                  'DATA16' ,  26), # ID#169 Data packet, size 16
(                                  'DATA32' ,  42), # ID#170 Data packet, size 32
(                                  'DATA64' ,  74), # ID#171 Data packet, size 64
(                                  'DATA96' , 106), # ID#172 Data packet, size 96
(                             'RANGEFINDER' ,  16), # ID#173 Rangefinder reporting
(                        'AIRSPEED_AUTOCAL' ,  56), # ID#174 Airspeed auto-calibration
(                             'RALLY_POINT' ,  27), # ID#175 A rally point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS
(                       'COMPASSMOT_STATUS' ,  28), # ID#177 Status of compassmot calibration
(                                   'AHRS2' ,  32), # ID#178 Status of secondary AHRS filter if available
(                           'CAMERA_STATUS' ,  37), # ID#179 Camera Event
(                         'CAMERA_FEEDBACK' ,  53), # ID#180 Camera Capture Feedback
(                                'BATTERY2' ,  12), # ID#181 Deprecated. Use BATTERY_STATUS instead. 2nd Battery status
(                                   'AHRS3' ,  48), # ID#182 Status of third AHRS filter if available. This is for ANU research group (Ali and Sean)
(               'AUTOPILOT_VERSION_REQUEST' ,  10), # ID#183 Request the autopilot version from the system/component.
(                   'REMOTE_LOG_DATA_BLOCK' , 214), # ID#184 Send a block of log data to remote location
(                        'MAG_CAL_PROGRESS' ,  35), # ID#191 Reports progress of compass calibration.
(                          'MAG_CAL_REPORT' ,  52), # ID#192 Reports results of completed compass calibration. Sent until MAG_CAL_ACK received.
(                       'EKF_STATUS_REPORT' ,  30), # ID#193 EKF Status message including flags and variances
(                              'PID_TUNING' ,  33), # ID#194 PID tuning information
(                           'GIMBAL_REPORT' ,  50), # ID#200 3 axis gimbal mesuraments
(                'GIMBAL_TORQUE_CMD_REPORT' ,  16), # ID#214 100 Hz gimbal torque command telemetry
(                         'GOPRO_HEARTBEAT' ,  11), # ID#215 Heartbeat from a HeroBus attached GoPro
(                      'GOPRO_SET_RESPONSE' ,  10), # ID#219 Response from a GOPRO_COMMAND set request
(                                     'RPM' ,  16), # ID#226 RPM sensor output
(                        'ESTIMATOR_STATUS' ,  50), # ID#230 Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS enum definition for further information. The innovaton test ratios show the magnitude of the sensor innovation divided by the innovation check threshold. Under normal operation the innovaton test ratios should be below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should be optional and controllable by the user.
(                                'WIND_COV' ,  48), # ID#231 
(                               'GPS_INPUT' ,  71), # ID#232 GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position estimate of the system.
(                           'GPS_RTCM_DATA' , 190), # ID#233 RTCM message for injecting into the onboard GPS (used for DGPS)
(                            'HIGH_LATENCY' ,  48), # ID#234 Message appropriate for high latency connections like Iridium
(                               'VIBRATION' ,  40), # ID#241 Vibration levels and accelerometer clipping
(                           'HOME_POSITION' ,  60), # ID#242 This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitly set by the operator before or after. The position the system will return to and land on. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
(                        'MESSAGE_INTERVAL' ,  14), # ID#244 This interface replaces DATA_STREAM
(                      'EXTENDED_SYS_STATE' ,  10), # ID#245 Provides state for additional features
(                            'ADSB_VEHICLE' ,  46), # ID#246 The location and information of an ADSB vehicle
(                               'COLLISION' ,  27), # ID#247 Information about a potential collision
(                            'V2_EXTENSION' , 262), # ID#248 Message implementing parts of the V2 payload specs in V1 frames for transitional support.
(                             'MEMORY_VECT' ,  44), # ID#249 Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
(                              'DEBUG_VECT' ,  38), # ID#250 
(                       'NAMED_VALUE_FLOAT' ,  26), # ID#251 Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
(                         'NAMED_VALUE_INT' ,  26), # ID#252 Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
(                              'STATUSTEXT' ,  59), # ID#253 Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
(                                   'DEBUG' ,  17), # ID#254 Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
])
