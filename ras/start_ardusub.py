"""
ArduSub routine module to set parameters, load in pre-defined mission file, arm 
throttles, and set the sub into 'AUTO' mode to carry out mission.

Some comments taken directly from ArduSub docs.
"""

# Disable "Bare exception" warning
# pylint: disable=W0702

import time
import json
import math
import multiprocessing

from datetime import datetime as date

from pymavlink import mavutil

# from ardusub_dict import mavlink_dict

class AutoSubRoutine:
    """
    Automated routine to handle ArduSub input and data output
    """

    def __init__(self, mission_file): #TODO: import python script with pre-defined dictionary
        
        # Mission name unique w/ datetime stamp
        self.mission_name = f"UUV Baseline Mission - Tsunami - {date.today()}"

        self.mission_date = f"{date.now()}"

        # Start empty dictionary with mission name
        self.sim_dict = {
            "mission_name": self.mission_name,
        }

        # Recorder module start time
        self.start_time = time.time()

        # Set predefined mission
        self.mission = mission_file

        # MISSION_CURRENT.mission_state pymavlink packet 
        #   flag to signal that mission has finished.
        #   Options:
        #       
        self.finish_flag = 5

        # Set simulation timer
        self.msg_freq = 0.75

        # Track all received message types for the current run
        self.received_message_types = set()

        # Create the connection
        #   If using a companion computer
        #   the default connection is available
        #   at ip 192.168.2.1 and the port 14550
        # Note: The connection is done with 'udpin' and not 'udpout'.
        #   You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
        #   uses a 'udpbcast' (client) and not 'udpin' (server).
        #   If you want to use QGroundControl in parallel with your python script,
        #   it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
        #   E.g: --out udpbcast:192.168.2.255:yourport
        # Since we are using the simulated ArduSub module, the comms link is
        #   done through tcp at ip 127.0.0.1 port 5762
        self.sub = mavutil.mavlink_connection('tcp:127.0.0.1:5762', dialect='ardupilotmega')

        # Check if connection is valid
        self.sub.wait_heartbeat()

        # Set up multiprocessing queue for incoming messages
        self.queue = multiprocessing.Queue(maxsize=100)

        # Multiprocessing event to stop recording
        self.stop_record = multiprocessing.Event()

        # MongoDB setup
        # self.mongo_uri = ('')
        # self.client = MongoClient(self.mongo_uri)
        # self.db = self.client['ras_data']


    def ready_sub(self, mode):
        """
        Set ArduSub parameters, send specific commands, arm sub to prep for mode initialization,
        launch custom mode to start sub. 
        """

        # Set battery parameters (Matching Lithium-ion (14.8V, 18Ah) BlueROV2 battery)
        # Parameter types are as follows:
        #   INT8, INT16, INT32 or REAL32
        # self.set_param('FRAME_CONFIG', 4, 'REAL32')
        self.set_param('BATT_CAPACITY', 18000, 'REAL32')
        # self.set_param('BATT_LOW_MAH', 1800, 'REAL32')
        # self.set_param('BATT_FS_LOW_ACT', 3, 'REAL32')

        # Set simulation battery parameters TODO: Is this neccessary?
        self.set_param('SIM_BATT_VOLTAGE', 14.8, 'REAL32')
        self.set_param('SIM_BATT_CAP_AH', 18, 'REAL32')

        # TODO: Further define set_param function, investigate parameters,
        #       add as needed

        # # self.sub.mav.param_set_send(
        # #     self.sub.target_system,
        # #     self.sub.target_component,
        # #     b'BATT_AMP_PERVLT',
        # #     ,
        # #     mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        # # )

        # # Read param set ACK
        # ack = self.sub.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        # print('name: %s\tvalue: %d' %
        #     (ack['param_id'], ack['param_value']))

        # Buffer to not send command too early
        time.sleep(1)
        
        # List for waypionts
        waypoints = []

        with open(self.mission, 'r') as f:
            lines = f.readlines()
        
        # Skip file header
        lines = lines[1:]

        # Parse through MAVLink mission file
        for line in lines:
            fields = line.strip().split('\t')
            seq = int(fields[0])  # Sequence number
            frame = int(fields[2])  # Coordinate frame
            command = int(fields[3])  # MAVLink command
            lat = float(fields[8])  # Latitude
            lon = float(fields[9])  # Longitude
            alt = float(fields[10])  # Altitude
            autocontinue = int(fields[11])  # Auto-continue flag
            waypoints.append((seq, frame, command, lat, lon, alt, autocontinue))

        for wp in waypoints:
            itr, frame, command, lat, lon, alt, autocontinue = wp
            self.sub.mav.mission_item_int_send(
                self.sub.target_system,
                self.sub.target_component,
                itr,  # Waypoint iteration
                frame,  # Frame
                command,  # Command
                0,  # Current WP
                autocontinue,  # Auto-continue
                0, 0, 0, 0,  # Params 1-4 (unused for basic waypoints)
                int(lat * 1e7),  # Latitude (in 1e7)
                int(lon * 1e7),  # Longitude (in 1e7)
                alt  # Altitude
            )
            print(f"Loaded {itr}: lat={lat}, lon={lon}, alt={alt}")
        
        # Buffer to not send command too early
        time.sleep(2)

        print('Sending mission to sub...')

        # Send waypoints to sub
        self.sub.mav.mission_count_send(
            self.sub.target_system,
            self.sub.target_component,
            len(waypoints)
        )
        
        # Buffer to not send command too early
        time.sleep(2)

        self.sub.mav.command_long_send(
            1, # autopilot system id
            1, # autopilot component id
            400, # command id, ARM/DISARM
            0, # confirmation
            1, # arm!
            0,0,0,0,0,0 # unused parameters for this command
        )

        # Buffer to not send command too early
        time.sleep(2)

        # Set sub to Auto mode to carry out mission automatically
        self.sub.mav.set_mode_send(
            self.sub.target_system, # System target
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, # Enable custom mode
            self.sub.mode_mapping()[mode] # Start auto mode
        )

        print('ArduSub underway...')

    
    def set_param(self, param_name, param_value, data_type):
        """
        Set parameter to specific value or flag if no parameter uses
        flags to control actions

        Args:
            parameter_name (str): Parameter name to be set
            parameter_value (int/float): Desired parameter value
            value_type (str): Desired parameter data type from ArduPilot compatible
                              data types: INT8, INT16, INT32 or REAL32             
        """

        param = param_name.encode('utf-8')
        
        # Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot.
        # Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM.
        # The parameter variable type is described by MAV_PARAM_TYPE in 
        # http://mavlink.org/messages/common.
        if data_type == 'INT8':
            self.sub.mav.param_set_send(
                self.sub.target_system,
                self.sub.target_component,
                param,
                param_value,
                mavutil.mavlink.MAV_PARAM_TYPE_INT8
            )
        elif data_type == 'INT16':
            self.sub.mav.param_set_send(
                self.sub.target_system,
                self.sub.target_component,
                param,
                param_value,
                mavutil.mavlink.MAV_PARAM_TYPE_INT16
            )
        elif data_type == 'INT32':
            self.sub.mav.param_set_send(
                self.sub.target_system,
                self.sub.target_component,
                param,
                param_value,
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )
        elif data_type == 'REAL32':
            self.sub.mav.param_set_send(
                self.sub.target_system,
                self.sub.target_component,
                param,
                param_value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
        else:
            print(f'{data_type} not supported. Check module doc string.')
            return

        # Read param set ACK
        # IMPORTANT: The receiving component should acknowledge the new parameter value 
        # by sending a param_value message to all communication partners.
        # This will also ensure that multiple GCS all have an up-to-date list of all parameters.
        # If the sending GCS did not receive a PARAM_VALUE message within its timeout time,
        # it should re-send the PARAM_SET message.
        ack = self.sub.recv_match(type='PARAM_VALUE', blocking=True, timeout=1).to_dict()
        print('Parameter: %s\tvalue: %d' %
            (ack['param_id'], ack['param_value']))

        # Buffer to not send messages too early
        time.sleep(1)


    def queue_messages(self):
        """
        Multiprocessing shared queue to store messages as are received.
        """
        while not self.stop_record.is_set():
            try:
                message = self.sub.recv_match(blocking=False, timeout=1)
                if message:
                    self.queue.put(message.to_dict(), timeout=5)
            except Exception as e:
                print(f'Message error: {e}')


    def request_and_record(self):
        """
        Listen for all MAVLink messages output by ArduSub and
        update sim_dict dictionary and timer while filling missing keys with 0
        if mavpackettype is not recieved at constant interval.
        """

        # Store previous entries to carry data over if message type
        # is not sent when other messsages are received
        previous_entry = {}

        start_time = time.time()

        # Flag to get one final reading before stopping recording session
        loop = True
        
        while not self.stop_record.is_set():
            print(self.sim_dict, "\n")
            try:
                # Calculate elapsed time and format as h:m:s
                elapsed_time = int(time.time() - start_time)
                hours, remainder = divmod(elapsed_time, 3600)
                minutes, seconds = divmod(remainder, 60)
                formatted_time = f'{hours:02}:{minutes:02}:{seconds:02}'

                # Create new entries with timestamp for time series format
                current_entry = {}

                # Track which keys were updated
                updated_keys = set()

                # Receive MAVLink message
                while not self.queue.empty():
                    # Convert message to dict (pymavlink method)
                    message_content = self.queue.get()
                    packet_type = message_content.get('mavpackettype', '')

                    # Ignore messages with Unknown mavpackettype
                    if packet_type.startswith("UNKNOWN_") or packet_type.startswith("BAD_"):
                        continue

                    # Remove redundant "mavpackettype" field
                    message_content.pop('mavpackettype', None)

                    # Add message received to current entry and update keys
                    current_entry[packet_type] = message_content
                    updated_keys.add(packet_type)

                    # Add packet type to set
                    self.received_message_types.add(packet_type)

                # Fill missing keys with 0
                # for message_type in self.received_message_types:
                #     if message_type not in updated_keys:
                #         current_entry[message_type] = math.nan

                # Place previous entry into entry if no message has arrived at
                #   defined message frequency
                # TODO: Replace previous entry values with NaN if there is no entry at
                #       time step
                if previous_entry:
                    for message_type in self.received_message_types:
                        if message_type not in updated_keys:
                            current_entry[message_type] = math.nan

                # Add the current timestamp entry to the time-series data
                self.sim_dict[formatted_time] = current_entry

                # Copy current message content over to previous entry dict to store
                previous_entry = current_entry.copy()

                # Control msg frequency
                time.sleep(self.msg_freq)

                # Flag to stop recording if mission complete
                if self.sim_dict[formatted_time]['MISSION_CURRENT']['mission_state'] == self.finish_flag:
                    break

            except Exception as e:
                print(f'Error receiving message: {e}')
                
                # Grab one final group of messages before stopping recording session
                if loop:
                    loop = False
                else:
                    break
                
        print('Stopping recorder...')

        self.stop_recording_and_post()


    def stop_recording_and_post(self):
        """
        Stop the recorder and post outcome to MongoDB
        """

        # Event trigger to stop recording
        self.stop_record.set()

        # Specify MongoDB collection
        # collection = self.db["ardusub_tsunami_mission_baseline_data"]

        # TODO: Loop to correctly format latitude, longitude, and temperature
        #         to floating point values
        # Latitude and Longitude are incorrectly formatted wherever 
        #   lat/latitude/lon/lng/longitude are read
        for entry in self.sim_dict:
            if entry == {'timestamp': '00:00:00'}:
                data.remove(entry)
            for key, value in list(entry.items()):
                if isinstance(value, dict):
                    for sub_key, sub_value in list(value.items()):
                        if sub_key in (
                            "lat",
                            "lon",
                            "lng",
                            "latitude",
                            "longitude"
                            ) and isinstance(sub_value, int):
                                str_value = str(sub_value)
                                if str_value.startswith('-'): 
                                    value[sub_key] = float(str_value[:3] + "." + str_value[3:])
                                else:
                                    value[sub_key] = float(str_value[:2] + "." + str_value[2:])

        with open(f'{self.mission_name}.txt', 'w') as converted_file: 
            converted_file.write(json.dumps(self.sim_dict))


if __name__ == "__main__":
    launch = AutoSubRoutine('missions/short_wp_test.txt')

    # Potential user input
    vehicle_mode = 'AUTO'

    launch.ready_sub(vehicle_mode)

    # Creating threads to record data on module boot before mission start
    message_queue = multiprocessing.Process(target=launch.queue_messages)
    record_messages = multiprocessing.Process(target=launch.request_and_record)

    # Start processes (ArduPilot will always run as baskground task)
    message_queue.start()
    record_messages.start()

    # TODO: Add flag that tells processes to join after certain condition is passed
    #       since ArduSub is a constand process in itself

    # try:
    #     # TODO: Add function that works for flag
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     # Put dictionary into txt file
    #     launch.stop_recording_and_post()

    # Join processes to end
    message_queue.join()
    record_messages.join()
