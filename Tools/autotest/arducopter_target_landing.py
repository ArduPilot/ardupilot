import os
import numpy as np
from typing import TypedDict, Dict, List
from numpy.typing import NDArray
import csv
import types
import shutil
import datetime

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import Test
from arducopter import AutoTestCopter

from Target_Landing.Platform import Platform
from Target_Landing.Landing_Error_Visualization.simulation_env import SeaState, SimulationEnvironment, TestInfo, WindSpeed, PositionSensor
from Target_Landing.DroneStabilityMonitor import DroneStabilityMonitor
from Target_Landing.AsyncMavlinkHandler import AsyncMavlinkHandler, MAVMessages

testdir = os.path.dirname(os.path.realpath(__file__))
drone_lat_initial = 12.992006
drone_lng_initial = 80.236649
SITL_START_LOCATION = mavutil.location(drone_lat_initial,drone_lng_initial,0,0)

class AutoTestCopterTargetLanding(AutoTestCopter):

    target_origin = np.zeros((3,))
    target_v0 = np.array([5, 0, 0])
    target_state = np.zeros((6,))
    target_state_noisy = np.zeros((6,))
    drone_state = np.zeros((6,))
    target: Platform
    target_pos_last_updated_on = 0
    drone_traj = []
    drone_att = []
    drone_battery_info = []

    drone_operation_timeout = 3*60          # Timeout in secs to stop the test when drone cannot land

    R = 6378137.0  # Radius of earth in meters

    log_folder_path = os.path.expanduser("~/UAV_Landing/hor_clearance_5")

    sea_states: List[SeaState] = SimulationEnvironment.sea_states



    def get_acceleration(self, a, acc_last_update_time, tstart):
        
        now = self.get_sim_time_cached()
        acc_duration = 2
        i = int(((now - tstart)/acc_duration)/a.shape[0])

        if now - acc_last_update_time > acc_duration:
            acc_last_update_time = now
        
        return a[i, :], acc_last_update_time

    def calculate_target_origin(self, distance_from_drone: float):
        '''
        Calculate the origin of target from given distance from the drone start location

        (Args):
            distance_from_drone in meters
        (Return):
            lat (degE7) and lng (degE7) and alt (m) of target origin
        '''

        theta = np.random.uniform(low=0, high=2*np.pi)

        dx = distance_from_drone*np.cos(theta)
        dy = distance_from_drone*np.sin(theta)

        dlat = self.target._dx_to_dlat(dx)
        dlng = self.target._dy_to_dlng(dy, drone_lat_initial)

        target_origin = np.array([
            drone_lat_initial*1e7 + dlat,
            drone_lng_initial*1e7 + dlng,
            0
        ])
        return target_origin

    def initialize_target(self, test_config: TestInfo):
        '''
        Intialize the position and velocity of the moving target. Initial position
        of the target is defined as the origin
        
        Args: test_config
        '''
        self.target_v0 = test_config['velocity']

        self.target = Platform()
        self.target_origin = self.calculate_target_origin(test_config["target_distance_from_drone"])
        self.target.spawn(self.target_origin, self.target_v0)

        self.target_state[0:3] = self.target_origin
        self.target_state[3:6] = self.target_v0
        self.target_state_noisy = self.target_state  + self.target.position_error(test_config["drone_pos_sensor"]['error'], self.target_state)


    def move_target(self, test_config: TestInfo):

        self.target.dt = 1/test_config["target_pos_sensor"]['update_rate']
        self.target.an = test_config['acceleration'][0]
        self.target.at = test_config["acceleration"][1]
        self.target_state = self.target.move(self.target_state[0:3], self.target_state[3:6])

        self.target_state_noisy =  self.target_state + self.target.position_error(test_config["target_pos_sensor"]['error'], self.target_state)

        # z = self.mavlink.get_elevation(target_state[0], target_state[1])
        # if z == None:
        #     z = 0
        # target_state[2] = z

    def set_follow_parameters(self):
        """
        Set and verify AP_Follow parameters
        """
        # Distance related parameters
        self.set_parameter("FOLL_DIST_MAX", 100)    # Maximum follow distance in meters
        
        # Following behavior parameters
        self.set_parameter("FOLL_ENABLE", 1)        # Enable following (0:Disabled, 1:Enabled)
        self.set_parameter("FOLL_POS_P", 0.5)       # Position error P gain
        
        # Yaw behavior
        self.set_parameter("FOLL_YAW_BEHAVE", 1)    # Yaw behavior (0:None, 1:Face Lead, 2:Same as Lead, 3:Direction of Travel)

        self.set_parameter("TERRAIN_ENABLE", 0)
        self.set_parameter("SIM_SPEEDUP", 1)


        # Verify parameters were set correctly
        self.progress("Verifying AP_Follow parameters...")
        if (self.get_parameter("FOLL_DIST_MAX") != 100 or
            self.get_parameter("FOLL_ENABLE") != 1 or
            self.get_parameter("FOLL_POS_P") != 0.5 or
            self.get_parameter("FOLL_YAW_BEHAVE") != 1 or 
            self.get_parameter("TERRAIN_ENABLE") != 0):
            raise NotAchievedException("Failed to set AP_Follow parameters correctly")
        
        self.progress("AP_Follow parameters set successfully")

    def set_wind_parameters(self, wind_speed: WindSpeed):

        self.set_parameters({
            "SIM_WIND_SPD": SimulationEnvironment.get_wind_spd_from_sea_state(wind_speed['sea_state']),
            "SIM_WIND_DIR": wind_speed['dir'],
            "SIM_WIND_TURB": wind_speed['turbulence']
        })

    def set_drone_parameters(self, drone_pos_sensor: PositionSensor):

        self.set_parameter("FOLL_GPS_N", drone_pos_sensor['error']['sigma'])
        self.set_parameter("SIM_GPS_HZ", drone_pos_sensor['update_rate'])
        self.set_parameter("SIM_GPS_LAG_MS", drone_pos_sensor["update_latency"]*1000)


    def send_target_pos(self, time_boot):
        gpi = self.mav.mav.global_position_int_encode(
                    int(time_boot * 1000), # time_boot_ms
                    int(self.target_state_noisy[0]),
                    int(self.target_state_noisy[1]),
                    int(self.target_state_noisy[2] * 1000), # alt in mm
                    int(self.target_state_noisy[2]* 1000), # relative alt - urp.
                    vx=int(self.target_state_noisy[3]*100),
                    vy=int(self.target_state_noisy[4]*100),
                    vz=int(self.target_state_noisy[5]*100),
                    hdg=0
                )
        gpi.pack(self.mav.mav)
        self.mav.mav.send(gpi)

    def get_drone_pos(self):
        '''
        Fetches the drone position from mavlink and returns lat (deg) and lng (deg) of drone
        '''
        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        drone_pos = self.mav.location()

        return drone_pos, np.array([msg.vx/100, msg.vy/100, msg.vz/100])
    
    def request_message(self, msg_id, rate):
            message = self.mav.mav.command_long_encode(
            self.mav.target_system,  # Target system ID
            self.mav.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
            0,  # Confirmation
            msg_id,  # param1: Message ID to be streamed
            rate, # param2: Interval in microseconds
            0,       # param3 (unused)
            0,       # param4 (unused)
            0,       # param5 (unused)
            0,       # param5 (unused)
            0        # param6 (unused)
            )

            # Send the COMMAND_LONG
            self.mav.mav.send(message)

            while True:
                response = self.mav.recv_match(type='COMMAND_ACK', blocking=True)
                if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("Command accepted")
                    return True
                else:
                    print("Command failed")
                    return False
    
    def test_landing_on_moving_target(self, test_config: TestInfo):
        '''Take off and follow the moving target, then land on the moving target  ''' 

        # Reboot sitl for every test case
        self.reboot_sitl()
        self.initialize_target(test_config)
        self.request_message(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1000)
        self.request_message(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10000)
        self.request_message(mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 10000)

        # drone_stablilty_monitor = DroneStabilityMonitor(self.mav)
        mav_msg_handler = AsyncMavlinkHandler(self.mav)

        # Create folder and file to log target and drone position
        test_data_log_folder_path = os.path.join(self.log_folder_path, test_config['test_name'])
        if not os.path.exists(test_data_log_folder_path):
            os.makedirs(test_data_log_folder_path)

        # set wind parameters
        self.set_wind_parameters(test_config['wind_speed'])
        self.set_drone_parameters(test_config['drone_pos_sensor'])

        # Take of the drone to 40m altitude
        self.change_mode('GUIDED', 10)
        self.takeoff(alt_min=test_config['drone_alt'], mode='GUIDED')

        # Set the necessary parameters and change the mode to TARLAND
        self.set_follow_parameters()
        self.send_target_pos(self.get_sim_time_cached())
        # self.change_mode('TARLAND')
        self.run_cmd_do_set_mode_with_number(29)

        self.target_pos_last_updated_on = self.get_sim_time_cached()

        drone_pos, drone_vel = self.get_drone_pos()
        target_pos = Location(self.target_state[0]*1e-7, self.target_state[1]*1e-7)
        distance = self.get_distance(drone_pos, target_pos)
        alt_diff = drone_pos.alt - self.target_state[2]
        
        test_start = self.get_sim_time_cached()
        now = self.get_sim_time_cached()

        target_traj = []
        target_traj_noisy = []
        self.drone_traj = []
        is_landed = False

        handlers = self.define_mav_msg_handlers(test_config)
        mav_msg_handler.register_handlers(handlers)
        mav_msg_handler.start()

        try:
            while True and now - test_start < self.drone_operation_timeout:
                now  = self.get_sim_time_cached()

                self.move_target(test_config)

                delay = self.calculate_target_pos_sensor_delay(test_config["target_pos_sensor"]['update_rate'], 
                                                            test_config["target_pos_sensor"]['update_latency'])
                if (now - self.target_pos_last_updated_on) > delay:
                    self.send_target_pos(now)
                    self.target_pos_last_updated_on = now
                    target_traj_noisy.append(self.format_trajectory_data(self.target_pos_last_updated_on, self.target_state_noisy))

                target_traj.append(self.format_trajectory_data(self.target_pos_last_updated_on, self.target_state))


                # print(f"Loop execution time {self.get_sim_time_cached()-now} s")
                # print(f"Real time {time.time() - loop_last_update_time}s")

                if not self.armed():
                    is_landed = True
                    break
            mav_msg_handler.stop()

            # drone_loc = Location(self.drone_traj[-1]["lat"]*1e-7, self.drone_traj[-1]["lng"]*1e-7)
            target_loc = Location(target_traj[-1]["lat"]*1e-7, target_traj[-1]["lng"]*1e-7)

            target_timestamp = [obj["time"] for obj in target_traj]
            drone_timestamp = [obj["time"] for obj in self.drone_traj]

            drone_end_i = np.abs(np.array(drone_timestamp) - target_timestamp[-1]).argmin()

            drone_loc = Location(self.drone_traj[drone_end_i]["lat"]*1e-7, self.drone_traj[drone_end_i]["lng"]*1e-7)

            distance = self.get_distance(drone_loc, target_loc)
            alt_diff = self.drone_traj[-1]["alt"] - target_traj[-1]["alt"]
            self.log_trajectory(os.path.join(test_data_log_folder_path, 'target.csv'), target_traj)
            self.log_trajectory(os.path.join(test_data_log_folder_path, 'target_noisy.csv'), target_traj_noisy)
            self.log_trajectory(os.path.join(test_data_log_folder_path, 'drone.csv'), self.drone_traj)
            self.log_test_config(self.log_folder_path, test_config, {
                'is_landed': is_landed,
                'alt_diff': alt_diff,
                'distance': distance
            })
            self.log_trajectory(os.path.join(test_data_log_folder_path, 'drone_att.csv'), self.drone_att)
            self.log_trajectory(os.path.join(test_data_log_folder_path, 'drone_battery_info.csv'), self.drone_battery_info)

            if self.armed:
                self.disarm_vehicle()
                self.wait_disarmed()

            
            self.progress("Success")

        except KeyboardInterrupt:
            print("keyboard interrupt - saving the variables")
            distance = self.get_distance(drone_loc, target_loc)
            alt_diff = self.drone_traj[-1]["alt"] - target_traj[-1]["alt"]
            self.log_trajectory(os.path.join(test_data_log_folder_path, 'target.csv'), target_traj)
            self.log_trajectory(os.path.join(test_data_log_folder_path, 'target_noisy.csv'), target_traj_noisy)
            self.log_trajectory(os.path.join(test_data_log_folder_path, 'drone.csv'), self.drone_traj)
            self.log_test_config(self.log_folder_path, test_config, {
                'is_landed': is_landed,
                'alt_diff': alt_diff,
                'distance': distance
            })
            self.log_trajectory(os.path.join(test_data_log_folder_path, 'drone_att.csv'), self.drone_att)
            self.log_trajectory(os.path.join(test_data_log_folder_path, 'drone_battery_info.csv'), self.drone_battery_info)

            if self.armed:
                self.disarm_vehicle()
                self.wait_disarmed()


        

    def format_trajectory_data(self, time, data):
        return {
            'time': time,
            'lat': data[0],'lng': data[1],'alt': data[2],
            'vx': data[3],'vy': data[4],'vz': data[5],
        }
    
    def log_trajectory_point(self, file_path, data):

        mode = 'a'
        
        with open(file_path, mode, newline='', encoding='utf-8') as csvfile:

            writer = csv.DictWriter(csvfile,
                                    fieldnames=data.keys(),
                                    delimiter=",")
            if not os.path.exists(file_path):
                writer.writeheader()
            writer.writerow(data)

    def log_trajectory(self, file_path, data: List[Dict[str, float]]):

        mode = 'w'
        
        with open(file_path, mode, newline='', encoding='utf-8') as csvfile:

            writer = csv.DictWriter(csvfile,
                                    fieldnames=data[0].keys(),
                                    delimiter=",")
            writer.writeheader()
            writer.writerows(data)


    def rename_autotest_bin_logs(self, log_dir, new_dir, test_name):
        """
        Rename ArduPilot autotest bin log files based on the test name and current timestamp.
        
        Args:
            log_dir (str): Directory containing the bin logs
            new_dir (str): Directory to log the renamed files
            test_name (str): Name of the test being run
        
        Returns:
            list: Paths to the renamed log files, or empty list if no logs found
        """
        # Ensure the log directory exists
        if not os.path.exists(log_dir):
            print(f"Log directory {log_dir} does not exist.")
            return []

        # Get current timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Find all .bin log files
        bin_log_files = [
            os.path.join(log_dir, f) for f in os.listdir(log_dir) 
            if os.path.isfile(os.path.join(log_dir, f)) 
            and f.endswith('.bin')
        ]

        if not bin_log_files:
            print(f"No .bin log files found in {log_dir}")
            return []

        # List to store renamed log paths
        renamed_logs = []
        
        # Rename each .bin log file
        for log_file in bin_log_files:
            try:
                # Extract the original filename without extension
                original_filename = os.path.basename(log_file)
                
                # Create new filename with test name and timestamp
                base_filename = f"{test_name}_{timestamp}_{original_filename}"
                new_log_path = os.path.join(new_dir, base_filename)
                
                # Copy the log file with the new name
                shutil.copy2(log_file, new_log_path)
                
                renamed_logs.append(new_log_path)
                print(f"Bin log renamed to: {base_filename}")
            
            except Exception as e:
                print(f"Error renaming log {log_file}: {e}")
        
        return renamed_logs

    def define_mav_msg_handlers(self, test_config: TestInfo):

        def drone_pos_handler(msg):
            data = [msg.lat, msg.lon, msg.alt/1000, msg.vx/100, msg.vy/100, msg.vz/100]
            self.drone_traj.append(
                 self.format_trajectory_data(msg.time_boot_ms*1e-3, data)
            )
           
        def drone_att_handler(msg):
            self.drone_att.append(
                msg.to_dict()
            )

        def drone_battery_status_handler(msg):
            self.drone_battery_info.append(msg.to_dict())

        return [
            {
                "message_name": MAVMessages.drone_gps_pos,
                "handler": drone_pos_handler
            },
            {
                "message_name": MAVMessages.attitude,
                "handler": drone_att_handler
            },
            {
                "message_name": MAVMessages.battery_status,
                "handler": drone_battery_status_handler
            }
        ]

    def log_test_config(self, folder_path, test_config: TestInfo, traj_info: Dict[str, any]):  
        config = {
                'name': test_config['test_name'],
                'initial_distance_from_target': test_config['target_distance_from_drone'],

                'wind_speed': test_config['wind_speed']['sea_state'],
                'wind_dir': test_config['wind_speed']['dir'],
                'wind_turb': test_config['wind_speed']['turbulence'],

                'velocity': test_config['velocity'],
                'acceleration': test_config['acceleration'],

                'target_pos_error_sigma': test_config['target_pos_sensor']['error']['sigma'],
                'target_update_rate': test_config['target_pos_sensor']['update_rate'],
                'target_pos_update_latency': test_config['target_pos_sensor']['update_latency'],

                'drone_pos_error_sigma': test_config['drone_pos_sensor']['error']['sigma'],
                'drone_update_rate': test_config['drone_pos_sensor']['update_rate'],
                'drone_pos_update_latency': test_config['drone_pos_sensor']['update_latency'],

                'is_landed': traj_info['is_landed'],
                'distance': traj_info['distance'],
                'alt_diff': traj_info['alt_diff']
            }
        
        file_name = os.path.join(folder_path, 'test_config.csv')
        file_exists = os.path.exists(file_name)
        # mode = 'a' if file_exists else 'w'
        mode =  'a'
        with open(file_name, mode, newline='', encoding='utf-8') as csvfile:
            writer = csv.DictWriter(csvfile, 
                                    fieldnames=config.keys(),
                                    delimiter=",")
            if not file_exists:
                # Write the header
                writer.writeheader()
            
            # Write row
            writer.writerow(config)
    
    def calculate_target_pos_sensor_delay(self, target_pos_update_rate, target_pos_update_latency):
        return (1/target_pos_update_rate) + target_pos_update_latency

    def create_function(self,name, docstring, func_body):
        """
        Create a function with a specified name, docstring, and body.
        
        Args:
            name (str): The name of the function.
            docstring (str): The docstring for the function.
            func_body (function): The body of the function.
        
        Returns:
            function: The dynamically created function.
        """
        new_func = types.FunctionType(
            func_body.__code__, 
            func_body.__globals__, 
            name, 
            func_body.__defaults__, 
            func_body.__closure__
        )
        new_func.__doc__ = docstring
        return new_func

    def tests(self):

        test_configs = SimulationEnvironment.generate_test_configs()
        # test_configs = SimulationEnvironment.specific_test_case()

        ret = []
        #314
        #1554
        #2450
        #2742
        for i in range(0, len(test_configs), 1):

            fun = self.create_function(test_configs[i]['test_name'], f"test-{test_configs[i]['test_name']}",self.test_landing_on_moving_target)
            ret.append(Test(fun, kwargs={
                "self": self,
                "test_config": test_configs[i],
            }))
        
        # ret = [
        #         self.test_wind_params,
        #         self.check_test_configs,
        #         self.print_test_config
        #         ]
        return ret
    
    def print_test_config(self):
        '''Prints test config'''

        test_configs = SimulationEnvironment.generate_test_configs()

        print(f"Total test cases {len(test_configs)}")

        # test_configs = SimulationEnvironment.specific_test_case()

        print(test_configs[0])

    def check_test_configs(self):
        '''Check the no of test configs to be tested'''
        test_configs = SimulationEnvironment.generate_test_configs()

        target_pos_error = []
        gps_freq = []
        target_pos_update_latency = []
        drone_pos_error = []
        drone_pos_update_rate = []
        drone_pos_update_latency = []
        wind_spd = []
        wind_dir = []
        wind_turb = []
        dist_from_target = []
        drone_alt = []

        for config in test_configs:

            target_pos_error.append(config['drone_pos_sensor']["error"]["sigma"])
            gps_freq.append(config['drone_pos_sensor']['update_rate'])
            target_pos_update_latency.append(config['drone_pos_sensor']['update_latency'])

            drone_pos_error.append(config['drone_pos_sensor']["error"]["sigma"])
            drone_pos_update_rate.append(config['drone_pos_sensor']['update_rate'])
            drone_pos_update_latency.append(config['drone_pos_sensor']['update_latency'])

            wind_spd.append(config['wind_speed']['sea_state'])
            wind_dir.append(config['wind_speed']['dir'])
            wind_turb.append(config['wind_speed']['turbulence'])
            dist_from_target.append(config['target_distance_from_drone'])

            drone_alt.append(config['drone_alt'])

        target_pos_error = list(set(target_pos_error))
        expected_target_pos_error = 5

        gps_freq = list(set(gps_freq))
        expected_gps_freq = 2

        target_pos_update_latency = list(set(target_pos_update_latency))
        expected_gps_lat = 2

        drone_pos_error = list(set(drone_pos_error))
        expected_drone_pos_error = 5

        drone_pos_update_rate = list(set(drone_pos_update_rate))
        expected_drone_pos_update_rate = 2

        drone_pos_update_latency = list(set(drone_pos_update_latency))
        expected_drone_pos_update_latency = 2

        wind_spd = list(set(wind_spd))
        expected_wind_spd = 6

        wind_dir = list(set(wind_dir))
        expected_wind_dir = 4

        wind_turb = list(set(wind_turb))
        expected_wind_turb = 1

        dist_from_target = list(set(dist_from_target))
        expected_dist_from_target = 2

        drone_alt = list(set(drone_alt))
        expected_drone_alt = 2
 
        if(len(target_pos_error) != expected_target_pos_error or
           len(gps_freq) != expected_gps_freq or
           len(target_pos_update_latency) != expected_gps_lat or
           len(drone_pos_error) != expected_drone_pos_error or
           len(drone_pos_update_rate) != expected_drone_pos_update_rate or
           len(drone_pos_update_latency) != expected_drone_pos_update_latency or
           len(wind_spd) != expected_wind_spd or
           len(wind_dir) != expected_wind_dir or
           len(wind_turb) != expected_wind_turb or
           len(dist_from_target) != expected_dist_from_target or
           len(drone_alt) != expected_drone_alt) :
            print(f"target_pos_error expected {expected_target_pos_error} got {len(target_pos_error)}")
            print(f"gps_freq expected {expected_gps_freq} got {len(gps_freq)}")
            print(f"target_pos_update_latency expected {expected_gps_lat} got {len(target_pos_update_latency)}")
            print(f"wind_spd expected {expected_wind_spd} got {len(wind_spd)}")
            print(f"wind_dir expected {expected_wind_dir} got {len(wind_dir)}")
            print(f"wind_turb expected {expected_wind_turb} got {len(wind_turb)}")
            print(f"distance_from_target expected {expected_dist_from_target} got {len(dist_from_target)}")
            print(f"drone_alt expected {expected_drone_alt} got {len(drone_alt)}")
            raise NotAchievedException("There is a mismatch in test config")

    def test_wind_params(self):
        '''Test wind param setting'''
        sigma = 0.5
        gps_freq = 5
        target_pos_update_latency = 0.2
        dist_from_target = 25
        sea_state = 2
        wind_dir = 180
        wind_turb = 0.2
        alt = 20

        pos_error = {
            'type': "Gaussian",
            'sigma': sigma,
            'mu': 0
        }
        pos_sensor = {
            'error': pos_error,
            'update_latency': target_pos_update_latency,
            'update_rate': gps_freq
        }

        test_config: TestInfo = {
            'test_name': "",
            'acceleration': np.array([0,0,0]),
            'velocity': np.array([5, 0, 0]),
            'target_pos_sensor': pos_sensor,
            'drone_pos_sensor': pos_sensor,
            'target_distance_from_drone': dist_from_target,
            'wind_speed':{
                'sea_state': sea_state,
                'dir': wind_dir,
                'turbulence': wind_turb
            },
            'drone_alt': alt
        }

        self.test_landing_on_moving_target(test_config)
        
        if(round(self.get_parameter('SIM_WIND_SPD'), 2) != 3.06):
            raise NotAchievedException(f"Expected wind speed {3.06} actual wind speed {round(self.get_parameter('SIM_WIND_SPD'), 2)}")
        if(self.get_parameter('SIM_WIND_DIR') != wind_dir):
            raise NotAchievedException(self.exception_str("SIM_WIND_DIR",wind_dir, self.get_parameter('SIM_WIND_DIR')))
        if(round(self.get_parameter('SIM_WIND_TURB'), 2) != wind_turb):
            raise NotAchievedException(self.exception_str('SIM_WIND_TURB', wind_turb, self.get_parameter('SIM_WIND_TURB')))

    def exception_str(self, param, expected, actual):
        return f"{param} expected {expected} actual {actual}"
    
class Location:

    def __init__(self, lat, lon):
        self.lat = lat
        self.lng = lon