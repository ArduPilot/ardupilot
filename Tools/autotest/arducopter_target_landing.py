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

testdir = os.path.dirname(os.path.realpath(__file__))
drone_lat_initial = 12.992006
drone_lng_initial = 80.236649
SITL_START_LOCATION = mavutil.location(drone_lat_initial,drone_lng_initial,0,0)

class WindSpeed(TypedDict):
    speed: float
    dir: float
    turbulence: float

class TestInfo(TypedDict):
    test_name: str
    velocity: NDArray[np.float64]
    acceleration: NDArray[np.float64]
    gps_error:  Dict[str, any]
    gps_latency: float
    gps_frequency: float
    wind_speed: WindSpeed
    target_distance_from_drone: float

class SeaState(TypedDict):
    state: int
    wind_speed: NDArray[np.float64] # in kmph
    wave_height: NDArray[np.float64] # in kmph

class AutoTestCopterTargetLanding(AutoTestCopter):

    target_origin = np.zeros((3,))
    target_v0 = np.array([5, 0, 0])
    target_state = np.zeros((6,))
    target_state_noisy = np.zeros((6,))
    drone_state = np.zeros((6,))
    target: Platform
    target_pos_last_updated_on = 0

    drone_operation_timeout = 5*60          # Timeout in secs to stop the test when drone cannot land

    R = 6378137.0  # Radius of earth in meters

    target_log_file_path = os.path.expanduser("~/UAV_Landing/logs/test")

    sea_states: List[SeaState] = [
        {
            "state": 0,
            "wind_speed": np.array([0, 1]),
            "wave_height": np.array([])
        },
        {
            "state": 1,
            "wind_speed": np.array([1, 5]),
            "wave_height": np.array([])
        },
        {
            "state": 2,
            "wind_speed": np.array([6, 11]),
            "wave_height": np.array([])
        },
        {
            "state": 3,
            "wind_speed": np.array([12, 19]),
            "wave_height": np.array([])
        },
        {
            "state": 4,
            "wind_speed": np.array([20, 28]),
            "wave_height": np.array([])
        },
        {
            "state": 5,
            "wind_speed": np.array([29, 38]),
            "wave_height": np.array([])
        }
    ]


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
        self.target_state_noisy = self.target_state  + self.target.position_error(test_config['gps_error'], self.target_state)


    def move_target(self, test_config: TestInfo):

        self.target.dt = 1/test_config['gps_frequency']
        self.target_state = self.target.move(self.target_state[0:3], self.target_state[3:6])

        self.target_state_noisy =  self.target_state + self.target.position_error(test_config['gps_error'], self.target_state)

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
            "SIM_WIND_SPD": wind_speed['speed']*5/18, # convert wind speed to m/s
            "SIM_WIND_DIR": wind_speed['dir'],
            "SIM_WIND_TURB": wind_speed['turbulence']
        })

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

    def test_landing_on_moving_target(self, test_config: TestInfo):
        '''Take off and follow the moving target, then land on the moving target  ''' 

        # Reboot sitl for every test case
        self.reboot_sitl()
        self.initialize_target(test_config)

        # Create folder and file to log target and drone position
        log_file_path = os.path.join(self.target_log_file_path, test_config['test_name'])
        if not os.path.exists(log_file_path):
            os.makedirs(log_file_path)

        # set wind parameters
        self.set_wind_parameters(test_config['wind_speed'])

        # Take of the drone to 40m altitude
        self.change_mode('GUIDED', 10)
        self.takeoff(alt_min=40, mode='GUIDED')

        # Set the necessary parameters and change the mode to TARLAND
        self.set_follow_parameters()
        self.send_target_pos(self.get_sim_time_cached())
        self.change_mode('TARLAND')
        self.target_pos_last_updated_on = self.get_sim_time_cached()

        drone_pos, drone_vel = self.get_drone_pos()
        target_pos = Location(self.target_state[0]*1e-7, self.target_state[1]*1e-7)
        distance = self.get_distance(drone_pos, target_pos)
        
        test_start = self.get_sim_time_cached()
        acc_last_update_time = test_start

        while True and self.get_sim_time_cached() - test_start < self.drone_operation_timeout:
            now  = self.get_sim_time_cached()

            # acc, acc_last_update_time = self.get_acceleration(test_config['acceleration'], acc_last_update_time, tstart)
            # self.target.at = acc[0]
            # self.target.an = acc[1]
            self.move_target(test_config)

            if now - self.target_pos_last_updated_on > self.calculate_gps_time_delay(test_config['gps_frequency'], test_config['gps_latency']):
                self.send_target_pos(now)
                self.target_pos_last_updated_on = now
                self.log_trajectory(os.path.join(log_file_path,'target_noisy.csv'), 
                            {
                                'time': self.target_pos_last_updated_on,
                                'lat': self.target_state_noisy[0],
                                'lng': self.target_state_noisy[1],
                                'alt': self.target_state_noisy[2],
                                'vx': self.target_state_noisy[3],
                                'vy': self.target_state_noisy[4],
                                'vz': self.target_state_noisy[5],
                            })

            # Log drone data
            self.log_trajectory(os.path.join(log_file_path,'target.csv'), 
                            {
                                'time': self.target_pos_last_updated_on,
                                'lat': self.target_state[0],
                                'lng': self.target_state[1],
                                'alt': self.target_state[2],
                                'vx': self.target_state[3],
                                'vy': self.target_state[4],
                                'vz': self.target_state[5],
                            })
            

            drone_pos, drone_vel = self.get_drone_pos()
            target_pos = Location(self.target_state[0]*1e-7, self.target_state[1]*1e-7)

            # Log drone data
            self.log_trajectory(os.path.join(log_file_path,'drone.csv'), 
                                {
                                    'time': now,
                                    'lat': drone_pos.lat,
                                    'lng': drone_pos.lng,
                                    'alt': drone_pos.alt,
                                    'vx': drone_vel[0],
                                    'vy': drone_vel[1],
                                    'vz': drone_vel[2],
                                })
            # self.log_test_data(log_file_path, drone_pos)
            
            distance = self.get_distance(drone_pos, target_pos)
            alt_diff = drone_pos.alt - self.target_state[2]

            print(f"horizontal distance {distance}, alt difference {alt_diff}")

            if not self.armed():
                break
        
        if self.armed:
            self.disarm_vehicle()
            self.wait_disarmed()

        # log test config
        self.log_test_config(self.target_log_file_path, test_config)
        self.progress("Success")
        self.rename_autotest_bin_logs("~/UAV_Landing/ardupilot/Tools/autotest/logs", test_config['test_name'])

    def log_trajectory(self, file_path, data):

        mode = 'a'
        
        with open(file_path, mode, newline='', encoding='utf-8') as csvfile:

            writer = csv.DictWriter(csvfile,
                                    fieldnames=data.keys(),
                                    delimiter=",")
            if not os.path.exists(file_path):
                writer.writeheader()
            writer.writerow(data)
        # with open(file_path, 'ab') as f:
        #     np.savetxt(f, [data], delimiter=",")

    def rename_autotest_bin_logs(self, log_dir, test_name):
        """
        Rename ArduPilot autotest bin log files based on the test name and current timestamp.
        
        Args:
            log_dir (str): Directory containing the bin logs
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

        print(bin_log_files)
        
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
                new_log_path = os.path.join(log_dir, base_filename)
                
                # Copy the log file with the new name
                shutil.copy2(log_file, new_log_path)
                
                renamed_logs.append(new_log_path)
                print(f"Bin log renamed to: {base_filename}")
            
            except Exception as e:
                print(f"Error renaming log {log_file}: {e}")
        
        return renamed_logs



    def log_test_data(self, log_file_path, drone_pos):

        with open(os.path.join(log_file_path,'target_pos_noisy.csv'), 'ab') as f:
                    np.savetxt(f, [self.target_pos_last_updated_on, self.target_state_noisy], delimiter=',', fmt='%f')

        with open(os.path.join(log_file_path,'target_pos.csv'), 'ab') as f:
                np.savetxt(f, [self.target_pos_last_updated_on, self.target_state], delimiter=',', fmt='%f')

        with open(os.path.join(log_file_path,'drone_pos.csv'), 'ab') as f:
                np.savetxt(f, [np.array([drone_pos.lat, drone_pos.lng, drone_pos.alt])], delimiter=',', fmt='%f')

    def log_test_config(self, folder_path, test_config: TestInfo):  
        config = {
                'name': test_config['test_name'],
                'wind_speed': test_config['wind_speed']['speed'],
                'wind_dir': test_config['wind_speed']['dir'],
                'wind_turb': test_config['wind_speed']['turbulence'],
                'velocity': test_config['velocity'],
                'acceleration': test_config['acceleration'],
                'gps_error_sigma': test_config['gps_error']['sigma'],
                'gps_update_frequency': test_config['gps_frequency'],
                'gps_latency': test_config['gps_latency']
            }
        
        file_name = os.path.join(folder_path, 'test_config.csv');
        file_exists = os.path.exists(file_name)
        mode = 'a' if file_exists else 'w'
        with open(file_name, mode, newline='', encoding='utf-8') as csvfile:
            writer = csv.DictWriter(csvfile, 
                                    fieldnames=config.keys(),
                                    delimiter=",")
            if not file_exists:
                # Write the header
                writer.writeheader()
            
            # Write row
            writer.writerow(config)
    
    def calculate_gps_time_delay(self, gps_frequency, gps_latency):
        return (1/gps_frequency) + gps_latency

    def generate_test_configs(self):

        target_distance_from_drone = [25, 50, 75, 100]
        
        gps_error: Dict[str, any] = {
                "type": "Gaussian",
                "mu": 0,
                "sigma": np.array([0.2, 0.4, 0.5, 1, 1.5, 2])
        }

        gps_time: Dict[str, float] = {
            'frequency': [0.5, 1, 2],
            'latency': [0.2, 0.5, 1]
        }

        winds: Dict[str, any] = {
            # "sea_states": np.array([1, 2, 3]),
            "sea_states": np.array([3]),
            "wind_direction": np.array([0, 90, 180, 270]),
            "turbulence": np.array([0.2])
            # Low turbulence (0.0-0.3): Stable, predictable wind
            # Medium turbulence (0.3-0.7): Occasional gusts
            # High turbulence (0.7-1.0): Frequent, strong gusts
        }

        test_configs: List[TestInfo] = []

        for dist in target_distance_from_drone:
            for sea_state in range(0, len(winds['sea_states'])):
                for dir in range(0, len(winds['wind_direction'])):
                    for turb in range(0, len(winds['turbulence'])):
                        for sigma in gps_error['sigma']:
                            for gps_frequency in gps_time['frequency']:
                                for gps_latency in gps_time['latency']:
                        
                                    sea_state_info = self.sea_states[sea_state]
                                    test_configs.append({
                                        "test_name": f"sigma-{round(sigma, 3)}__sea_state-{sea_state}__dir-{dir}__turb{turb}",
                                        "wind_speed": {
                                            'speed':np.random.uniform(low=sea_state_info['wind_speed'][0], high=sea_state_info['wind_speed'][1]),
                                            'dir': winds['wind_direction'][dir],
                                            "turbulence": winds['turbulence'][turb]
                                        },
                                        "velocity": np.array([5, 0, 0]),
                                        "acceleration": np.array([0, 0, 0]),
                                        "gps_error": {
                                            "type": gps_error['type'],
                                            "mu": gps_error["mu"],
                                            "sigma": sigma
                                        },
                                        "gps_frequency": gps_frequency,
                                        "gps_latency": gps_latency,
                                        "target_distance_from_drone": dist
                                    })

        return test_configs
    
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

        test_configs = self.generate_test_configs()

        ret = []
        for i in range(1, 2, 1):

            fun = self.create_function(test_configs[i]['test_name'], f"test-{test_configs[i]['test_name']}",self.test_landing_on_moving_target)
            ret.append(Test(fun, kwargs={
                "self": self,
                "test_config": test_configs[i]
            }))
        
        return ret

class Location:

    def __init__(self, lat, lon):
        self.lat = lat
        self.lng = lon