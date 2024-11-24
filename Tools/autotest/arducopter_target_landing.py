import os
import numpy as np
from typing import TypedDict, Dict, List
from numpy.typing import NDArray
import csv
import types

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import Test
from arducopter import AutoTestCopter

from Target_Landing.Platform import Platform

testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = mavutil.location(12.992006,80.236649,0,0)

class WindSpeed(TypedDict):
    speed: float
    dir: float
    turbulence: float

class TestInfo(TypedDict):
    test_name: str
    velocity: NDArray[np.float64]
    acceleration: NDArray[np.float64]
    gps_error:  Dict[str, any]
    wind_speed: WindSpeed

class SeaState(TypedDict):
    state: int
    wind_speed: NDArray[np.float64] # in kmph
    wave_height: NDArray[np.float64] # in kmph

class AutoTestCopterTargetLanding(AutoTestCopter):

    target_origin = np.array([129920050, 802368336, 0])
    target_v0 = np.array([5, 0, 0])
    target_state = np.zeros((6,))
    target_state_noisy = np.zeros((6,))
    drone_state = np.zeros((6,))
    target: Platform
    target_update_time = 1

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

    target_log_file_path = os.path.expanduser("~/UAV_Landing/logs/test")

    def get_acceleration(self, a, acc_last_update_time, tstart):
        
        now = self.get_sim_time_cached()
        acc_duration = 2
        i = int(((now - tstart)/acc_duration)/a.shape[0])

        if now - acc_last_update_time > acc_duration:
            acc_last_update_time = now
        
        return a[i, :], acc_last_update_time



    def initialize_target(self, test_config: TestInfo):
        '''
        Intialize the position and velocity of the moving target. Initial position
        of the target is defined as the origin
        
        Args: Initial velocity
        '''
        self.target_v0 = test_config['velocity']

        self.target = Platform()
        self.target.spawn(self.target_origin, self.target_v0)

        self.target_state[0:3] = self.target_origin
        self.target_state[3:6] = self.target_v0
        self.target_state_noisy = self.target_state  + self.target.position_error(test_config['gps_error'])


    def move_target(self, test_config: TestInfo):

        self.target.dt = self.target_update_time
        self.target_state = self.target.move(self.target_state[0:3], self.target_state[3:6])

        self.target_state_noisy =  self.target_state + self.target.position_error(test_config['gps_error'])

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
        self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        drone_pos = self.mav.location()

        return drone_pos

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

        last_sent = 0
        drone_pos = self.get_drone_pos()
        target_pos = Location(self.target_state[0]*1e-7, self.target_state[1]*1e-7)
        distance = self.get_distance(drone_pos, target_pos)
        tstart = self.get_sim_time_cached()
        acc_last_update_time = tstart

        while True and self.get_sim_time_cached() - tstart < 5*60:
            now  = self.get_sim_time_cached()

            if now - last_sent > self.target_update_time:

                # acc, acc_last_update_time = self.get_acceleration(test_config['acceleration'], acc_last_update_time, tstart)
                # self.target.at = acc[0]
                # self.target.an = acc[1]

                self.move_target(test_config)
                self.send_target_pos(now)
                last_sent = self.get_sim_time_cached()

            drone_pos = self.get_drone_pos()
            target_pos = Location(self.target_state[0]*1e-7, self.target_state[1]*1e-7)

            # Log test data
            self.log_test_data(log_file_path, drone_pos)
            

            distance = self.get_distance(drone_pos, target_pos)
            alt_diff = drone_pos.alt - self.target_state[2]

            print(f"horizontal distance {distance}, alt difference {alt_diff}")

            # self.wait_disarmed()
            if not self.armed():
                break
        
        if self.armed:
            self.disarm_vehicle()
            self.wait_disarmed()

        # log test config
        self.log_test_config(self.target_log_file_path, test_config)
        self.progress("Success")

    def log_test_data(self, log_file_path, drone_pos):

        with open(os.path.join(log_file_path,'target_pos_noisy.csv'), 'ab') as f:
                    np.savetxt(f, [self.target_state_noisy], delimiter=',', fmt='%f')

        with open(os.path.join(log_file_path,'target_pos.csv'), 'ab') as f:
                np.savetxt(f, [self.target_state], delimiter=',', fmt='%f')

        with open(os.path.join(log_file_path,'drone_pos.csv'), 'ab') as f:
                np.savetxt(f, [np.array([drone_pos.lat, drone_pos.lng, drone_pos.alt])], delimiter=',', fmt='%f')

    def log_test_config(self, folder_path, test_config):  
        config = {
                'name': test_config['test_name'],
                'wind_speed': test_config['wind_speed']['speed'],
                'wind_dir': test_config['wind_speed']['dir'],
                'wind_turb': test_config['wind_speed']['turbulence'],
                'velocity': test_config['velocity'],
                'acceleration': test_config['acceleration'],
                'gps_error_sigma': test_config['gps_error']['sigma']
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
        

    def generate_test_configs(self):
        
        gps_error: Dict[str, any] = {
                "type": "Gaussian",
                "mu": 0,
                "sigma": np.linspace(0.05, 0.5, 10)
        }

        winds: Dict[str, any] = {
            "sea_states": np.array([0, 1, 2, 3]),
            "wind_direction": np.linspace(0, 330, 8),
            "turbulence": np.array([0.1])
            # Low turbulence (0.0-0.3): Stable, predictable wind
            # Medium turbulence (0.3-0.7): Occasional gusts
            # High turbulence (0.7-1.0): Frequent, strong gusts
        }

        test_configs: List[TestInfo] = []

        for sigma in gps_error['sigma']:
            for sea_state in range(0, len(winds['sea_states'])):
                for dir in range(0, len(winds['wind_direction'])):
                    for turb in range(0, len(winds['turbulence'])):
                        
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
                            }
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
        for i in range(31, 64, 1):

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