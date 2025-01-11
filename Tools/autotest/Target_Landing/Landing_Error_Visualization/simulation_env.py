import numpy as np

from numpy.typing import NDArray
from typing import List, Dict, TypedDict

class SeaState(TypedDict):
    state: int
    wind_speed: NDArray[np.float64] # in kmph
    wave_height: NDArray[np.float64] # in kmph

class WindSpeed(TypedDict):
    sea_state: int
    dir: float
    turbulence: float

class PositionSensor(TypedDict):
    error: Dict[str, any]
    update_rate: float
    update_latency: float

class TestInfo(TypedDict):
    test_name: str
    velocity: NDArray[np.float64]
    acceleration: NDArray[np.float64]

    target_pos_sensor: PositionSensor
    drone_pos_sensor: PositionSensor

    wind_speed: WindSpeed
    target_distance_from_drone: float

    drone_alt: float

class SimulationEnvironment:

    sea_states: List[SeaState] = [
        # Wind speeds are in Km/hr
                {
                    "state": 0,
                    # 0 - 0.27 m/s
                    # 0 - 0.54 Knot
                    "wind_speed": np.array([0, 1]),
                    "wave_height": np.array([])
                },
                {
                    "state": 1,
                    # 0.27 - 1.38 m/s
                    # 0.54 - 2.7 Knots
                    "wind_speed": np.array([1, 5]),
                    "wave_height": np.array([])
                },
                {
                    "state": 2,
                    # 1.67 - 3.06 m/s
                    # 3.23 - 5.94 Knots
                    "wind_speed": np.array([6, 11]),
                    "wave_height": np.array([])
                },
                {
                    "state": 3,
                    # 3.33 - 5.27 m/s
                    # 6.47 - 10.25 Knots
                    "wind_speed": np.array([12, 19]),
                    "wave_height": np.array([])
                },
                {
                    "state": 4,
                    # 5.55 - 7.78 m/s
                    # 10.8 - 15.12 Knots
                    "wind_speed": np.array([20, 28]),
                    "wave_height": np.array([])
                },
                {
                    "state": 5,
                    # 8.05 - 10.55 m/s
                    # 15.65  - 20.52 Knots
                    "wind_speed": np.array([29, 38]),
                    "wave_height": np.array([])
                }
            ]
    
    ship_speed = 7.72 # 15 Knots
    drone_speed = 15
    
    drone_alts = [40]

    initial_distance_from_target = [100]

    winds: Dict[str, any] = {
                    # "sea_states": np.array([0, 1, 2, 3, 4, 5]),
                    # "wind_direction": np.array([0,90,180,270]),
                    # "turbulence": np.array([0.2])
                    "sea_states": np.array([0]),
                    "wind_direction": np.array([0]),
                    "turbulence": np.array([0])
                    # Low turbulence (0.0-0.3): Stable, predictable wind
                    # Medium turbulence (0.3-0.7): Occasional gusts
                    # High turbulence (0.7-1.0): Frequent, strong gusts
                }
    

    pos_error: Dict[str, any] = {
                        "type": "Gaussian",
                        "mu": 0,
                        # "sigma": np.array([0.5, 1, 1.5, 2])
                        "sigma": np.array([0, 1, 2, 3, 4])
                }
    
    pos_sensor: PositionSensor = {
        "error": pos_error,
        "update_rate": [1, 5],
        "update_latency": [0, 0.2]
        # "update_latency": [0]
    }

    drone_pos_sensor: PositionSensor = {
        "error": pos_error,
        "update_rate": [5, 10],     # Ardupilot GPS frequency is from 5Hz - 20Hz
        "update_latency": [0, 0.2] # Ardupilot latency is from 0 - 250ms
        # "update_latency": [0]

    }
    

    @staticmethod
    def get_wind_spd_from_sea_state(sea_state: int):
        '''
        Returns the wind speed from sea state in m/s
        (Args): Sea state number
        (Returns): Max possible velocity in that sea state in m/s
        '''
        
        wind_info = list(filter(lambda obj: obj["state"] == sea_state, SimulationEnvironment.sea_states))[0]
        wind_spd_kmph = wind_info["wind_speed"][1]
        wind_spd_mps = round(wind_spd_kmph*5/18, 2)
        return wind_spd_mps
    
    @staticmethod
    def get_sea_state_wind_speed(sea_states: List[int]):
        '''
        Returns the maximum possible wind speed in a given sea state in m/s
        (Args): sea states
        (Returns): max speed in m/s in the correspoding sea state
        '''

        wind_speed: List[float] = []

        for sea_state in sea_states:
            wind_speed.append(SimulationEnvironment.get_wind_spd_from_sea_state(sea_state))

        return wind_speed
    

    @staticmethod
    def get_test_name(drone_alt, dist, sea_state, dir, turb, sigma, gps_freq, target_pos_update_latency):
        return f"drone_alt-{drone_alt}__dist-{dist}__sea_state-{sea_state}__dir-{dir}__turb-{turb}__sigma-{sigma}__gps_freq-{gps_freq}__gps_lat-{target_pos_update_latency}"
    
    @staticmethod
    def generate_test_configs():

        initial_distance_from_target = SimulationEnvironment.initial_distance_from_target
        pos_error: Dict[str, any] = SimulationEnvironment.pos_sensor["error"]
        pos_sensor = SimulationEnvironment.pos_sensor
        winds: Dict[str, any] = SimulationEnvironment.winds
        drone_alts = SimulationEnvironment.drone_alts

        ship_traj = "circle" # circle or constant velocity

        test_configs: List[TestInfo] = []

        for drone_alt in drone_alts:
            for dist in initial_distance_from_target:
                for sea_state in winds['sea_states']:
                    for wind_dir in range(0, len(winds['wind_direction'])):
                        for turb in range(0, len(winds['turbulence'])):
                            for sigma_i, sigma in enumerate(pos_error['sigma']):
                                for update_rate_i, target_pos_update_rate in enumerate(pos_sensor['update_rate']):
                                    for latency_i, target_pos_update_latency in enumerate(pos_sensor['update_latency']):

                                        target_pos_sensor: PositionSensor = {
                                            "error": {
                                                "type": pos_error['type'],
                                                "mu": pos_error["mu"],
                                                "sigma": sigma
                                            },
                                            "update_latency": target_pos_update_latency,
                                            "update_rate": target_pos_update_rate
                                        }

                                        drone_pos_sensor: PositionSensor = {
                                             "error": {
                                                "type": pos_error['type'],
                                                "mu": pos_error["mu"],
                                                "sigma": sigma
                                            },
                                            "update_latency": SimulationEnvironment.drone_pos_sensor["update_latency"][latency_i],
                                            "update_rate":  SimulationEnvironment.drone_pos_sensor["update_rate"][update_rate_i]
                                        }

                                        ship_vel, ship_acc = SimulationEnvironment.get_ship_vel_and_acc(ship_traj)
                            
                                        test_configs.append({
                                            "test_name": SimulationEnvironment.get_test_name(drone_alt, dist, sea_state, wind_dir, turb, sigma, target_pos_update_rate, target_pos_update_latency),
                                            "wind_speed": {
                                                'sea_state': sea_state,
                                                'dir': winds['wind_direction'][wind_dir],
                                                "turbulence": winds['turbulence'][turb]
                                            },
                                            "velocity": ship_vel,
                                            "acceleration": np.array([0, 0, 0]),
                                            "target_pos_sensor": target_pos_sensor,
                                            "drone_pos_sensor": drone_pos_sensor,
                                            "target_distance_from_drone": dist,
                                            "drone_alt": drone_alt
                                    })

        return test_configs
    
    @staticmethod
    def get_ship_vel_and_acc(motion_type: str):

        angle = np.random.uniform(0, 2*np.pi)
        ship_dir = np.array([np.cos(angle), np.sin(angle), 0])
        ship_vel = SimulationEnvironment.ship_speed*ship_dir

        match motion_type:

            case "constant_velocity":
                ship_acc = np.array([0, 0, 0])

                return ship_vel, ship_acc
            
            case "circle":
                R = 100
                ship_acc = np.array([SimulationEnvironment.ship_speed**2/R, 0, 0])  # an, at, az

                return ship_vel, ship_acc


    @staticmethod
    def specific_test_case():

        sigma = 0
        target_pos_update_latency = 0.2
        drone_pos_update_latency = 0.05
        target_pos_update_rate = 1
        drone_pos_update_rate = 5
        sea_state = 0
        wind_dir = 0
        wind_turb = 0
        dist = 25
        drone_alt = 40

        winds: Dict[str, any] = SimulationEnvironment.winds

        target_pos_sensor: PositionSensor = {
                                            "error": {
                                                "type": "Gaussian",
                                                "mu": 0,
                                                "sigma": sigma
                                            },
                                            "update_latency": target_pos_update_latency,
                                            "update_rate": target_pos_update_rate
                                        }

        target_pos_sensor: PositionSensor = {
                                    "error": {
                                        "type": "Gaussian",
                                        "mu": 0,
                                        "sigma": sigma
                                    },
                                    "update_latency": drone_pos_update_latency,
                                    "update_rate": drone_pos_update_rate
                                }

        test_config: List[TestInfo] = [{
            'test_name': SimulationEnvironment.get_test_name(drone_alt, dist, sea_state, wind_dir, wind_turb, sigma, target_pos_update_rate, target_pos_update_latency),
             'wind_speed': {
                'sea_state': sea_state,
                'dir': winds['wind_direction'][wind_dir],
                "turbulence": winds['turbulence'][wind_turb]
            },
            'velocity': np.array([SimulationEnvironment.ship_speed, 0, 0]),
            'acceleration': np.array([0, 0]),
            'target_pos_error': {
                'type': "Gaussian",
                'mu': 0,
                'sigma': sigma
            },
             "target_pos_sensor": target_pos_sensor,
            "drone_pos_sensor": target_pos_sensor,
            "target_distance_from_drone": dist,
            "drone_alt": drone_alt

        }]

        return test_config