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

class TestInfo(TypedDict):
    test_name: str
    velocity: NDArray[np.float64]
    acceleration: NDArray[np.float64]
    gps_error:  Dict[str, any]
    gps_latency: float
    gps_frequency: float
    wind_speed: WindSpeed
    target_distance_from_drone: float

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
    
    initial_distance_from_target = [25, 50, 75, 100]

    winds: Dict[str, any] = {
                    "sea_states": np.array([0, 1, 2, 3, 4, 5]),
                    "wind_direction": np.array([0,90,180,270]),
                    "turbulence": np.array([0.2])
                    # Low turbulence (0.0-0.3): Stable, predictable wind
                    # Medium turbulence (0.3-0.7): Occasional gusts
                    # High turbulence (0.7-1.0): Frequent, strong gusts
                }
    
    gps_error: Dict[str, any] = {
                        "type": "Gaussian",
                        "mu": 0,
                        # "sigma": np.array([0.5, 1, 1.5, 2])
                        "sigma": np.array([0.5, 1])
                }
    
    gps_time: Dict[str, float] = {
            'frequency': [0.5, 1],
            'latency': [0.2, 0.5, 1]
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
    def get_test_name(dist, sea_state, dir, turb, sigma, gps_freq, gps_latency):
        return f"dist-{dist}__sea_state-{sea_state}__dir-{dir}__turb-{turb}__sigma-{sigma}__gps_freq-{gps_freq}__gps_lat-{gps_latency}"
    
    @staticmethod
    def generate_test_configs():

        initial_distance_from_target = SimulationEnvironment.initial_distance_from_target
        gps_error: Dict[str, any] = SimulationEnvironment.gps_error
        gps_time: Dict[str, float] = SimulationEnvironment.gps_time
        winds: Dict[str, any] = SimulationEnvironment.winds

        test_configs: List[TestInfo] = []

        for dist in initial_distance_from_target:
            for sea_state in winds['sea_states']:
                for dir in range(0, len(winds['wind_direction'])):
                    for turb in range(0, len(winds['turbulence'])):
                        for sigma in gps_error['sigma']:
                            for gps_frequency in gps_time['frequency']:
                                for gps_latency in gps_time['latency']:
                        
                                    test_configs.append({
                                        "test_name": SimulationEnvironment.get_test_name(dist, sea_state, dir, turb, sigma, gps_frequency, gps_latency),
                                        "wind_speed": {
                                            'sea_state': sea_state,
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
    
    @staticmethod
    def specific_test_case():

        sigma = 0
        gps_latency = 0.2
        gps_frequency = 1
        sea_state = 0
        wind_dir = 0
        wind_turb = 0
        dist = 25

        initial_distance_from_target = SimulationEnvironment.initial_distance_from_target
        gps_error: Dict[str, any] = SimulationEnvironment.gps_error
        gps_time: Dict[str, float] = SimulationEnvironment.gps_time
        winds: Dict[str, any] = SimulationEnvironment.winds

        test_config: List[TestInfo] = [{
            'test_name': SimulationEnvironment.get_test_name(dist, sea_state, wind_dir, wind_turb, sigma, gps_frequency, gps_latency),
            'acceleration': np.array([0, 0]),
            'velocity': np.array([5, 0, 0]),
            'gps_error': {
                'type': "Gaussian",
                'mu': 0,
                'sigma': sigma
            },
            'gps_latency': gps_latency,
            'gps_frequency': gps_frequency,
            'target_distance_from_drone': dist,
            'wind_speed': {
                'sea_state': sea_state,
                'dir': winds['wind_direction'][wind_dir],
                "turbulence": winds['turbulence'][wind_turb]
            },
        }]

        return test_config