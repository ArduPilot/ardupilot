import numpy as np

from numpy.typing import NDArray
from typing import List, Dict, TypedDict


class SeaState(TypedDict):
    state: int
    wind_speed: NDArray[np.float64] # in kmph
    wave_height: NDArray[np.float64] # in kmph

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

gps_error: Dict[str, any] = {
                "type": "Gaussian",
                "mu": 0,
                "sigma": np.linspace(0.05, 0.5, 10)
        }

winds: Dict[str, any] = {
            "sea_states": np.linspace(0, 5, 6),
            "wind_direction": np.linspace(0, 330, 12),
            "turbulence": np.linspace(0, 1, 11)
            # Low turbulence (0.0-0.3): Stable, predictable wind
            # Medium turbulence (0.3-0.7): Occasional gusts
            # High turbulence (0.7-1.0): Frequent, strong gusts
        }
