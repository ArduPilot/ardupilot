import numpy as np
import os
import matplotlib.pyplot as plt
import csv
import seaborn as sns
import pandas as pd

from typing import Dict, List, TypedDict
from numpy.typing import NDArray

from simulation_env import TestInfo, sea_states
from Data_Visualizer import DataVisualizer

class ErrorAnalysis(TypedDict):
     gps_error_sd: float
     wind_speed: float
     wind_direction: float
     wind_turbulence:  float
     target_velocity: NDArray[np.float64]
     target_acceleration: NDArray[np.float64]
     has_landed: str
     landing_error: float

def generate_test_configs():
        
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
                        
                        sea_state_info = sea_states[sea_state]
                        test_configs.append({
                            "test_name": f"sigma-{sigma}__sea_state-{sea_state}__dir-{dir}__turb{turb}",
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

def load_data(folder_path, folder):
        
        # Construct file paths
        target_noisy_file = os.path.join(folder_path, folder, "target_pos_noisy.csv")
        target_file = os.path.join(folder_path, folder, "target_pos.csv")
        drone_file = os.path.join(folder_path, folder, "drone_pos.csv")
        
        # Load data
        print(target_noisy_file)
        target_state_noisy = np.loadtxt(target_noisy_file, delimiter=",", dtype=float)
        target_state = np.loadtxt(target_file, delimiter=",", dtype=float)
        drone_data = np.loadtxt(drone_file, delimiter=",", dtype=float)
        drone_position = drone_data[:, 0:3]
        drone_position[:, 0:2] *= 1e7

        return target_state, target_state_noisy, drone_position

def calculate_landing_error():
     
    test_configs: List[TestInfo] = generate_test_configs()
    data_folder_path = os.path.expanduser("~/UAV_Landing/logs")

    error_analysis_file = os.path.join(data_folder_path, "error_analysis.csv")
    error_analysis: List[ErrorAnalysis] = []
    for config in test_configs:
        
        folder_name = config["test_name"]

        if not os.path.exists(os.path.join(data_folder_path, folder_name)):
             continue
        
        target_state, target_state_noisy, drone_position = load_data(data_folder_path, folder_name)

        visualizer = DataVisualizer(target_state, drone_position)
        
        drone_pos_m = visualizer.get_drone_pos_neu_m()
        target_pos_m = visualizer.get_target_pos_neu_m()

        desired_landing_pos = target_pos_m[-1, :]
        actual_landing_pos = drone_pos_m[-1, :]

        has_landed = "NOT LANDED"
        landing_error = np.inf

        if actual_landing_pos[2] <= 1:
             has_landed = "LANDED"
             landing_error = np.linalg.norm(actual_landing_pos[0:2] - desired_landing_pos[0:2])


        error_analysis.append({
             'gps_error_sd': config["gps_error"]['sigma'],
             'wind_speed': config["wind_speed"]["speed"],
             'wind_direction': config["wind_speed"]["dir"],
             'wind_turbulence': config["wind_speed"]["turbulence"],
             'has_landed': has_landed,
             'landing_error': landing_error,
             'target_velocity': config["velocity"],
             'target_acceleration': config["acceleration"]
        })

    compute_landing_correlations(pd.DataFrame(error_analysis, columns=['wind_speed', 'wind_direction', 'gps_error_sd', 'landing_error']))

    with open(error_analysis_file, 'w', newline='', encoding='utf-8') as csvfile:
            
        writer = csv.DictWriter(csvfile,
                                    fieldnames=error_analysis[0].keys(),
                                    delimiter=",")
        writer.writeheader()
        writer.writerows(error_analysis)


def compute_landing_correlations(simulation_data):
    """
    Compute and visualize correlations between landing parameters
    
    Parameters:
    simulation_data (pd.DataFrame): DataFrame with columns:
    - wind_speed
    - wind_direction
    - wind_turbulence
    - gps_accuracy
    - landing_error
    
    Returns:
    - Correlation matrix
    # - Statistical significance matrix
    """
    # Compute Pearson correlation
    correlation_matrix = simulation_data.corr(method='pearson')
    
    # def calculate_pvalue(x, y):
    #     return stats.pearsonr(x, y)[1]
    
    # pvalue_matrix = simulation_data.corr(method=lambda x, y: calculate_pvalue(x, y))
    
    # Visualization
    plt.figure(figsize=(10, 8))
    sns.heatmap(
        correlation_matrix, 
        annot=True, 
        cmap='coolwarm', 
        vmin=-1, 
        vmax=1, 
        center=0,
        square=True
    )
    plt.title('Parameter Correlation Heatmap')
    plt.tight_layout()


    df = simulation_data["landing_error"].replace(np.inf, np.nan)
    df = df.dropna()
    print(len(df))
    plt.figure(figsize=(10, 8))
    plt.hist(df)
    plt.xlabel("Landing error, m")
    plt.ylabel("Frequency")
    plt.title("Landing error - Histogram")
    plt.show()


    
    return correlation_matrix
        
        


def visualize_logs():
    # Commented out folders for easy modification
    test_configs: List[TestInfo] = generate_test_configs()
    
    data_folder_path = os.path.expanduser("~/UAV_Landing/logs")

    for config in test_configs:
        # # Construct file paths
        # target_file = os.path.join(data_folder_path, folder, "target_pos_noisy.csv")
        # drone_file = os.path.join(data_folder_path, folder, "drone_pos.csv")
        
        # # Load data
        # target_state = np.loadtxt(target_file)
        # drone_data = np.loadtxt(drone_file)
        # drone_position = drone_data[:, 0:3]
        # drone_position[:, 0:2] *= 1e7

        folder = config["test_name"]
        if not os.path.exists(os.path.join(data_folder_path, folder)):
             continue
        
        target_state, target_state_noisy, drone_position = load_data(data_folder_path, folder)
        
        # Initialize data visualizer
        visualizer = DataVisualizer(target_state, drone_position)
        
        drone_pos_m = visualizer.get_drone_pos_neu_m()
        target_pos_m = visualizer.get_target_pos_neu_m()
        target_pos_noisy_m = visualizer._get_position_NEU(target_state_noisy[:,0:3])
        
        # XY Position Plot
        plt.figure()
        plt.plot(drone_pos_m[:, 0], drone_pos_m[:, 1], linewidth=1.5, label="Drone Position")
        plt.plot(target_pos_m[:, 0], target_pos_m[:, 1], linewidth=1.5, label="Target Position")
        plt.plot(target_pos_noisy_m[:, 0], target_pos_noisy_m[:, 1], linewidth=1.5, label="Target Position - Noisy")
        plt.plot(drone_pos_m[0, 0], drone_pos_m[0, 1], 'o')
        plt.plot(target_pos_m[0, 0], target_pos_m[0, 1], 'o')
        plt.xlabel('x, m')
        plt.ylabel('y, m')
        plt.title("XY Position")
        plt.legend()
        plt.grid(True)
        plt.savefig(os.path.join(data_folder_path, f"{folder}-xy-pos.png"))
        plt.close()
        
        # Drone Altitude Plot
        plt.figure()
        plt.plot(drone_pos_m[:, 2], linewidth=1.5)
        plt.title("Drone Altitude, m")
        plt.grid(True)
        plt.savefig(os.path.join(data_folder_path, f"{folder}-drone-alt.png"))
        plt.close()
        
        # 3D Trajectory Plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(drone_pos_m[:, 0], drone_pos_m[:, 1], drone_pos_m[:, 2], linewidth=1.5, label="Drone Position")
        ax.plot(target_pos_m[:, 0], target_pos_m[:, 1], target_pos_m[:, 2], linewidth=1.5, label="Target Position")
        ax.plot(target_pos_noisy_m[:, 0], target_pos_noisy_m[:, 1], target_pos_noisy_m[:, 2], linewidth=1.5, label="Target Position - Noisy")
        ax.set_title("Trajectories")
        ax.legend()
        ax.grid(True)
        plt.savefig(os.path.join(data_folder_path, f"{folder}-3d-traj.png"))
        plt.close()

if __name__ == "__main__":
    # visualize_logs()
    calculate_landing_error()