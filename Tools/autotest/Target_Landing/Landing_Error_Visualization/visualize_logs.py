import numpy as np
import os
import matplotlib.pyplot as plt
import csv
import seaborn as sns
import pandas as pd
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

from typing import Dict, List, TypedDict
from numpy.typing import NDArray

from simulation_env import TestInfo, SimulationEnvironment
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

log_file_path = "~/UAV_Landing/logs2"

def load_data(folder_path, folder):
        
        # Construct file paths
        target_noisy_file = os.path.join(folder_path, folder, "target_noisy.csv")
        target_file = os.path.join(folder_path, folder, "target.csv")
        drone_file = os.path.join(folder_path, folder, "drone.csv")
        
        # Load data
        target_state_noisy = np.loadtxt(target_noisy_file, delimiter=",", dtype=float, usecols=(1,2,3,4,5))
        target_state = np.loadtxt(target_file, delimiter=",", dtype=float, usecols=(1,2,3,4,5))
        drone_data = np.loadtxt(drone_file, delimiter=",", dtype=float, usecols=(1,2,3,4,5))
        drone_position = drone_data[:, 0:3]
        # drone_position_raw = drone_data[:, 6:10]
        drone_position_raw = []

        time = np.loadtxt(drone_file, delimiter=",", dtype=float, usecols=0)
        time_gps = np.loadtxt(target_noisy_file, delimiter=",", dtype=float, usecols=0)

        return time, time_gps, target_state, target_state_noisy, drone_position, drone_position_raw

def calculate_landing_error():
     
    test_configs: List[TestInfo] = SimulationEnvironment.generate_test_configs()
    # test_configs = SimulationEnvironment.specific_test_case()
    data_folder_path = os.path.expanduser(log_file_path)

    error_analysis_file = os.path.join(data_folder_path, "error_analysis.csv")
    error_analysis: List[ErrorAnalysis] = []
    for config in test_configs:
        
        folder_name = config["test_name"]

        if not os.path.exists(os.path.join(data_folder_path, folder_name)):
             continue
        
        time, time_gps, target_state, target_state_noisy, drone_position, drone_position_raw = load_data(data_folder_path, folder_name)

        visualizer = DataVisualizer(target_state, drone_position)
        
        drone_pos_m = visualizer.get_drone_pos_neu_m()
        target_pos_m = visualizer.get_target_pos_neu_m()

        desired_landing_pos = target_pos_m[-1, :]
        actual_landing_pos = drone_pos_m[-1, :]

        has_landed = False
        # landing_error = np.inf
        landing_error = np.linalg.norm(actual_landing_pos[0:2] - desired_landing_pos[0:2])

        landing_off_x = actual_landing_pos[0] - desired_landing_pos[0]
        landing_off_y = actual_landing_pos[1] - desired_landing_pos[1] 


        if actual_landing_pos[2] <= 1.5:
             has_landed = True


        error_analysis.append({
             'name': config['test_name'],
             'wind_speed': SimulationEnvironment.get_wind_spd_from_sea_state(config["wind_speed"]["sea_state"]),
             'wind_dir': config["wind_speed"]["dir"],
             'wind_turb': config["wind_speed"]["turbulence"],
             'velocity': config["velocity"],
             'acceleration': config["acceleration"],
             'gps_error_sigma': config["gps_error"]['sigma'],
             'gps_update_frequency': config['gps_frequency'],
             'gps_latency': config['gps_latency'],
             'initial_distance_from_target': config['target_distance_from_drone'],
             'is_landed': has_landed,
             'landing_error': landing_error,
             'alt_diff': drone_pos_m[-1,2],
             'landing_off_x': landing_off_x,
             'landing_off_y': landing_off_y
        })

    # compute_landing_correlations(pd.DataFrame(error_analysis, columns=['wind_speed', 'wind_dir', 'gps_error_sigma', 'landing_error']))

    # plot_bar(error_analysis)
    plot_landing_error(error_analysis)

    if os.path.exists(error_analysis_file):
        raise Exception("File already exists")

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
        
# def plot_parameter_specific_hist(sim_data_df: pd.DataFrame, param: str, bins):
     

#     landed_df = sim_data_df[sim_data_df["is_landed"] == True]
#     not_landed_df = sim_data_df[sim_data_df["is_landed"] == False]

#     plt.hist(not_landed_df[param],
#              label='Not landed',
#              bins=bins,
#              alpha=0.5, rwidth=0.5)
#     plt.hist(landed_df[param],
#             label='Landed',
#             bins=bins,
#             alpha=0.5, rwidth=0.5)
#     plt.legend()
#     plt.xlabel(param)
#     plt.ylabel("Frequency")

def plot_landing_error(sim_data: List[ErrorAnalysis]):
     
    parameters = ["wind_speed", "wind_dir", "gps_error_sigma", "gps_update_frequency", 
                                                  "gps_latency", "initial_distance_from_target"]
         
    sim_data_df = pd.DataFrame(sim_data, columns=["wind_speed", "wind_dir", "gps_error_sigma", "gps_update_frequency", 
                                                  "gps_latency", "initial_distance_from_target", "landing_error", "landing_off_x", "landing_off_y"]) 

    sim_data_df = sim_data_df[sim_data_df["landing_error"] <= 5]
    print(sim_data_df.shape)

   


    print(sim_data_df.shape)

    for i, param in enumerate(parameters):
          
        # subplt = plt.subplot(3,2, i+1)
        plt.rcParams.update({'font.size': 20})
        fig = plt.figure(figsize=(12,12))
        fig.suptitle('Effect of real world scenarios on landing', fontsize=16)
        values = get_bins(param)

        for value in values:

            df = sim_data_df[sim_data_df[param] == value]
            plt.scatter(df["landing_off_x"], df["landing_off_y"], label=value)

        plot_square(2)
        plot_square(5)
        plot_square(10)
        plt.legend()
        plt.xlabel(get_xlabel(param))
        plt.xlim([-5, 5])
        plt.ylim([-5, 5])
        
        # plot_parameter_specific_hist(sim_data_df, param, bins=get_bins(param))

    plt.subplots_adjust(left=0.1,
                    bottom=0.1, 
                    right=0.9, 
                    top=0.9, 
                    wspace=0.4, 
                    hspace=0.4)
    plt.show()

def plot_square(side):

    plt.plot([-side/2, side/2], [-side/2, -side/2], 'black')
    plt.plot([side/2, side/2], [-side/2, side/2], 'black')
    plt.plot([side/2, -side/2], [side/2, side/2], 'black')
    plt.plot([-side/2, -side/2], [side/2, -side/2], 'black')

def plot_bar(sim_data: List[ErrorAnalysis]):
     
    parameters = ["wind_speed", "wind_dir", "gps_error_sigma", "gps_update_frequency", 
                                                  "gps_latency", "initial_distance_from_target"]
     
    sim_data_df = pd.DataFrame(sim_data, columns=["wind_speed", "wind_dir", "gps_error_sigma", "gps_update_frequency", 
                                                  "gps_latency", "initial_distance_from_target", "is_landed"]) 
    
    print(sim_data_df.shape)
    # sim_data_df = sim_data_df[(sim_data_df["gps_error_sigma"] == 0.5) &
    #                           (sim_data_df["gps_latency"] != 1) &
    #                           (sim_data_df["gps_update_frequency"] == 1) &
    #                           (sim_data_df["wind_dir"] >= 90) &
    #                           (sim_data_df["wind_speed"] <= 6)]

    plt.rcParams.update({'font.size': 20})
    fig = plt.figure(figsize=(12,10))
    fig.suptitle('Effect of real world scenarios on landing', fontsize=16)

    landed_df = sim_data_df[sim_data_df["is_landed"] == True]
    not_landed_df = sim_data_df[sim_data_df["is_landed"] == False]


    print(f"Successful landing {landed_df.shape[0]}")
    print(f"Failed landing {not_landed_df.shape[0]}")
    print(f"Total Attempts {landed_df.shape[0] + not_landed_df.shape[0]}, {sim_data_df.shape[0]}")

    for i, param in enumerate(parameters):
          
        subplt = plt.subplot(3,2, i+1)

        
        l = landed_df.groupby(param).size()
        n = not_landed_df.groupby(param).size()

        x1 = np.array(get_bins(param))
        if len(x1) > 1:
            width = (x1[1] - x1[0])*0.25
        else: 
            width = 0.1


        x2 = x1 + width

        bottom = 0
        plt.bar(x1, l.values,
                 width=width, 
                 label="Landed", bottom=bottom)
        plt.bar(x2, n.values, 
                width=width, 
                label="Not Landed", bottom=bottom)
        # plt.text(x1[0]/2, l.values[0]/2, f"{x1[0]}")
        plt.legend()
        plt.xlabel(get_xlabel(param))
        plt.ylabel("# of Occurences")

        if param ==  "gps_error_sigma" or param == "gps_update_frequency" or param == "gps_latency":
            plt.xlim(0, 1.5)

        # plot_parameter_specific_hist(sim_data_df, param, bins=get_bins(param))

    plt.subplots_adjust(left=0.1,
                    bottom=0.1, 
                    right=0.9, 
                    top=0.9, 
                    wspace=0.4, 
                    hspace=0.4)
    plt.show()

def get_xlabel(param: str):
    
    match param:
        case "wind_speed":
            return "Wind Speed (m/s)"
        case "wind_dir":
            return "Wind Direction (deg)"
        case "gps_error_sigma":
            return "GPS Error Std Deviation"
        case "gps_update_frequency":
            return "Target GPS Frequency (Hz)"
        case "gps_latency":
            return "Target GPS latency (s)"
        case "initial_distance_from_target":
            return "Initial distance from target (m)"

def get_bins(param:str):
     
    match param:
        case "wind_speed":
            return SimulationEnvironment.get_sea_state_wind_speed(SimulationEnvironment.winds["sea_states"])
            # return SimulationEnvironment.get_sea_state_wind_speed([0, 1, 2, 3])
        case "wind_dir":
            return SimulationEnvironment.winds["wind_direction"]
            # return [90, 180, 270]
        case "gps_error_sigma":
            return SimulationEnvironment.gps_error["sigma"]
            # return [0.5]
        case "gps_update_frequency":
            return SimulationEnvironment.gps_time["frequency"]
            # return [1]
        case "gps_latency":
            return SimulationEnvironment.gps_time["latency"]
            # return [0.2, 0.5]
        case "initial_distance_from_target":
            return [25, 50, 75, 100]


def visualize_logs():
    test_configs: List[TestInfo] = SimulationEnvironment.generate_test_configs()
    # test_configs = SimulationEnvironment.specific_test_case()
    
    data_folder_path = os.path.expanduser(log_file_path)

    for config in test_configs:
        
        folder = config["test_name"]
        if not os.path.exists(os.path.join(data_folder_path, folder)):
             continue
        
        save_path = os.path.join(data_folder_path, folder)
        time, time_gps, target_state, target_state_noisy, drone_position, drone_position_raw = load_data(data_folder_path, folder)
        
        # Initialize data visualizer
        visualizer = DataVisualizer(target_state, drone_position)
        
        drone_pos_m = visualizer.get_drone_pos_neu_m()
        target_pos_m = visualizer.get_target_pos_neu_m()
        target_pos_noisy_m = visualizer._get_position_NEU(target_state_noisy[:,0:3])
        drone_pos_raw_m = visualizer._get_position_NEU(drone_position_raw[:,0:3])
        
        # # XY Position Plot
        # plt.figure(figsize=(10,8))

        # plt.plot(drone_pos_m[:, 0], drone_pos_m[:, 1], linewidth=1.5, label="Drone Position")
        # plt.plot(target_pos_m[:, 0], target_pos_m[:, 1], linewidth=1.5, label="Target Position")
        # plt.plot(target_pos_noisy_m[:, 0], target_pos_noisy_m[:, 1], linewidth=1.5, label="Target Position - Noisy")
        # # plt.scatter(drone_pos_m[:, 0], drone_pos_m[:, 1], s=30, c='blue', label="Drone Position", marker='o')
        # # plt.scatter(target_pos_m[:, 0], target_pos_m[:, 1], s=30, c='red', label="Target Position", marker='o')
        # # plt.scatter(target_pos_noisy_m[:, 0], target_pos_noisy_m[:, 1], s=30, c='green', label="Target Position Noisy", marker='o')

        # plt.plot(drone_pos_m[0, 0], drone_pos_m[0, 1], 'o')
        # plt.plot(target_pos_m[0, 0], target_pos_m[0, 1], 'o')
        # plt.xlabel('x, m')
        # plt.ylabel('y, m')
        # plt.title("XY Position")
        # plt.legend()
        # plt.grid(True)
        # plt.savefig(os.path.join(save_path, f"{folder}-xy-pos.png"))
        
        # # Drone Altitude Plot
        # plt.figure(figsize=(10,8))
        # plt.plot(drone_pos_m[:, 2], linewidth=1.5)
        # plt.title("Drone Altitude, m")
        # plt.grid(True)
        # plt.savefig(os.path.join(save_path, f"{folder}-drone-alt.png"))
        
        # # 3D Trajectory Plot
        # fig = plt.figure(figsize=(10,8))
        # ax = fig.add_subplot(111, projection='3d')
        # ax.plot(drone_pos_m[:, 0], drone_pos_m[:, 1], drone_pos_m[:, 2], linewidth=1.5, label="Drone Position")
        # ax.plot(target_pos_m[:, 0], target_pos_m[:, 1], target_pos_m[:, 2], linewidth=1.5, label="Target Position")
        # ax.plot(target_pos_noisy_m[:, 0], target_pos_noisy_m[:, 1], target_pos_noisy_m[:, 2], linewidth=1.5, label="Target Position - Noisy")
        # ax.set_title("Trajectories")
        # ax.legend()
        # ax.grid(True)
        # plt.savefig(os.path.join(save_path, f"{folder}-3d-traj.png"))

        # With respect to time

        plt.figure(figsize=(10,8))
        # Scatter plot for drone and target positions
        plt.scatter(time, drone_pos_m[:, 0], s=30, c='blue', label="Drone Position", marker='o')
        plt.scatter(time, drone_pos_raw_m[:,0],  s=30, c='red', label="Drone Position Raw", marker='o')
        # plt.scatter(time, target_pos_m[:, 0], s=30, c='green', label="Target Position", marker='o')
        # plt.scatter(time_gps, target_pos_noisy_m[:, 0], s=30, c='red', label="Target Position - Noisy", marker='^')

        # Labels and title
        plt.xlabel('t, s')
        plt.ylabel('x, m')
        plt.title("X Position")
        plt.legend()
        plt.grid(True)
        plt.show()

       
        # t = np.linspace(0, 125)
        # print(len(t))
        # animate_trajectory(drone_pos_m.transpose(), target_pos_m.transpose(), np.linspace(0, 125, 126))

        break
        # plt.close('all')
        # plt.show()

def animate_trajectory(r1, r2, t):

    # Set up the figure and 3D animation
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(0, max(r1[0,:]) * 1.1)
    ax.set_ylim(0, max(r1[1,:]) * 1.1)
    ax.set_zlim(0, max(r1[2,:]) * 1.1)
    ax.set_title('Landing Trajectory')
    ax.set_xlabel('X Distance (m)')
    ax.set_ylabel('Y Distance (m)')
    ax.set_zlabel('Z Height (m)')

    # Initialize an empty line
    line1, = ax.plot([], [], [], 'b-', lw=2, label='Trajectory 1')
    point1, = ax.plot([], [], [], 'ro', markersize=10)

    line2, = ax.plot([], [], [], 'b-', lw=2, label='Trajectory 2')
    point2, = ax.plot([], [], [], 'ro', markersize=10)

    # Initialization function
    def init():
        line1.set_data([], [])
        line1.set_3d_properties([])
        point1.set_data([], [])
        point1.set_3d_properties([])

        line2.set_data([], [])
        line2.set_3d_properties([])
        point2.set_data([], [])
        point2.set_3d_properties([])
        return line1, point1, line2, point2

    # Animation update function
    def update(frame):
        line1.set_data(r1[0,:frame], r1[1,:frame])
        line1.set_3d_properties(r1[2,:frame])
        point1.set_data(r1[0,frame:frame+1], r1[0,frame:frame+1])
        point1.set_3d_properties(r1[2,frame:frame+1])

        line2.set_data(r2[0,:frame], r2[1,:frame])
        line2.set_3d_properties(r2[2,:frame])
        point2.set_data(r2[0,frame:frame+1], r2[0,frame:frame+1])
        point2.set_3d_properties(r2[2,frame:frame+1])
        return line1, point1, line2, point2

    # Create animation
    anim = animation.FuncAnimation(
        fig, 
        update, 
        frames=len(t), 
        init_func=init, 
        interval=100, 
        blit=True
    )

    # Optional: Rotate the view during animation
    ax.view_init(elev=20, azim=45)

    anim.save('trajectory_animation.gif', writer='pillow')
    # anim.save('trajectory_animation.mp4', writer='ffmpeg')

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # visualize_logs()
    calculate_landing_error()