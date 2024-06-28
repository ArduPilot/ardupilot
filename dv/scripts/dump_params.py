import time
import pandas as pd
import numpy as np
import sys

# Import mavutil
from pymavlink import mavutil

pd.set_option('display.max_columns', None)
pd.set_option('display.max_rows', None)
np.set_printoptions(precision=6, suppress=True)

# connect_string = 'udpin:0.0.0.0:14551'
connect_string = '/dev/ttyACM0'
# Create the connection
master = mavutil.mavlink_connection(connect_string, baud=57600)
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Request all parameters
master.mav.param_request_list_send(
    master.target_system, master.target_component
)

params = []

stop_read = False

param_indexes = np.zeros(2000, dtype=bool)

while not stop_read:
    try:
        message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=10).to_dict()

        if message["param_index"] == np.iinfo(np.uint16).max:
            continue

        if len(params) == 0:
            param_indexes = np.zeros(message["param_count"], dtype=bool)

        params.append(message)

        print(message["param_count"] - len(params))
        print(message)

        param_indexes[message["param_index"]] = True

        stop_read = np.all(param_indexes)

    except Exception as error:
        print(error)
        break

    time.sleep(0.01)

params_df = pd.DataFrame(params)

params_df = params_df.drop(columns=["mavpackettype", "param_count"])
params_df = params_df.set_index("param_index", drop=True)
assert params_df.index.is_monotonic_increasing
params_df = params_df.set_index("param_id", drop=True)
params_df.sort_index(inplace=True)

# Round paramever values as mission planner does. It simplifies diffing.
params_df["param_value"] = np.round(params_df["param_value"], decimals=6)

# params_df.to_csv("cube_orange_default_params.csv", header=None)
params_df["param_value"].to_csv("cube_orange_default_params.param", header=False)

# %%
# Get single param.

master.mav.param_request_read_send(
    master.target_system, master.target_component,
    b'ACRO_RP_EXPO',
    -1
)

message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()

print(message)

# %%
# Sorts parameter list from mission planner

file_name = "KDV01_005_green_CUBEPLUS_20240409.param"

params_df = pd.read_csv(file_name, header=None)

params_df = params_df.set_index(params_df.keys()[0], drop=True)
params_df.sort_index(inplace=True)

params_df.to_csv(file_name.replace(".param", "") + "_sorted.param", header=False)
