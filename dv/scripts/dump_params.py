import time
import pandas as pd
import numpy as np
import sys

# Import mavutil
from pymavlink import mavutil

pd.set_option('display.max_columns', None)
pd.set_option('display.max_rows', None)
np.set_printoptions(precision=6, suppress=True)

connect_string = 'udpin:0.0.0.0:14551'
# Create the connection
master = mavutil.mavlink_connection(connect_string)
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

        # print('name: %s\tvalue: %d' % (message['param_id'],
        #                                message['param_value']))
    except Exception as error:
        print(error)
        break
        # sys.exit(0)

    time.sleep(0.01)

params_df = pd.DataFrame(params)

params_df = params_df.drop(columns=["mavpackettype", "param_count"])
params_df = params_df.set_index("param_index", drop=True)
assert params_df.index.is_monotonic_increasing
params_df = params_df.set_index("param_id", drop=True)
params_df.sort_index(inplace=True)

# pd.options.display.float_format = '{:.6f}'.format
# print(params_df[params_df["param_id"] == "GPS_POS1_X"])

params_df.to_csv("some_params.csv", header=None)
params_df["param_value"].to_csv("some_params.param", header=None)

# %%


KDV01_green_first_flyable = pd.read_csv("KDV01_green_first_flyable.param")

KDV01_green_first_flyable = KDV01_green_first_flyable.set_index(KDV01_green_first_flyable.keys()[0], drop=True)
KDV01_green_first_flyable.sort_index(inplace=True)

KDV01_green_first_flyable.to_csv("KDV01_green_first_flyable___.param", header=None)
