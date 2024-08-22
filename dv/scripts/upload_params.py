import time
import pandas as pd
import numpy as np
import sys

# Import mavutil
from pymavlink import mavutil

pd.set_option('display.max_columns', None)
pd.set_option('display.max_rows', None)
np.set_printoptions(precision=6, suppress=True)

# connect_string = 'udpin:0.0.0.0:14700'
connect_string = '/dev/ttyACM0'
# Create the connection
master = mavutil.mavlink_connection(connect_string)
# Wait a heartbeat before sending commands
master.wait_heartbeat()


# %%

def readout_param_messages():
    while True:
        m = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if not m:
            break
        else:
            m = m.to_dict()
            print(m)


def read_param(l_master, l_param_id):
    m = {"param_id": ''}
    for iter in range(10):
        l_master.mav.param_request_read_send(
            l_master.target_system, l_master.target_component,
            l_param_id.encode(),
            -1
        )

        try:
            m = l_master.recv_match(type='PARAM_VALUE', blocking=True, timeout=10)
            if not m:
                continue
            else:
                m = m.to_dict()
        except:
            print("Timeout expired?")

        if m['param_id'] == l_param_id:
            return m
        else:
            print(m['param_id'], " !=", l_param_id)
            continue

    return None


def write_param(l_master, l_param_id, new_param_value):
    old_param_message = read_param(master, l_param_id)

    if old_param_message is None:
        print("old_param_message is None")
        return None

    if old_param_message["param_value"] == new_param_value:
        print(f"Already set to: {new_param_value}, skipping")
        return old_param_message  # Already set.

    updated_param = {"param_value": None}

    for liter in range(10):
        l_master.mav.param_set_send(
            l_master.target_system, l_master.target_component,
            l_param_id.encode(),
            new_param_value,
            old_param_message["param_type"]
        )

        # m = l_master.recv_match(type='PARAM_VALUE', blocking=True, timeout=10)
        # if m is None:
        #     continue
        # else:
        #     m.to_dict()["param_id"] == l_param_id:
        #

        n = 0
        while new_param_value != updated_param["param_value"]:
            updated_param = read_param(master, l_param_id)
            if updated_param is None:
                return None

            # print(updated_param)
            time.sleep(0.5)
            n = n + 1
            if n > 10:
                return None

    return updated_param


readout_param_messages()

# %%

res = write_param(master, "ACRO_BAL_PITCH", 3.0)
assert res is not None

print(res)
res2 = read_param(master, "ACRO_BAL_PITCH")
print(res2)
assert res["param_value"] == res2["param_value"]

# %%


file_path = "/home/slovak/remote-id/dv-kd-sw-rid/ardupilot/dv/parameters/CubeOrangePlus-dv.param"

params_df = pd.read_csv(file_path, header=None, names=["param_id", "param_value"])

# prams_to_write = [
#     "FRAME_CLASS",
#     "DID_BARO_ACC",
#     "DID_CANDRIVER",
#     "DID_ENABLE",
#     "DID_MAVPORT",
#     "DID_OPTIONS",
# ]
#
# SR2_EXTRA1,10.0
# SR2_EXTRA2,10.0
# SR2_EXTRA3,2.0
# SR2_EXT_STAT,2.0
# SR2_POSITION,10.0
# SR2_RAW_SENS,2.0
# SR2_RC_CHAN,2.0


# %%

for index, row in params_df.iterrows():
    print(row['param_id'])
    res = write_param(master, row["param_id"], row["param_value"])
    if res is None:
        print(f"Cannot set {row['param_id']}, maybe readonly?")
    else:
        print("OK\n")
    time.sleep(0.5)
    readout_param_messages()

# %%

master.reboot_autopilot()

# %%

res2 = read_param(master, "FRAME_CLASS")
# res2 = read_param(master, "SERIAL1_BAUD")
print(res2)



# %%
res = write_param(master, "SERIAL1_BAUD", 921.0)
assert res is not None
print(res)

# %%
res2 = read_param(master, "FRAME_CLASS")
print(res2)

# %%
res = write_param(master, "FRAME_CLASS", 1)
assert res is not None
print(res)

# %%
res2 = read_param(master, "FORMAT_VERSION")
print(res2)

# %%
res = write_param(master, "FORMAT_VERSION", 0)
assert res is not None
print(res)

master.reboot_autopilot()



# %%

readout_param_messages()
res2 = read_param(master, "DID_MAVPORT")
print(res2)


# DID_BARO_ACC,0.0
# DID_CANDRIVER,0.0
# DID_ENABLE,1.0
# DID_MAVPORT,0.0
# DID_OPTIONS,1.0

res = write_param(master, "DID_MAVPORT", 1)
assert res is not None
print(res)
