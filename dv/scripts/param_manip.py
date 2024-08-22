import time
import pandas as pd
import numpy as np
import sys

# Import mavutil
from pymavlink import mavutil

pd.set_option('display.max_columns', None)
pd.set_option('display.max_rows', None)
np.set_printoptions(precision=6, suppress=True)

calib_param_ids = [
    'AHRS_TRIM_X',
    'AHRS_TRIM_Y',
    'COMPASS_DIA_X',
    'COMPASS_DIA_Y',
    'COMPASS_DIA_Z',
    'COMPASS_ODI_X',
    'COMPASS_ODI_Y',
    'COMPASS_ODI_Z',
    'COMPASS_OFS_X',
    'COMPASS_OFS_Y',
    'COMPASS_OFS_Z',
    'INS_ACC2OFFS_X',
    'INS_ACC2OFFS_Y',
    'INS_ACC2OFFS_Z',
    'INS_ACC2SCAL_X',
    'INS_ACC2SCAL_Y',
    'INS_ACC2SCAL_Z',
    'INS_ACCOFFS_X',
    'INS_ACCOFFS_Y',
    'INS_ACCOFFS_Z',
    'INS_ACCSCAL_X',
    'INS_ACCSCAL_Y',
    'INS_ACCSCAL_Z',
    'INS_GYR2OFFS_X',
    'INS_GYR2OFFS_Y',
    'INS_GYR2OFFS_Z',
    'INS_GYROFFS_X',
    'INS_GYROFFS_Y',
    'INS_GYROFFS_Z',
]


def readout_param_messages(some_master):
    while True:
        m = some_master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if not m:
            break
        # else:
        #     m = m.to_dict()
        # print(m)


def read_param(l_master, l_param_id):
    print(f"Reading {l_param_id} ...")
    m = {"param_id": ''}
    for _ in range(10):
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
            m["param_value"] = np.float32(m["param_value"])
            return m
        else:
            # print(m['param_id'], " !=", l_param_id)
            continue

    print(f"Cannot read {l_param_id}")
    return None


def write_param(l_master, l_param_id, new_param_value):
    current_param_message = read_param(l_master, l_param_id)

    if current_param_message is None:
        return False

    current_param_value = current_param_message['param_value']

    if np.isclose(current_param_value, new_param_value):
        print(f"{l_param_id} already set to desired new_param_value == current_param_value [{new_param_value} == {current_param_value}]")
        return True

    print(f"Writing {l_param_id}: {current_param_value} ==> {new_param_value}")

    updated_param_value = np.nan
    n = 0

    while updated_param_value is None and not np.isclose(new_param_value, updated_param_value):
        print(f"{new_param_value} != {updated_param_value}")
        l_master.mav.param_set_send(
            l_master.target_system, l_master.target_component,
            l_param_id.encode(),
            new_param_value,
            current_param_message["param_type"]
        )

        updated_param_value = read_param(l_master, l_param_id)["param_value"]
        print(f"updated_param_value: {updated_param_value}, {type(updated_param_value)}")

        time.sleep(0.5)

        n += 1

        if n > 10:
            return False

    return True


def get_all_params_from_device(some_master):
    some_master.mav.param_request_list_send(
        some_master.target_system, some_master.target_component
    )

    params = []

    stop_read = False

    param_indexes = np.zeros(2000, dtype=bool)

    while not stop_read:
        try:
            message = some_master.recv_match(type='PARAM_VALUE', blocking=True, timeout=10).to_dict()

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

    param_index_diff = np.diff(np.sort(params_df["param_index"].to_numpy()))
    assert np.all((param_index_diff[0] == param_index_diff))

    params_df = params_df[["param_id", "param_value"]]

    params_df.sort_values("param_id", inplace=True)
    params_df.reset_index(drop=True, inplace=True)

    # Round paramever values as mission planner does. It simplifies diffing.
    # params_df["param_value"] = np.round(params_df["param_value"], decimals=6)
    return params_df


def get_selected_params_from_device(some_master, param_list):
    params = []
    param_indexes = np.zeros(len(param_list), dtype=bool)

    for idx, param_id in enumerate(param_list):
        param = read_param(some_master, param_id)
        if param is not None:
            params.append({"param_id": param["param_id"], "param_value": param["param_value"]})
            param_indexes[idx] = True

    assert np.all(param_indexes)

    params_df = pd.DataFrame(params)
    params_df = params_df.astype({'param_value': 'float32'})

    params_df.sort_values("param_id", inplace=True)

    # Round paramever values as mission planner does. It simplifies diffing.
    # params_df["param_value"] = np.round(params_df["param_value"], decimals=6)

    return params_df


def write_to_device(some_master, params_df):

    params_df["write_status"] = False

    for row_idx in range(params_df.shape[0]):
        param_id = params_df.iloc[row_idx]['param_id']
        param_value = params_df.iloc[row_idx]['param_value']
        res = write_param(some_master, param_id, param_value)
        if not res:
            print(f"{param_id} set failed")
        else:
            params_df.loc[row_idx, "write_status"] = True
        time.sleep(0.1)
        readout_param_messages(some_master)

    print(params_df[params_df["write_status"] == False])


def read_params_from_file(file_path):
    params_df = pd.read_csv(file_path, header=None, names=["param_id", "param_value"]).astype({'param_value': 'float32'})
    params_df.sort_values("param_id", inplace=True)

    # Round paramever values as mission planner does. It simplifies diffing.
    # params_df["param_value"] = np.round(params_df["param_value"], decimals=6)

    return params_df


def write_params_to_file(params_df, to_file):
    params_df = params_df.set_index("param_id", drop=True)
    params_df["param_value"].to_csv(to_file, header=False)


def sort_paramets_from_mission_planner(file_name):
    params_df = pd.read_csv(file_name, header=None)
    params_df = params_df.set_index(params_df.keys()[0], drop=True)
    params_df.sort_index(inplace=True)
    params_df.to_csv(file_name.replace(".param", "") + "_sorted.param", header=False)


def save_calibs(some_master, calib_file_path="calib_params.param"):
    some_calib_params = get_selected_params_from_device(some_master, calib_param_ids)
    write_params_to_file(some_calib_params, calib_file_path)


def restore_calibs(some_master, calib_file_path="calib_params.param"):
    calib_params2 = read_params_from_file(calib_file_path)
    write_to_device(some_master, calib_params2)

# %%

if __name__ == "main":
    pass

# %%
#     connect_string = '/dev/ttyACM3'
#
#     gmaster = mavutil.mavlink_connection(connect_string)
#     gmaster.wait_heartbeat()
#
#     all_params = get_all_params_from_device(gmaster)
#
#     calib_params = get_selected_params_from_device(gmaster, calib_param_ids)
#
#     calib_file_path = "/home/slovak/remote-id/dv-kd-sw-rid/ardupilot/dv/scripts/calit_params.param"
#
#     write_params_to_file(calib_params, calib_file_path)
#
#     calib_params2 = read_params_from_file(calib_file_path)
#
#     write_to_device(gmaster, calib_params2)
#
#
# # %%
