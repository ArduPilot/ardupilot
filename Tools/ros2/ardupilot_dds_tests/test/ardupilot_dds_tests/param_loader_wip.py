# Copyright 2023 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

"""Loader parameters for test cases (WIP)."""

# from pathlib import Path
#
# class DDSParams(object):
#     def __init__(self):
#         super().__init__()
#
#         # default params
#         self.mra_serial_device = f"./dev/ttyROS0"
#         self.mra_serial_baud = 115200
#         self.mra_refs_file = "dds_xrce_profile.xml"
#
#         self.ap_sim_vehicle_cmd = "sim_vehicle.py"
#         self.ap_serial_device = f"./dev/ttyROS1"
#         self.ap_serial_baud = 115200
#         self.ap_vehicle = "ArduCopter"
#         self.ap_frame = "quad"
#
#         pkg = get_package_share_directory("ardupilot_sitl")
#
#         # The micro_ros_agent and ardupilot nodes do not expose params
#         # as ROS params, parse the config file and send them in as node args.
#         params = Path(pkg, "config", "ardupilot_dds.yaml")
#
#         with open(params, "r") as f:
#             params_str = f.read()
#             params = yaml.safe_load(params_str)
#             print(params)
#
#             mra_params = params["/micro_ros_agent"]
#             if mra_params["transport"] == "serial":
#                 self.mra_serial_baud = f"{mra_params['serial_baud']}"
#                 self.mra_serial_device = f"{mra_params['serial_device']}"
#                 self.mra_refs_file = f"{mra_params['refs_file']}"
#
#             ap_params = params["/ardupilot"]
#             if ap_params:
#                 self.ap_sim_vehicle_cmd = ap_params["sim_vehicle_cmd"]
#                 self.ap_vehicle = ap_params["vehicle"]
#                 self.ap_frame = ap_params["frame"]
#                 self.ap_serial_device = ap_params["serial_device"]
#                 self.ap_serial_baud = ap_params["serial_baud"]
