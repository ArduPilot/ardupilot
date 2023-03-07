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

"""
Launch ArduPilot SITL.

Run with default arguments:

ros2 launch ardupilot_sitl sitl.launch.py

Show launch arguments:

ros2 launch ardupilot_sitl sitl.launch.py --show-args
"""
from typing import List

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

# Labels for the optional uart launch arguments.
uart_labels = ["A", "B", "C", "D", "E", "F", "H", "I", "J"]
max_serial_ports = 10


def launch_sitl(
    context: LaunchContext, *args, **kwargs
) -> List[LaunchDescriptionEntity]:
    """Return a SITL process."""
    # Retrieve launch arguments.
    command = LaunchConfiguration("command").perform(context)
    model = LaunchConfiguration("model").perform(context)
    speedup = LaunchConfiguration("speedup").perform(context)
    slave = LaunchConfiguration("slave").perform(context)
    sim_address = LaunchConfiguration("sim_address").perform(context)
    instance = LaunchConfiguration("instance").perform(context)
    defaults = LaunchConfiguration("defaults").perform(context)

    # Display launch arguments.
    print(f"command:          {command}")
    print(f"model:            {model}")
    print(f"speedup:          {speedup}")
    print(f"slave:            {slave}")
    print(f"sim_address:      {sim_address}")
    print(f"instance:         {instance}")
    print(f"defaults:         {defaults}")

    # Required arguments.
    cmd_args = [
        f"{command} ",
        f"--model {model} ",
        f"--speedup {speedup} ",
        f"--slave {slave} ",
        f"--sim-address={sim_address} ",
        f"--instance {instance} ",
        f"--defaults {defaults} ",
    ]

    # Optional arguments.
    wipe = LaunchConfiguration("wipe").perform(context)
    if wipe == "True":
        cmd_args.append("--wipe ")
        print(f"wipe:             {wipe}")

    synthetic_clock = LaunchConfiguration("synthetic_clock").perform(context)
    if synthetic_clock == "True":
        cmd_args.append("--synthetic-clock ")
        print(f"synthetic_clock:  {synthetic_clock}")

    home = LaunchConfiguration("home").perform(context)
    if home:
        cmd_args.append(f"--home {home} ")
        print(f"home:             {home}")

    # Optional uart arguments.
    for label in uart_labels:
        arg = LaunchConfiguration(f"uart{label}").perform(context)
        if arg:
            cmd_args.append(f"--uart{label} {arg} ")
            print(f"uart{label}:            {arg}")

    # Optional serial arguments.
    for label in range(10):
        arg = LaunchConfiguration(f"serial{label}").perform(context)
        if arg:
            cmd_args.append(f"--serial{label} {arg} ")
            print(f"serial{label}:          {arg}")

    rate = LaunchConfiguration("rate").perform(context)
    if rate:
        cmd_args.append(f"--rate {rate} ")
        print(f"rate:             {rate}")

    gimbal = LaunchConfiguration("gimbal").perform(context)
    if gimbal:
        cmd_args.append(f"--gimbal {gimbal} ")
        print(f"gimbal:           {gimbal}")

    base_port = LaunchConfiguration("base_port").perform(context)
    if base_port:
        cmd_args.append(f"--base-port {base_port} ")
        print(f"base_port:        {base_port}")

    rc_in_port = LaunchConfiguration("rc_in_port").perform(context)
    if rc_in_port:
        cmd_args.append(f"--rc-in-port {rc_in_port} ")
        print(f"rc_in_port:       {rc_in_port}")

    sim_port_in = LaunchConfiguration("sim_port_in").perform(context)
    if sim_port_in:
        cmd_args.append(f"--sim-port-in {sim_port_in} ")
        print(f"sim_port_in:      {sim_port_in}")

    sim_port_out = LaunchConfiguration("sim_port_out").perform(context)
    if sim_port_out:
        cmd_args.append(f"--sim-port-out {sim_port_out} ")
        print(f"sim_port_out:     {sim_port_out}")

    irlock_port = LaunchConfiguration("irlock_port").perform(context)
    if irlock_port:
        cmd_args.append(f"--irlock-port {irlock_port} ")
        print(f"irlock_port:      {irlock_port}")

    start_time = LaunchConfiguration("start_time").perform(context)
    if start_time:
        cmd_args.append(f"--start-time {start_time} ")
        print(f"start_time:       {start_time}")

    sysid = LaunchConfiguration("sysid").perform(context)
    if sysid:
        cmd_args.append(f"--sysid {sysid} ")
        print(f"sysid:            {sysid}")

    # Create action.
    sitl_process = ExecuteProcess(
        cmd=[cmd_args],
        shell=True,
        output="both",
        respawn=False,
    )
    return [sitl_process]


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description for SITL."""
    launch_arguments = generate_launch_arguments()

    return LaunchDescription(
        launch_arguments
        + [
            OpaqueFunction(function=launch_sitl),
        ]
    )


def generate_launch_arguments() -> List[DeclareLaunchArgument]:
    """Generate a list of launch arguments."""
    launch_args = [
        # Required launch arguments.
        DeclareLaunchArgument(
            "command",
            default_value="arducopter",
            description="Run ArduPilot SITL.",
            choices=[
                "antennatracker",
                "arducopter-heli",
                "ardurover",
                "blimp",
                "arducopter",
                "arduplane",
                "ardusub",
            ],
        ),
        DeclareLaunchArgument(
            "model",
            default_value="quad",
            description="Set simulation model.",
        ),
        DeclareLaunchArgument(
            "slave",
            default_value="0",
            description="Set the number of JSON slaves.",
        ),
        DeclareLaunchArgument(
            "sim_address",
            default_value="127.0.0.1",
            description="Set address string for simulator.",
        ),
        DeclareLaunchArgument(
            "speedup",
            default_value="1",
            description="Set simulation speedup.",
        ),
        DeclareLaunchArgument(
            "instance",
            default_value="0",
            description="Set instance of SITL "
            "(adds 10*instance to all port numbers).",
        ),
        DeclareLaunchArgument(
            "defaults",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ardupilot_sitl"),
                    "config",
                    "default_params",
                    "copter.parm",
                ]
            ),
            description="Set path to defaults file.",
        ),
        # Optional launch arguments.
        DeclareLaunchArgument(
            "wipe",
            default_value="False",
            description="Wipe eeprom.",
            choices=["True", "False"],
        ),
        DeclareLaunchArgument(
            "synthetic_clock",
            default_value="False",
            description="Set synthetic clock mode.",
            choices=["True", "False"],
        ),
        DeclareLaunchArgument(
            "home",
            default_value="",
            description="Set start location (lat,lng,alt,yaw) or location name.",
        ),
        DeclareLaunchArgument(
            "rate",
            default_value="",
            description="Set SITL framerate.",
        ),
        DeclareLaunchArgument(
            "gimbal",
            default_value="",
            description="Enable simulated MAVLink gimbal.",
        ),
        DeclareLaunchArgument(
            "base_port",
            default_value="",
            description="Set port num for base port(default 5670) "
            "must be before -I option.",
        ),
        DeclareLaunchArgument(
            "rc_in_port",
            default_value="",
            description="Set port num for rc in.",
        ),
        DeclareLaunchArgument(
            "sim_port_in",
            default_value="",
            description="Set port num for simulator in.",
        ),
        DeclareLaunchArgument(
            "sim_port_out",
            default_value="",
            description="Set port num for simulator out.",
        ),
        DeclareLaunchArgument(
            "irlock_port",
            default_value="",
            description="Set port num for irlock.",
        ),
        DeclareLaunchArgument(
            "start_time",
            default_value="",
            description="Set simulation start time in UNIX timestamp.",
        ),
        DeclareLaunchArgument(
            "sysid",
            default_value="",
            description="Set SYSID_THISMAV.",
        ),
    ]

    # UART launch arguments.
    uart_args = []
    for label in uart_labels:
        arg = DeclareLaunchArgument(
            f"uart{label}",
            default_value="",
            description=f"set device string for UART{label}.",
        )
        uart_args.append(arg)

    # Serial launch arguments.
    serial_args = []
    for label in range(max_serial_ports):
        arg = DeclareLaunchArgument(
            f"serial{label}",
            default_value="",
            description=f"set device string for SERIAL{label}.",
        )
        serial_args.append(arg)

    return launch_args + uart_args + serial_args
