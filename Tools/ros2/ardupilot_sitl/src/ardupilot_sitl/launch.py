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

"""Launch actions for ArduPilot."""
from typing import List
from typing import Dict
from typing import Text
from typing import Tuple

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from .actions import ExecuteFunction

TRUE_STRING = "True"
FALSE_STRING = "False"
BOOL_STRING_CHOICES = set([TRUE_STRING, FALSE_STRING])

class VirtualPortsLaunch:
    """Launch functions for creating virtual ports using `socat`."""

    @staticmethod
    def generate_action(context: LaunchContext, *args, **kwargs) -> ExecuteProcess:
        """Generate a launch action."""
        # Declare commands.
        command = "socat"

        # Retrieve launch arguments.
        tty0 = LaunchConfiguration("tty0").perform(context)
        tty1 = LaunchConfiguration("tty1").perform(context)

        # Display launch arguments.
        print(f"command:          {command}")
        print(f"tty0:             {tty0}")
        print(f"tty1:             {tty1}")

        # Create action.
        action = ExecuteProcess(
            cmd=[
                [
                    "socat ",
                    "-d -d ",
                    f"pty,raw,echo=0,link={tty0} ",
                    f"pty,raw,echo=0,link={tty1} ",
                ]
            ],
            shell=True,
            output="both",
            respawn=False,
            cached_output=True,
        )
        return action

    @staticmethod
    def generate_launch_description_with_actions() -> Tuple[LaunchDescription, Dict[Text, ExecuteFunction]]:
        """Generate a launch description with actions."""
        launch_arguments = VirtualPortsLaunch.generate_launch_arguments()

        action = ExecuteFunction(function=VirtualPortsLaunch.generate_action)

        ld = LaunchDescription(
            launch_arguments
            + [
                action,
            ]
        )
        actions = {
            "virtual_ports": action,
        }
        return ld, actions

    @staticmethod
    def generate_launch_description() -> LaunchDescription:
        """Generate a launch description."""
        ld, _ = VirtualPortsLaunch.generate_launch_description_with_actions()
        return ld

    @staticmethod
    def generate_launch_arguments() -> List[DeclareLaunchArgument]:
        """Generate a list of launch arguments."""
        return [
            DeclareLaunchArgument(
                "tty0",
                default_value="tty0",
                description="Symbolic link name for tty0.",
            ),
            DeclareLaunchArgument(
                "tty1",
                default_value="tty1",
                description="Symbolic link name for tty1.",
            ),
        ]


class MicroRosAgentLaunch:
    """Launch functions for the micro ROS agent node."""

    @staticmethod
    def generate_action(context: LaunchContext, *args, **kwargs) -> ExecuteProcess:
        """Launch the micro_ros_agent node."""
        # ROS arguments.
        micro_ros_agent_ns = LaunchConfiguration("micro_ros_agent_ns").perform(context)

        # Common arguments.
        transport = LaunchConfiguration("transport").perform(context)
        middleware = LaunchConfiguration("middleware").perform(context)
        verbose = LaunchConfiguration("verbose").perform(context)
        discovery = LaunchConfiguration("discovery").perform(context)

        # IPvX arguments.
        port = LaunchConfiguration("port").perform(context)

        # Serial arguments.
        device = LaunchConfiguration("device").perform(context)
        baudrate = LaunchConfiguration("baudrate").perform(context)

        # Display launch arguments.
        print(f"namespace:        {micro_ros_agent_ns}")
        print(f"transport:        {transport}")
        print(f"middleware:       {middleware}")
        print(f"verbose:          {verbose}")
        print(f"discovery:        {discovery}")

        # Required arguments
        args = [
            transport,
            "--middleware",
            middleware,
            "--verbose",
            verbose,
        ]

        if transport in ["udp4", "udp6", "tcp4", "tcp6"]:
            # IPvX arguments
            port = LaunchConfiguration("port").perform(context)
            args.append("--port")
            args.append(port)
            print(f"port:             {port}")
        elif transport in ["serial", "multiserial", "pseudoterminal"]:
            # Serial arguments
            baudrate = LaunchConfiguration("baudrate").perform(context)
            args.append("--baudrate")
            args.append(baudrate)
            print(f"baudrate:         {baudrate}")

            device = LaunchConfiguration("device").perform(context)
            args.append("--dev")
            args.append(device)
            print(f"device:           {device}")
        else:
            # transport must be canfd
            pass

        # Optional arguments.
        refs = LaunchConfiguration("refs").perform(context)
        if refs:
            args.append("--refs")
            args.append(refs)
            print(f"refs:             {refs}")

        # Create action.
        node = Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent",
            namespace=f"{micro_ros_agent_ns}",
            output="both",
            arguments=args,
        )
        return node

    @staticmethod
    def generate_launch_description_with_actions() -> Tuple[LaunchDescription, Dict[Text, ExecuteFunction]]:
        """Generate a launch description with actions."""
        launch_arguments = MicroRosAgentLaunch.generate_launch_arguments()

        action = ExecuteFunction(function=MicroRosAgentLaunch.generate_action)

        ld = LaunchDescription(
            launch_arguments
            + [
                action,
            ]
        )
        actions = {
            "micro_ros_agent": action,
        }
        return ld, actions

    @staticmethod
    def generate_launch_description() -> LaunchDescription:
        """Generate a launch description."""
        ld, _ = MicroRosAgentLaunch.generate_launch_description_with_actions()
        return ld

    @staticmethod
    def generate_launch_arguments() -> List[DeclareLaunchArgument]:
        """Generate a list of launch arguments."""
        return [
            DeclareLaunchArgument(
                "micro_ros_agent_ns",
                default_value="",
                description="Set the micro_ros_agent namespace.",
            ),
            DeclareLaunchArgument(
                "transport",
                default_value="udp4",
                description="Set the transport.",
                choices=[
                    "udp4",
                    "udp6",
                    "tcp4",
                    "tcp6",
                    "canfd",
                    "serial",
                    "multiserial",
                    "pseudoterminal",
                ],
            ),
            DeclareLaunchArgument(
                "middleware",
                default_value="dds",
                description="Set the middleware.",
                choices=["ced", "dds", "rtps"],
            ),
            DeclareLaunchArgument(
                "refs",
                default_value="",
                description="Set the refs file.",
            ),
            DeclareLaunchArgument(
                "verbose",
                default_value="4",
                description="Set the verbosity level.",
                choices=["0", "1", "2", "3", "4", "5", "6"],
            ),
            DeclareLaunchArgument(
                "discovery",
                default_value="7400",
                description="Set the dsicovery port.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="2019",
                description="Set the port number.",
            ),
            DeclareLaunchArgument(
                "baudrate",
                default_value="115200",
                description="Set the baudrate.",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="",
                description="Set the device.",
            ),
        ]


class MAVProxyLaunch:
    """Launch functions for MAVProxy."""

    @staticmethod
    def generate_action(context: LaunchContext, *args, **kwargs) -> ExecuteProcess:
        """Return a non-interactive MAVProxy process."""
        # Declare the command.
        command = "mavproxy.py"

        # Retrieve launch arguments.
        master = LaunchConfiguration("master").perform(context)
        out = LaunchConfiguration("out").perform(context)
        sitl = LaunchConfiguration("sitl").perform(context)
        console = LaunchConfiguration("console").perform(context)
        map = LaunchConfiguration("map").perform(context)

        # Display launch arguments.
        print(f"command:          {command}")
        print(f"master:           {master}")
        print(f"sitl:             {sitl}")
        print(f"out:              {out}")
        print(f"console:          {console}")
        print(f"map:              {map}")

        cmd = [
            f"{command} ",
            f"--out {out} ",
            "--out ",
            "127.0.0.1:14551 ",
            f"--master {master} ",
            f"--sitl {sitl} ",
            "--non-interactive ",
        ]

        if console == TRUE_STRING:
            cmd.append("--console ")

        if map == TRUE_STRING:
            cmd.append("--map ")

        # Create action.
        mavproxy_process = ExecuteProcess(
            cmd=cmd,
            shell=True,
            output="both",
            respawn=False,
        )
        return mavproxy_process

    @staticmethod
    def generate_launch_description_with_actions() -> Tuple[LaunchDescription, Dict[Text, ExecuteFunction]]:
        """Generate a launch description for MAVProxy."""
        launch_arguments = MAVProxyLaunch.generate_launch_arguments()

        action = ExecuteFunction(function=MAVProxyLaunch.generate_action)

        ld = LaunchDescription(
            launch_arguments
            + [
                action,
            ]
        )
        actions = {
            "mavproxy": action,
        }
        return ld, actions

    @staticmethod
    def generate_launch_description() -> LaunchDescription:
        """Generate a launch description."""
        ld, _ = MAVProxyLaunch.generate_launch_description_with_actions()
        return ld

    @staticmethod
    def generate_launch_arguments() -> List[DeclareLaunchArgument]:
        """Generate a list of launch arguments."""
        return [
            DeclareLaunchArgument(
                "master",
                default_value="tcp:127.0.0.1:5760",
                description="MAVLink master port and optional baud rate.",
            ),
            DeclareLaunchArgument(
                "out",
                default_value="127.0.0.1:14550",
                description="MAVLink output port and optional baud rate.",
            ),
            DeclareLaunchArgument(
                "sitl",
                default_value="127.0.0.1:5501",
                description="SITL output port.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="False",
                description="Enable MAVProxy Map.",
                choices=BOOL_STRING_CHOICES
            ),
            DeclareLaunchArgument(
                "console",
                default_value="False",
                description="Enable MAVProxy Console.",
                choices=BOOL_STRING_CHOICES
            ),
        ]


class SITLLaunch:
    """Launch functions for ArduPilot SITL."""

    # Labels for the optional uart launch arguments.
    UART_LABELS = ["A", "B", "C", "D", "E", "F", "H", "I", "J"]
    MAX_SERIAL_PORTS = 10

    @staticmethod
    def generate_action(context: LaunchContext, *args, **kwargs) -> ExecuteProcess:
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
        if wipe == TRUE_STRING:
            cmd_args.append("--wipe ")
            print(f"wipe:             {wipe}")

        synthetic_clock = LaunchConfiguration("synthetic_clock").perform(context)
        if synthetic_clock == TRUE_STRING:
            cmd_args.append("--synthetic-clock ")
            print(f"synthetic_clock:  {synthetic_clock}")

        home = LaunchConfiguration("home").perform(context)
        if home:
            cmd_args.append(f"--home {home} ")
            print(f"home:             {home}")

        # Optional uart arguments.
        for label in SITLLaunch.UART_LABELS:
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
        return sitl_process

    @staticmethod
    def generate_launch_description_with_actions() -> Tuple[LaunchDescription, Dict[Text, ExecuteFunction]]:
        """Generate a launch description for SITL."""
        launch_arguments = SITLLaunch.generate_launch_arguments()

        action = ExecuteFunction(function=SITLLaunch.generate_action)

        ld = LaunchDescription(
            launch_arguments
            + [
                action,
            ]
        )
        actions = {
            "sitl": action,
        }
        return ld, actions

    @staticmethod
    def generate_launch_description() -> LaunchDescription:
        """Generate a launch description."""
        ld, _ = SITLLaunch.generate_launch_description_with_actions()
        return ld

    @staticmethod
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
                description="Set instance of SITL " "(adds 10*instance to all port numbers).",
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
                choices=BOOL_STRING_CHOICES,
            ),
            DeclareLaunchArgument(
                "synthetic_clock",
                default_value="False",
                description="Set synthetic clock mode.",
                choices=BOOL_STRING_CHOICES,
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
                description="Set port num for base port(default 5670) " "must be before -I option.",
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
        for label in SITLLaunch.UART_LABELS:
            arg = DeclareLaunchArgument(
                f"uart{label}",
                default_value="",
                description=f"set device string for UART{label}.",
            )
            uart_args.append(arg)

        # Serial launch arguments.
        serial_args = []
        for label in range(SITLLaunch.MAX_SERIAL_PORTS):
            arg = DeclareLaunchArgument(
                f"serial{label}",
                default_value="",
                description=f"set device string for SERIAL{label}.",
            )
            serial_args.append(arg)

        return launch_args + uart_args + serial_args
