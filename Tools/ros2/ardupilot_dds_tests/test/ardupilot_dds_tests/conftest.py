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

# flake8: noqa

"""Common fixtures."""
import os
import sys
import pytest

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import LaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessIO

from pathlib import Path

from ardupilot_sitl.launch import VirtualPortsLaunch
from ardupilot_sitl.launch import MicroRosAgentLaunch
from ardupilot_sitl.launch import MAVProxyLaunch
from ardupilot_sitl.launch import SITLLaunch

# ---------------------------------------------------------------------------
# Debug helpers — log every byte of IO from every launched process so that
# CI logs show exactly what each process is outputting and whether the
# trigger strings ever appear.  Output goes to stderr so colcon captures it
# even when a test hangs and times out.
# ---------------------------------------------------------------------------


def _io_logger(stream_label):
    """Return an OnProcessIO callback that dumps raw output to stderr."""

    def handler(info):
        # info.action is the ExecuteProcess/Node that emitted the text.
        # Use .name if available (Node), otherwise the class name.
        action = info.action
        name = getattr(action, "name", None) or type(action).__name__
        for line in info.text.splitlines(keepends=True):
            sys.stderr.write(f"[dds-io] {name} {stream_label}: {line!r}\n")
        sys.stderr.flush()
        return []

    return handler


_log_stdout = _io_logger("stdout")
_log_stderr = _io_logger("stderr")


@pytest.fixture(scope="function")
def device_dir(tmp_path_factory):
    """Fixture to create a temporary directory for devices."""
    path = tmp_path_factory.mktemp("devices")

    # Create dev directory for virtual ports.
    os.mkdir(Path(path, "dev"))

    yield path


@pytest.fixture(scope="function")
def virtual_ports(device_dir):
    """Fixture to create virtual ports."""
    tty0 = Path(device_dir, "dev", "tty0").resolve()
    tty1 = Path(device_dir, "dev", "tty1").resolve()

    vp_ld, vp_actions = VirtualPortsLaunch.generate_launch_description_with_actions()

    ld = IncludeLaunchDescription(
        LaunchDescriptionSource(vp_ld),
        launch_arguments={
            "tty0": str(tty0),
            "tty1": str(tty1),
        }.items(),
    )
    yield ld, vp_actions


@pytest.fixture(scope="function")
def micro_ros_agent_serial(device_dir):
    """Fixture to create a micro_ros_agent node."""
    tty0 = Path(device_dir, "dev", "tty0").resolve()

    mra_ld, mra_actions = MicroRosAgentLaunch.generate_launch_description_with_actions()

    ld = IncludeLaunchDescription(
        LaunchDescriptionSource(mra_ld),
        launch_arguments={
            "transport": "serial",
            "baudrate": "115200",
            "device": str(tty0),
        }.items(),
    )
    yield ld, mra_actions


@pytest.fixture(scope="function")
def micro_ros_agent_udp():
    """Fixture to create a micro_ros_agent node."""
    mra_ld, mra_actions = MicroRosAgentLaunch.generate_launch_description_with_actions()

    ld = IncludeLaunchDescription(
        LaunchDescriptionSource(mra_ld),
        launch_arguments={
            "transport": "udp4",
        }.items(),
    )
    yield ld, mra_actions


@pytest.fixture(scope="function")
def mavproxy():
    """Fixture to bring up MAVProxy."""
    mp_ld, mp_actions = MAVProxyLaunch.generate_launch_description_with_actions()

    ld = IncludeLaunchDescription(
        LaunchDescriptionSource(mp_ld),
        launch_arguments={
            "master": "tcp:127.0.0.1:5760",
            "sitl": "127.0.0.1:5501",
        }.items(),
    )
    yield ld, mp_actions


@pytest.fixture(scope="function")
def sitl_copter_dds_serial(device_dir, virtual_ports, micro_ros_agent_serial, mavproxy):
    """
    Fixture to bring up ArduPilot SITL DDS over serial with sequenced startup.

    Process startup is serialised to avoid the race where ArduPilot sends its
    DDS CREATE_PARTICIPANT request before micro_ros_agent is listening on the
    virtual serial port:

      socat  --[stderr "starting data transfer loop"]--> micro_ros_agent
      micro_ros_agent  --[stdout/stderr "running..."]--> SITL

    mavproxy is started immediately alongside socat; it handles reconnection so
    it does not need to wait for SITL to be up first.
    """
    tty1 = Path(device_dir, "dev", "tty1").resolve()

    vp_ld, vp_actions = virtual_ports
    mra_ld, mra_actions = micro_ros_agent_serial
    mp_ld, mp_actions = mavproxy
    sitl_ld, sitl_actions = SITLLaunch.generate_launch_description_with_actions()

    sitl_ld_args = IncludeLaunchDescription(
        LaunchDescriptionSource(sitl_ld),
        launch_arguments={
            "command": "arducopter",
            "synthetic_clock": "True",
            # "wipe": "True",
            "wipe": "False",
            "model": "quad",
            "speedup": "10",
            "slave": "0",
            "instance": "0",
            "serial1": f"uart:{str(tty1)}",
            "defaults": str(
                Path(
                    get_package_share_directory("ardupilot_sitl"),
                    "config",
                    "default_params",
                    "copter.parm",
                )
            )
            + ","
            + str(
                Path(
                    get_package_share_directory("ardupilot_sitl"),
                    "config",
                    "default_params",
                    "dds_serial.parm",
                )
            ),
        }.items(),
    )

    # Start micro_ros_agent only once socat has created the PTY pair.
    # socat prints "starting data transfer loop" to stderr when both ends of the
    # PTY are open, which guarantees tty0 and tty1 symlinks exist.
    _mra_started = [False]

    def on_socat_stderr(info):
        sys.stderr.write(
            f"[dds-trigger] socat stderr check: mra_started={_mra_started[0]}"
            f" action_set={vp_actions['virtual_ports'].action is not None}"
            f" action_match={info.action is vp_actions['virtual_ports'].action}"
            f" text={info.text!r}\n"
        )
        sys.stderr.flush()
        if (
            not _mra_started[0]
            and vp_actions["virtual_ports"].action is not None
            and info.action is vp_actions["virtual_ports"].action
            and b"starting data transfer loop" in info.text
        ):
            sys.stderr.write("[dds-trigger] socat ready — launching micro_ros_agent\n")
            sys.stderr.flush()
            _mra_started[0] = True
            return [mra_ld]
        return []

    # Start SITL only once micro_ros_agent has opened the serial device.
    # micro_ros_agent prints "running..." when its Termios transport is ready
    # to accept XRCE-DDS connections, so ArduPilot's CREATE_PARTICIPANT request
    # will reach a listening agent instead of racing against startup.
    _sitl_started = [False]

    def on_mra_output(info):
        sys.stderr.write(
            f"[dds-trigger] mra output check: sitl_started={_sitl_started[0]}"
            f" action_set={mra_actions['micro_ros_agent'].action is not None}"
            f" action_match={info.action is mra_actions['micro_ros_agent'].action}"
            f" text={info.text!r}\n"
        )
        sys.stderr.flush()
        if (
            not _sitl_started[0]
            and mra_actions["micro_ros_agent"].action is not None
            and info.action is mra_actions["micro_ros_agent"].action
            and b"running..." in info.text
        ):
            sys.stderr.write("[dds-trigger] micro_ros_agent ready — launching SITL\n")
            sys.stderr.flush()
            _sitl_started[0] = True
            return [sitl_ld_args]
        return []

    ld = LaunchDescription(
        [
            vp_ld,
            mp_ld,
            # Log all process IO unconditionally so CI output shows exactly
            # what each process is emitting and whether trigger strings appear.
            RegisterEventHandler(OnProcessIO(on_stdout=_log_stdout, on_stderr=_log_stderr)),
            RegisterEventHandler(OnProcessIO(on_stderr=on_socat_stderr)),
            RegisterEventHandler(
                OnProcessIO(on_stdout=on_mra_output, on_stderr=on_mra_output)
            ),
        ]
    )
    actions = {}
    actions.update(vp_actions)
    actions.update(mra_actions)
    actions.update(mp_actions)
    actions.update(sitl_actions)
    yield ld, actions


@pytest.fixture(scope="function")
def sitl_copter_dds_udp(micro_ros_agent_udp, mavproxy):
    """Fixture to bring up ArduPilot SITL DDS."""
    mra_ld, mra_actions = micro_ros_agent_udp
    mp_ld, mp_actions = mavproxy
    sitl_ld, sitl_actions = SITLLaunch.generate_launch_description_with_actions()

    sitl_ld_args = IncludeLaunchDescription(
        LaunchDescriptionSource(sitl_ld),
        launch_arguments={
            "command": "arducopter",
            "synthetic_clock": "True",
            # "wipe": "True",
            "wipe": "False",
            "model": "quad",
            "speedup": "10",
            "slave": "0",
            "instance": "0",
            "defaults": str(
                Path(
                    get_package_share_directory("ardupilot_sitl"),
                    "config",
                    "default_params",
                    "copter.parm",
                )
            )
            + ","
            + str(
                Path(
                    get_package_share_directory("ardupilot_sitl"),
                    "config",
                    "default_params",
                    "dds_udp.parm",
                )
            ),
        }.items(),
    )

    ld = LaunchDescription(
        [
            mra_ld,
            mp_ld,
            sitl_ld_args,
        ]
    )
    actions = {}
    actions.update(mra_actions)
    actions.update(mp_actions)
    actions.update(sitl_actions)
    yield ld, actions


@pytest.fixture(scope="function")
def sitl_copter_dds_udp_use_ns(micro_ros_agent_udp, mavproxy):
    """Fixture to bring up ArduPilot SITL DDS."""
    mra_ld, mra_actions = micro_ros_agent_udp
    mp_ld, mp_actions = mavproxy
    sitl_ld, sitl_actions = SITLLaunch.generate_launch_description_with_actions()

    sitl_ld_args = IncludeLaunchDescription(
        LaunchDescriptionSource(sitl_ld),
        launch_arguments={
            "command": "arducopter",
            "synthetic_clock": "True",
            # "wipe": "True",
            "wipe": "False",
            "model": "quad",
            "speedup": "10",
            "slave": "0",
            "instance": "0",
            "defaults": str(
                Path(
                    get_package_share_directory("ardupilot_sitl"),
                    "config",
                    "default_params",
                    "copter.parm",
                )
            )
            + ","
            + str(
                Path(
                    get_package_share_directory("ardupilot_sitl"),
                    "config",
                    "default_params",
                    "dds_udp.parm",
                )
            )
            + ","
            + str(
                Path(
                    get_package_share_directory("ardupilot_sitl"),
                    "config",
                    "default_params",
                    "dds_use_ns.parm",
                )
            ),
        }.items(),
    )

    ld = LaunchDescription(
        [
            mra_ld,
            mp_ld,
            sitl_ld_args,
        ]
    )
    actions = {}
    actions.update(mra_actions)
    actions.update(mp_actions)
    actions.update(sitl_actions)
    yield ld, actions


@pytest.fixture(scope="function")
def sitl_plane_dds_serial(device_dir, virtual_ports, micro_ros_agent_serial, mavproxy):
    """
    Fixture to bring up ArduPilot SITL DDS over serial with sequenced startup.

    See sitl_copter_dds_serial for the rationale behind the sequencing.
    """
    tty1 = Path(device_dir, "dev", "tty1").resolve()

    vp_ld, vp_actions = virtual_ports
    mra_ld, mra_actions = micro_ros_agent_serial
    mp_ld, mp_actions = mavproxy
    sitl_ld, sitl_actions = SITLLaunch.generate_launch_description_with_actions()

    sitl_ld_args = IncludeLaunchDescription(
        LaunchDescriptionSource(sitl_ld),
        launch_arguments={
            "command": "arduplane",
            "synthetic_clock": "True",
            # "wipe": "True",
            "wipe": "False",
            "model": "plane",
            "speedup": "10",
            "slave": "0",
            "instance": "0",
            "serial1": f"uart:{str(tty1)}",
            "defaults": str(
                Path(
                    get_package_share_directory("ardupilot_sitl"),
                    "config",
                    "models",
                    "plane.parm",
                )
            )
            + ","
            + str(
                Path(
                    get_package_share_directory("ardupilot_sitl"),
                    "config",
                    "default_params",
                    "dds_serial.parm",
                )
            ),
        }.items(),
    )

    _mra_started = [False]

    def on_socat_stderr(info):
        sys.stderr.write(
            f"[dds-trigger] socat stderr check: mra_started={_mra_started[0]}"
            f" action_set={vp_actions['virtual_ports'].action is not None}"
            f" action_match={info.action is vp_actions['virtual_ports'].action}"
            f" text={info.text!r}\n"
        )
        sys.stderr.flush()
        if (
            not _mra_started[0]
            and vp_actions["virtual_ports"].action is not None
            and info.action is vp_actions["virtual_ports"].action
            and b"starting data transfer loop" in info.text
        ):
            sys.stderr.write("[dds-trigger] socat ready — launching micro_ros_agent\n")
            sys.stderr.flush()
            _mra_started[0] = True
            return [mra_ld]
        return []

    _sitl_started = [False]

    def on_mra_output(info):
        sys.stderr.write(
            f"[dds-trigger] mra output check: sitl_started={_sitl_started[0]}"
            f" action_set={mra_actions['micro_ros_agent'].action is not None}"
            f" action_match={info.action is mra_actions['micro_ros_agent'].action}"
            f" text={info.text!r}\n"
        )
        sys.stderr.flush()
        if (
            not _sitl_started[0]
            and mra_actions["micro_ros_agent"].action is not None
            and info.action is mra_actions["micro_ros_agent"].action
            and b"running..." in info.text
        ):
            sys.stderr.write("[dds-trigger] micro_ros_agent ready — launching SITL\n")
            sys.stderr.flush()
            _sitl_started[0] = True
            return [sitl_ld_args]
        return []

    ld = LaunchDescription(
        [
            vp_ld,
            mp_ld,
            # Log all process IO unconditionally so CI output shows exactly
            # what each process is emitting and whether trigger strings appear.
            RegisterEventHandler(OnProcessIO(on_stdout=_log_stdout, on_stderr=_log_stderr)),
            RegisterEventHandler(OnProcessIO(on_stderr=on_socat_stderr)),
            RegisterEventHandler(
                OnProcessIO(on_stdout=on_mra_output, on_stderr=on_mra_output)
            ),
        ]
    )
    actions = {}
    actions.update(vp_actions)
    actions.update(mra_actions)
    actions.update(mp_actions)
    actions.update(sitl_actions)
    yield ld, actions


@pytest.fixture(scope="function")
def sitl_plane_dds_udp(device_dir, virtual_ports, micro_ros_agent_udp, mavproxy):
    """Fixture to bring up ArduPilot SITL DDS."""
    tty1 = Path(device_dir, "dev", "tty1").resolve()

    vp_ld, vp_actions = virtual_ports
    mra_ld, mra_actions = micro_ros_agent_udp
    mp_ld, mp_actions = mavproxy
    sitl_ld, sitl_actions = SITLLaunch.generate_launch_description_with_actions()

    sitl_ld_args = IncludeLaunchDescription(
        LaunchDescriptionSource(sitl_ld),
        launch_arguments={
            "command": "arduplane",
            "synthetic_clock": "True",
            # "wipe": "True",
            "wipe": "False",
            "model": "plane",
            "speedup": "10",
            "slave": "0",
            "instance": "0",
            "defaults": str(
                Path(
                    get_package_share_directory("ardupilot_sitl"),
                    "config",
                    "models",
                    "plane.parm",
                )
            )
            + ","
            + str(
                Path(
                    get_package_share_directory("ardupilot_sitl"),
                    "config",
                    "default_params",
                    "dds_udp.parm",
                )
            ),
        }.items(),
    )

    ld = LaunchDescription(
        [
            vp_ld,
            mra_ld,
            mp_ld,
            sitl_ld_args,
        ]
    )
    actions = {}
    actions.update(vp_actions)
    actions.update(mra_actions)
    actions.update(mp_actions)
    actions.update(sitl_actions)
    yield ld, actions
