import threading
import time
from pymavlink import mavutil
import argparse
import socket
import struct

# Define CAMERA_CAP_FLAGS as constants
CAMERA_CAP_FLAGS_CAPTURE_VIDEO = 1                          # Camera is able to record video
CAMERA_CAP_FLAGS_CAPTURE_IMAGE = 2                          # Camera is able to capture images
CAMERA_CAP_FLAGS_HAS_MODES = 4                              # Camera has separate Video and Image/Photo modes
CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8        # Camera can capture images while in video mode
CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16       # Camera can capture videos while in Photo/Image mode
CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE = 32                 # Camera has image survey mode
CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM = 64                        # Camera has basic zoom control
CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS = 128                      # Camera has basic focus control
CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM = 256                     # Camera has video streaming capabilities
CAMERA_CAP_FLAGS_HAS_TRACKING_POINT = 512                   # Camera supports tracking of a point
CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE = 1024              # Camera supports tracking of a selection rectangle
CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS = 2048             # Camera supports tracking geo status

class CameraTrackingScript:
    def __init__(self, ip, port, sysid, compid, resh, resv):
        self.ip = ip
        self.port = port
        self.sysid = sysid
        self.compid = compid
        self.connection = None
        self.udp_ip = "127.0.0.1"  # Localhost
        self.udp_port = 14580      # Port to send to
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.resh = resh
        self.resv = resv

    def connect_to_mavlink(self):
        self.connection = mavutil.mavlink_connection(f'udp:{self.ip}:{self.port}', source_system=self.sysid)
        print("Searching Vehicle")
        while not self.connection.probably_vehicle_heartbeat(self.connection.wait_heartbeat()):
            print(".", end="")
        print("\nFound Vehicle")
        self.connection.wait_heartbeat()
        print("Heartbeat received from system (system %u component %u)" % (self.connection.target_system, self.connection.target_component))
        self.connection.mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=mavutil.mavlink.MAV_STATE_UNINIT,
            mavlink_version=3
        )

    def send_camera_information(self):
        flags = (
            CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
            CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
            CAMERA_CAP_FLAGS_HAS_MODES |
            CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
            CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
            CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
            CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM |
            CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS |
            CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM |
            CAMERA_CAP_FLAGS_HAS_TRACKING_POINT |
            CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE |
            CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS
        )
        self.connection.mav.camera_information_send(
            int(time.time() * 1000) & 0xFFFFFFFF,          # time_boot_ms
            b"CameraVendor123" + b'\0' * (32 - len("CameraVendor123")),  # vendor_name
            b"CameraModel123" + b'\0' * (32 - len("CameraModel123")),    # model_name
            (1 << 24) | (0 << 16) | (0 << 8) | 1,          # firmware_version
            float('nan'),                                  # focal_length
            float('nan'),                                  # sensor_size_h
            float('nan'),                                  # sensor_size_v
            self.resh,                                     # resolution_h
            self.resv,                                     # resolution_v
            0,                                             # lens_id
            flags,                                         # flags
            0,                                             # cam_definition_version
            b"",                                           # cam_definition_uri
            0                                              # gimbal_device_id
        )
        print("Camera information message sent")

    def handle_camera_track_point(self, msg):
        print("Received MAV_CMD_CAMERA_TRACK_POINT command.")
        # These are already floats
        param1 = msg.param1
        param2 = msg.param2
        print(f"Tracking point parameters: param1={param1}, param2={param2}")

    def handle_camera_track_rectangle(self, msg):
        print("Received MAV_CMD_CAMERA_TRACK_RECTANGLE command.")
        # These should remain as floats (normalized coordinates)
        norm_x = msg.param1
        norm_y = msg.param2
        norm_w = msg.param3
        norm_h = msg.param4
        print(f"Tracking rectangle parameters: norm_x={norm_x}, norm_y={norm_y}, norm_w={norm_w}, norm_h={norm_h}")

        # Send normalized coordinates as floats to the OpenCV script
        self.sock.sendto(struct.pack('!ffff', norm_x, norm_y, norm_w, norm_h), (self.udp_ip, self.udp_port))
        print("Sent normalized tracking coordinates to OpenCV script.")

    def send_heartbeat(self):
        while True:
            self.connection.mav.heartbeat_send(
                type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=mavutil.mavlink.MAV_STATE_UNINIT,
                mavlink_version=3
            )
            time.sleep(1)

    def run(self):
        self.connect_to_mavlink()
        self.send_camera_information()

        # Start the heartbeat thread
        heartbeat_thread = threading.Thread(target=self.send_heartbeat)
        heartbeat_thread.daemon = True
        heartbeat_thread.start()

        while True:
            msg = self.connection.recv_match(type='COMMAND_LONG', blocking=True)
            if msg and msg.get_type() == 'COMMAND_LONG':
                if msg.target_system == self.sysid:
                    if msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT:
                        self.handle_camera_track_point(msg)
                    elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_RECTANGLE:
                        self.handle_camera_track_rectangle(msg)
                else:
                    print("Received but not for us")

def main():
    parser = argparse.ArgumentParser(description="A script to send camera information and Forward track rectangle and track point messages to tracking script")
    parser.add_argument('--sysid', type=int, help='System ID', required=True)
    parser.add_argument('--compid', type=int, help='Component ID', required=True)
    parser.add_argument('--resh', type=int, help='Video Resolution horizontal', required=True)
    parser.add_argument('--resv', type=int, help='Video Resolution vertical', required=True)
    args = parser.parse_args()

    ip = "127.0.0.1"
    port = 14560

    script = CameraTrackingScript(ip, port, args.sysid, args.compid, args.resh, args.resv)
    script.run()

if __name__ == "__main__":
    main()
