import cv2
import socket
import struct
import threading
import math
from pymavlink import mavutil
import time

class GimbalControl:
    def __init__(self, connection_str):
        self.master = mavutil.mavlink_connection(connection_str)
        self.master.wait_heartbeat()
        print("Heartbeat from the system (system %u component %u)" % 
              (self.master.target_system, self.master.target_component))
        self.center_x = 0
        self.center_y = 0
        self.lock = threading.Lock()  # Initialize a lock for shared variables
        self.control_thread = threading.Thread(target=self.send_command)
        self.control_thread.start()

    def send_gimbal_manager_pitch_yaw_angles(self, pitch, yaw, pitch_rate, yaw_rate):
        msg = self.master.mav.gimbal_manager_set_pitchyaw_encode(
            self.master.target_system,
            self.master.target_component,
            0,
            0,
            pitch,
            yaw,
            pitch_rate,
            yaw_rate
        )
        self.master.mav.send(msg)

    def send_command(self):
        while True:
            start_time = time.time()  # Record the start time of the loop
            
            with self.lock:  # Lock when accessing shared variables
                centre_x_copy = int(self.center_x)
                centre_y_copy = int(self.center_y)

            if (centre_x_copy == 0 and centre_y_copy == 0):
                diff_x = 0
                diff_y = 0
            else:
                diff_x = (centre_x_copy - (640 / 2)) / 2
                diff_y = -(centre_y_copy - (480 / 2)) / 2

            self.send_gimbal_manager_pitch_yaw_angles(float("NaN"), float("NaN"), math.radians(diff_y), math.radians(diff_x))

            # 50Hz
            elapsed_time = time.time() - start_time
            sleep_time = max(0, 0.02 - elapsed_time)  # Ensure no negative sleep time
            time.sleep(sleep_time)

    def update_center(self, x, y):
        with self.lock:  # Lock when updating shared variables
            self.center_x = x
            self.center_y = y


class VideoStreamer:
    def __init__(self, input_port, output_port):
        self.input_port = input_port
        self.output_port = output_port
        self.cap = self.initialize_video_capture()
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.out = self.initialize_video_writer()
        self.frame = None

        self.tracker = cv2.legacy.TrackerCSRT_create()  # Change to legacy namespace
        self.initBB = None
        self.last_change_time = 0

    def initialize_video_capture(self):
        cap = cv2.VideoCapture(
            f'udpsrc port={self.input_port} ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert ! appsink',
            cv2.CAP_GSTREAMER
        )
        if not cap.isOpened():
            print("Error: Could not open video stream.")
            exit()
        return cap

    def initialize_video_writer(self):
        out = cv2.VideoWriter(
            f'appsrc ! videoconvert ! x264enc speed-preset=ultrafast tune=zerolatency ! rtph264pay config-interval=1 ! udpsink host=127.0.0.1 port={self.output_port}',
            cv2.CAP_GSTREAMER,
            0,
            30,
            (self.frame_width, self.frame_height)
        )
        if not out.isOpened():
            print("Error: Could not open video writer.")
            self.cap.release()
            exit()
        return out

    def update_tracker(self, new_roi):
        self.initBB = new_roi
        self.tracker = cv2.legacy.TrackerCSRT_create()  # Change to legacy namespace
        if self.frame is not None:
            self.tracker.init(self.frame, new_roi)

    def process_frame(self, gimbal_control):
        while True:
            ret, self.frame = self.cap.read()
            if not ret:
                print("Error: Unable to read frame from video stream.")
                break

            # If a bounding box exists, update the tracker and draw the rectangle
            if self.initBB is not None:
                (success, box) = self.tracker.update(self.frame)
                if success:
                    (x, y, w, h) = [int(v) for v in box]
                    center_x = x + (w // 2)
                    center_y = y + (h // 2)
                    gimbal_control.update_center(center_x, center_y)
                    cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    self.last_change_time = time.time()
                else:
                    print("Tracking failure detected, reinitializing tracker.")
                    self.reinitialize_tracker(gimbal_control)

            # Write the frame to the RTSP stream
            self.out.write(self.frame)

            # Display the frame
            cv2.imshow("Frame", self.frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        self.cleanup()

    def reinitialize_tracker(self, gimbal_control):
        """Reinitialize tracker on loss."""
        gimbal_control.update_center(0, 0)
        self.initBB = None  # Reset the bounding box
        # Optionally, use object detection to find the new location of the object and update tracker

        # Example: Use an object detection method to reinitialize the bounding box
        # detection_box = self.object_detection_method(self.frame)
        # if detection_box:
        #     self.update_tracker(detection_box)

    def cleanup(self):
        self.cap.release()
        self.out.release()
        cv2.destroyAllWindows()


class UDPReceiver:
    def __init__(self, gimbal_control, video_streamer, port=14580):
        self.gimbal_control = gimbal_control
        self.video_streamer = video_streamer
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', port))
        self.udp_thread = threading.Thread(target=self.receive_udp_data)
        self.udp_thread.daemon = True
        self.udp_thread.start()

    def receive_udp_data(self):
        while True:
            data, addr = self.sock.recvfrom(16)
            x, y, w, h = struct.unpack('!ffff', data)
            print(f"Received new tracking coordinates: x={x}, y={y}, w={w}, h={h}")
            # Convert normalized coordinates to pixel values
            x, y, w, h = int(x * self.video_streamer.frame_width), int(y * self.video_streamer.frame_height), int(w * self.video_streamer.frame_width), int(h * self.video_streamer.frame_height)
            self.video_streamer.update_tracker((x, y, w-x, h-y))


def main():
    gimbal_control = GimbalControl('127.0.0.1:14570')
    video_streamer = VideoStreamer(input_port=5600, output_port=5700)
    udp_receiver = UDPReceiver(gimbal_control, video_streamer)

    video_streamer.process_frame(gimbal_control)

if __name__ == "__main__":
    main()