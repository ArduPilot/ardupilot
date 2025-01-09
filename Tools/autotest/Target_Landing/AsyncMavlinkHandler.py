from typing import Dict, Callable
import time
import threading

from pymavlink import mavutil

class AsyncMavlinkHandler:

    def __init__(self, mav):
        """
        Initialize mavlink message handler

        Args:
            mavlink object
        """

        self.mav = mav
        self.message_handlers: Dict[str, Callable] = {}
        self.running = False
        self.receive_thread = None

    def register_handlers(self, handlers):

        for handler in handlers:
            self._register_handler(handler["message_name"], handler["handler"])

    def _register_handler(self, message_type: str, handler):
        """Register a handler function for a specific message type"""

        self.message_handlers[message_type] = handler

    def _receive_loop(self):
        """Internal message receiving loop to run in separate thread"""
        while self.running:
            gps_msg = self.mav.recv_match(type="GLOBAL_POSITION_INT")
            self.handle_msg_response(gps_msg)

            att_msg = self.mav.recv_match(type="ATTITUDE")
            self.handle_msg_response(att_msg)

            batt_msg = self.mav.recv_match(type="BATTERY_STATUS")
            self.handle_msg_response(batt_msg)
            # if msg is not None:
            #     msg_type = msg.get_type()
                
            #     if msg_type == "BAD_DATA":
            #         print(msg)

            #     if msg_type in self.message_handlers:
            #         try:
            #             self.message_handlers[msg_type](msg)
            #         except Exception as e:
            #             print(f"Error handling message {msg_type}: {e}")
            
            time.sleep(0.01)

    def handle_msg_response(self,msg):
        if msg is not None:
            msg_type = msg.get_type()
            
            if msg_type == "BAD_DATA":
                print(msg)

            if msg_type in self.message_handlers:
                try:
                    self.message_handlers[msg_type](msg)
                except Exception as e:
                    print(f"Error handling message {msg_type}: {e}")

    def start(self):
        """Start the message receiving thread"""
        if self.receive_thread is None or not self.receive_thread.is_alive():
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop)
            self.receive_thread.daemon = True  # Thread will close when main program exits
            self.receive_thread.start()
            print("MAVLink receiver thread started")
        
            
    def stop(self):
        """Stop the message receiving thread"""
        self.running = False
        if self.receive_thread is not None:
            self.receive_thread.join(timeout=1.0)  # Wait up to 1 second for thread to finish
            print("MAVLink receiver thread stopped")

    def send_message(self, message_type: str, **kwargs):
        """Send a MAVLink message"""
        try:
            self.mav.send(
                mavutil.mavlink.MAVLink_message(message_type, **kwargs)
            )
        except Exception as e:
            print(f"Error sending message {message_type}: {e}")

class MAVMessages:

    drone_gps_pos = "GLOBAL_POSITION_INT"
    attitude = "ATTITUDE"
    battery_status = "BATTERY_STATUS"