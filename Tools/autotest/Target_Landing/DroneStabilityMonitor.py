import os
import csv

class DroneStabilityMonitor:

    PITCH_THRESHOLD = 70*(3.14159/180)
    ROLL_THRESHOLD = 70*(3.14159/180)
    CURRENT_THRESHOLD = 90 # Assuming 90% of max current is saturation

    euler_angles = []
    angular_speed = []

    battery_voltage_percent = 0
    battery_current_amps = 0

    is_limit_exceeded = {
        "roll": False,
        "pitch": False,
    }

    
    def __init__(self, mav):

        self.mav = mav

    def check_attitude_unstable(self):

        msg = self.mav.recv_match(type="ATTITUDE", blocking=True)
        self.euler_angles = [msg.roll, msg.pitch, msg.yaw]
        self.angular_speed = [msg.rollspeed, msg.pitchspeed, msg.yawspeed]

        if abs(msg.roll) >= self.ROLL_THRESHOLD:
            self.is_limit_exceeded['roll'] = True

        if abs(msg.pitch) >= self.PITCH_THRESHOLD:
            self.is_limit_exceeded['pitch'] = True


    def check_current_saturate(self):

        msg = self.mav.recv_match(type="BATTERY_STATUS", blocking=True)
        self.battery_voltage_percent = msg.battery_remaining
        self.battery_current_amps = msg.current_battery/100
    
    def log_stability_data(self, folder_path, test_name):

        mode = 'a'

        data = {
            'roll': self.euler_angles[0],
            'pitch': self.euler_angles[1],
            'yaw': self.euler_angles[2],
            'roll_rate': self.angular_speed[0],
            'pitch_rate': self.angular_speed[1],
            'yaw_rate': self.angular_speed[2],
            'battery_voltage_percent': self.battery_voltage_percent,
            'battery_current_amps': self.battery_current_amps,
            'test_name': test_name
        }

        file_path = os.path.join(folder_path, "stability_info.csv")
        
        with open(file_path, mode, newline='', encoding='utf-8') as csvfile:

            writer = csv.DictWriter(csvfile,
                                    fieldnames=data.keys(),
                                    delimiter=",")
            if not os.path.exists(file_path):
                writer.writeheader()
            writer.writerow(data)


    def is_any_limit_exceeded(self):

        return self.is_limit_exceeded["roll"] or self.is_limit_exceeded["pitch"]

    def monitor(self):
        """Main monitoring loop"""

        self.check_attitude_unstable()
        self.check_current_saturate()

        return 
    


