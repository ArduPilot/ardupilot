import numpy as np

class DataVisualizer:
    """
    A class for converting and visualizing geospatial data positions
    """
    
    def __init__(self, target_state, drone_position):
        """
        Initialize the DataVisualizer with target state and drone position
        
        :param target_state: numpy array of target positions and velocities
        :param drone_position: numpy array of drone position
        """
        self.R = 6378137.0  # Radius of earth in meters
        
        self.target_position = target_state[:, 0:3]
        self.target_velocity = target_state[:, 3:6]
        self.drone_position = drone_position
        
        # Choose target's initial position as origin
        self.origin = self.target_position[0, :]

    def get_drone_pos_neu_m(self):
        """
        Get drone position in North-East-Up (NEU) coordinates
        
        :return: numpy array of drone position in meters
        """
        return self._get_position_NEU(self.drone_position)

    def get_target_pos_neu_m(self):
        """
        Get target position in North-East-Up (NEU) coordinates
        
        :return: numpy array of target position in meters
        """
        return self._get_position_NEU(self.target_position)

    def _shift_position_with_origin(self, pos):
        """
        Shifts the position vectors with respect to the origin
        
        :param pos: numpy array of positions (lat, lng, alt)
        :return: shifted positions
        """
        return pos - self.origin

    def _lat_lng_to_meters(self, dlat, dlng):
        """
        Convert latitude and longitude differences to meters
        
        :param dlat: latitude difference in degE7
        :param dlng: longitude difference in degE7
        :return: numpy array of x, y distances in meters
        """
        scaling_factor = 2 * np.pi * self.R / 360
        x = dlat * 1e-7 * scaling_factor
        y = dlng * 1e-7 * (np.cos(np.deg2rad(self.origin[0])) * scaling_factor)
        
        return np.column_stack([x, y])

    def _get_position_NEU(self, pos_deg):
        """
        Convert position vector to North-East-Up coordinates in meters
        
        :param pos_deg: numpy array of positions (lat, lng, alt)
        :return: numpy array of NEU coordinates
        """
        pos_origin_shifted = self._shift_position_with_origin(pos_deg)
        
        pos_neu_m = np.zeros_like(pos_origin_shifted, dtype=float)
        pos_neu_m[:, 0:2] = self._lat_lng_to_meters(pos_origin_shifted[:, 0], pos_origin_shifted[:, 1])
        pos_neu_m[:, 2] = pos_origin_shifted[:, 2]
        
        return pos_neu_m