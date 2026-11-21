import unittest

from actuator_frames import encode_frame
from rcoutput_mapping import load_motor_order, motors_in_esp_order, parse_observation


class RCOutputMappingTests(unittest.TestCase):
    def test_native_forward_output_maps_to_provisional_esp_ids(self):
        values = [1550, 1550, 1450, 1450, 1500, 1500, 1500, 1500] + [0] * 8
        armed, channels = parse_observation(("ATAL_RCOUT,1,1," + ",".join(map(str, values))).encode())
        self.assertTrue(armed)
        mapped = motors_in_esp_order(channels, load_motor_order())
        self.assertEqual(mapped, (1450, 1500, 1450, 1500, 1500, 1550, 1550, 1500))
        self.assertIn("," + ",".join(map(str, mapped)) + ",", encode_frame("PWM", 0xAABBCCDD, 2, mapped))

    def test_incomplete_and_unsafe_observations_fail_closed(self):
        good = b"ATAL_RCOUT,1,1," + b",".join([b"1500"] * 16)
        for packet in (good[:-1], good.replace(b",1500", b",1900", 1),
                       good.replace(b"RCOUT", b"OUTPUT"), good + b",0",
                       good.replace(b",1,1,", b",1,2,", 1)):
            with self.subTest(packet=packet):
                with self.assertRaises(ValueError):
                    parse_observation(packet)


if __name__ == "__main__":
    unittest.main()
