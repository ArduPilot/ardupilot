import unittest

from actuator_frames import encode_frame


class ActuatorFramesTests(unittest.TestCase):
    def test_wire_format_and_crc(self):
        self.assertEqual(
            encode_frame("ARM", 0x1234ABCD, 3),
            "A1,ARM,1234ABCD,3,A9C1",
        )
        self.assertEqual(len(encode_frame("PWM", 0, 1, [1500] * 8).split(",")), 13)

    def test_invalid_commands_never_encode(self):
        for args in [
            ("ARM", 0, 0, None),
            ("PWM", 0, 1, [1500] * 7),
            ("PWM", 0, 1, [1500] * 7 + [1701]),
            ("ARM", 0, 1, [1500] * 8),
            ("INVALID", 0, 1, None),
        ]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                encode_frame(*args)


if __name__ == "__main__":
    unittest.main()
