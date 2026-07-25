#!/usr/bin/env python3

import time
from pymavlink import mavutil

master = mavutil.mavlink_connection("udp::14555", input=False, dialect="array_test")
while True:
    master.mav.system_time_send(1,2)
    master.mav.array_test_0_send(1, [-3, -2, -1, 0], [1,2,3,4], [5,6,7,8], [9,10,11,12])
    master.mav.array_test_1_send([1,2,3,4])
    master.mav.array_test_3_send(1, [2,3,4,5])
    master.mav.array_test_4_send([1,2,3,4], 5)
    master.mav.array_test_5_send("test1", "test2")
    master.mav.array_test_6_send(1,2,3, [4,5], [6,7], [8,9], [10,11], [12,13], [14,15], "long value", [1.1, 2.2], [3.3, 4.4])
    master.mav.array_test_7_send([1.1, 2.2], [3.3, 4.4],
            [4,5], [6,7], [8,9], [10,11], [12,13], [14,15], "long value")
    master.mav.array_test_8_send(1, [2.2, 3.3], [14,15])
    time.sleep(1)

master.close()

