#!/usr/bin/env python3

#
# An example script that receives images from a WebotsArduVehicle 
# on port 5599 and displays using OpenCV.
# Requires opencv-python (`pip3 install opencv-python`)
#

# flake8: noqa

import cv2
import socket
import struct
import numpy as np

# connect to WebotsArduVehicle
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5599))

header_size = struct.calcsize("=HH")
while True:
    # receive header
    header = s.recv(header_size)
    if len(header) != header_size:
        print("Header size mismatch")
        break

    # parse header
    width, height = struct.unpack("=HH", header)

    # receive image
    bytes_to_read = width * height
    img = bytes()
    while len(img) < bytes_to_read:
        img += s.recv(min(bytes_to_read - len(img), 4096))

    # convert incoming bytes to a numpy array (a grayscale image)
    img = np.frombuffer(img, np.uint8).reshape((height, width))

    # Do cool stuff with the image here
    # ...

    # display image
    cv2.imshow("image", img)
    if cv2.waitKey(1) == ord("q"):
        break

s.close()