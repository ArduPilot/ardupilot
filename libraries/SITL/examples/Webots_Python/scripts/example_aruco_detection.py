#!/usr/bin/env python3

#
# An example script that receives images from a WebotsArduVehicle on port 5599 
# and displays them overlayed with any ArUco markers using OpenCV.
# Requires opencv-python (`pip3 install opencv-python`)
#

import cv2
import socket
import struct
import numpy as np

# connect to WebotsArduVehicle
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5599))

# ArUco setup
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()

header_size = struct.calcsize("=HH")
while True:
    # receive header
    header = s.recv(header_size)
    if len(header) != header_size:
        print("Header size mismatch")
        break

    # parse header
    width, height = struct.unpack("=HH", header)

    # for CV applications we may want camera intrinsics such as focal length: 
    # https://stackoverflow.com/questions/61555182/webot-camera-default-parameters-like-pixel-size-and-focus
    # cam_focal_length = 2 * np.arctan(np.tan(cam_fov * 0.5) / (cam_width / cam_height))

    # receive image
    bytes_to_read = width * height
    img = bytes()
    while len(img) < bytes_to_read:
        img += s.recv(min(bytes_to_read - len(img), 4096))

    # convert to numpy array
    img = np.frombuffer(img, np.uint8).reshape((height, width))
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # detect ArUco markers
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # draw the bounding box of the ArUCo detection
            cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)
            # compute and draw the center (x, y)-coordinates of the ArUco
            # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
            # draw the ArUco marker ID on the image
            cv2.putText(img, str(markerID),
                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)

    # display image
    cv2.imshow("image", img)
    if cv2.waitKey(1) == ord("q"):
        break

s.close()