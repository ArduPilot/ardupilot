
# Real-Time Object Tracking with Gimbal Control (Simulation)

## Overview

## Requirements and Installations
1. We assume you have already setup Ardupilot with SITL and gazebo simulation
2. Install Gstreamer

```
sudo apt update
```

Install build tools and Python development libraries and GStreamer related plugins
```

sudo apt install -y build-essential cmake git pkg-config libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev

sudo apt install -y python3-dev python3-numpy python3-pip

sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
                    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
                    gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools
```

3. default opencv from pip should be uninstalled and again built with gstreamer support (If already did this go to next step)
```
git clone https://github.com/opencv/opencv.git
```

```
git clone https://github.com/opencv/opencv_contrib.git
```

```
cd opencv
mkdir build
cd build
```

Build Configurations
```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D PYTHON3_EXECUTABLE=$(which python3) \
      -D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
      -D PYTHON3_LIBRARY=$(python3 -c "from distutils.sysconfig import get_config_var; print(get_config_var('LIBDIR'))") \
      -D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
      -D WITH_GSTREAMER=ON \
      -D WITH_FFMPEG=ON \
      -D BUILD_opencv_python3=ON \
      -D BUILD_EXAMPLES=OFF ..

```

```
make -j$(nproc)
```

```
sudo make install
sudo ldconfig
```



## Description
There are two main Python scripts:

1. **`tracking.py`**: This script handles video streaming and object tracking. It uses OpenCV to process video frames, applies an object tracking algorithm, and sends commands to the gimbal to adjust its orientation based on the object's position in the frame.

2. **`send_camera_information.py`**: This script communicates with the UAV's autopilot system using MAVLink. It sends the necessary camera and gimbal information to ensure the UAV's camera is correctly oriented.


## Runing the scripts
just run

  ```bash run_tracking.sh```