# Video Stream Information

This script updates the ArduPilot Camera library with the values required to populate the VIDEO_STREAM_INFORMATION
mavlink message allow the GCS to find and display live video to the user

# Parameters

- VID1_CAMMODEL: Video stream camera model (1:Siyi A8, 2:Siyi ZR10, 3:Siyi ZR30, 4:Siyi ZT30 Zoom, 5:Siyi ZT30 Wide, 6:Siyi ZT30 IR, 7:Siyi ZT6 RGB, 8:Siyi ZT6 IR, 9:Herelink WifiAP, 10:Herelink USB-tethering, 11:Topotek 1080p, 12:Topotek 480p, 13:Viewpro)
- VID1_ID: Video stream id
- VID1_TYPE: Video stream type (0:RTSP, 1:RTPUDP, 2:TCP_MPEG, 3:MPEG_TS)
- VID1_FLAG: Video stream flags (Bitmask: 0:Running,1:Thermal,2:Thermal Range Enabled)
- VID1_FRAME_RATE: Video stream frame rate
- VID1_HRES: Video stream horizontal resolution
- VID1_VRES: Video stream vertical resolution
- VID1_BITR: Video stream bitrate
- VID1_HFOV: Video stream horizontal FOV in degrees
- VID1_ENCODING: Video stream encoding (0:Unknown, 1:H264, 2:H265)
- VID1_IPADDR0: Video stream IP Address first octet
- VID1_IPADDR1: Video stream IP Address second octet
- VID1_IPADDR2: Video stream IP Address third octet
- VID1_IPADDR3: Video stream IP Address fourth octet
- VID1_IPPORT: Video Stream IP Address Port

# How To Use

1. Setup the camera gimbal as described on the ArduPilot wiki including ethernet setup
2. Check the IP address of the camera gimbal
3. Setup scripting per https://ardupilot.org/plane/docs/common-lua-scripts.html and reboot the autopilot
4. Copy this script to the vehicle autopilot's "scripts" directory
5. Set the VID1_CAMMODEL parameter to the camera gimbal model and adjust the new VID1 params as required
6. If necessary using the GCS's mavlink inspector to confirm the VIDEO_STREAM_INFORMATION URI field is correct
7. Confirm the ground station can display the live video
