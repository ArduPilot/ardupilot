"""
Capture a UDP video stream and pipe to an GStreamer RTSP server 

The Gazebo GStreamerPlugin with <use_basic_pipeline>1</use_basic_pipeline>
has elements:

source:     appsrc
              caps='video/x-raw, format=I420, width=640, height=480, framerate=50'
              is-live=1
              do-timestamp=1
              stream-type=0
queue:      queue
converter:  videoconvert
encoder:    x264enc bitrate=800 speed-preset=6 tune=4 key-int-max=10
payloader:  rtph264pay
sink:       udpsink host=127.0.0.1 port=5600


A udpsink example streaming a test pattern using the same settings as the basic
pipeline in the Gazebo GstCameraPlugin.

1. udpsink

gst-launch-1.0 -v videotestsrc ! 'video/x-raw,format=I420,width=640,height=480,framerate=50/1' ! queue ! videoconvert ! x264enc bitrate=800 speed-preset=6 tune=4 key-int-max=10 ! rtph264pay ! udpsink host=127.0.0.1 port=5600

2a. Display the udpsink directly

udpsrc

gst-launch-1.0 -v udpsrc address=127.0.0.1 port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false

2b. Or run the udpsink, this script to convert to RTSP, and view the RTSP stream:

python ./gst_udp_to_rtsp.py

3. rtspsrc

gst-launch-1.0 rtspsrc location=rtsp://localhost:8554/camera latency=50 ! decodebin ! autovideosink


Acknowledgments
---------------

GStreamer RTSP server example adapted from code by Jerome Carretero (Tamaggo)

https://github.com/tamaggo/gstreamer-examples
https://github.com/tamaggo/gstreamer-examples/blob/master/test_gst_rtsp_server.py
"""

# Copyright (c) 2015 Tamaggo Inc.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst
from gi.repository import GstRtspServer
from gi.repository import GLib


class GstUdpMediaFactory(GstRtspServer.RTSPMediaFactory):
    """
    Create a GStreamer pipeline for a udpsrc source.
    """

    def __init__(self, address="127.0.0.1", port="5600"):
        GstRtspServer.RTSPMediaFactory.__init__(self)

        # udpsrc options
        self.address = address
        self.port = port

    def do_create_element(self, url):
        source = f"udpsrc address={self.address} port={self.port}"

        codec = "application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264"

        s_h264 = "x264enc tune=zerolatency"
        pipeline_str = f"( {source} ! {codec} ! queue max-size-buffers=1 name=q_enc ! {s_h264} ! rtph264pay name=pay0 pt=96 )"
        print(pipeline_str)
        return Gst.parse_launch(pipeline_str)


class GstServer:
    """
    A GStreamer RTSP server streaming a udpsrc to:

    rtsp://127.0.0.1:8554/camera
    """

    def __init__(self):
        self.server = GstRtspServer.RTSPServer()
        self.server.set_address("127.0.0.1")
        self.server.set_service("8554")

        media_factory = GstUdpMediaFactory(address="127.0.0.1", port="5600")
        media_factory.set_shared(True)

        mount_points = self.server.get_mount_points()
        mount_points.add_factory("/camera", media_factory)

        self.server.attach(None)


def main():
    Gst.init(None)
    server = GstServer()
    loop = GLib.MainLoop()
    loop.run()


if __name__ == "__main__":
    main()
