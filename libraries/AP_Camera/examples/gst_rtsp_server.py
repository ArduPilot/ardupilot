"""
GST RTSP Server Example

Usage
-----

1. Start the server:

python ./gst_rtsp_server.py

2. Display the RTSP stream

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

import sys
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst
from gi.repository import GstRtspServer
from gi.repository import GLib


class VideoTestMediaFactory(GstRtspServer.RTSPMediaFactory):
    """
    Create a GStreamer pipeline for a videotestsrc source.

    The default videotestsrc uses the smpte test pattern. Other test patterns
    may be specified when initialising the class.
    """
    def __init__(self, pattern="smpte"):
        GstRtspServer.RTSPMediaFactory.__init__(self)
        self.pattern = pattern

    def do_create_element(self, url):
        s_src = f"videotestsrc pattern={self.pattern} ! video/x-raw,rate=30,width=640,height=480,format=I420"
        s_h264 = "x264enc tune=zerolatency"
        pipeline_str = f"( {s_src} ! queue max-size-buffers=1 name=q_enc ! {s_h264} ! rtph264pay name=pay0 pt=96 )"
        if len(sys.argv) > 1:
            pipeline_str = " ".join(sys.argv[1:])
        print(pipeline_str)
        return Gst.parse_launch(pipeline_str)


class GstServer:
    """
    A GStreamer RTSP server streaming three different test patterns:

    rtsp://127.0.0.1:8554/camera
    rtsp://127.0.0.1:8554/ball
    rtsp://127.0.0.1:8554/snow
    """
    def __init__(self):
        self.server = GstRtspServer.RTSPServer()
        self.server.set_address("127.0.0.1")
        self.server.set_service("8554")

        media_factory1 = VideoTestMediaFactory()
        media_factory1.set_shared(True)

        media_factory2 = VideoTestMediaFactory("ball")
        media_factory2.set_shared(True)

        media_factory3 = VideoTestMediaFactory("snow")
        media_factory3.set_shared(True)

        mount_points = self.server.get_mount_points()
        mount_points.add_factory("/camera", media_factory1)
        mount_points.add_factory("/ball", media_factory2)
        mount_points.add_factory("/snow", media_factory3)

        self.server.attach(None)


def main():
    Gst.init(None)
    server = GstServer()
    loop = GLib.MainLoop()
    loop.run()


if __name__ == "__main__":
    main()
