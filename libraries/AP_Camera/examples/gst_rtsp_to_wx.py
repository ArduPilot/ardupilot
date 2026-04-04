""""
Capture a RTSP video stream and display in wxpython

Usage
-----

1. rtsp server

python ./gst_rtsp_server.py

3. display in wxpython

python ./gst_rtsp_to_wx.py

Acknowledgments
---------------

Video class to capture GStreamer frames
  https://www.ardusub.com/developers/opencv.html

ImagePanel class to display openCV images in wxWidgets
  https://stackoverflow.com/questions/14804741/opencv-integration-with-wxpython
"""

# flake8: noqa

import copy
import cv2
import gi
import numpy as np
import threading
import wx


gi.require_version("Gst", "1.0")
from gi.repository import Gst


class VideoStream:
    """BlueRov video capture class constructor - adapted to capture rtspsrc

    Attributes:
        address (string): RTSP address
        port (int): RTSP port
        mount_point (string): video stream mount point
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
        latest_frame (np.ndarray): Latest retrieved video frame
    """

    def __init__(
        self, address="127.0.0.1", port=8554, mount_point="/camera", latency=50
    ):
        Gst.init(None)

        self.address = address
        self.port = port
        self.mount_point = mount_point
        self.latency = latency

        self.latest_frame = self._new_frame = None

        self.video_source = (
            f"rtspsrc location=rtsp://{address}:{port}{mount_point} latency={latency}"
        )

        # Python does not have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = (
            "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert"
        )
        # Create a sink to get data
        self.video_sink_conf = (
            "! appsink emit-signals=true sync=false max-buffers=2 drop=true"
        )

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = [
                "videotestsrc ! decodebin",
                "! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert",
                "! appsink",
            ]

        command = " ".join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name("appsink0")

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps_structure = sample.get_caps().get_structure(0)
        array = np.ndarray(
            (caps_structure.get_value("height"), caps_structure.get_value("width"), 3),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8,
        )
        return array

    def frame(self):
        """Get Frame

        Returns:
            np.ndarray: latest retrieved image frame
        """
        if self.frame_available:
            self.latest_frame = self._new_frame
            # reset to indicate latest frame has been 'consumed'
            self._new_frame = None
        return self.latest_frame

    def frame_available(self):
        """Check if a new frame is available

        Returns:
            bool: true if a new frame is available
        """
        return self._new_frame is not None

    def run(self):
        """Get frame to update _new_frame"""

        self.start_gst(
            [
                self.video_source,
                self.video_decode,
                self.video_sink_conf,
            ]
        )

        self.video_sink.connect("new-sample", self.callback)

    def callback(self, sink):
        sample = sink.emit("pull-sample")
        self._new_frame = self.gst_to_opencv(sample)

        return Gst.FlowReturn.OK


class ImagePanel(wx.Panel):
    def __init__(self, parent, video_stream, fps=30):
        wx.Panel.__init__(self, parent)

        self._video_stream = video_stream

        # Shared between threads
        self._frame_lock = threading.Lock()
        self._latest_frame = None

        print("Waiting for video stream...")
        waited = 0
        while not self._video_stream.frame_available():
            waited += 1
            print("\r  Frame not available (x{})".format(waited), end="")
            cv2.waitKey(30)
        print("\nSuccess! Video stream available")

        if self._video_stream.frame_available():
            # Only retrieve and display a frame if it's new
            frame = copy.deepcopy(self._video_stream.frame())

            # Frame size
            height, width, _ = frame.shape

            parent.SetSize((width, height))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            self.bmp = wx.Bitmap.FromBuffer(width, height, frame)

            self.timer = wx.Timer(self)
            self.timer.Start(int(1000 / fps))

            self.Bind(wx.EVT_PAINT, self.OnPaint)
            self.Bind(wx.EVT_TIMER, self.NextFrame)

    def OnPaint(self, evt):
        dc = wx.BufferedPaintDC(self)
        dc.DrawBitmap(self.bmp, 0, 0)

    def NextFrame(self, event):
        if self._video_stream.frame_available():
            frame = copy.deepcopy(self._video_stream.frame())

            # Convert frame to bitmap for wxFrame
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.bmp.CopyFromBuffer(frame)
            self.Refresh()


def main():

    # create the video stream
    video_stream = VideoStream(mount_point="/camera")

    # app must run on the main thread
    app = wx.App()
    wx_frame = wx.Frame(None)

    # create the image panel
    image_panel = ImagePanel(wx_frame, video_stream, fps=30)

    wx_frame.Show()
    app.MainLoop()


if __name__ == "__main__":
    main()
