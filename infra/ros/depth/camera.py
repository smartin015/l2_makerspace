import numpy as np
import pyrealsense2 as rs

MAX_DEPTH = 32000 # 99th percentile of values is <= 32000

class RealsenseCamera():
    def __init__(self, path=None, dim=None, hz=None):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.colorizer = rs.colorizer()
        # https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.colorizer.html?highlight=color#pyrealsense2.colorizer.colorize
        # 0 - Jet 
        # 1 - Classic 
        # 2 - WhiteToBlack 
        # 3 - BlackToWhite 
        # 4 - Bio 
        # 5 - Cold 
        # 6 - Warm 
        # 7 - Quantized 
        # 8 - Pattern
        self.colorizer.set_option(rs.option.color_scheme, 0)
        if path is not None:
            print("Playback from bag:", path, "(ignoring dimension & frequency params)")
            self.config.enable_device_from_file(path)
        else:
            print("Depth stream:", dim, "@", hz, "hz")
            self.config.enable_stream(rs.stream.depth, dim[0], dim[1], rs.format.z16, hz)

    def spin_forever(self, cb=None, colorized_cb=None):
        self.pipeline.start(self.config)
        while(True):
            try:
                frame = self.pipeline.wait_for_frames()
                if cb is not None:
                    frame_data = np.asanyarray(frame.get_depth_frame().get_data())
                    cb(frame.get_frame_number(), frame_data)
                if colorized_cb is not None:
                    colorized_cb(frame.get_frame_number(), np.asanyarray(self.colorizer.colorize(frame.get_depth_frame()).get_data()))
            except KeyboardInterrupt:
                return

