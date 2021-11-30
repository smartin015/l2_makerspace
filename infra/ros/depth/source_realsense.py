import numpy as np
import pyrealsense2 as rs

MAX_DEPTH = 32000 # 99th percentile of values is <= 32000

class RealsenseSource():
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

        self.sinks = None
        self.color_sinks = None

    def append_sink(self, sink):
        if sink.COLORIZED_INPUT:
            if self.color_sinks is None:
                self.color_sinks = []
            self.color_sinks.append(sink.write)
        else:
            if self.sinks is None:
                self.sinks = []
            self.sinks.append(sink.write)

    def run_forever(self):
        self.pipeline.start(self.config)
        while(True):
            try:
                frame = self.pipeline.wait_for_frames()
                if self.sinks is not None:
                    frame_data = np.asanyarray(frame.get_depth_frame().get_data())
                    for cb in self.sinks:
                        cb(frame.get_frame_number(), frame_data)
                if self.color_sinks is not None:
                    frame_data = np.asanyarray(self.colorizer.colorize(frame.get_depth_frame()).get_data())
                    for cb in self.color_sinks:
                        cb(frame.get_frame_number(), frame_data)
            except KeyboardInterrupt:
                return

