from kernel_config import kernel_bounds
from numba import cuda
import rvl_cuda
from math import sqrt, ceil
import numpy as np
import cv2
import pyrealsense2 as rs
import socket
import time
from datetime import datetime

MAX_UDP_PACKET_BYTES = 65535

class Sender():
    def __init__(self, args):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.debug = args.debug

        if args.path is not None:
            print("Enabling recorded bag:", args.path)
            self.config.enable_device_from_file(args.path)
        else:
            print("Enabling depth stream", args.dim, "@", args.hz, "hz")
            self.config.enable_stream(rs.stream.depth, args.dim[0], args.dim[1], rs.format.z16, args.hz)

        self.dim = (args.dim[1], args.dim[0])
        if args.mproc is not None and args.mpthread is not None:
            self.KB, self.BLOCKS_PER_GRID, self.THREADS_PER_BLOCK, self.NUM_SECTOR = kernel_bounds(self.dim, mproc=args.mproc, sm_cores=args.mpthread)
        else:
            self.KB, self.BLOCKS_PER_GRID, self.THREADS_PER_BLOCK, self.NUM_SECTOR = kernel_bounds(self.dim)
        rvl_cuda.configure(self.KB, self.NUM_SECTOR)
        
        self.sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.dest = (args.dest, args.port)

        self.stats = dict([(n,0) for n in ['skipped_frames', 'avg_fetch_latency', 'avg_encode_latency', 'avg_send_latency', 'avg_latency']])
        self.print_pd = args.print_pd
        print("Printing stats every", self.print_pd, "seconds")
        self.last_stats_frame_num = 0
        self.last_frame_num = 0
        self.last_print_stats = time.perf_counter()

    def avg_ms(self, f, n_sec):
        self.stats[f] = self.stats[f] * 0.9 + (n_sec * 1000) * 0.1

    def print_stats(self):
        hz = (self.last_frame_num - self.last_stats_frame_num) / self.print_pd
        self.last_stats_frame_num = self.last_frame_num
        self.stats['skipped_frames'] /= self.print_pd
        print(("#{0:<5} ({1:4.1f}hz,{skipped_frames:4.1f}skip/s):" +
               "{avg_fetch_latency:10.2f}ms fetch" +
               "{avg_encode_latency:10.2f}ms encode" +
               "{avg_send_latency:10.2f}ms send" +
               "{avg_latency:10.2f}ms total").format(
                    self.last_frame_num, hz - self.stats['skipped_frames'],
                    **self.stats))
        self.stats['skipped_frames'] = 0

    def spin_forever(self):
        encoded = cuda.mapped_array((self.NUM_SECTOR, rvl_cuda.SECTOR_LEN), dtype=np.uint64)
        # encoded = np.zeros((self.NUM_SECTOR, rvl_cuda.SECTOR_LEN), dtype=np.uint64)
        print(self.NUM_SECTOR, "sectors, length", rvl_cuda.SECTOR_LEN)
        sectors_per_packet = int(MAX_UDP_PACKET_BYTES / (rvl_cuda.SECTOR_LEN * np.dtype(np.uint64).itemsize))
        print("Starting UDP send pipeline (%d sectors/packet, %d packets/frame)" % (sectors_per_packet, int(self.NUM_SECTOR / sectors_per_packet)))
        self.pipeline.start(self.config)
        delta = cuda.mapped_array(self.dim, dtype=np.uint16)
        last_frame = delta
        while(True):
            try:
                start = time.perf_counter()
                has, frame = self.pipeline.try_wait_for_frames()
                if not has:
                    break
                frame_data = np.asanyarray(frame.get_depth_frame().get_data())
                fnum = frame.get_frame_number()
                # PERF_TODO actually take delta between frames
                #delta[:,:] = frame_data - last_frame
                delta[:,:] = frame_data
                last_frame = frame_data

                enc_start = time.perf_counter()
                rvl_cuda.encode[self.BLOCKS_PER_GRID, self.THREADS_PER_BLOCK](delta, encoded)
                enc_end_send_start = time.perf_counter()
                prep_amt = 0
                for i in range(1, encoded.shape[0], sectors_per_packet):
                    # PERF_TODO restrict width to max packet width
                    # width = np.max(np.right_shift(encoded[i:(i+sectors_per_packet),0], 32))
                    self.sock.sendto(encoded[i:(i+sectors_per_packet),:].tobytes(), self.dest)
                send_end = time.perf_counter()

                if self.debug:
                    cv2.imshow("frame", frame_data / max(1, np.max(frame_data)))
                    cv2.imshow("encoded", encoded / max(1, np.max(encoded)))
                    cv2.waitKey(1)

                # Update stats
                self.stats['skipped_frames'] += frame.get_frame_number() - self.last_frame_num - 1
                self.avg_ms("avg_fetch_latency", enc_start - start)
                self.avg_ms("avg_encode_latency", enc_end_send_start - enc_start)
                self.avg_ms("avg_send_latency", send_end - enc_end_send_start)
                self.avg_ms("avg_latency", send_end - start)
                self.last_frame_num = frame.get_frame_number()
                if send_end - self.last_print_stats > self.print_pd:
                    self.print_stats()
                    self.last_print_stats = send_end
            except KeyboardInterrupt:
                return

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Send RVL-encoded depth camera frames to a network destination over UDP.")
    parser.add_argument("--playback", dest='path', type=str, help="When specified, play from playback file instead of from device.")
    parser.add_argument("dest", metavar='DEST', type=str, help="destination ip address", default="0.0.0.0")
    parser.add_argument("port", metavar='PORT', type=int, help="port number", default=19823)
    parser.add_argument("--dim", nargs=2, metavar=('width', 'height'), help="Dimensions of depth image", default=(848, 480))
    parser.add_argument("--hz", type=int, help="Frequency of depth image", default=30)
    parser.add_argument("--print_pd", type=float, help="Period (in seconds) to print performance stats", default=2.0)
    parser.add_argument("--debug", type=bool, default=True, help="Show debug visualization")
    parser.add_argument("--mproc", type=int, default=None, help="Override GPU mproc count")
    parser.add_argument("--mpthread", type=int, default=None, help="Override max threads per mproc")
    args = parser.parse_args()

    s = Sender(args)
    try: 
        s.spin_forever()
    finally: 
        cuda.close()


