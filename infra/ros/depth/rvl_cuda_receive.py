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

class Receiver():
    def __init__(self, args):
        self.dim = (args.dim[1], args.dim[0])
        self.deproject = args.deproject
        # NOTE: we match the details of the sender so that we get the same block & segment dimensions/counts
        # Warp: 32        Multiprocessors: 1      Thread limit per mproc: 128
        self.KB, self.BLOCKS_PER_GRID, self.THREADS_PER_BLOCK, self.NUM_SECTOR = kernel_bounds(self.dim, warp=32, mproc=1, sm_cores=128)
        rvl_cuda.configure(self.KB, self.NUM_SECTOR)
        self.sectors_per_packet = 1 # int(MAX_UDP_PACKET_BYTES / (rvl_cuda.SECTOR_LEN * np.dtype(np.uint64).itemsize))
        print("Sector length:", rvl_cuda.SECTOR_LEN, "\tSectors per packet:", self.sectors_per_packet, "\tPackets/frame:",  int(self.NUM_SECTOR / self.sectors_per_packet))

        self.sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.sock.bind((args.host, args.port))

        self.stats = dict([(n,0) for n in ['avg_recv_latency', 'avg_decode_latency', 'avg_latency']])
        self.print_pd = args.print_pd
        self.last_print_stats = time.perf_counter()
        self.num_frames = 0
        print("Printing stats every", self.print_pd, "seconds")

    def avg_ms(self, f, n_sec):
        self.stats[f] = self.stats[f] * 0.9 + (n_sec * 1000) * 0.1

    def print_stats(self):
        hz = self.num_frames / self.print_pd
        print(("{0:4.1f}hz:" +
               "{avg_recv_latency:10.2f}ms recv" +
               "{avg_decode_latency:10.2f}ms decode" +
               "{avg_latency:10.2f}ms total").format(hz, **self.stats))
        self.num_frames = 0

    def spin_forever(self):
        encoded = {}
        decoded = {}
        print("UDP server listening")
        while(True):
            start = time.perf_counter()
            nsec = 0
            while nsec < self.NUM_SECTOR:
                (data, src) = self.sock.recvfrom(rvl_cuda.SECTOR_LEN * min(self.sectors_per_packet, self.NUM_SECTOR-nsec) * np.dtype(np.uint64).itemsize)
                src=src[0]
                l = len(data)
                if l == 0:
                    continue

                if encoded.get(src) is None:
                    print("New client", src)
                    encoded[src] = np.zeros((self.NUM_SECTOR, rvl_cuda.SECTOR_LEN), dtype=np.uint64)
                    decoded[src] = np.zeros(self.dim, dtype=np.uint16)
        
                # TODO handle calibration packets (intrinsics / extrinsics)
                # TODO handle variable package width
                # TODO handle delta frames
                dsec = l // np.dtype(np.uint64).itemsize // rvl_cuda.SECTOR_LEN
                encoded[src][nsec:nsec+dsec,:] = np.frombuffer(data, dtype=np.uint64).reshape(dsec, rvl_cuda.SECTOR_LEN)
                print(encoded[src][nsec:nsec+dsec,0:5])
                nsec += self.sectors_per_packet
                # print(nsec)
    
            decode_start = time.perf_counter()
            rvl_cuda.decode[self.BLOCKS_PER_GRID, self.THREADS_PER_BLOCK](encoded[src], decoded[src], self.deproject)
            decode_end = time.perf_counter()
            cv2.imshow(src + "dec", decoded[src] / max(1, np.max(decoded[src])))
            cv2.imshow(src + "enc", encoded[src] / max(1, np.max(encoded[src])))
            cv2.waitKey(1)
            end = time.perf_counter()
            self.avg_ms('avg_recv_latency', decode_start - start)
            self.avg_ms('avg_decode_latency', decode_end - decode_start)
            self.avg_ms('avg_latency', end - start)
            self.num_frames += 1
            if end - self.last_print_stats > self.print_pd:
                self.print_stats()
                self.last_print_stats = end


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Receive RVL-encoded depth camera frames over UDP.")
    parser.add_argument("--port", metavar='PORT', type=int, help="port number", default=19823)
    parser.add_argument("--host", type=str, help="ip address", default="0.0.0.0")
    parser.add_argument("--dim", nargs=2, metavar=('width', 'height'), help="Dimensions of depth image", default=(848, 480))
    parser.add_argument("--print_pd", type=float, help="Period (in seconds) to print performance stats", default=2.0)
    parser.add_argument("--deproject", type=bool, help="Whether to deproject depth frames into point clouds", default=False)
    args = parser.parse_args()

    r = Receiver(args)
    try: 
        r.spin_forever()
    finally: 
        cuda.close()


