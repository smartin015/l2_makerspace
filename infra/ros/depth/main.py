from source_realsense import RealsenseSource, MAX_DEPTH
from source_udp import UDPSource
from web_display import start_server_daemon
import cv2

def frame2web(frame_num, colorized):
    done, buf = cv2.imencode(".jpg", colorized)
    if not done:
        return
    infeed.write(buf)

class LoopbackSink():
    def __init__(self, dim, debug=False, cpu_encode=False, cpu_decode=False):
        self.debug = debug
        if cpu_encode or cpu_decode:
            raise Exception("CPU encode/decode on loopback not implemented")
        encoded = np.zeros((NUM_SECTOR, rvl_cuda.SECTOR_LEN), dtype=np.uint64)
        if DEPROJECT:
            decoded = np.zeros((np.prod(dim), 3), dtype=np.float32)
        else:
            decoded = np.zeros(dim, dtype=np.uint16)

        print("Running on img shape {}, output shape {} (Ctl+C to stop)...".format(dim, encoded.shape))
        sample = None
        nsamp = 0
        timings = [[], []]

    def write(self, frame):
        print(frame)
        data = np.asanyarray(frame.get_depth_frame().get_data())
        start = datetime.now()
        rvl_cuda.encode[BLOCKS_PER_GRID, THREADS_PER_BLOCK](data, encoded)
        timings[0].append(datetime.now() - start)
        
        # print([hex(e) for e in encoded[529][0:10]]) # 529 80w624h
        start = datetime.now()
        rvl_cuda.decode[4, int(NUM_SECTOR/4)](encoded, decoded, DEPROJECT)
        timings[1].append(datetime.now() - start)
        #print(rvl_decode.inspect_types())
        #import sys
        #sys.exit(0)
        nsamp += 1

        if self.debug:
            cv2.imshow("original", data / np.max(data))
            cv2.imshow("encoded", encoded / max(1, np.max(encoded)))

            if DEPROJECT:
                import matplotlib.pyplot as plt
                from mpl_toolkits import mplot3d
                ax = plt.axes(projection='3d')
                ax.scatter(decoded[:,0], decoded[:,1], decoded[:,2], s=1)
                plt.show()
            else:
                cv2.imshow("decoded", decoded / max(1, np.max(decoded)))

class WebServerSink():
    def __init__(self, port):
       # TODO
       pass

class FileSink():
    def __init__(self, path):
        # Unknown if pyrealsense bag format is equivalent to ros2 bag format... need to confirm
        raise Exception("Not implemented")

IP_PORT_RE = r'([0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3})\:?([0-9]{1,5})?'
def main(args):
    source = None
    # Setup source
    ip_match = re.match(IP_PORT_RE, args.source)
    if ip_match:
        source = UDPSource(ip_match[1], ip_match[2], args.dim, mproc=args.mproc, mpthread=args.mpthread, cpu_decode=args.cpu_decode)
    if args.source  == 'device':
        source = RealsenseSource(dim=args.dim, hz=args.hz)
    else:
        source = RealsenseSource(path=args.source)

    # Register sinks
    for sink in args.sink:
        if sink == "loopback":
            source.append_sink(LoopbackSink(args.dim, args.verbose, args.cpu_encode, args.cpu_decode))
            continue
        if sink.startswith("web"):
            port = sink.split(':')[1:2]
            source.append_sink(StreamingServerSink(int(port) if len(port) > 0 else None))
            continue
        ip_match = re.match(IP_PORT_RE, sink)
        if ip_match:
            source.append_sink(UDPSink(ip_match[1], ip_match[2]))
            continue
        else:
            source.append_sink(FileSink(sink))

    # Run the main loop
    try:
        source.run_forever()
    finally:
        cuda.close()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Send/receive compressed/decompressed realsense depth camera frames")
    parser.add_argument("--source", type=str, help="Source ip+port, file path, or 'device' for local camera", default="device")
    parser.add_argument("--sink", type=str, help="Destination ip+port, file path, 'web:####' for web display on port, or 'loopback' to test transcoding locally", 
                        default=["loopback"], nargs='+')
    parser.add_argument("--dim", nargs=2, metavar=('width', 'height'), help="Dimensions of depth image (if source is not file path)", default=(848, 480))
    parser.add_argument("--hz", type=int, help="Frequency of depth image (if source is 'device')", default=30)
    parser.add_argument("--stats_pd", type=float, help="Period (in seconds) to print performance stats", default=2.0)
    parser.add_argument("--verbose", type=bool, default=True, help="Show extra details (may open new windows)")
    parser.add_argument('--cpu_encode', action='store_true', default=False, help="Use CPU for RVL encoding (default is GPU)")
    parser.add_argument('--cpu_decode', action='store_true', default=False, help="Use CPU for RVL decoding (default is GPU)")
    parser.add_argument("--mproc", type=int, default=None, help="Override GPU mproc count")
    parser.add_argument("--mpthread", type=int, default=None, help="Override max threads per mproc")
    args = parser.parse_args(argv)
    main()
