from camera import RealsenseCamera, MAX_DEPTH
from web_display import start_server_daemon
import cv2

def profile_test():

    from numba import cuda
    import rvl_cuda
    import numpy as np
    import pyrealsense2 as rs
    from datetime import datetime
    DEPROJECT = True
    pipeline = rs.pipeline()
    config = rs.config()
    rs.config.enable_device_from_file(config, "./a.bag")
    #config.enable_stream(rs.stream.depth, dim[0], dim[1], rs.format.z16, 30)
    pipeline.start(config)

    encoded = np.zeros((NUM_SECTOR, rvl_cuda.SECTOR_LEN), dtype=np.uint64)
    if DEPROJECT:
        decoded = np.zeros((np.prod(dim), 3), dtype=np.float32)
    else:
        decoded = np.zeros(dim, dtype=np.uint16)

    print("Running on img shape {}, output shape {} (Ctl+C to stop)...".format(dim, encoded.shape))
    sample = None
    nsamp = 0
    timings = [[], []]
    while nsamp < 20:
        try:
            has, frame = pipeline.try_wait_for_frames()
            if not has:
                break
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
        except KeyboardInterrupt:
            break

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

    cuda.close()

    print("""=== Summary Performance Stats ===
        #samples: 
            {} encode
            {} decode
        First frame: 
            {} encode
            {} decode
        Warmed-up wall time (avg): 
            {} encode
            {} decode""".format(
            len(timings[0]), len(timings[1]),
            timings[0][0],  timings[1][0],
            np.mean(timings[0][1:]), np.mean(timings[1][1:])
        ))


    cv2.waitKey(0)


def main():
    cam = RealsenseCamera(dim=(848, 480), hz=30)
    infeed = start_server_daemon('', 8000)
    def frame2web(frame_num, colorized):
        done, buf = cv2.imencode(".jpg", colorized)
        if not done:
            return
        infeed.write(buf)
    cam.spin_forever(colorized_cb=frame2web)

if __name__ == "__main__":
    main()
