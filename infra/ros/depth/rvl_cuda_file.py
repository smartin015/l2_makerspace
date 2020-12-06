# envelope calculations
# 7200B output @ 20x compression + 28B IP header + 3B preamble -> 391 bytes per thread-frame
# Frame overhead is 0.83%, contributes 3*256*30*8=184kbps to total bitrate. Each byte is ~61kbps
# @ 30hz across 256 threads -> 3MB -> 24mbps
# Original is 1280*720*30*2*8 -> 442mbps, so 18.4x reduction
# Frame footprint vs TRVL research paper is (1280*720) / (640*576) = 2.5X

from numba import cuda
import rvl_cuda
from math import sqrt, ceil
import numpy as np
import cv2
import pyrealsense2 as rs
from datetime import datetime

DEPROJECT = True

# Load test image M*N*bytes
# Note that column order is height-first
PX_B = 2
input_dim_realsense = (720, 1280)
input_dim_realsense_reg = (480, 848)
input_dim_trvl = (576, 640)
dim = input_dim_realsense_reg
print("Image dimensions", dim)

# Evaluate the attached GPU to see what we're working with
# Cuda relations:
# - Grids --> GPUs
# - Blocks --> Multi Processors (MP)
# - Threads --> Stream Processors (SP)
# - Warps --> 32 threads executing simultaneously
# https://stackoverflow.com/questions/63823395/how-can-i-get-the-number-of-cuda-cores-in-my-gpu-using-python-and-numba
CC_CORES_PER_SM_DICT = {
    (2,0) : 32,
    (2,1) : 48,
    (3,0) : 192,
    (3,5) : 192,
    (3,7) : 192,
    (5,0) : 128,
    (5,2) : 128,
    (5,3) : 128, # Jetson nano
    (6,0) : 64,
    (6,1) : 128,
    (7,0) : 64,
    (7,5) : 64,
    (8,0) : 64,
}
WARP = 32
d = cuda.get_current_device()
MPROC = getattr(d, 'MULTIPROCESSOR_COUNT', 1)
MAX_THREADS_PER_SM = CC_CORES_PER_SM_DICT.get(getattr(d, 'COMPUTE_CAPABILITY'), 0)
print({WARP, MPROC, MAX_THREADS_PER_SM})

# Compute kernel bounds based on estimated thread count. Example:
# 1280x720 -> 256 threads -> 3600 16z samples -> 7200B max output per thread 
# X*Y = M*N/T, X=K*M, Y=K*N, X*Y = K^2*M*N, K = sqrt((X*Y)/(M*N)) = sqrt((M*N/T)/(M*N)) = sqrt(1/T),
# X = 1280*sqrt(1/256) = 80
# Y = 720*sqrt(1/256) = 45
# In this example, each thread is an 80x45 segment of the 16z image
def kernel_bounds(dim, mproc, sm_cores):
    # Come up with a guess for the number of threads & CUDA kernel shape
    if dim[0] >= dim[1]:
         raise Exception("Image dimensions must be X,Y where X < Y")
    px = np.prod(dim)

    # Threads per block must be multiple of warp size, typically between 128 and 512
    if px % WARP != 0:
        raise Exception("Not possible to make # kernels a multiple of WARP {}; total pixel count {}".format(WARP, numKernels))

    # Subtract factors of 2 until we get WARP - this gives us certainty in matching warp size
    # This assumes that WARP is a power of 2
    dimSub = list(dim)
    fact = [1,1]
    while np.prod(fact) < WARP:
        if dimSub[0] % 1 != 0 or dimSub[1] % 1 != 0:
            raise Exception("Could not factor {} from image dimensions - got to {} with {}".format(WARP, fact, dimSub))
        if dimSub[0] < dimSub[1]:
            dimSub[1] = dimSub[1] / 2
            fact[1] = fact[1] * 2
        else:
           dimSub[0] = dimSub[0] / 2
           fact[0] = fact[0] * 2

    # Good starting guess, but maybe not evenly dividing the pixels
    threadguess = (WARP*4) * (mproc*2)
    K = sqrt(1.0/threadguess)
    raw = [ceil(dimSub[0]*K), ceil(dimSub[1]*K)]
    print("Raw bounds %s (ballpark %d threads)" % (raw, threadguess))

    # Align bounds so that they evenly divide the dimensions
    while dimSub[0] % raw[0] != 0:
        raw[0] += 1
    while dimSub[1] % raw[1] != 0:
        raw[1] += 1
    raw = np.multiply(raw, fact) # Add the factored values back in
    print("rawfact", raw)
    numKernels = np.divide(dim, raw)
    print("Adjusted %s" % raw)
    print("numKernels", numKernels)


    # Size bpg so it fits evenly given the kernel
    # Number of blocks in grid should be 2-4x the number of multiprocessors (1) -> 2-4.
    # bpgScale[X/Y] * bpg = threads[X/Y]
    # thread[X/Y] / bpgScale[X/Y] = bpg[X/Y] > 2*MPROC 
    # thread[X/Y] / 2*MPROC > bpgScale[X/Y]
    bpgScale = 1
    while True:
        bpg = tuple(np.ceil(np.divide(numKernels, bpgScale)).astype(int))
        if np.prod(bpg) < 4*mproc:
            break
        bpgScale += 1
    print("bpgScale", bpgScale)

    # Now come up with runnable 2d dimensions for the kernel
    # We want to size the threads per block so they fit within MAX_THREADS_PER_SM
    # and so that when multiplied elementwise by BLOCKS_PER_GRID they don't exceed 
    tpb = tuple(np.ceil(np.divide(numKernels, bpg)).astype(int))
    return (tuple(raw), bpg, tpb)

KB, BLOCKS_PER_GRID, THREADS_PER_BLOCK = kernel_bounds(dim, MPROC, MAX_THREADS_PER_SM)
print({
  "KB": KB, 
  "BLOCKS_PER_GRID": BLOCKS_PER_GRID, 
  "THREADS_PER_BLOCK": THREADS_PER_BLOCK,
  "dim composed": np.multiply(np.multiply(KB, THREADS_PER_BLOCK), BLOCKS_PER_GRID),
})

# One row per thread, with length allowing maximum compression size
NUM_SECTOR = np.product(np.multiply(BLOCKS_PER_GRID, THREADS_PER_BLOCK))

# Intrinsics provided by call to `rs-sensor-control`
rvl_cuda.configure(KB, NUM_SECTOR, {
    "ppx": 422.946, "ppy": 238.06,
    "fx": 426.192, "fy": 426.192,
})
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
