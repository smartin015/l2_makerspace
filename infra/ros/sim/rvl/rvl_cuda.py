from numba import cuda
from math import sqrt
import numpy as np
import cv2
import pyrealsense2 as rs

# 128 cores
# Threads per block must be multiple of warp size (32), typically between 128 and 512
# Number of blocks in grid should be 2-4x the number of multiprocessors (1) -> 2-4.
WARP = 32
MPROC = 1

THREADS_PER_BLOCK = WARP*4
BLOCKS_PER_GRID = MPROC*2
NTHREADS = THREADS_PER_BLOCK * BLOCKS_PER_GRID

print("%d threads (%d per block, %d blocks per grid)" % (
    NTHREADS, THREADS_PER_BLOCK, BLOCKS_PER_GRID))

# Load test image M*N*bytes
PX_B = 2
input_dim_realsense = (1280, 720)
input_dim_trvl = (640, 576)
dim = input_dim_realsense

# Allocate destination space M*(header+N)
# header = 1B command + 2B identifier
dest = "TODO"

# Compute kernel bounds. Example:
# 1280x720 -> 256 threads -> 3600 16z samples -> 7200B max output per thread 
# X*Y = M*N/T, X=K*M, Y=K*N, X*Y = K^2*M*N, K = sqrt((X*Y)/(M*N)) = sqrt((M*N/T)/(M*N)) = sqrt(1/T),
# X = 1280*sqrt(1/256) = 80
# Y = 720*sqrt(1/256) = 45
# Each thread is an 80x45 segment of the 16z image
K = sqrt(1.0/NTHREADS)
def kernel_bounds(dim):
    px = np.prod(dim)
    tpx = px/NTHREADS
    return (int(dim[0]*K), int(dim[1]*K))


KB = kernel_bounds(dim)
CUDA_THREAD_DIM = tuple(np.divide(dim, KB).astype(int))
PX_KB = np.prod(KB)
print("Kernel bounds: {}, thread shape: {}".format(KB, CUDA_THREAD_DIM))

# =========== CALCULATIONS ===========
# CUDA kernel. For each thread
@cuda.jit
def rvl_kernel(im, out):
    x, y = cuda.grid(2)
    outx = CUDA_THREAD_DIM[0]*y + x # Linear output id
    x = KB[0]*x
    y = KB[1]*y
    im[x][y] += 1
    
    # TODO preamble
    outIdx = 0
    out[outx][outIdx] = 5
    outIdx += 1
    
    i = 0
    while i < PX_KB:
        zeros = 0
        zstop = False
        zadd = 1
        while i < PX_KB and zadd:
            ix = x + (i % KB[0])
            iy = y + (i // KB[0])
            zadd = (im[ix][iy] == 0) # Or below delta
            zeros += zadd
            i += 1
        # TODO variable length encoding
        out[outx][outIdx] = zeros
        outIdx += 1
        
        nonzeros = 0
        nzadd = 1
        while i+nonzeros < PX_KB and nzadd:
            ix = x + (i % KB[0])
            iy = y + (i // KB[0])
            nzadd = (im[ix][iy] != 0) # And above delta
            nonzeros += nzadd
            i += 1
        out[outx][outIdx] = nonzeros
        outIdx += 1
    
        #j = 0
        #while j < nonzeros:
        #    # TODO delta, VLE
        #    jx = x + (j % KB[0])
        #    jy = y + (i // KB[0])
        #    out[outx][outIdx] = im[jx][jy]
        #    outIdx += 1

     #TODO flush
    

block_per_grid = (1,1) # tuple(np.divide(img.shape, CUDA_THREAD_DIM).astype(int))
from datetime import datetime

timings = []
# One row per thread, with length allowing maximum compression size
result = np.zeros((NTHREADS, PX_KB+1), dtype=np.uint16)

pipeline = rs.pipeline()
config = rs.config()
rs.config.enable_device_from_file(config, "./a.bag")
#config.enable_stream(rs.stream.depth, dim[0], dim[1], rs.format.z16, 30)
pipeline.start(config)

print("Running on img shape {}, output shape {} (Ctl+C to stop)...".format(dim, result.shape))
while True:
    try:
        has, frame = pipeline.try_wait_for_frames()
        if not has:
            break
        print(frame)
        start = datetime.now()
        data = np.asanyarray(frame.get_depth_frame().get_data())
        rvl_kernel[block_per_grid, CUDA_THREAD_DIM](data, result)
        timings.append(datetime.now() - start)
    except KeyboardInterrupt:
        break

print("Result ({} initial, {} post-avg, {} samples):\n{}".format(timings[0], np.mean(timings[1:]), len(timings[1:]), result[0:8][0:20]))

cv2.imshow("img", result / np.max(result))
cv2.waitKey(0)

# Compute thread image bounds Ioffs, Iwidth, Iheight = (threadID * stride, M/(stride1/N), N/(stride2/M))
# Loop i = 0...Iwidth*Iheight
# Run RVL algorithm
# If dest space exceeded,  resent command to "raw", then copy image bounds right shifted 1
# Send each row in dest as UDP packet
# 7200B output @ 20x compression + 28B IP header + 3B preamble -> 391 bytes per thread-frame
# Frame overhead is 0.83%, contributes 3*256*30*8=184kbps to total bitrate. Each byte is ~61kbps
# @ 30hz across 256 threads -> 3MB -> 24mbps
# Original is 1280*720*30*2*8 -> 442mbps, so 18.4x reduction
# Frame footprint vs TRVL research paper is (1280*720) / (640*576) = 2.5X
