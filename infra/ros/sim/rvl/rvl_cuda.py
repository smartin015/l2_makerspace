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
def rvl_kernel(im, out, dbg):
    x, y = cuda.grid(2)
    outx = CUDA_THREAD_DIM[0]*y + x # Linear output id
    x = KB[0]*x
    y = KB[1]*y
    # im[x][y] += 1

    # Variable length encoding tracking
    word = 0
    nibblesWritten = 0
    
    # Write XY top left corner in pixel space
    out[outx][0] = (y << 16) | x
    outIdx = 1

    dbgIdx = 1    
    i = 0
    while i < PX_KB:
        dbg[outx][0] += 1 # Num iterations
        # Count zeros and nonzeros
        zeros = 0
        nonzeros = 0
        z = 1
        while i < PX_KB and (zeros == 0 or nonzeros == 0 or not z):
            ix = x + (i % KB[0])
            iy = y + (i // KB[0])
            z = (im[ix][iy] == 0) # Or below delta
            zeros += z
            nonzeros += (not z)
            i += 1
        
        # Write out segment (zeros, nonzeros, values[])
        j = i - nonzeros
        while j < i and outIdx < PX_KB:
            if zeros != -1:
                value = zeros
                zeros = -1
                dbg[outx][dbgIdx] = value
                dbgIdx += 1
            elif nonzeros != -1:
                value = nonzeros
                nonzeros = -1
                dbg[outx][dbgIdx] = value
                dbgIdx += 1
            else:
                jx = x + (j % KB[0])
                jy = y + (j // KB[0])

                # TODO add filtering, delta, time stuff
                value = im[jx][jy]
                j += 1

            # Embed negative values in positive space 
            # sequence is 0, 1, -1, 2, -2...
            value = (value << 1) ^ (value >> 31)

            # Write data (variable length encoding)
            while True:
                # Pop lower 3 bits from value
                nibble = value & 0x7
                value = (value >> 3)

                # Indicate more data if some value remains
                if value != 0:
                    nibble |= 0x8
                
                # Append nibble to word
                word = (word << 4) | nibble 
                nibblesWritten += 1

                # Flush word when complete (Tegra is 64bit architecture)
                if nibblesWritten == 16: 
                    out[outx][outIdx] = word
                    outIdx += 1
                    nibblesWritten = 0
                    word = 0

                # We're done when there's no more data
                if value == 0:
                    break
     #TODO flush
    

block_per_grid = (1,1) # tuple(np.divide(img.shape, CUDA_THREAD_DIM).astype(int))
from datetime import datetime

timings = []
# One row per thread, with length allowing maximum compression size
# TODO size PX_KB properly
result = np.zeros((NTHREADS, PX_KB+1), dtype=np.uint64)
dbg = np.zeros((NTHREADS, 256), dtype=np.int16) # TODO remove for better performance

pipeline = rs.pipeline()
config = rs.config()
rs.config.enable_device_from_file(config, "./a.bag")
#config.enable_stream(rs.stream.depth, dim[0], dim[1], rs.format.z16, 30)
pipeline.start(config)

ctx = cuda.current_context()
ctx.reset()
print(ctx.get_memory_info())

print("Running on img shape {}, output shape {} (Ctl+C to stop)...".format(dim, result.shape))
sample = None
nsamp = 0
while nsamp < 3:
    try:
        has, frame = pipeline.try_wait_for_frames()
        if not has:
            break
        print(frame)
        start = datetime.now()
        data = np.asanyarray(frame.get_depth_frame().get_data())
        rvl_kernel[block_per_grid, CUDA_THREAD_DIM](data, result, dbg)
        timings.append(datetime.now() - start)
        sample = data[0:KB[0]][0:KB[1]]
        nsamp += 1
    except KeyboardInterrupt:
        break


def decodeSector(sector):
    x = int(sector[0]) & 0xffff
    y = (int(sector[0]) >> 16) & 0xffff
    print("decoding sector {}, {}".format(x,y))

    i = 1
    word = 0
    nibblesWritten = 0

    dbg = [0]

    zeros = None
    nonzeros = None
    written = 0
    result = np.zeros(KB, dtype=np.uint16)
    while i < len(sector):
        # Decode variable
        nibble = 0
        value = 0
        bits = 61 # 64 - 3 bits
        while True:
            if not nibblesWritten:
                word = int(sector[i])
                i += 1
                nibblesWritten = 16
            nibble = word & 0xf000000000000000
            value = value | (nibble << 1) >> bits
            word = (word << 4)
            nibblesWritten -= 1
            bits -= 3
            if not (nibble & 0x8000000000000000):
                break
            
        # Value is now assigned; reverse positive flipping from compression
        value = (value >> 1) ^ -(value & 1)
        if zeros is None:
            zeros = value
            written += zeros
        elif nonzeros is None:
            nonzeros = value
        elif zeros == 0 and nonzeros == 0:
            return (result, dbg)
        elif written < nonzeros:
            result[written % KB[0]][written // KB[0]] = value
            written += 1
        else:
            dbg[0] += 1 # Num sections written
            dbg.append(zeros)
            dbg.append(nonzeros)
            zeros = None
            nonzeros = None
            written = 0
    return result




cuda.close()
# np.set_printoptions(threshold=np.inf)
print("{} result ({} initial, {} post-avg, {} samples):\n"
      "top left quadrant\n{}\npre-encoded\n{}".format(
          result.shape, timings[0], np.mean(timings[1:]), len(timings[1:]), 
          sample, dbg[0]))

# Count zeroes and nonzeros from debug info
pre_zeros = 0
pre_nonzeros = 0
for i in range(1, len(dbg[0])):
    if i % 2:
        pre_nonzeros += dbg[0][i]
    else:
        pre_zeros += dbg[0][i]
sample_nonzeros = np.count_nonzero(sample)
print("pre-encoded sums: %d zeros, %d zeros\nactual image: %d zeros, %d nonzeros" % (pre_zeros, pre_nonzeros, len(sample)-sample_nonzeros, sample_nonzeros))

decoded, decodeDbg = decodeSector(result[0])
print("decode dbg\n{}\ndecoded\n{}".format(decodeDbg, decoded))

cv2.imshow("sample encoded", sample / np.max(sample))
cv2.imshow("img", result / np.max(result))
cv2.imshow("sample decoded", decoded / np.max(decoded))
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
