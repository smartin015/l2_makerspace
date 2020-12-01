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
# Note that column order is height-first
PX_B = 2
input_dim_realsense = (720, 1280)
input_dim_realsense_reg = (480, 848)
input_dim_trvl = (576, 640)
dim = input_dim_realsense_reg

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
print("Kernel bounds: {} ({} pixels), thread shape: {}".format(KB, PX_KB, CUDA_THREAD_DIM))

# =========== CALCULATIONS ===========
# CUDA kernel. For each thread
@cuda.jit
def rvl_kernel(im, out):
    x, y = cuda.grid(2)
    outx = cuda.gridsize(2)[1]*x + y
    x = KB[0]*x
    y = KB[1]*y

    # Variable length encoding tracking
    word = 0
    nibblesWritten = 0
    
    # Write XY top left corner in pixel space
    out[outx][0] = (y << 16) | x
    outIdx = 1

    i = 0
    # TODO failing to write last section
    while i < PX_KB:
        zeros = 0
        nonzeros = 0
        while i < PX_KB:
            ix = x + (i % KB[0])
            iy = y + (i // KB[0])
            z = 0
            if im[ix][iy] == 0: # Or below delta
                z = 1

            # Stop when we've hit a new string of zeros
            if zeros != 0 and nonzeros != 0 and z == 1:
                break
            
            zeros += z
            nonzeros += (1-z)
            i += 1

        # Write out segment (zeros, nonzeros, values[])
        j = i - nonzeros
        while True:
            if zeros != -1:
                value = zeros
                zeros = -1
            elif nonzeros != -1:
                value = nonzeros
                nonzeros = -1
            else:
                jx = x + (j % KB[0])
                jy = y + (j // KB[0])

                # TODO add filtering, delta, time stuff
                value = im[jx][jy]
                j += 1

            # Embed negative values in positive space 
            # sequence is 0, 1, -1, 2, -2...
            value = (value << 1) ^ (value >> 15) # 16-bit values

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
                    if outIdx >= PX_KB:
                        break
                    out[outx][outIdx] = word
                    outIdx += 1
                    nibblesWritten = 0
                    word = 0

                # We're done when there's no more data
                if value == 0:
                    break
            
            # Check end condition after start of loop 
            if zeros == -1 and nonzeros == -1 and j >= i:
                break

    # Flush any remaining values, offsetting word to end first
    word = (word << (4 * (16-nibblesWritten)))
    out[outx][outIdx] = word
    

block_per_grid = (1,1) # tuple(np.divide(img.shape, CUDA_THREAD_DIM).astype(int))
from datetime import datetime

timings = []
# One row per thread, with length allowing maximum compression size
# TODO size PX_KB properly
result = np.zeros((NTHREADS, int((PX_KB / 2) + 1)), dtype=np.uint64)

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
while nsamp < 20:
    try:
        has, frame = pipeline.try_wait_for_frames()
        if not has:
            break
        print(frame)
        start = datetime.now()
        data = np.asanyarray(frame.get_depth_frame().get_data())
        rvl_kernel[block_per_grid, CUDA_THREAD_DIM](data, result)
        timings.append(datetime.now() - start)
        sample = data[0:KB[0], 0:KB[1]]
        nsamp += 1
    except KeyboardInterrupt:
        break

print("(Actual image shape received: {} dtype {})".format(data.shape, data.dtype))

def decodeSector(sector, result):
    start = datetime.now()

    # Extract header
    x = int(sector[0]) & 0xffff
    y = (int(sector[0]) >> 16) & 0xffff

    i = 1 # Skip header word
    word = 0
    nibblesWritten = 0
    zeros = None
    nonzeros = None
    outIdx = 0
    written = 0
    while True:
        # Decode variable
        nibble = np.uint64(0)
        value = np.uint16(0)
        bits = 61 # 64 - 3 bits
        while True:
            if not nibblesWritten:
                if i >= len(sector):
                    break
                word = int(sector[i])
                i += 1
                nibblesWritten = 16
            nibble = word & 0xf000000000000000
            value = value | ((nibble << 1) & 0xffffffffffffffff) >> bits
            word = (word << 4) & 0xffffffffffffffff
            nibblesWritten -= 1
            bits -= 3
            if not (nibble & 0x8000000000000000):
                break
        
        # Value is now assigned; reverse positive flipping from compression
        value = int(value)
        value = (value >> 1) ^ -(value & 1)

        # Reset counts if we've written a set of [zero, nonzero, data]
        if zeros == 0 and nonzeros == 0:
            return ((x,y), result, datetime.now() - start)
        elif zeros is not None and nonzeros is not None and written >= nonzeros:
            zeros = None
            nonzeros = None
            written = 0

        if zeros is None:
            zeros = value
            outIdx += zeros
        elif nonzeros is None:
            nonzeros = value
        else: # written < nonzeros
            if outIdx >= PX_KB:
                return ((x,y), result, datetime.now() - start)
            result[x+int(outIdx % KB[0])][y+int(outIdx // KB[0])] = value
            written += 1
            outIdx += 1

    return ((x,y), result, datetime.now() - start)




cuda.close()
# np.set_printoptions(threshold=np.inf)
print("{} result ({} initial, {} post-avg, {} samples)".format(
          result.shape, timings[0], np.mean(timings[1:]), len(timings[1:])))

cv2.imshow("original", data / np.max(data))
cv2.imshow("encoded", result / np.max(result))

# Decode each part of the original image
import concurrent.futures
decodeImg = np.zeros(dim, dtype=np.uint16)
cpu_timings = []
allstart = datetime.now()
with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
    futures = []
    for i in range(result.shape[0]):
        futures.append(executor.submit(decodeSector, result[i], decodeImg))
    for future in concurrent.futures.as_completed(futures):
        (s, result, threadtime) = future.result()
        cpu_timings.append(threadtime.total_seconds())
        print("decoded {}".format(s))
        cv2.imshow("decodeImg", decodeImg / np.max(decodeImg))
        cv2.waitKey(30)
walltime = datetime.now() - allstart

print("""=== Summary Performance Stats ===
    Encode first frame: {}
    Encode warmed-up wall time (avg): {}
    Encode #samples: {}
    Decode wall time: {}
    Decode avg sec/thread: {}
    Decode fastest thread (s): {}
    Decode slowest thread (s): {}""".format(
        timings[0], np.mean(timings[1:]), len(timings[1:]),
        walltime, np.mean(cpu_timings), np.min(cpu_timings), np.max(cpu_timings)
    ))


cv2.waitKey(0)
#sample = sample / np.max(sample)
#ss_re = cv2.resize(sample, resize, interpolation=cv2.INTER_AREA)
#cv2.imshow("sample sector", ss_re)
# de_re = cv2.resize(decoded / np.max(decoded), resize, interpolation=cv2.INTER_AREA)
# cv2.imshow("decoded", de_re) 
#diff = np.abs(sample - decoded)
#diff_re = cv2.resize(diff, resize, interpolation=cv2.INTER_AREA)
#cv2.imshow("sample vs decode diff (black=good)", diff_re / max(np.max(diff_re), 1))

# Compute thread image bounds Ioffs, Iwidth, Iheight = (threadID * stride, M/(stride1/N), N/(stride2/M))
# Loop i = 0...Iwidth*Iheight
# Run RVL algorithm
# If dest space exceeded,  reset command to "raw", then copy image bounds right shifted 1
# Send each row in dest as UDP packet
# 7200B output @ 20x compression + 28B IP header + 3B preamble -> 391 bytes per thread-frame
# Frame overhead is 0.83%, contributes 3*256*30*8=184kbps to total bitrate. Each byte is ~61kbps
# @ 30hz across 256 threads -> 3MB -> 24mbps
# Original is 1280*720*30*2*8 -> 442mbps, so 18.4x reduction
# Frame footprint vs TRVL research paper is (1280*720) / (640*576) = 2.5X
