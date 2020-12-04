from numba import cuda
from math import sqrt, ceil
import numpy as np
import cv2
import pyrealsense2 as rs

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
PX_KB = np.prod(KB)
print({
  "KB": KB, 
  "PX_KB": PX_KB, 
  "BLOCKS_PER_GRID": BLOCKS_PER_GRID, 
  "THREADS_PER_BLOCK": THREADS_PER_BLOCK,
  "dim composed": np.multiply(np.multiply(KB, THREADS_PER_BLOCK), BLOCKS_PER_GRID),
})

# =========== CALCULATIONS ===========
# CUDA kernel. For each thread
@cuda.jit
def rvl_encode(im, out):
    x, y = cuda.grid(2)
    outx = cuda.gridsize(2)[1]*x + y
    #print(x, y, g[0], g[1])
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


@cuda.jit()
def rvl_decode(enc, out):
    encx = cuda.grid(1)
    # Extract header & start on next word
    x = enc[encx][0] & 0xffff
    y = (enc[encx][0] >> 16) & 0xffff
    i = 1

    word = np.uint64(0)
    nibblesWritten = 0
    zeros = -1
    nonzeros = -1
    outIdx = 0
    written = 0
    bits = 61
    value = 0
    while True:
        # Decode variable
        value = 0
        nibble = 0
        bits = 61 # 64 - 3 bits
        while True:
            if i >= SECTOR_LEN:
                return
            if nibblesWritten == 0:
                word = enc[encx][i]
                i += 1
                nibblesWritten = 16
            nibble = word & 0xf000000000000000
            value = value | (nibble << 1) >> bits
            word = word << 4
            nibblesWritten -= 1
            bits -= 3
            if not (nibble & 0x8000000000000000):
                break
        
        # Value is now assigned; reverse positive flipping from compression
        value = (value >> 1) ^ -(value & 1)

        # Reset counts if we've written a set of [zero, nonzero, data]
        if zeros == 0 and nonzeros == 0:
            return
        elif zeros != -1 and nonzeros != -1 and written >= nonzeros:
            zeros = -1
            nonzeros = -1
            written = 0

        if zeros == -1:
            zeros = value
            outIdx += zeros
        elif nonzeros == -1:
            nonzeros = value
        else: # written < nonzeros
            if outIdx >= PX_KB:
                return
            out[x + (outIdx % KB[0])][y + (outIdx // KB[0])] = value
            written += 1
            outIdx += 1

        # Check end conditions
        #if i >= SECTOR_LEN or outIdx >= PX_KB:
        #    break

from datetime import datetime
# One row per thread, with length allowing maximum compression size
SECTOR_LEN = int(PX_KB / 2) # TODO size SECTOR_LEN properly
NUM_SECTOR = np.product(np.multiply(BLOCKS_PER_GRID, THREADS_PER_BLOCK))
encoded = np.zeros((NUM_SECTOR, SECTOR_LEN), dtype=np.uint64)
decoded = np.zeros(dim, dtype=np.uint16)

pipeline = rs.pipeline()
config = rs.config()
rs.config.enable_device_from_file(config, "./a.bag")
#config.enable_stream(rs.stream.depth, dim[0], dim[1], rs.format.z16, 30)
pipeline.start(config)

ctx = cuda.current_context()
ctx.reset()
print(ctx.get_memory_info())
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
        rvl_encode[BLOCKS_PER_GRID, THREADS_PER_BLOCK](data, encoded)
        timings[0].append(datetime.now() - start)
        
	print([hex(e) for e in encoded[529][0:10]]) # 529 80w624h
        start = datetime.now()
        rvl_decode[4, int(NUM_SECTOR/4)](encoded, decoded)
        timings[1].append(datetime.now() - start)
        #print(rvl_decode.inspect_types())
        #import sys
        #sys.exit(0)
        print(decoded[80][624:634])
        nsamp += 1
    except KeyboardInterrupt:
        break

cv2.imshow("original", data / np.max(data))
cv2.imshow("encoded", encoded / max(1, np.max(encoded)))
cv2.imshow("decoded", decoded / max(1, np.max(decoded)))

cuda.close()

print("""=== Summary Performance Stats ===
    #samples: {}
    First frame: 
        {} encode
        {} decode
    Warmed-up wall time (avg): 
        {} encode
        {} decode""".format(
        len(timings[1:]),
        timings[0][0], np.mean(timings[1][1:]), 
        timings[1][0], np.mean(timings[1][1:])
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
