from numba import cuda
import numpy as np
from math import tan, sqrt
KB = None
PX_KB = None
SECTOR_LEN = None
NUM_SECTOR = None
FLT_EPSILON = 0.000001
# ppx, ppy, fx, fy, coeffs[0...3]
PPX = 0.0
PPY = 0.0
FX = 0.0
FY = 0.0
#COEFFS = [0.0, 0.0, 0.0, 0.0]

def configure(kb, num_sector, intrinsics=None):
    global KB, PX_KB, SECTOR_LEN, NUM_SECTOR, PPX, PPY, FX, FY, COEFFS
    KB = kb
    PX_KB = kb[0] * kb[1]
    SECTOR_LEN = int(PX_KB / 2) # TODO size properly
    NUM_SECTOR = num_sector
    if intrinsics is not None:
        PPX = intrinsics["ppx"]
        PPY = intrinsics["ppy"]
        FX = intrinsics["fx"]
        FY = intrinsics["fy"]
        #COEFFS = intrinsics["coeffs"]

# =========== CALCULATIONS ===========
# CUDA kernel. For each thread
@cuda.jit
def encode(im, out):
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
def decode(enc, out, deproject=False):
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
            if not deproject:
                # Write out to image
                out[x + (outIdx % KB[0])][y + (outIdx // KB[0])] = value
            else: # Simple deprojection (assume no distortion model)
                # Copied from https://github.com/IntelRealSense/librealsense/blob/master/include/librealsense2/rsutil.h#L67
                px = float(x + (outIdx % KB[0]))
                py = float(y + (outIdx // KB[0]))
                offs = int((x + y*KB[0]) + outIdx)
                out[offs][0] = value * (px - PPX) / FX 
                out[offs][1] = value * (py - PPY) / FY
                out[offs][2] = value
            written += 1
            outIdx += 1
