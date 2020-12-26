from functools import reduce
import numpy as np
from numba import cuda
from math import sqrt, ceil

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

def factors(n):    
        return set(reduce(list.__add__, 
                            ([i, n//i] for i in range(1, int(n**0.5) + 1) if n % i == 0)))

# Compute kernel bounds based on estimated thread count. Example:
# 1280x720 -> 256 threads -> 3600 16z samples -> 7200B max output per thread 
# X*Y = M*N/T, X=K*M, Y=K*N, X*Y = K^2*M*N, K = sqrt((X*Y)/(M*N)) = sqrt((M*N/T)/(M*N)) = sqrt(1/T),
# X = 1280*sqrt(1/256) = 80
# Y = 720*sqrt(1/256) = 45
# In this example, each thread is an 80x45 segment of the 16z image
def kernel_bounds(dim, mproc=MPROC, sm_cores=MAX_THREADS_PER_SM):
    print("Warp: {0}\tMultiprocessors: {1}\tThread limit per mproc: {2}".format(WARP, MPROC, MAX_THREADS_PER_SM))
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
    print("Warp factor:", fact)
    # Good starting guess, but maybe not evenly dividing the pixels
    threadguess = (WARP*2) * (mproc*8)
    print("Threadcount guess:", threadguess)
    
    best = (1,1,dimSub[0]*dimSub[1]-threadguess)
    for i in factors(dimSub[0]):
        for j in factors(dimSub[1]):
            diff = abs((dimSub[0]/i) * (dimSub[1]/j) - threadguess)
            if diff < best[2]:
                best = (i,j,diff)
    
    # Align bounds so that they evenly divide the dimensions
    raw = np.multiply([int(best[0]), int(best[1])], fact) # Add the factored values back in
    numKernels = np.divide(dim, raw)

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

    # Now come up with runnable 2d dimensions for the kernel
    # We want to size the threads per block so they fit within MAX_THREADS_PER_SM
    # and so that when multiplied elementwise by BLOCKS_PER_GRID they don't exceed 
    tpb = tuple(np.ceil(np.divide(numKernels, bpg)).astype(int))
    nsec = np.product(np.multiply(bpg, tpb))
    print("{kb} kernel size ({numKernels} per frame), {bpg} blocks per grid, {tpb} threads per block, {nsec} sectors".format(kb=tuple(raw), numKernels=numKernels, bpg=bpg, tpb=tpb, nsec=nsec))
    return (tuple(raw), bpg, tpb, nsec)
