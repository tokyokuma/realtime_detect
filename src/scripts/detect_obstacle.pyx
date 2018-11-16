# -*- coding: utf-8 -*-
#cython: boundscheck=False
#cython: wraparound=False

import numpy as np
cimport numpy as np
cimport cython

DTYPE = np.int64
ctypedef np.int64_t DTYPE_t
cdef int height = 360 / 2
cdef int width = 640 / 2
cdef int all_pixels = height * width

def calc_area(np.ndarray[DTYPE_t, ndim=2] f):
    cdef float[:] obstacle_coordination = np.zeros([2,2,2], dtype=np.float32)
    cdef int y, x, i, j, pixel
    for y in range(0, height):
        y = y * 2
        for x in range(0, width):
            x = x * 2
            pixel = f.item(y,x)
            if pixel == 0 or pixel == 1:
                obstacle_coordination[0][0][0]
                obstacle_coordination[0][0][1]

    return
