# -*- coding: utf-8 -*-
#cython: boundscheck=False
#cython: wraparound=False

import numpy as np
cimport numpy as np
cimport cython

DTYPE = np.int64
ctypedef np.int64_t DTYPE_t
cdef int all_class_num = 9
cdef int height = 360 / 2
cdef int width = 640 / 2
cdef int all_pixels = height * width

def calc_area(np.ndarray[DTYPE_t, ndim=2] f):
    cdef float[:] class_area = np.zeros(all_class_num, dtype=np.float32)
    cdef int y, x, i, j, pixel
    for y in range(0, height):
        y = y * 2
        for x in range(0, width):
            x = x * 2
            pixel = f.item(y,x)
            for i in range(0, all_class_num):
                if pixel == i:
                    class_area[i] = class_area[i] + 1
                else:
                    pass

    for j in range(0, all_class_num):
        class_area[j] = class_area[j] / all_pixels * 100
        print str(j) + ' : ' + str(class_area[j]) + '%'
