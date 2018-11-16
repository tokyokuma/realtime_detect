# -*- coding: utf-8 -*-

import numpy as np
import cythonTest as cp
import time

test = np.ones(10000000)

#pure python + cython
start = time.time()

cp.loopTest(test)

elapsed_time = time.time() - start
print(elapsed_time)



#cython with ctypedef
start = time.time()

cp.loopTestC(test)

elapsed_time = time.time() - start
print(elapsed_time)
