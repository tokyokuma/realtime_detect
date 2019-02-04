from distutils.core import setup
from Cython.Build import cythonize
import numpy

setup(
     ext_modules = cythonize("/home/nvidia/tools/ESPNetv2/segmentation/cnn/fast_ESPNetv2.pyx"),
     include_dirs = [numpy.get_include()]
)
