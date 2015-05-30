# Benchmarking of direct kinematic computation

The pantilt.py is to benchmark the computer installation with respect to direct kinematic computation and its Jacobian.

The performances are very linked to the use of numpy and the use of matrix computation acceleration like BLAS or ATLAS. To see the configuration of your numpy, type:

'''
import numpy
numpy.show_config()
'''

The model used here is a small robot pan tilt turret which configuration is described below.

(./pan_tilt.JPG)