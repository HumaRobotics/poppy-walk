# Benchmarking of direct kinematic computation

The pantilt.py is to benchmark the computer installation with respect to direct kinematic computation and its Jacobian.

The performances are very linked to the use of numpy and the use of matrix computation acceleration like BLAS or ATLAS. To see the configuration of your numpy, type:

```
import numpy
numpy.show_config()
```

If the computer is optimized for matrix computation, the clue is to use as far as possible matrix multiplication with numpy. Here, the Jacobian computation is done with numpy.cross function. But this fonction only use multiplication in python. If the cross function is done via matrix multiplication, it can go 3 times faster!

# robot model used for benchmark

The model used here is a small robot pan tilt turret which configuration is described below.

![sketch](pan_tilt.JPG)


# results on different computers
For the installation on Poppy "Chouchou" version on ODROID U3, the results are :

```
end effector
[ 0.09999924  0.00017453  0.04965093]
computation time in micros
179.43406105
computation time in micros without Jacobian
115.551948547
computation time in micros without BLAS
292.736053467
BLAS gain
2.77361060831
```

For a intel core i5 with Pythonxy the results are :

```
end effector
[ 0.09999924  0.00017453  0.04965093]
computation time in micros
46.9999313354
computation time in micros without Jacobian
15.0001049042
computation time in micros without BLAS
77.999830246
BLAS gain
1.96875209549
```
