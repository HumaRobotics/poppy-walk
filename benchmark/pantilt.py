# -*- coding: utf-8 -*-

from numpy import *
import time

def MGD(q,mode = "direct kinematic and jacobian",blas = "with blas"):

    #geo metric parameters (m)
    # length of the arm    
    L = 0.1
    # height of the neck
    H = 0.05
    
    #definition of axes
    x=array([1,0,0])
    y=array([0,1,0])
    z=array([0,0,1])
    
    #computation of sin and cos
    s = sin(q*pi/180.0)
    c = cos(q*pi/180.0)
    
    #function to compute rotation matrices
    def rot_mat(axe,s,c):
        if axe == "x":
            return array([[1,0,0],[0,c,-s],[0,s,c]])
        elif axe == "y":
            return array([[c,0,s],[0,1,0],[-s,0,c]])
        elif axe == "z":
            return array([[c,-s,0],[s,c,0],[0,0,1]])
        else:
            return 0
            
    #individual rotation matrices
    # the first articulation links the base to the turret around z axis
    rot0=rot_mat("z",s[0],c[0])
    # the second articulation is about the pitch of the turret around y axis
    rot1=rot_mat("y",s[1],c[1])
    
    # composition of rotations
    t0 = rot0
    t1 = dot(rot0,rot1)
    
    # computation of individual vectors
    OA = H*dot(t0,z)
    AM = L*dot(t1,x)
    
    # sum of vectors
    OM = OA+AM
    
    # computation of Jacobian of M (it is the speed vector around the considered articulation)
    
    def cross_prod(a,b):
        ax = array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
        return dot(ax,b)
        
    if mode == "direct kinematic and jacobian":
      JM = zeros((3,2))
      if blas == "with blas":
        JM[:,0] = cross_prod(z,OM)
        JM[:,1] = cross_prod(dot(t0,y),AM)
      else:
        JM[:,0] = cross(z,OM)
        JM[:,1] = cross(dot(t0,y),AM)
    return OM



q = array([0.1,0.2])
T0 = time.time()
for i in range(1000): 
    OM = MGD(q)
dT = time.time()-T0

# without J computation
T0 = time.time()
for i in range(1000): 
    OM = MGD(q,mode="direct kinematic")
dTwoJ = time.time()-T0

# without BLAS
# without J computation
T0 = time.time()
for i in range(1000): 
    OM = MGD(q,blas="without blas")
dTwoBLAS = time.time()-T0

print "end effector"
print OM
print "computation time in micros"
print dT*1000
print "computation time in micros without Jacobian"
print dTwoJ*1000
print "computation time in micros without BLAS"
print dTwoBLAS*1000

print "BLAS gain"
print (dTwoBLAS-dTwoJ)/(dT-dTwoJ)
