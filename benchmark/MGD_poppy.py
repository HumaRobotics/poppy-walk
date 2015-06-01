from numpy import *
import time

def MGD(q, theta, phi):
    # q shape
    # q[0] : abs_y
    # q[1] : abs_x
    # q[2] : abs_z
    # q[3] : bust_y
    # q[4] : bust_x
    # q[5] : l_shoulder_y
    # q[6] : l_shoulder_x
    # q[7] : l_arm_z
    # q[8] : l_elbow_y
    # q[9] : r_shoulder_y
    # q[10] : r_shoulder_x
    # q[11] : r_arm_z
    # q[12] : r_elbow_y
    # q[13] : head_z
    # q[14] : head_y
    # q[15] : l_hip_x
    # q[16] : l_hip_z
    # q[17] : l_hip_y
    # q[18] : l_knee_y
    # q[19] : l_ankle_y
    # q[20] : r_hip_x
    # q[21] : r_hip_z
    # q[22] : r_hip_y
    # q[23] : r_knee_y
    # q[24] : r_ankle_y
    
    # geometric data
    # shoulder position on y
    ysh = 0.11
    # length of upper arm
    uarm = 0.15
    # lenght of forearm
    farm = 0.12
    # distance between sternum and shoulder axis
    chest = 0.05
    # distance between stomach and sternum
    stom = 0.14
    # length of neck
    neck = 0.13
    # length of nose
    nose = 0.1
    # distance between stomach and pelvis axis
    pelv = 0.07
    # hip position on y
    yhip = 0.05
    # length of thigh
    thigh = 0.18
    # length of leg
    leg = 0.23
    # toe position on x
    xtoe = 0.12
    # heel position on x
    xheel = 0.04
    
    #definition of axes
    x=array([1,0,0])
    y=array([0,1,0])
    z=array([0,0,1])
    
    # computation of sin and cos
    s = sin(q*pi/180.0)
    c = cos(q*pi/180.0)
    sphi = sin(phi*pi/180.0)
    cphi = cos(phi*pi/180.0)
    stheta = sin(theta*pi/180.0)
    ctheta = cos(theta*pi/180.0)
    
    def rot_mat(axe,s,c):
        if axe == "x":
            return array([[1,0,0],[0,c,-s],[0,s,c]])
        elif axe == "y":
            return array([[c,0,s],[0,1,0],[-s,0,c]])
        elif axe == "z":
            return array([[c,-s,0],[s,c,0],[0,0,1]])
        else:
            return 0
    
    rot0=rot_mat("y",stheta,ctheta)
    rot1=rot_mat("x",sphi,cphi)
    
    rott0=rot_mat("y",s[0],c[0])
    rott1=rot_mat("x",s[1],c[1])
    rott2=rot_mat("z",s[2],c[2])
    rott3=rot_mat("y",s[3],c[3])
    rott4=rot_mat("x",s[4],c[4])
    
    rotal0=rot_mat("y",s[5],c[5])
    rotal1=rot_mat("x",s[6],c[6])
    rotal2=rot_mat("z",s[7],c[7])
    rotal3=rot_mat("y",s[8],c[8])
    
    rotar0=rot_mat("y",s[9],c[9])
    rotar1=rot_mat("x",s[10],c[10])
    rotar2=rot_mat("z",s[11],c[11])
    rotar3=rot_mat("y",s[12],c[12])
    
    roth0=rot_mat("z",s[13],c[13])
    roth1=rot_mat("y",s[14],c[14])
    
    rotll0=rot_mat("x",s[15],c[15])
    rotll1=rot_mat("z",s[16],c[16])
    rotll2=rot_mat("y",s[17],c[17])
    rotll3=rot_mat("y",s[18],c[18])
    rotll4=rot_mat("y",s[19],c[19])
    
    rotlr0=rot_mat("x",s[20],c[20])
    rotlr1=rot_mat("z",s[21],c[21])
    rotlr2=rot_mat("y",s[22],c[22])
    rotlr3=rot_mat("y",s[23],c[23])
    rotlr4=rot_mat("y",s[24],c[24])
    
    # transformations
    t1 = dot(rot0,rot1)
    tt0 = dot(t1,rott0)
    tt1 = dot(tt0,rott1)
    tt3 = dot(tt1,dot(rott2,rott3))
    tt4 = dot(tt3,rott4)
    tar0 = dot(tt4,rotar0)
    tar1 = dot(tar0,rotar1)
    tar2 = dot(tar1,rotar2)
    tar3 = dot(tar2,rotar3)
    tal0 = dot(tt4,rotal0)
    tal1 = dot(tal0,rotal1)
    tal2 = dot(tal1,rotal2)
    tal3 = dot(tal2,rotal3)
    th0 = dot(tt4,roth0)
    th1 = dot(th0,roth1)
    tlr0 = dot(t1,rotlr0)
    tlr2 = dot(tlr0,dot(rotlr1,rotlr2))
    tlr3 = dot(tlr2,rotlr3)
    tlr4 = dot(tlr3,rotlr4)
    tll0 = dot(t1,rotll0)
    tll2 = dot(tll0,dot(rotll1,rotll2))
    tll3 = dot(tll2,rotll3)
    tll4 = dot(tll3,rotll4)
    
    def cross_prod(a,b):
        ax = array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
        return dot(ax,b)
        
    # computation of end effectors
    # D point (sternum)
    D={}
    GD = -dot(tt1,z)*stom
    D["pos"] = GD
    D["J"] = zeros((3,25))
    D["J"][:,10] = cross_prod(y,GD)
    D["J"][:,11] = cross_prod(dot(tt0,x),GD)
    
    # C point (right shoulder)
    C={}
    DC = dot(tt4,y)*ysh-dot(tt4,z)*chest
    GC = GD+DC
    C["pos"] = GC
    C["J"] = zeros((3,25))
    C["J"][:,10] = cross_prod(y,GC)
    C["J"][:,11] = cross_prod(dot(tt0,x),GC)
    C["J"][:,12] = cross_prod(dot(tt1,z),GC)
    C["J"][:,13] = cross_prod(dot(tt3,y),DC)
    C["J"][:,14] = cross_prod(dot(tt4,x),DC)
    
    # E point (left shoulder)
    E={}
    DE = -dot(tt4,y)*ysh-dot(tt4,z)*chest
    GE = GD+DE
    E["pos"] = GE
    E["J"] = zeros((3,25))
    E["J"][:,10] = cross_prod(y,GE)
    E["J"][:,11] = cross_prod(dot(tt0,x),GE)
    E["J"][:,12] = cross_prod(dot(tt1,z),GE)
    E["J"][:,13] = cross_prod(dot(tt3,y),DE)
    E["J"][:,14] = cross_prod(dot(tt4,x),DE)
    
    # B point (right elbow)
    B = {}
    CB = dot(tar2,z)*uarm
    DB = DC+CB
    GB = GC+CB
    B["pos"] = GB
    B["J"] = zeros((3,25))
    B["J"][:,10] = cross_prod(y,GB)
    B["J"][:,11] = cross_prod(dot(tt0,x),GB)
    B["J"][:,12] = cross_prod(dot(tt1,z),GB)
    B["J"][:,13] = cross_prod(dot(tt3,y),DB)
    B["J"][:,14] = cross_prod(dot(tt4,x),DB)
    B["J"][:,4] = cross_prod(dot(tar0,y),CB)
    B["J"][:,5] = cross_prod(dot(tar1,x),CB)
    B["J"][:,6] = cross_prod(dot(tar2,z),CB)
    
    # F point (left elbow)
    F = {}
    EF = dot(tal2,z)*uarm
    DF = DE+EF
    GF = GE+EF
    F["pos"] = GF
    F["J"] = zeros((3,25))
    F["J"][:,10] = cross_prod(y,GF)
    F["J"][:,11] = cross_prod(dot(tt0,x),GF)
    F["J"][:,12] = cross_prod(dot(tt1,z),GF)
    F["J"][:,13] = cross_prod(dot(tt3,y),DF)
    F["J"][:,14] = cross_prod(dot(tt4,x),DF)
    F["J"][:,0] = cross_prod(dot(tal0,y),EF)
    F["J"][:,1] = cross_prod(dot(tal1,x),EF)
    F["J"][:,2] = cross_prod(dot(tal2,z),EF)
    
    # A point (right hand)
    A = {}
    BA = dot(tar3,z)*farm
    DA = DB+BA
    CA = CB+BA
    GA = GB+BA
    A["pos"] = GA
    A["J"] = zeros((3,25))
    A["J"][:,10] = cross_prod(y,GA)
    A["J"][:,11] = cross_prod(dot(tt0,x),GA)
    A["J"][:,12] = cross_prod(dot(tt1,z),GA)
    A["J"][:,13] = cross_prod(dot(tt3,y),DA)
    A["J"][:,14] = cross_prod(dot(tt4,x),DA)
    A["J"][:,4] = cross_prod(dot(tar0,y),CA)
    A["J"][:,5] = cross_prod(dot(tar1,x),CA)
    A["J"][:,6] = cross_prod(dot(tar2,z),CA)
    A["J"][:,7] = cross_prod(dot(tar3,y),BA)
    
    # H point (left hand)
    H = {}
    FH = dot(tal3,z)*farm
    DH = DF+FH
    EH = EF+FH
    GH = GF+FH
    F["pos"] = GH
    F["J"] = zeros((3,25))
    F["J"][:,10] = cross_prod(y,GF)
    F["J"][:,11] = cross_prod(dot(tt0,x),GH)
    F["J"][:,12] = cross_prod(dot(tt1,z),GH)
    F["J"][:,13] = cross_prod(dot(tt3,y),DH)
    F["J"][:,14] = cross_prod(dot(tt4,x),DH)
    F["J"][:,0] = cross_prod(dot(tal0,y),EH)
    F["J"][:,1] = cross_prod(dot(tal1,x),EH)
    F["J"][:,2] = cross_prod(dot(tal2,z),EH)
    F["J"][:,3] = cross_prod(dot(tal3,y),FH)
    
#    GD = -dot(tt1,z)*stom
#    DC = dot(tt4,y)*ysh-dot(tt4,z)*chest
#    DE = -dot(tt4,y)*ysh-dot(tt4,z)*chest
#    CB = dot(tar2,z)*uarm
#    EF = dot(tal2,z)*uarm
#    BA = dot(tar3,z)*farm
#    FH = dot(tal3,z)*farm
#    GU = dot(t1,z)*pelv
#    UK = dot(tlr0,y)*yhip
#    UP = -dot(tll0,y)*yhip
#    KL = dot(tlr2,z)*thigh
#    PQ = dot(tll2,z)*thigh
#    LM = dot(tlr3,z)*leg
#    QR = dot(tll3,z)*leg
#    MN = dot(tlr4,x)*xtoe
#    RS = dot(tll4,x)*xtoe
#    MO = -dot(tlr4,x)*xheel
#    RT = -dot(tll4,x)*xheel
#    DI = -dot(th0,z)*neck
#    IJ = dot(th1,x)*nose
    
    # composition
#    GC = GD+DC
#    GB = GC+CB
#    GA = GB+BA
#    GE = GD+DE
#    GF = GE+EF
#    GH = GF+FH
#    GI = GD+DI
#    GJ = GI+IJ
#    GK = GU+UK
#    GL = GK+KL
#    GM = GL+LM
#    GN = GM+MN
#    GO = GM+MO
#    GP = GU+UP
#    GQ = GP+PQ
#    GR = GQ+QR
#    GS = GR+RS
#    GT = GR+RT
    return A
    
q = zeros((25,))
T0 = time.time()
for i in range(1000): 
    a=MGD(q,0.0,0.0)
dT = time.time()-T0   
print "computation time in micros"
print dT*1000