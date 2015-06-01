from numpy import *
import time

def MGD(q, theta, phi, Jacobian=True, rleg=True, lleg=True, head=True, rarm=True, larm=True, bust=True):

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
    ysh = 0.106
    # length of upper arm
    uarm = 0.150
    # lenght of forearm
    farm = 0.120
    # distance between sternum and shoulder axis
    chest = 0.045
    # distance between stomach and sternum
    stom = 0.134
    # length of neck
    neck = 0.128
    # length of nose
    nose = 0.100
    # distance between stomach and pelvis axis
    pelv = 0.060
    # hip position on y
    yhip = 0.045
    # length of thigh
    zthigh = 0.200
    # shift of thigh
    ythigh = 0.030
    # length of leg
    leg = 0.210
    # toe position on x
    xtoe = 0.120
    
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
    
    # global frame
    rot0=rot_mat("y",stheta,ctheta)
    rot1=rot_mat("x",sphi,cphi)
    t1 = dot(rot0,rot1)
    
    if rarm or larm or head or bust:
      # bust chain
      rott0=rot_mat("y",s[0],c[0])
      rott1=rot_mat("x",s[1],c[1])
      rott2=rot_mat("z",s[2],c[2])
      rott3=rot_mat("y",s[3],c[3])
      rott4=rot_mat("x",s[4],c[4])
      tt0 = dot(t1,rott0)
      tt1 = dot(tt0,rott1)
      tt3 = dot(tt1,dot(rott2,rott3))
      tt4 = dot(tt3,rott4)
    
    if larm:
      # left arm chain
      rotal0=rot_mat("y",s[5],c[5])
      rotal1=rot_mat("x",s[6],c[6])
      rotal2=rot_mat("z",s[7],c[7])
      rotal3=rot_mat("y",s[8],c[8])
      tal0 = dot(tt4,rotal0)
      tal1 = dot(tal0,rotal1)
      tal2 = dot(tal1,rotal2)
      tal3 = dot(tal2,rotal3)
    
    if rarm:
      # right arm chain
      rotar0=rot_mat("y",s[9],c[9])
      rotar1=rot_mat("x",s[10],c[10])
      rotar2=rot_mat("z",s[11],c[11])
      rotar3=rot_mat("y",s[12],c[12])
      tar0 = dot(tt4,rotar0)
      tar1 = dot(tar0,rotar1)
      tar2 = dot(tar1,rotar2)
      tar3 = dot(tar2,rotar3)
    
    if head:
      # head chain
      roth0=rot_mat("z",s[13],c[13])
      roth1=rot_mat("y",s[14],c[14])
      th0 = dot(tt4,roth0)
      th1 = dot(th0,roth1)
    
    if lleg:
      # left leg chain
      rotll0=rot_mat("x",s[15],c[15])
      rotll1=rot_mat("z",s[16],c[16])
      rotll2=rot_mat("y",s[17],c[17])
      rotll3=rot_mat("y",s[18],c[18])
      rotll4=rot_mat("y",s[19],c[19])
      tll0 = dot(t1,rotll0)
      tll1 = dot(tll0,rotll1)
      tll2 = dot(tll1,rotll2)
      tll3 = dot(tll2,rotll3)
      tll4 = dot(tll3,rotll4)
    
    if rleg:
      # right leg chain
      rotlr0=rot_mat("x",s[20],c[20])
      rotlr1=rot_mat("z",s[21],c[21])
      rotlr2=rot_mat("y",s[22],c[22])
      rotlr3=rot_mat("y",s[23],c[23])
      rotlr4=rot_mat("y",s[24],c[24])
      tlr0 = dot(t1,rotlr0)
      tlr1 = dot(tlr0,rotlr1)
      tlr2 = dot(tlr1,rotlr2)
      tlr3 = dot(tlr2,rotlr3)
      tlr4 = dot(tlr3,rotlr4)
    
    # computation of end effectors
    points = []
    if bust or rarm or larm or head:
      # D point (sternum)
      D={}
      D["name"]="D"
      GD = dot(tt1,z)*stom
      D["pos"] = GD
      if Jacobian:
        D["J"] = zeros((3,25))
        D["J"][:,0] = cross_prod(dot(t1,y),GD)
        D["J"][:,1] = cross_prod(dot(tt0,x),GD)
      points.append(D)
  
    if bust or rarm:
      # C point (right shoulder)
      C={}
      C["name"]="C"
      DC = -dot(tt4,y)*ysh+dot(tt4,z)*chest
      GC = GD+DC
      C["pos"] = GC
      if Jacobian:
        C["J"] = zeros((3,25))
        C["J"][:,0] = cross_prod(dot(t1,y),GC)
        C["J"][:,1] = cross_prod(dot(tt0,x),GC)
        C["J"][:,2] = cross_prod(dot(tt1,z),GC)
        C["J"][:,3] = cross_prod(dot(tt3,y),DC)
        C["J"][:,4] = cross_prod(dot(tt4,x),DC)
      points.append(C)
  
  
    if bust or larm:
      # E point (left shoulder)
      E={}
      E["name"]="E"
      DE = dot(tt4,y)*ysh+dot(tt4,z)*chest
      GE = GD+DE
      E["pos"] = GE
      if Jacobian:
        E["J"] = zeros((3,25))
        E["J"][:,0] = cross_prod(dot(t1,y),GE)
        E["J"][:,1] = cross_prod(dot(tt0,x),GE)
        E["J"][:,2] = cross_prod(dot(tt1,z),GE)
        E["J"][:,3] = cross_prod(dot(tt3,y),DE)
        E["J"][:,4] = cross_prod(dot(tt4,x),DE)
      points.append(E)
  
    if rarm:
      # B point (right elbow)
      B = {}
      B["name"]="B"
      CB = -dot(tar2,z)*uarm
      GB = GC+CB
      B["pos"] = GB
      if Jacobian:
        DB = DC+CB
        B["J"] = zeros((3,25))
        B["J"][:,0] = cross_prod(dot(t1,y),GB)
        B["J"][:,1] = cross_prod(dot(tt0,x),GB)
        B["J"][:,2] = cross_prod(dot(tt1,z),GB)
        B["J"][:,3] = cross_prod(dot(tt3,y),DB)
        B["J"][:,4] = cross_prod(dot(tt4,x),DB)
        B["J"][:,9] = cross_prod(dot(tar0,y),CB)
        B["J"][:,10] = cross_prod(dot(tar1,x),CB)
        B["J"][:,11] = cross_prod(dot(tar2,z),CB)
      points.append(B)
  
    if larm:
      # F point (left elbow)
      F = {}
      F["name"]="F"
      EF = -dot(tal2,z)*uarm
      GF = GE+EF
      F["pos"] = GF
      if Jacobian:
        DF = DE+EF
        F["J"] = zeros((3,25))
        F["J"][:,0] = cross_prod(dot(t1,y),GF)
        F["J"][:,1] = cross_prod(dot(tt0,x),GF)
        F["J"][:,2] = cross_prod(dot(tt1,z),GF)
        F["J"][:,3] = cross_prod(dot(tt3,y),DF)
        F["J"][:,4] = cross_prod(dot(tt4,x),DF)
        F["J"][:,5] = cross_prod(dot(tal0,y),EF)
        F["J"][:,6] = cross_prod(dot(tal1,x),EF)
        F["J"][:,7] = cross_prod(dot(tal2,z),EF)
      points.append(F)
  
    if rarm:
      # A point (right hand)
      A = {}
      A["name"]="A"
      BA = -dot(tar3,z)*farm
      GA = GB+BA
      A["pos"] = GA
      if Jacobian:
        DA = DB+BA
        CA = CB+BA
        A["J"] = zeros((3,25))
        A["J"][:,0] = cross_prod(dot(t1,y),GA)
        A["J"][:,1] = cross_prod(dot(tt0,x),GA)
        A["J"][:,2] = cross_prod(dot(tt1,z),GA)
        A["J"][:,3] = cross_prod(dot(tt3,y),DA)
        A["J"][:,4] = cross_prod(dot(tt4,x),DA)
        A["J"][:,9] = cross_prod(dot(tar0,y),CA)
        A["J"][:,10] = cross_prod(dot(tar1,x),CA)
        A["J"][:,11] = cross_prod(dot(tar2,z),CA)
        A["J"][:,12] = cross_prod(dot(tar3,y),BA)
      points.append(A)
  
    if larm:
      # H point (left hand)
      H = {}
      H["name"]="H"
      FH = -dot(tal3,z)*farm
      GH = GF+FH
      H["pos"] = GH
      if Jacobian:
        DH = DF+FH
        EH = EF+FH
        H["J"] = zeros((3,25))
        H["J"][:,0] = cross_prod(dot(t1,y),GH)
        H["J"][:,1] = cross_prod(dot(tt0,x),GH)
        H["J"][:,2] = cross_prod(dot(tt1,z),GH)
        H["J"][:,3] = cross_prod(dot(tt3,y),DH)
        H["J"][:,4] = cross_prod(dot(tt4,x),DH)
        H["J"][:,5] = cross_prod(dot(tal0,y),EH)
        H["J"][:,6] = cross_prod(dot(tal1,x),EH)
        H["J"][:,7] = cross_prod(dot(tal2,z),EH)
        H["J"][:,8] = cross_prod(dot(tal3,y),FH)
      points.append(H)
      
    if head:
      # I point (neck)
      I = {}
      I["name"]="I"
      DI = dot(th0,z)*neck
      GI = GD+DI
      I["pos"] = GI
      if Jacobian:
        I["J"] = zeros((3,25))
        I["J"][:,0] = cross_prod(dot(t1,y),GI)
        I["J"][:,1] = cross_prod(dot(tt0,x),GI)
        I["J"][:,2] = cross_prod(dot(tt1,z),GI)
        I["J"][:,3] = cross_prod(dot(tt3,y),DI)
        I["J"][:,4] = cross_prod(dot(tt4,x),DI)
      points.append(I)
      
      # J point (nose)
      J = {}
      J["name"]="J"
      IJ = dot(th1,x)*nose
      GJ = GI+IJ
      J["pos"] = GJ
      if Jacobian:
        DJ = DI+IJ
        J["J"] = zeros((3,25))
        J["J"][:,0] = cross_prod(dot(t1,y),GJ)
        J["J"][:,1] = cross_prod(dot(tt0,x),GJ)
        J["J"][:,2] = cross_prod(dot(tt1,z),GJ)
        J["J"][:,3] = cross_prod(dot(tt3,y),DJ)
        J["J"][:,4] = cross_prod(dot(tt4,x),DJ)
        J["J"][:,13] = cross_prod(dot(th0,z),IJ)
        J["J"][:,14] = cross_prod(dot(th0,y),IJ)
      points.append(J)
    
    if lleg or rleg:
      # U point (pelvis)
      U = {}
      U["name"]="U"
      GU = -dot(t1,z)*pelv
      U["pos"] = GU
      if Jacobian:
        U["J"] = zeros((3,25))
      points.append(U)
    
    if rleg:
      # K point (right hip)
      K = {}
      K["name"]="K"
      UK = -dot(tlr0,y)*yhip
      GK = GU+UK
      K["pos"] = GK
      if Jacobian:
        K["J"] = zeros((3,25))
        K["J"][:,20] = cross_prod(dot(t1,x),UK)
      points.append(K)
    
    if lleg:
      # P point (left hip)
      P = {}
      P["name"]="P"
      UP = dot(tll0,y)*yhip
      GP = GU+UP
      P["pos"] = GP
      if Jacobian:
        P["J"] = zeros((3,25))
        P["J"][:,15] = cross_prod(dot(t1,x),UP)
      points.append(P)
    
    if rleg:
      # L point (right knee)
      L = {}
      L["name"]="L"
      KL = -dot(tlr2,z)*zthigh+dot(tlr2,y)*ythigh
      GL = GK+KL
      L["pos"] = GL
      if Jacobian:
        UL = UK+KL
        L["J"] = zeros((3,25))
        L["J"][:,20] = cross_prod(dot(t1,x),UL)
        L["J"][:,21] = cross_prod(dot(tlr0,z),KL)
        L["J"][:,22] = cross_prod(dot(tlr1,y),KL)
      points.append(L)
    
    if lleg:
      # Q point (left knee)
      Q = {}
      Q["name"]="Q"
      PQ = -dot(tll2,z)*zthigh-dot(tll2,y)*ythigh
      GQ = GP+PQ
      Q["pos"] = GQ
      if Jacobian:
        UQ = UP+PQ
        Q["J"] = zeros((3,25))
        Q["J"][:,15] = cross_prod(dot(t1,x),UQ)
        Q["J"][:,16] = cross_prod(dot(tll0,z),PQ)
        Q["J"][:,17] = cross_prod(dot(tll1,y),PQ)
      points.append(Q)
  
    if rleg:
      # M point (right ankle)
      M = {}
      M["name"]="M"
      LM = -dot(tlr3,z)*leg
      GM = GL+LM
      M["pos"] = GM
      if Jacobian:
        UM = UL+LM
        KM = KL+LM
        M["J"] = zeros((3,25))
        M["J"][:,20] = cross_prod(dot(t1,x),UM)
        M["J"][:,21] = cross_prod(dot(tlr0,z),KM)
        M["J"][:,22] = cross_prod(dot(tlr1,y),KM)
        M["J"][:,23] = cross_prod(dot(tlr2,y),LM)
      points.append(M)
    
    if lleg:
      # R point (left ankle)
      R = {}
      R["name"]="R"
      QR = -dot(tll3,z)*leg
      GR = GQ+QR
      R["pos"] = GR
      if Jacobian:
        UR = UQ+QR
        PR = PQ+QR
        R["J"] = zeros((3,25))
        R["J"][:,15] = cross_prod(dot(t1,x),UR)
        R["J"][:,16] = cross_prod(dot(tll0,z),PR)
        R["J"][:,17] = cross_prod(dot(tll1,y),PR)
        R["J"][:,18] = cross_prod(dot(tll2,y),QR)
      points.append(R)
    
    if rleg:
      # N point (right toe)
      N = {}
      N["name"]="N"
      MN = dot(tlr4,x)*xtoe
      GN = GM+MN
      N["pos"] = GN
      if Jacobian:
        UN = UM+MN
        KN = KM+MN
        LN = LM+MN
        N["J"] = zeros((3,25))
        N["J"][:,20] = cross_prod(dot(t1,x),UN)
        N["J"][:,21] = cross_prod(dot(tlr0,z),KN)
        N["J"][:,22] = cross_prod(dot(tlr1,y),KN)
        N["J"][:,23] = cross_prod(dot(tlr2,y),LN)
        N["J"][:,24] = cross_prod(dot(tlr3,y),MN)
      points.append(N)
  
    if lleg:
      # S point (left toe)
      S = {}
      S["name"]="S"
      RS = dot(tll4,x)*xtoe
      GS = GR+RS
      S["pos"] = GS
      if Jacobian:
        US = UR+RS
        PS = PR+RS
        QS = QR+RS
        S["J"] = zeros((3,25))
        S["J"][:,15] = cross_prod(dot(t1,x),US)
        S["J"][:,16] = cross_prod(dot(tll0,z),PS)
        S["J"][:,17] = cross_prod(dot(tll1,y),PS)
        S["J"][:,18] = cross_prod(dot(tll2,y),QS)
        S["J"][:,19] = cross_prod(dot(tll3,y),RS)
      points.append(S)
    return points
      
    
def rot_mat(axe,s,c):
  if axe == "x":
      return array([[1,0,0],[0,c,-s],[0,s,c]])
  elif axe == "y":
      return array([[c,0,s],[0,1,0],[-s,0,c]])
  elif axe == "z":
      return array([[c,-s,0],[s,c,0],[0,0,1]])
  else:
      return 0
      
def cross_prod(a,b):
  ax = array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
  return dot(ax,b)

if __name__ == "__main__":
  q = zeros((25,))
  T0 = time.time()
  for i in range(1000): 
    a=MGD(q,0.0,0.0,Jacobian=True,bust=False, rarm=False, larm=False, head=False, rleg=True, lleg=True)
  dT = time.time()-T0
  print "computation time in micros"
  print dT*1000