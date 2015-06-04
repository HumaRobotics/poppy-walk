from numpy import *



class Kinematics:
	"""
		The purpose of this class is to compute the geometrical model of Poppy robot including Jacobian matrix.
	"""
	def __init__(self, calcJacobian=True, calcRleg=True, calcLleg=True, calcHead=True, calcRarm=True, calcLarm=True, calcBust=True):
		# geometric data
		# shoulder position on y
		self.ysh = 0.106
		# length of upper arm
		self.uarm = 0.150
		# length of forearm
		self.farm = 0.120
		# distance between sternum and shoulder axis
		self.chest = 0.045
		# distance between stomach and sternum
		self.stom = 0.134
		# length of neck
		self.neck = 0.128
		# length of nose
		self.nose = 0.100
		# distance between stomach and pelvis axis
		self.pelv = 0.060
		# hip position on y
		self.yhip = 0.045
		# length of thigh
		self.zthigh = 0.200
		# shift of thigh
		self.ythigh = 0.030
		# length of leg
		self.leg = 0.210
		# toe position on x
		self.xtoe = 0.120
		# specify the body branches to compute
		self.calcJacobian = calcJacobian
		self.calcRleg = calcRleg
		self.calcLleg = calcLleg
		self.calcHead = calcHead
		self.calcRarm = calcRarm
		self.calcLarm = calcLarm
		self.calcBust = calcBust
		self.articulationNames = [	"abs_y","abs_x","abs_z","bust_y","bust_x",
									"l_shoulder_y","l_shoulder_x","l_arm_z","l_elbow_y",
									"r_shoulder_y","r_shoulder_x","r_arm_z","r_elbow_y",
									"head_z","head_y",
									"l_hip_x","l_hip_z","l_hip_y","l_knee_y","l_ankle_y",
									"r_hip_x","r_hip_z","r_hip_y","r_knee_y","r_ankle_y"]

		# dictionnary of points
		self.points = {}

	def updateModel(self, q, theta, phi):
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

		if self.calcRarm or self.calcLarm or self.calcHead or self.calcBust:
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

		if self.calcLarm:
			# left arm chain
			rotal0=rot_mat("y",s[5],c[5])
			rotal1=rot_mat("x",s[6],c[6])
			rotal2=rot_mat("z",s[7],c[7])
			rotal3=rot_mat("y",s[8],c[8])
			tal0 = dot(tt4,rotal0)
			tal1 = dot(tal0,rotal1)
			tal2 = dot(tal1,rotal2)
			tal3 = dot(tal2,rotal3)

		if self.calcRarm:
			# right arm chain
			rotar0=rot_mat("y",s[9],c[9])
			rotar1=rot_mat("x",s[10],c[10])
			rotar2=rot_mat("z",s[11],c[11])
			rotar3=rot_mat("y",s[12],c[12])
			tar0 = dot(tt4,rotar0)
			tar1 = dot(tar0,rotar1)
			tar2 = dot(tar1,rotar2)
			tar3 = dot(tar2,rotar3)

		if self.calcHead:
			# head chain
			roth0=rot_mat("z",s[13],c[13])
			roth1=rot_mat("y",s[14],c[14])
			th0 = dot(tt4,roth0)
			th1 = dot(th0,roth1)

		if self.calcLleg:
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

		if self.calcRleg:
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

		# sternum point (D point)
		if self.calcBust or self.calcRarm or self.calcLarm or self.calcHead:
			self.points["sternum"]={}
			GD = dot(tt1,z)*self.stom
			self.points["sternum"]["position"] = GD
			if self.calcJacobian:
				self.points["sternum"]["jacobian"] = zeros((3,25))
				self.points["sternum"]["jacobian"][:,0] = cross_prod(dot(t1,y),GD)
				self.points["sternum"]["jacobian"][:,1] = cross_prod(dot(tt0,x),GD)

		# right shoulder point (C point)
		if self.calcBust or self.calcRarm:
			self.points["r_shoulder"]={}
			DC = -dot(tt4,y)*self.ysh+dot(tt4,z)*self.chest
			GC = GD+DC
			self.points["r_shoulder"]["position"] = GC
			if self.calcJacobian:
				self.points["r_shoulder"]["jacobian"] = zeros((3,25))
				self.points["r_shoulder"]["jacobian"][:,0] = cross_prod(dot(t1,y),GC)
				self.points["r_shoulder"]["jacobian"][:,1] = cross_prod(dot(tt0,x),GC)
				self.points["r_shoulder"]["jacobian"][:,2] = cross_prod(dot(tt1,z),GC)
				self.points["r_shoulder"]["jacobian"][:,3] = cross_prod(dot(tt3,y),DC)
				self.points["r_shoulder"]["jacobian"][:,4] = cross_prod(dot(tt4,x),DC)

		# left shoulder point (E point)
		if self.calcBust or self.calcLarm:
			self.points["l_shoulder"]={}
			DE = dot(tt4,y)*self.ysh+dot(tt4,z)*self.chest
			GE = GD+DE
			self.points["l_shoulder"]["position"] = GE
			if self.calcJacobian:
				self.points["l_shoulder"]["jacobian"] = zeros((3,25))
				self.points["l_shoulder"]["jacobian"][:,0] = cross_prod(dot(t1,y),GE)
				self.points["l_shoulder"]["jacobian"][:,1] = cross_prod(dot(tt0,x),GE)
				self.points["l_shoulder"]["jacobian"][:,2] = cross_prod(dot(tt1,z),GE)
				self.points["l_shoulder"]["jacobian"][:,3] = cross_prod(dot(tt3,y),DE)
				self.points["l_shoulder"]["jacobian"][:,4] = cross_prod(dot(tt4,x),DE)

		# right elbow (B point)
		if self.calcRarm:
			self.points["r_elbow"] = {}
			CB = -dot(tar2,z)*self.uarm
			GB = GC+CB
			self.points["r_elbow"]["position"] = GB
			if self.calcJacobian:
				DB = DC+CB
				self.points["r_elbow"]["jacobian"] = zeros((3,25))
				self.points["r_elbow"]["jacobian"][:,0] = cross_prod(dot(t1,y),GB)
				self.points["r_elbow"]["jacobian"][:,1] = cross_prod(dot(tt0,x),GB)
				self.points["r_elbow"]["jacobian"][:,2] = cross_prod(dot(tt1,z),GB)
				self.points["r_elbow"]["jacobian"][:,3] = cross_prod(dot(tt3,y),DB)
				self.points["r_elbow"]["jacobian"][:,4] = cross_prod(dot(tt4,x),DB)
				self.points["r_elbow"]["jacobian"][:,9] = cross_prod(dot(tar0,y),CB)
				self.points["r_elbow"]["jacobian"][:,10] = cross_prod(dot(tar1,x),CB)
				self.points["r_elbow"]["jacobian"][:,11] = cross_prod(dot(tar2,z),CB)

		# left elbow (F point)
		if self.calcLarm:
			self.points["l_elbow"] = {}
			EF = -dot(tal2,z)*self.uarm
			GF = GE+EF
			self.points["l_elbow"]["position"] = GF
			if self.calcJacobian:
				DF = DE+EF
				self.points["l_elbow"]["jacobian"] = zeros((3,25))
				self.points["l_elbow"]["jacobian"][:,0] = cross_prod(dot(t1,y),GF)
				self.points["l_elbow"]["jacobian"][:,1] = cross_prod(dot(tt0,x),GF)
				self.points["l_elbow"]["jacobian"][:,2] = cross_prod(dot(tt1,z),GF)
				self.points["l_elbow"]["jacobian"][:,3] = cross_prod(dot(tt3,y),DF)
				self.points["l_elbow"]["jacobian"][:,4] = cross_prod(dot(tt4,x),DF)
				self.points["l_elbow"]["jacobian"][:,5] = cross_prod(dot(tal0,y),EF)
				self.points["l_elbow"]["jacobian"][:,6] = cross_prod(dot(tal1,x),EF)
				self.points["l_elbow"]["jacobian"][:,7] = cross_prod(dot(tal2,z),EF)

		# right hand (A point)
		if self.calcRarm:
			self.points["r_hand"] = {}
			BA = -dot(tar3,z)*self.farm
			GA = GB+BA
			self.points["r_hand"]["position"] = GA
			if self.calcJacobian:
				DA = DB+BA
				CA = CB+BA
				self.points["r_hand"]["jacobian"] = zeros((3,25))
				self.points["r_hand"]["jacobian"][:,0] = cross_prod(dot(t1,y),GA)
				self.points["r_hand"]["jacobian"][:,1] = cross_prod(dot(tt0,x),GA)
				self.points["r_hand"]["jacobian"][:,2] = cross_prod(dot(tt1,z),GA)
				self.points["r_hand"]["jacobian"][:,3] = cross_prod(dot(tt3,y),DA)
				self.points["r_hand"]["jacobian"][:,4] = cross_prod(dot(tt4,x),DA)
				self.points["r_hand"]["jacobian"][:,9] = cross_prod(dot(tar0,y),CA)
				self.points["r_hand"]["jacobian"][:,10] = cross_prod(dot(tar1,x),CA)
				self.points["r_hand"]["jacobian"][:,11] = cross_prod(dot(tar2,z),CA)
				self.points["r_hand"]["jacobian"][:,12] = cross_prod(dot(tar3,y),BA)

		# left hand (H point)        
		if self.calcLarm:
			self.points["l_hand"] = {}
			FH = -dot(tal3,z)*self.farm
			GH = GF+FH
			self.points["l_hand"]["position"] = GH
			if self.calcJacobian:
				DH = DF+FH
				EH = EF+FH
				self.points["l_hand"]["jacobian"] = zeros((3,25))
				self.points["l_hand"]["jacobian"][:,0] = cross_prod(dot(t1,y),GH)
				self.points["l_hand"]["jacobian"][:,1] = cross_prod(dot(tt0,x),GH)
				self.points["l_hand"]["jacobian"][:,2] = cross_prod(dot(tt1,z),GH)
				self.points["l_hand"]["jacobian"][:,3] = cross_prod(dot(tt3,y),DH)
				self.points["l_hand"]["jacobian"][:,4] = cross_prod(dot(tt4,x),DH)
				self.points["l_hand"]["jacobian"][:,5] = cross_prod(dot(tal0,y),EH)
				self.points["l_hand"]["jacobian"][:,6] = cross_prod(dot(tal1,x),EH)
				self.points["l_hand"]["jacobian"][:,7] = cross_prod(dot(tal2,z),EH)
				self.points["l_hand"]["jacobian"][:,8] = cross_prod(dot(tal3,y),FH)

		# neck (I point) 		
		if self.calcHead:
			self.points["neck"] = {}
			DI = dot(th0,z)*self.neck
			GI = GD+DI
			self.points["neck"]["position"] = GI
			if self.calcJacobian:
				self.points["neck"]["jacobian"] = zeros((3,25))
				self.points["neck"]["jacobian"][:,0] = cross_prod(dot(t1,y),GI)
				self.points["neck"]["jacobian"][:,1] = cross_prod(dot(tt0,x),GI)
				self.points["neck"]["jacobian"][:,2] = cross_prod(dot(tt1,z),GI)
				self.points["neck"]["jacobian"][:,3] = cross_prod(dot(tt3,y),DI)
				self.points["neck"]["jacobian"][:,4] = cross_prod(dot(tt4,x),DI)

		# nose (J point)
		if self.calcHead:
			self.points["nose"] = {}
			IJ = dot(th1,x)*self.nose
			GJ = GI+IJ
			self.points["nose"]["position"] = GJ
			if self.calcJacobian:
				DJ = DI+IJ
				self.points["nose"]["jacobian"] = zeros((3,25))
				self.points["nose"]["jacobian"][:,0] = cross_prod(dot(t1,y),GJ)
				self.points["nose"]["jacobian"][:,1] = cross_prod(dot(tt0,x),GJ)
				self.points["nose"]["jacobian"][:,2] = cross_prod(dot(tt1,z),GJ)
				self.points["nose"]["jacobian"][:,3] = cross_prod(dot(tt3,y),DJ)
				self.points["nose"]["jacobian"][:,4] = cross_prod(dot(tt4,x),DJ)
				self.points["nose"]["jacobian"][:,13] = cross_prod(dot(th0,z),IJ)
				self.points["nose"]["jacobian"][:,14] = cross_prod(dot(th0,y),IJ)

		# pelvis (U point)		
		if self.calcLleg or self.calcRleg:
			self.points["pelvis"] = {}
			GU = -dot(t1,z)*self.pelv
			self.points["pelvis"]["position"] = GU
			if self.calcJacobian:
				self.points["pelvis"]["jacobian"] = zeros((3,25))

		# right hip (K point)
		if self.calcRleg:
			self.points["r_hip"] = {}
			UK = -dot(tlr0,y)*self.yhip
			GK = GU+UK
			self.points["r_hip"]["position"] = GK
			if self.calcJacobian:
				self.points["r_hip"]["jacobian"] = zeros((3,25))
				self.points["r_hip"]["jacobian"][:,20] = cross_prod(dot(t1,x),UK)

		# left hip (P point)
		if self.calcLleg:
			self.points["l_hip"] = {}
			UP = dot(tll0,y)*self.yhip
			GP = GU+UP
			self.points["l_hip"]["position"] = GP
			if self.calcJacobian:
				self.points["l_hip"]["jacobian"] = zeros((3,25))
				self.points["l_hip"]["jacobian"][:,15] = cross_prod(dot(t1,x),UP)

		# right knee (L point)
		if self.calcRleg:
			self.points["r_knee"] = {}
			KL = -dot(tlr2,z)*self.zthigh+dot(tlr2,y)*self.ythigh
			GL = GK+KL
			self.points["r_knee"]["position"] = GL
			if self.calcJacobian:
				UL = UK+KL
				self.points["r_knee"]["jacobian"] = zeros((3,25))
				self.points["r_knee"]["jacobian"][:,20] = cross_prod(dot(t1,x),UL)
				self.points["r_knee"]["jacobian"][:,21] = cross_prod(dot(tlr0,z),KL)
				self.points["r_knee"]["jacobian"][:,22] = cross_prod(dot(tlr1,y),KL)

		# left knee (Q point)
		if self.calcLleg:
			self.points["l_knee"] = {}
			PQ = -dot(tll2,z)*self.zthigh-dot(tll2,y)*self.ythigh
			GQ = GP+PQ
			self.points["l_knee"]["position"] = GQ
			if self.calcJacobian:
				UQ = UP+PQ
				self.points["l_knee"]["jacobian"] = zeros((3,25))
				self.points["l_knee"]["jacobian"][:,15] = cross_prod(dot(t1,x),UQ)
				self.points["l_knee"]["jacobian"][:,16] = cross_prod(dot(tll0,z),PQ)
				self.points["l_knee"]["jacobian"][:,17] = cross_prod(dot(tll1,y),PQ)

		# right ankle (M point)
		if self.calcRleg:
			self.points["r_ankle"] = {}
			LM = -dot(tlr3,z)*self.leg
			GM = GL+LM
			self.points["r_ankle"]["position"] = GM
			if self.calcJacobian:
				UM = UL+LM
				KM = KL+LM
				self.points["r_ankle"]["jacobian"] = zeros((3,25))
				self.points["r_ankle"]["jacobian"][:,20] = cross_prod(dot(t1,x),UM)
				self.points["r_ankle"]["jacobian"][:,21] = cross_prod(dot(tlr0,z),KM)
				self.points["r_ankle"]["jacobian"][:,22] = cross_prod(dot(tlr1,y),KM)
				self.points["r_ankle"]["jacobian"][:,23] = cross_prod(dot(tlr2,y),LM)

		# left ankle (R point)
		if self.calcLleg:
			self.points["l_ankle"] = {}
			QR = -dot(tll3,z)*self.leg
			GR = GQ+QR
			self.points["l_ankle"]["position"] = GR
			if self.calcJacobian:
				UR = UQ+QR
				PR = PQ+QR
				self.points["l_ankle"]["jacobian"] = zeros((3,25))
				self.points["l_ankle"]["jacobian"][:,15] = cross_prod(dot(t1,x),UR)
				self.points["l_ankle"]["jacobian"][:,16] = cross_prod(dot(tll0,z),PR)
				self.points["l_ankle"]["jacobian"][:,17] = cross_prod(dot(tll1,y),PR)
				self.points["l_ankle"]["jacobian"][:,18] = cross_prod(dot(tll2,y),QR)

		# right toe (N point)
		if self.calcRleg:
			self.points["r_toe"] = {}
			MN = dot(tlr4,x)*self.xtoe
			GN = GM+MN
			self.points["r_toe"]["position"] = GN
			if self.calcJacobian:
				UN = UM+MN
				KN = KM+MN
				LN = LM+MN
				self.points["r_toe"]["jacobian"] = zeros((3,25))
				self.points["r_toe"]["jacobian"][:,20] = cross_prod(dot(t1,x),UN)
				self.points["r_toe"]["jacobian"][:,21] = cross_prod(dot(tlr0,z),KN)
				self.points["r_toe"]["jacobian"][:,22] = cross_prod(dot(tlr1,y),KN)
				self.points["r_toe"]["jacobian"][:,23] = cross_prod(dot(tlr2,y),LN)
				self.points["r_toe"]["jacobian"][:,24] = cross_prod(dot(tlr3,y),MN)

		# left toe (S point)
		if self.calcLleg:
			self.points["l_toe"] = {}
			RS = dot(tll4,x)*self.xtoe
			GS = GR+RS
			self.points["l_toe"]["position"] = GS
			if self.calcJacobian:
				US = UR+RS
				PS = PR+RS
				QS = QR+RS
				self.points["l_toe"]["jacobian"] = zeros((3,25))
				self.points["l_toe"]["jacobian"][:,15] = cross_prod(dot(t1,x),US)
				self.points["l_toe"]["jacobian"][:,16] = cross_prod(dot(tll0,z),PS)
				self.points["l_toe"]["jacobian"][:,17] = cross_prod(dot(tll1,y),PS)
				self.points["l_toe"]["jacobian"][:,18] = cross_prod(dot(tll2,y),QS)
				self.points["l_toe"]["jacobian"][:,19] = cross_prod(dot(tll3,y),RS)


	""" array((3,)) = self.getPosition(string pointName) 
	This method returns the coordinates of a point with respect to the following orthonormal frame :
	- Center is G point (stomach)
	- Z axis is vertical with respect to gravity directed upward
	- X axis is linked to the pelvis solid and directed forward
	- Y axis is defined so that XYZ frame is direct
	The list of the available points is :
	sternum, r[l]_shoulder, r[l]_elbow, r[l]_hand, neck, nose, pelvis, r[l]_hip, r[l]_knee, r[l]_ankle, r[l]_toe
	"""
	def getPosition(self, pointName):
		if self.points.has_key(pointName):
			if self.points[pointName].has_key("position"):
				return self.points[pointName]["position"]
			else:
				raise Exception,"No position computed"
		else:
			raise Exception,"No point named: " + pointName
			
	""" array((3,25)) = self.getJacobian(string pointName) 
	This method returns the jacobian matrix of a point with respect to the following orthonormal frame :
	- Center is G point (stomach)
	- Z axis is vertical with respect to gravity directed upward
	- X axis is linked to the pelvis solid and directed forward
	- Y axis is defined so that XYZ frame is direct
	The jacobian is the sensitivity of point coordinate with respect to each articulation.
	The list of the available points is :
	sternum, r[l]_shoulder, r[l]_elbow, r[l]_hand, neck, nose, pelvis, r[l]_hip, r[l]_knee, r[l]_ankle, r[l]_toe
	"""
	def getJacobian(self, pointName):
		if self.points.has_key(pointName):
			if self.points[pointName].has_key("jacobian"):
				return self.points[pointName]["jacobian"]
			else:
				raise Exception,"No jacobian computed"
		else:
			raise Exception,"No point named: " + pointName
	
	"""
	This method helps to verify the geometric model and its Jacobian
	"""
	def getSensitivity(self,pointName):
		J = self.getJacobian(pointName)
		axes = ["X","Y","Z"]
		for i_axe in range(len(axes)):
			print axes[i_axe] + " is sensitive to:"
			for i_art in range(len(self.articulationNames)):
				if J[i_axe,i_art]>0:
					print "+" + self.articulationNames[i_art]
				elif J[i_axe,i_art]<0:
					print "-" + self.articulationNames[i_art]

				

	def getSpeed(self, fromRef, toRef, point = array([0.,0.,0.])):
		raise NotImplementedError

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
	
# validation of the geometric model at "zero" configuration
if __name__ == "__main__":
	k = Kinematics()
	k.updateModel(zeros((25,)),0.0,0.0)
	print "Positions of points with respect to stomach at zero position. "
	print "Sternum position:"
	print k.getPosition("sternum")
	k.getSensitivity("sternum")
	print "Right shoulder position:"
	print k.getPosition("r_shoulder")
	k.getSensitivity("r_shoulder")
	print "Left shoulder position:"
	print k.getPosition("l_shoulder")
	k.getSensitivity("l_shoulder")
	print "Right elbow position:"
	print k.getPosition("r_elbow")
	k.getSensitivity("r_elbow")
	print "Left elbow position:"
	print k.getPosition("l_elbow")
	k.getSensitivity("l_elbow")
	print "Right hand position:"
	print k.getPosition("r_hand")
	k.getSensitivity("r_hand")
	print "Left hand position:"
	print k.getPosition("l_hand")
	k.getSensitivity("l_hand")
	print "Neck position:"
	print k.getPosition("neck")
	k.getSensitivity("neck")
	print "Nose position:"
	print k.getPosition("nose")
	k.getSensitivity("nose")
	print "Pelvis position:"
	print k.getPosition("pelvis")
	k.getSensitivity("pelvis")
	print "Right hip position:"
	print k.getPosition("r_hip")
	k.getSensitivity("r_hip")
	print "Left hip position:"
	print k.getPosition("l_hip")
	k.getSensitivity("l_hip")
	print "Right knee position:"
	print k.getPosition("r_knee")
	k.getSensitivity("r_knee")
	print "Left knee position:"
	print k.getPosition("l_knee")
	k.getSensitivity("l_knee")
	print "Right ankle position:"
	print k.getPosition("r_ankle")
	k.getSensitivity("r_ankle")
	print "Left ankle position:"
	print k.getPosition("l_ankle")
	k.getSensitivity("l_ankle")
	print "Right toe position:"
	print k.getPosition("r_toe")
	k.getSensitivity("r_toe")
	print "Left toe position:"
	print k.getPosition("l_toe")
	k.getSensitivity("l_toe")
	
	
