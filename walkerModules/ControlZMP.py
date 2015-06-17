import WalkerModule
import random



class ControlZMP(WalkerModule.WalkerModule):
    def __init__(self, kinematics):
        WalkerModule.WalkerModule.__init__(self)
        self.kinematics = kinematics
        self.ZMPpos = []
        self.logs["xZMP"] = []
        self.logs["yZMP"] = []
    
    def stepRightExecute(self, motorPositions, motorNextPositions):
        #~ print "ZMP step right"
        ZMPpos = [] #position under left foot
        return self.controlZMP( motorPositions, motorNextPositions, "l_toe")
        
    def stepLeftExecute(self, motorPositions, motorNextPositions):
        #~ print "ZMP step left"
        ZMPpos = [] #position under right foot
        return self.controlZMP( motorPositions, motorNextPositions, "r_toe")
        
    def doubleSupportRightExecute(self, motorPositions, motorNextPositions):
        #~ print "ZMP DS right"
        ZMPpos = [] #between feet
        return self.controlZMP( motorPositions, motorNextPositions, "")
        
    def doubleSupportLeftExecute(self, motorPositions, motorNextPositions):
        #~ print "ZMP DS left"
        ZMPpos = [] #between feet
        return self.controlZMP( motorPositions, motorNextPositions, "")
        
        
    def controlZMP(self, motorPositions, motorNextPositions, referencePosition=""):
        print "----"
        #~ if referencePosition == "":
            #~ return motorNextPositions
        
        #~ print self.kinematics.points.keys()
        try:
            position = self.kinematics.getPosition("pelvis")
            #~ jacobian = self.kinematics.points["pelvis"]["jacobian"]
            #~ print jacobian
            acceleration = self.kinematics.getAcceleration("pelvis")
            print position
            #~ print acceleration

            
        except Exception,e: 
            print str(e)
            return motorNextPositions
            
        g = 9.81
        
        xZMP = position[0] - acceleration[0]*9.81/(position[2])
        yZMP = position[1] - acceleration[1]*9.81/(position[2])
        
        self.logs["xZMP"].append(xZMP)
        self.logs["yZMP"].append(yZMP)
        
        print "pos ZMP ",[xZMP, yZMP]
        
        #~ goalZMP = [-position[0] , -position[1] ]
        #~ goalZ = 0.40
        
        #~ coef = 1.
        #~ correctionX = xZMP  - goalZMP[0]
        #~ correctionY =  yZMP - goalZMP[1]
        #~ correctionZ =  -position[2] - goalZ
        
        #~ if correctionX + correctionY + correctionZ== 0.:
            #~ print "avoid dividing by zero in ZMP"
            #~ return motorNextPositions
            
        #~ coefX = coef*correctionX/(correctionX + correctionY + correctionZ)
        #~ coefY = coef*correctionY/(correctionX + correctionY+ correctionZ)
        #~ coefZ= coef*correctionZ/(correctionX + correctionY+ correctionZ)
        
        
        #~ anglesX = {}
        #~ numX = 0
        #~ anglesY = {}
        #~ numY = 0        
        #~ anglesZ = {}
        #~ numZ = 0       
        #~ minK = 0.1
        
        #~ for m in self.kinematics.articulationNames:
            #~ ki = jacobian[0][self.kinematics.articulationNames.index(m)]
            #~ if abs(ki) > minK:
                #~ numX +=1
                #~ anglesX[m] = -1/ki
                
            #~ ki = jacobian[1][self.kinematics.articulationNames.index(m)]
            #~ if abs(ki) > minK:
                #~ numY +=1
                #~ anglesY[m] = -1/ki
                
            #~ ki = jacobian[2][self.kinematics.articulationNames.index(m)]
            #~ if abs(ki) > minK:
                #~ numZ +=1
                #~ anglesZ[m] = -1/ki
                
        #~ print "mod for X"
        #~ for m in anglesX.keys():
            #~ anglesX[m] = anglesX[m] /numX
            #~ print "d", m, " ",anglesX[m] *coefX
            #~ motorNextPositions[m] += anglesX[m] *coefX

        #~ print "mod for Y"
        #~ for m in anglesY.keys():
            #~ anglesY[m] = anglesY[m] /numY
            #~ print "d", m, " ",anglesY[m] *coefY
            #~ motorNextPositions[m] += anglesY[m] *coefY

        #~ print "mod for Z"
        #~ for m in anglesZ.keys():
            #~ anglesZ[m] = anglesZ[m] /numZ
            #~ print "d", m, " ",anglesZ[m] *coefZ
            #~ motorNextPositions[m] += anglesZ[m] *coefZ            
    
        #...
        #~ print "controlling ZMP"
        return motorNextPositions
       
    def canLiftLeftFoot(self):
        
        #MOCK
        r = random.randint(0, 10)
        if r == 0:
            print "can lift left foot"
            return True
            
        #~ print "can't lift left foot"
        return False
        
         
    def canLiftRightFoot(self):
        
        #MOCK
        r = random.randint(0, 10)
        if r == 0:
            print "can lift right foot"
            return True
            
        #~ print "can't lift right foot"
        return False       
        
        
#########################"
 
class MOCKControlZMP(WalkerModule.WalkerModule):
    def __init__(self, kinematics):
        pass
        
    def canLiftLeftFoot(self):
        return True
        #MOCK
        #~ r = random.randint(0, 10)
        #~ if r == 0:
            #~ print "can lift left foot"
            #~ return True
            
        #~ print "can't lift left foot"
        #~ return False
        
         
    def canLiftRightFoot(self):
        return True
        #MOCK
        #~ r = random.randint(0, 10)
        #~ if r == 0:
            #~ print "can lift right foot"
            #~ return True
            
        #~ print "can't lift right foot"
        #~ return False       