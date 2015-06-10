import WalkerModule
import random

class ControlZMP(WalkerModule.WalkerModule):
    def __init__(self, kinematics):
        WalkerModule.WalkerModule.__init__(self)
        self.kinematics = kinematics
        self.ZMPpos = []
    
    def stepRightExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #position under right foot
        return self.controlZMP( motorPositions, motorNextPositions, "r_toe")
        
    def stepLeftExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #position under left foot
        return self.controlZMP( motorPositions, motorNextPositions, "l_toe")
        
    def doubleSupportRightExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #between feet
        return self.controlZMP( motorPositions, motorNextPositions, "")
        
    def doubleSupportLeftExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #between feet
        return self.controlZMP( motorPositions, motorNextPositions, "")
        
        
    def controlZMP(self, motorPositions, motorNextPositions, referencePosition):
        print "----"
        if referencePosition == "":
            return motorNextPositions
        
        #~ print self.kinematics.points.keys()
        try:
            position = self.kinematics.getPosition(referencePosition)
            jacobian = self.kinematics.points[referencePosition]["jacobian"]
            #~ print jacobian
            acceleration = self.kinematics.getAcceleration("pelvis")
            #~ print position
            #~ print acceleration
            
        except Exception,e: 
            print str(e)
            return motorNextPositions
            
        g = 9.81
        
        xZMP = -position[0] + self.kinematics.xtoe - acceleration[0]*9.81/(-position[2])
        yZMP = -position[1] - acceleration[1]*9.81/(-position[2])
        print "pos ZMP ",[xZMP, yZMP]
        
        goalZMP = [ 0., 0.]
        
        coef = 0.1
        correctionX = self.kinematics.xtoe - acceleration[0]*9.81/(-position[2]) - goalZMP[0]
        correctionY =  - acceleration[1]*9.81/(-position[2]) - goalZMP[1]
        
        if correctionX + correctionY== 0.:
            print "avoid dividing by zero in ZMP"
            return motorNextPositions
            
        coefX = coef*correctionX/(correctionX + correctionY)
        coefY = coef*correctionY/(correctionX + correctionY)
        
        #~ sumJacobianX = sum(jacobian[0])
        #~ sumJacobianY = sum(jacobian[1])
        
        anglesX = {}
        numX = 0
        
        minK = 0.01
        
        for m in self.kinematics.articulationNames:
            ki = jacobian[0][self.kinematics.articulationNames.index(m)]
            #~ print ki
            if abs(ki) > minK:
                numX +=1
                anglesX[m] = 1/ki
                
        for m in anglesX.keys():
            anglesX[m] = anglesX[m] /numX
            print "d", m, " ",anglesX[m] *coefX
            motorNextPositions[m] += anglesX[m] *coefX
            
    
        #...
        #~ print "controlling ZMP"
        return motorNextPositions
       
    def canLiftLeftFoot(self):
        
        #MOCK
        r = random.randint(0, 50)
        if r == 0:
            print "can lift left foot"
            return True
            
        #~ print "can't lift left foot"
        return False
        
         
    def canLiftRightFoot(self):
        
        #MOCK
        r = random.randint(0, 50)
        if r == 0:
            print "can lift right foot"
            return True
            
        #~ print "can't lift right foot"
        return False       
 