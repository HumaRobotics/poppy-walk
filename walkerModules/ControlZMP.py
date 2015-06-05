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
        return self.controlZMP( motorPositions, motorNextPositions, "r_toe")
        
    def doubleSupportLeftExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #between feet
        return self.controlZMP( motorPositions, motorNextPositions, "l_toe")
        
        
    def controlZMP(self, motorPositions, motorNextPositions, referencePosition):
        #~ print self.kinematics.points.keys()
        try:
            position = self.kinematics.getPosition(referencePosition)
            acceleration = self.kinematics.getAcceleration("pelvis")
            #~ print position
            #~ print acceleration
            
            g = 9.81
            
            xZMP = -position[0] + self.kinematics.xtoe - acceleration[0]*9.81/(-position[2])
            yZMP = -position[1] - acceleration[1]*9.81/(-position[2])
            print [xZMP, yZMP]
            
        except Exception,e: 
            print str(e)
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
 