import WalkerModule
import random

class ControlZMP(WalkerModule.WalkerModule):
    def __init__(self, kinematics):
        WalkerModule.WalkerModule.__init__(self)
        self.kinematics = kinematics
        self.ZMPpos = []
    
    def stepRightExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #position under right foot
        return self.controlZMP( motorPositions, motorNextPositions)
        
    def stepLeftExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #position under left foot
        return self.controlZMP( motorPositions, motorNextPositions)
        
    def doubleSupportRightExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #between feet
        return self.controlZMP( motorPositions, motorNextPositions)
        
    def doubleSupportLeftExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #between feet
        return self.controlZMP( motorPositions, motorNextPositions)
        
        
    def controlZMP(self, motorPositions, motorNextPositions):
        speed = self.kinematics.getSpeed("pelvis", "right_foot")
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
 