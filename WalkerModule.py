import random
import Kinematics

class WalkerModule:
    def __init__(self):
        pass
        
    def execute(self, motorPositions, motorNextPositions, phase=""):
        if phase == "right step":
            return self.stepRightExecute(motorPositions, motorNextPositions)
        elif phase == "right double support":
            return self.doubleSupportRightExecute(motorPositions, motorNextPositions)
        elif phase == "left step":
            return self.stepLeftExecute(motorPositions, motorNextPositions)
        elif phase == "left double support":
            return self.doubleSupportLeftExecute(motorPositions, motorNextPositions)
            
    def stepRightExecute(self, motorPositions, motorNextPositions):
        return motorNextPositions
        
    def stepLeftExecute(self, motorPositions, motorNextPositions):
        return motorNextPositions
        
    def doubleSupportRightExecute(self, motorPositions, motorNextPositions):
        return motorNextPositions
        
    def doubleSupportLeftExecute(self, motorPositions, motorNextPositions):
        return motorNextPositions
        
#################
        
from pypot.primitive.move import Move,  MovePlayer
        
        
class MOCKPlayStepModule(WalkerModule):
    def __init__(self, file):
        WalkerModule.__init__(self)

        
    def execute(self, motorPositions, motorNextPositions, phase=""):
        return motorNextPositions
        
        
    def footLanded(self):
        
        #MOCK
        r = random.randint(0, 10)
        if r == 0:
            print "foot landed"
            return True

        return False
        
        
class PlayStepModule(WalkerModule):
    def __init__(self, file):
        WalkerModule.__init__(self)
        self.filename = file
        self.move = None

        #file should be recorded with correct frequency
        with open(self.filename ) as f:
            self.move  = Move.load(f)
            
        self.init()
        
    def init(self):
        self.index = 0
        self.finished = False
        
    def execute(self, motorPositions, motorNextPositions, phase=""):
        if self.finished:
            print "move ",self.filename," finished"
            return motorNextPositions
        
        #play one step of the file
        nextPositions = self.move.positions()[self.index]
        for m in nextPositions.keys():
            motorNextPositions[m] = nextPositions[m]
        
        self.index +=1
        if self.index >= len(self.move.positions()):
            self.finished = True
        return motorNextPositions
        
    def footLanded(self):
        if self.finished:
            self.init()
            return True
        return False
        
        #~ #MOCK
        #~ r = random.randint(0, 10)
        #~ if r == 0:
            #~ print "foot landed"
            #~ return True

        #~ return False
        
###############      
        
class ControlZMP(WalkerModule):
    def __init__(self, kinematics):
        WalkerModule.__init__(self)
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
 
####################        
        
class AngularControl(WalkerModule):
    def __init__(self, masterMotor, slaveMotor, referenceMaster = 0., referenceSlave = 0., inverse=False, scale=1.):
        WalkerModule.__init__(self)
        
        self.master = masterMotor
        self.slave= slaveMotor
        self.referenceMaster = referenceMaster
        self.referenceSlave = referenceSlave
        self.inverse = inverse
        self.scale = scale
        
    def execute(self, motorPositions, motorNextPositions, phase=""):

        target = self.scale*(motorPositions[self.master] - self.referenceMaster)
        if self.inverse:
            target = -target
            
        
        motorNextPositions[self.slave] = target + self.referenceSlave
        
        #~ print "angular control: master ",motorPositions[self.master] , ", slave next ",motorNextPositions[self.slave]
            
        return motorNextPositions
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
    