import random

class WalkerModule:
    def __init__(self):
        pass
        
    def execute(self, motorPositions, motorNextPositions, phase=""):
        if phase == "right step":
            self.stepRightExecute(motorPositions, motorNextPositions)
        elif phase == "right double support":
            self.doubleSupportRightExecute(motorPositions, motorNextPositions)
        elif phase == "left step":
            self.stepLeftExecute(motorPositions, motorNextPositions)
        elif phase == "left double support":
            self.doubleSupportLeftExecute(motorPositions, motorNextPositions)
            
    def stepRightExecute(self, motorPositions, motorNextPositions):
        pass
        
    def stepLeftExecute(self, motorPositions, motorNextPositions):
        pass
        
    def doubleSupportRightExecute(self, motorPositions, motorNextPositions):
        pass
        
    def doubleSupportLeftExecute(self, motorPositions, motorNextPositions):
        pass
        
#################
        
from pypot.primitive.move import Move,  MovePlayer
        
        
class MOCKPlayStepModule(WalkerModule):
    def __init__(self, file):
        WalkerModule.__init__(self)

        
    def execute(self, motorPositions, motorNextPosition, phase=""):
        pass
        
        
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
            
        init()
        
    def init(self):
        self.index = 0
        self.finished = False
        
    def execute(self, motorPositions, motorNextPosition, phase=""):
        if self.finished:
            print "move ",self.filename," finished"
            return
        
        #play one step of the file
        nextPosition = self.move.positions[index]
        for m in nextPosition.keys():
            motorNextPosition[m] = nextPosition[m]
        
        index +=1
        if index >= len(self.move.positions):
            self.finished = True
        
        
    def footLanded(self):
        return self.finished
        
        #~ #MOCK
        #~ r = random.randint(0, 10)
        #~ if r == 0:
            #~ print "foot landed"
            #~ return True

        #~ return False
        
###############      
        
class ControlZMP(WalkerModule):
    def __init__(self):
        WalkerModule.__init__(self)
        self.ZMPpos = []
    
    def stepRightExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #position under right foot
        self.controlZMP( motorPositions, motorNextPositions)
        
    def stepLeftExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #position under left foot
        self.controlZMP( motorPositions, motorNextPositions)
        
    def doubleSupportRightExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #between feet
        self.controlZMP( motorPositions, motorNextPositions)
        
    def doubleSupportLeftExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #between feet
        self.controlZMP( motorPositions, motorNextPositions)
        
        
    def controlZMP(self, motorPositions, motorNextPositions):
        #~ print "controlling ZMP"
        pass
        
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
        return
        target = self.scale*(motorPositions[self.master] - self.referenceMaster)
        if self.inverse:
            target = -target
            
        
        motorNextPosition[self.slave] = target + self.referenceSlave
        
        print "angular control: master ",motorPositions[self.master] , ", slave next ",motorNextPosition[self.slave]
            
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
    