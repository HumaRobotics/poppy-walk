import random
#import Kinematics

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

        
class MOCKPlayJsonModule(WalkerModule):
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
        if self.master is "constant":
            target = self.referenceMaster
        else:
            target = self.scale*(motorPositions[self.master] - self.referenceMaster)

        if self.inverse:
            target = -target
            
        
        motorNextPositions[self.slave] = target + self.referenceSlave
        
        #~ print "angular control: master ",motorPositions[self.master] , ", slave next ",motorNextPositions[self.slave]
            
        return motorNextPositions
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
    