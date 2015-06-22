import random
#import Kinematics

class WalkerModule:
    def __init__(self, reset="always"):
        self.logs = {}
        #resetConditions = ["always", "never", "if finished"] #phase name !
        self.resetCondition = reset
        
    def execute(self, motorPositions, motorNextPositions, phase=""):
        if phase == "right step":
            return self.stepRightExecute(motorPositions, motorNextPositions)
        elif phase == "right double support":
            return self.doubleSupportRightExecute(motorPositions, motorNextPositions)
        elif phase == "left step":
            return self.stepLeftExecute(motorPositions, motorNextPositions)
        elif phase == "left double support":
            return self.doubleSupportLeftExecute(motorPositions, motorNextPositions)
        else:
            print "ERROR: walker module wrong phase name ",phase
            
    def stepRightExecute(self, motorPositions, motorNextPositions):
        return motorNextPositions
        
    def stepLeftExecute(self, motorPositions, motorNextPositions):
        return motorNextPositions
        
    def doubleSupportRightExecute(self, motorPositions, motorNextPositions):
        return motorNextPositions
        
    def doubleSupportLeftExecute(self, motorPositions, motorNextPositions):
        return motorNextPositions
        
    def reset(self, phase=""):
        pass
        
    def getLogs(self):
        return self.logs
        
#################

        
class MOCKWalkerModule(WalkerModule):
    def __init__(self):
        WalkerModule.__init__(self)

        
    def execute(self, motorPositions, motorNextPositions, phase=""):
        return motorNextPositions
        
        
    def footLanded(self):
        return self.randomBoolean()
        
    def canLiftLeftFoot(self):
        return self.randomBoolean()
 
    def canLiftRightFoot(self):
        return self.randomBoolean()
        
    def randomBoolen(self):
        #MOCK
        r = random.randint(0, 50)
        if r == 0:
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
            target = (1- self.scale)*motorPositions[self.slave]  + self.scale*(self.referenceMaster)
        else:
            target = self.scale*(motorPositions[self.master] - self.referenceMaster)

        if self.inverse:
            target = -target
            
        
        motorNextPositions[self.slave] = target + self.referenceSlave
        
        #~ print "angular control: master ",motorPositions[self.master] , ", slave next ",motorNextPositions[self.slave]
            
        return motorNextPositions
        
  
########################

class LoggerModule(WalkerModule):
    def __init__(self, motorsList):
        WalkerModule.__init__(self)
        self.motorsList = motorsList
        for m in self.motorsList:
            self.logs[m] = []
            
    def execute(self, motorPositions, motorNextPositions, phase=""):
        for m in self.motorsList:
            self.logs[m].append(motorPositions[m])       
        return motorNextPositions
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
    