
import WalkerModule

import random, time, copy

class Walker:
    def __init__(self, robot):
        self.robot = robot
        self.dt = 0.2 #seconds
        
        self.stepSide = "right"
        
        self.walkModules = {}
        self.rightStepModules = {}
        self.rightDoubleSupportModules = {}
        self.leftStepModules = {}
        self.leftDoubleSupportModules = {}
        
        self.rightStepModules["swingFoot"] = WalkerModule.MOCKPlayStepModule("rightStep.json")
        self.leftStepModules["swingFoot"] = WalkerModule.MOCKPlayStepModule("leftStep.json")
        
        self.walkModules["balancing"] = WalkerModule.ControlZMP()
        self.walkModules["torso vertical"] = WalkerModule.AngularControl("bust_x", "abs_x", inverse=True)
        
    ###
        
    def oneStep(self):
        if self.stepSide == "right":
            self.doubleSupportRight()
            self.stepSide = "left"
            self.stepLeft()
            
        else:
            self.doubleSupportLeft()
            self.stepSide = "right"
            self.stepRight()
            
    
    def readMotorPositions(self):
        motorPositions = {}
        if self.robot is not None:
            for m in self.robot.motors:
                motorPositions[m.name] = m.present_position
        #~ print motorPositions
        return motorPositions
        
    def setMotorPositions(self, positions):
        if self.robot is not None:
            for m in self.robot.motors:
                m.goto_position(positions[m.name], self.dt, wait=False)
        
    ###
        
    def stepRight(self):
        print "### starting step right ###"
        while not self.rightStepModules["swingFoot"].footLanded():
            
            #read motor positions
            motorPositions = self.readMotorPositions()
            
            motorNextPositions = copy.deepcopy(motorPositions)

        
            #modify motor next positions by each module
            for m in self.walkModules.values():
                motorNextPositions = m.execute(motorPositions, motorNextPositions, phase="right step")
                
            for m in self.rightStepModules.values():
                motorNextPositions = m.execute(motorPositions, motorNextPositions)
                
            #Apply modified values
            self.setMotorPositions(motorNextPositions)
            
            #TODO : improve to wait real time
            time.sleep(self.dt)
            
        
    def stepLeft(self):
        print "### starting step left ###"
        while not self.leftStepModules["swingFoot"].footLanded():
            
            #read motor positions
            motorPositions = self.readMotorPositions()
            motorNextPositions = copy.deepcopy(motorPositions)

        
            #modify motor next positions by each module
            for m in self.walkModules.values():
                motorNextPositions = m.execute(motorPositions, motorNextPositions, phase="left step")
                
            for m in self.leftStepModules.values():
                motorNextPositions = m.execute(motorPositions, motorNextPositions)
                
            #Apply modified values
            self.setMotorPositions(motorNextPositions)
            
            #TODO : improve to wait real time
            time.sleep(self.dt)
            
    def doubleSupportRight(self):
        print "### starting double support right ###"
        while not self.walkModules["balancing"].canLiftLeftFoot():
            
            #read motor positions
            motorPositions = self.readMotorPositions()
            motorNextPositions = copy.deepcopy(motorPositions)

        
            #modify motor next positions by each module
            for m in self.walkModules.values():
                motorNextPositions = m.execute(motorPositions, motorNextPositions, phase="right double support")
                #~ print motorNextPositions
            for m in self.rightDoubleSupportModules.values():
                motorNextPositions = m.execute(motorPositions, motorNextPositions)
                #~ print motorNextPositions                
            #Apply modified values
            self.setMotorPositions(motorNextPositions)
            
            #TODO : improve to wait real time
            time.sleep(self.dt)
        
    def doubleSupportLeft(self):
        print "### starting double support left ###"
        while not self.walkModules["balancing"].canLiftRightFoot():
            
            #read motor positions
            motorPositions = self.readMotorPositions()
            motorNextPositions = copy.deepcopy(motorPositions)

        
            #modify motor next positions by each module
            for m in self.walkModules.values():
                motorNextPositions = m.execute(motorPositions, motorNextPositions, phase="left double support")
                
            for m in self.leftDoubleSupportModules.values():
                motorNextPositions = m.execute(motorPositions, motorNextPositions)
                
            #Apply modified values
            self.setMotorPositions(motorNextPositions)
            
            #TODO : improve to wait real time
            time.sleep(self.dt)        
        
    ###
    
    def init(self):
        pass
        
    def startWalk(self):
        pass
        
        
    def stopWalk(self):
        pass
        
    def clean(self):
        pass
        
        