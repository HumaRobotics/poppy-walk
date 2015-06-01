
import WalkerModule

import random, time

class Walker:
    def __init__(self, robot):
        self.robot = robot
        self.dt = 0.2 #seconds
        
        self.walkModules = {}
        self.rightStepModules = {}
        self.rightDoubleSupportModules = {}
        self.leftStepModules = {}
        self.leftDoubleSupportModules = {}
        
        self.rightStepModules["swingFoot"] = WalkerModule.PlayStepModule("rightStep.json")
        self.leftStepModules["swingFoot"] = WalkerModule.PlayStepModule("leftStep.json")
        
        self.walkModules["ZMP"] = WalkerModule.ControlZMP()
        
        
    def stepRight(self):
        
        while not self.rightStepModules["swingFoot"].footLanded():
            
            #read motor positions
            motorPositions = {}
            motorNextPositions = {}
            #~ for m in self.robot.motors:
                #~ motorPositions[m.name] = m.present_position
                #~ motorNextPositions[m.name] = motorPositions[m.name] 
        
            #modify motor next positions by each module
            for m in self.walkModules.values():
                motorPositions = m.execute(motorPositions, motorNextPositions, phase="right step")
                
            for m in self.rightStepModules.values():
                motorPositions = m.execute(motorPositions, motorNextPositions)
                
            #Apply modified values
            #~ for m in self.robot.motors:
                #~ m.goto_position(motorNextPositions[m.name], self.dt, wait=False)
            
            #TODO : improve to wait real time
            time.sleep(self.dt)
            
        
    def stepLeft(self):
        
        while not self.leftStepModules["swingFoot"].footLanded():
            
            #read motor positions
            motorPositions = {}
            motorNextPositions = {}
            #~ for m in self.robot.motors:
                #~ motorPositions[m.name] = m.present_position
                #~ motorNextPositions[m.name] = motorPositions[m.name] 
        
            #modify motor next positions by each module
            for m in self.walkModules.values():
                motorPositions = m.execute(motorPositions, motorNextPositions, phase="left step")
                
            for m in self.rightStepModules.values():
                motorPositions = m.execute(motorPositions, motorNextPositions)
                
            #Apply modified values
            #~ for m in self.robot.motors:
                #~ m.goto_position(motorNextPositions[m.name], self.dt, wait=False)
            
            #TODO : improve to wait real time
            time.sleep(self.dt)