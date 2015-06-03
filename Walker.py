
import WalkerModule
import Kinematics

import random, time, copy

#############
# PARAMATERS
#############

HAS_REAL_ROBOT = False

### Robot config ###
HAS_IMU = False
imu_model = "razor"

HAS_FOOT_SENSORS = False
###

### Activated modules ###


HAS_FORWARD_KINEMATICS = False
HAS_INVERSE_KINEMATICS = False

USE_ZMP = False
USE_PHASE_DIAGRAM = False

up_foot_trajectory = "CPG" #CPG|play_move

DO_TORSO_STABILIZATION = False
torso_stabilization = "vertical"


###


class Walker:
    def __init__(self, robot):
        self.robot = robot
        self.dt = 0.05 #seconds
        
        self.kinematics = Kinematics.Kinematics()
        
        self.stepSide = "right"
        
        self.walkModules = {}
        self.rightStepModules = {}
        self.rightDoubleSupportModules = {}
        self.leftStepModules = {}
        self.leftDoubleSupportModules = {}
        
        # foot movement when not on the ground
        if HAS_REAL_ROBOT :
            self.rightStepModules["swingFoot"] = WalkerModule.PlayStepModule("json/rlegstep.json")
            self.leftStepModules["swingFoot"] = WalkerModule.PlayStepModule("json/llegstep.json")
        else:
            self.rightStepModules["swingFoot"] = WalkerModule.MOCKPlayStepModule("json/rlegstep.json")
            self.leftStepModules["swingFoot"] = WalkerModule.MOCKPlayStepModule("json/llegstep.json")    
        
        # control of torso
        if HAS_REAL_ROBOT :
            self.walkModules["torso vertical"] = WalkerModule.AngularControl("r_hip_x", "abs_x", inverse=True)
                    
        # balancing module
        self.walkModules["balancing"] = WalkerModule.ControlZMP(self.kinematics)
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
        motorPositionsList = []
        if self.robot is not None:
            for m in self.robot.motors:
                motorPositions[m.name] = m.present_position
                motorPositionsList.append(motorPositions[m.name])
        #~ print motorPositions
        self.kinematics.updateModel(motorPositionsList)
        
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
        
        