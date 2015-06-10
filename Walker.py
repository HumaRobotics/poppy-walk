
from walkerModules.WalkerModule import *
from walkerModules.PlayJsonModule import PlayJsonModule
from walkerModules.ControlZMP import ControlZMP
import Kinematics

import random, time, copy

from numpy import array

#############
# PARAMATERS
#############

HAS_REAL_ROBOT = True

### Robot config ###
#~ HAS_IMU = False
#~ imu_model = "razor"

#~ HAS_FOOT_SENSORS = False
###

### Activated modules ###


#~ HAS_FORWARD_KINEMATICS = False
#~ HAS_INVERSE_KINEMATICS = False

#~ USE_ZMP = False
#~ USE_PHASE_DIAGRAM = False

#~ up_foot_trajectory = "CPG" #CPG|play_move

#~ DO_TORSO_STABILIZATION = False
#~ torso_stabilization = "vertical"


###


class Walker:
    def __init__(self, robot):
        self.robot = robot
        self.dt = 0.05#seconds
        
        self.kinematics = Kinematics.Kinematics()
        
        self.stepSide = "right"
        
        self.walkModules = {}
        self.rightStepModules = {}
        self.rightDoubleSupportModules = {}
        self.leftStepModules = {}
        self.leftDoubleSupportModules = {}
        
        # foot movement when not on the ground
        #~ if HAS_REAL_ROBOT :
            #~ self.rightStepModules["swingFoot"] = PlayJsonModule("json/rlegstep.json")
            #~ self.leftStepModules["swingFoot"] = PlayJsonModule("json/llegstep.json")
        #~ else:
        self.rightStepModules["swingFoot"] = MOCKPlayJsonModule("json/rlegstep2.json")
        self.leftStepModules["swingFoot"] = MOCKPlayJsonModule("json/llegstep2.json")    
        
        # control of torso
        #~ if HAS_REAL_ROBOT :
            #~ self.walkModules["torso vertical"] = AngularControl("r_hip_x", "abs_x", inverse=True)

        self.walkModules["keep bust_x"] = AngularControl("constant", "bust_x")
        self.walkModules["keep bust_y"] = AngularControl("constant", "bust_y")                   
        self.walkModules["keep abs_z"] = AngularControl("constant", "abs_z")
        self.walkModules["keep abs_y"] = AngularControl("constant", "abs_y") 
        
        self.leftStepModules["keep r_hip_z"] = AngularControl("constant", "r_hip_z") 
        self.leftStepModules["keep r_hip_y"] = AngularControl("constant", "r_hip_y") 
        self.leftStepModules["keep r_hip_x"] = AngularControl("constant", "r_hip_x") 
        self.leftStepModules["keep r_knee_y"] = AngularControl("constant", "r_knee_y") 
        self.leftStepModules["keep r_ankle_y"] = AngularControl("constant", "r_ankle_y") 
        
        self.rightStepModules["keep l_hip_z"] = AngularControl("constant", "l_hip_z") 
        self.rightStepModules["keep l_hip_y"] = AngularControl("constant", "l_hip_y") 
        self.rightStepModules["keep l_hip_x"] = AngularControl("constant", "l_hip_x") 
        self.rightStepModules["keep l_knee_y"] = AngularControl("constant", "l_knee_y") 
        self.rightStepModules["keep l_ankle_y"] = AngularControl("constant", "l_ankle_y") 
        # balancing module
        self.walkModules["balancing"] = ControlZMP(self.kinematics)
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
        motorPositionsList = [0.]*25
        if self.robot is not None:
            for m in self.robot.motors:
                motorPositions[m.name] = m.present_position
                motorPositionsList[self.kinematics.articulationNames.index(m.name)] =motorPositions[m.name]
        #~ print motorPositions
        self.kinematics.updateModel(array(motorPositionsList), 0., 0.)
        
        return motorPositions
        
    def setMotorPositions(self, positions):
        if self.robot is not None:
            for m in self.robot.motors:
                m.goto_position(positions[m.name], self.dt, wait=False)
 
    def setMotorSpeeds(self, positions, positionsBefore):
        if self.robot is not None:
            for m in self.robot.motors:
                #~ if m.name == "l_knee_y":
                    
                speed = ( positions[m.name] - positionsBefore[m.name] )/self.dt
                #~ print m.name, speed
                if speed > 20:
                    speed = 20
                if speed < -20:
                    speed = -20
                m.goal_speed = speed        
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
            #~ self.setMotorPositions(motorNextPositions)
            self.setMotorSpeeds(motorNextPositions, motorPositions)
            
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
            #~ self.setMotorPositions(motorNextPositions)
            self.setMotorSpeeds(motorNextPositions, motorPositions)
            
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
            #~ self.setMotorPositions(motorNextPositions)
            self.setMotorSpeeds(motorNextPositions, motorPositions)
            
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
            #~ self.setMotorPositions(motorNextPositions)
            self.setMotorSpeeds(motorNextPositions, motorPositions)
            
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
        
        