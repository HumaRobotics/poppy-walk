
from walkerModules.WalkerModule import WalkerModule, AngularControl, LoggerModule
from walkerModules.PlayJsonModule import PlayJsonModule
from walkerModules.ControlZMP import ControlZMP, MOCKControlZMP
from walkerModules.CPGModule import CPGModule
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
    def __init__(self, robot, razor):
        self.robot = robot
        self.dt = 0.05#seconds
        
        self.kinematics = Kinematics.Kinematics()
        self.razor = razor
        
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
        #~ self.rightStepModules["swingFoot"] = MOCKPlayJsonModule("json/rlegstep2.json")
        #~ self.leftStepModules["swingFoot"] = MOCKPlayJsonModule("json/llegstep2.json")    
        CPGstepTime = 2.
        #~ self.rightStepModules["swingFoot"] = CPGModule("r_knee_y", self.dt, cycleTime = CPGcycleTime, amplitude = 30)
        self.leftStepModules["l_ankle_y"] = CPGModule("l_ankle_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = 10, stopRatio = 0.5)
        self.leftStepModules["l_knee_y"] = CPGModule("l_knee_y", self.dt, cycleTime =2*CPGstepTime, amplitude = 30, offset = 10)
        self.leftStepModules["l_hip_y"] = CPGModule("l_hip_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = -30, offset = -10)
        
        self.leftStepModules["r_knee_y"] = CPGModule("r_knee_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = -10, offset = 10)
        self.leftStepModules["r_hip_y"] = CPGModule("r_hip_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = 10, offset = -10)
    
        
        self.rightStepModules["r_ankle_y"] = CPGModule("r_ankle_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = 10, stopRatio = 0.5)
        self.rightStepModules["r_knee_y"] = CPGModule("r_knee_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = 30, offset = 10)
        self.rightStepModules["r_hip_y"] = CPGModule("r_hip_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = -30, offset = -10)
        
        self.rightStepModules["l_knee_y"] = CPGModule("l_knee_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = -10, offset = 10)
        self.rightStepModules["l_hip_y"] = CPGModule("l_hip_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = 10, offset = -10)
        
        CPGDSTime = 0.5
        
        self.walkModules["l_hip_x"] = CPGModule("l_hip_x", self.dt, cycleTime = 2*(CPGstepTime+CPGDSTime) , startRatio = (2*CPGstepTime+CPGDSTime) /(2*(CPGstepTime+CPGDSTime) ), amplitude = 10, offset = 0)
        self.walkModules["r_hip_x"] = CPGModule("r_hip_x", self.dt, cycleTime = 2*(CPGstepTime+CPGDSTime), startRatio = (CPGstepTime) /(2*(CPGstepTime+CPGDSTime) ), amplitude = -10, offset = 0)
        # control of torso
        #~ if HAS_REAL_ROBOT :
            #~ self.walkModules["torso vertical"] = AngularControl("r_hip_x", "abs_x", inverse=True)

        self.walkModules["keep bust_x"] = AngularControl("constant", "bust_x", scale = 0.1)
        self.walkModules["keep bust_y"] = AngularControl("constant", "bust_y", scale = 0.1)                   
        self.walkModules["keep abs_z"] = AngularControl("constant", "abs_z", scale = 0.1)
        self.walkModules["keep abs_y"] = AngularControl("constant", "abs_y", scale = 0.1) 
        self.walkModules["keep abs_x"] = AngularControl("constant", "abs_x", scale = 0.1) 
        
        #~ self.leftStepModules["keep r_hip_z"] = AngularControl("constant", "r_hip_z") 
        #~ self.leftStepModules["keep r_hip_y"] = AngularControl("constant", "r_hip_y") 
        #~ self.leftStepModules["keep r_hip_x"] = AngularControl("constant", "r_hip_x") 
        #~ self.leftStepModules["keep r_knee_y"] = AngularControl("constant", "r_knee_y") 
        #~ self.leftStepModules["keep r_ankle_y"] = AngularControl("constant", "r_ankle_y") 
        
        #~ self.rightStepModules["keep l_hip_z"] = AngularControl("constant", "l_hip_z") 
        #~ self.rightStepModules["keep l_hip_y"] = AngularControl("constant", "l_hip_y") 
        #~ self.rightStepModules["keep l_hip_x"] = AngularControl("constant", "l_hip_x") 
        #~ self.rightStepModules["keep l_knee_y"] = AngularControl("constant", "l_knee_y") 
        #~ self.rightStepModules["keep l_ankle_y"] = AngularControl("constant", "l_ankle_y") 
        
        self.walkModules["logger"] = LoggerModule(["l_hip_x", "l_knee_y", "r_knee_y"])
        
        # balancing module
        #~ self.walkModules["balancing"] = MOCKControlZMP(self.kinematics)
        
        self.leftFootLandedModule = "l_ankle_y"
        self.rightFootLandedModule = "r_ankle_y"
        self.canLiftLeftFootModule = "l_hip_x"
        self.canLiftRightFootModule = "r_hip_x"       
        
        
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
                
        else:
            for m in self.kinematics.articulationNames:
                motorPositions[m] = random.randint(-20, 20)
                motorPositionsList[self.kinematics.articulationNames.index(m)] =motorPositions[m]
                
        #~ print motorPositions
        if self.razor is not None:
            try:
                razorData = self.razor.eul()
            except:
                print "error could not read razor"
                razorData = [0., 0.]
        self.kinematics.updateModel(array(motorPositionsList), razorData[0], razorData[1])

        
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
        
        #~ for m in self.walkModules.values():
            #~ motorNextPositions = m.reset()
            
        for m in self.rightStepModules.values():
            motorNextPositions = m.reset()
        
        while not self.rightStepModules[self.rightFootLandedModule ].footLanded():
            
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


        #~ for m in self.walkModules.values():
            #~ motorNextPositions = m.reset()
            
        for m in self.leftStepModules.values():
            motorNextPositions = m.reset()


        while not self.leftStepModules[self.leftFootLandedModule ].footLanded():
            
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

        #~ for m in self.walkModules.values():
            #~ motorNextPositions = m.reset()
            
        for m in self.rightDoubleSupportModules.values():
            motorNextPositions = m.reset()


        while not self.walkModules[self.canLiftLeftFootModule ].canLiftLeftFoot():
            
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

        #~ for m in self.walkModules.values():
            #~ motorNextPositions = m.reset()
            
        for m in self.leftDoubleSupportModules.values():
            motorNextPositions = m.reset()


        while not self.walkModules[self.canLiftRightFootModule ].canLiftRightFoot():
            
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
        print "### stopping walk ###"
        if self.robot is not None:
            #~ try:
            for m in self.robot.motors:
                m.goal_speed = 0.
                
            time.sleep(0.5)
            
            for m in self.robot.motors:
                m.goto_position(0.0,2., wait=False)
                
            time.sleep(0.5)
            for m in self.robot.motors:
                m.compliant = True
                #~ self.robot.stand_position.start()
            #~ except:
                #~ pass
                
        print "### plotting ###"
        import pylab 
        
        pos1 = self.walkModules["logger"].pos["l_hip_x"]
        pos2 = self.walkModules["logger"].pos["l_knee_y"]
        pos3 = self.walkModules["logger"].pos["r_knee_y"]
        
        t = range(0, len(pos1))

        pylab.plot(t,pos1, "r-", t, pos2, "b-", t, pos3, "g-")

        pylab.grid(True)
        pylab.savefig("one_step.png")
        
    def clean(self):
        pass
        
        