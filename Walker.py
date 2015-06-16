
from walkerModules.WalkerModule import WalkerModule, AngularControl, LoggerModule, MOCKWalkerModule
from walkerModules.PlayJsonModule import PlayJsonModule
from walkerModules.ControlZMP import ControlZMP, MOCKControlZMP
from walkerModules.CPGModule import CPGModule
import Kinematics

import random, time, copy

from numpy import array

HAS_REAL_ROBOT = True


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
        
        if self.robot is None:
            self.walkModules["fake"] = MOCKWalkerModule() #mock module that implements footLanded, canLiftLeftFoot and canLiftRightFoot
            self.leftFootLandedModule = "fake" #name of a module implementing footLanded
            self.rightFootLandedModule = "fake"#name of a module implementing footLanded
            self.canLiftLeftFootModule = "fake"#name of a module implementing canLiftLeftFoot
            self.canLiftRightFootModule ="fake"   #name of a module implementing canLiftRightFoot
        else:
            
            self.leftFootLandedModule = "l_ankle_y"
            self.rightFootLandedModule = "r_ankle_y"
            self.canLiftLeftFootModule = "l_hip_x"
            self.canLiftRightFootModule = "r_hip_x"    

            #~ self.rightStepModules["swingFoot"] = PlayJsonModule("json/rlegstep2.json")
            #~ self.leftStepModules["swingFoot"] = PlayJsonModule("json/llegstep2.json")    
            
            
            CPGstepTime = 2. #time of step phase
            CPGDSTime = 0.5 #time of double support phase
            
            #############
            ## walkModules : active during all phases
            
            #transfer weight
            self.walkModules["l_hip_x"] = CPGModule("l_hip_x", self.dt, cycleTime = 2*(CPGstepTime+CPGDSTime) , startRatio = (2*CPGstepTime+CPGDSTime) /(2*(CPGstepTime+CPGDSTime) ), amplitude = 10, offset = 0)
            self.walkModules["r_hip_x"] = CPGModule("r_hip_x", self.dt, cycleTime = 2*(CPGstepTime+CPGDSTime), startRatio = (CPGstepTime) /(2*(CPGstepTime+CPGDSTime) ), amplitude = -10, offset = 0)
           
           #keep bust at 0
            self.walkModules["keep bust_x"] = AngularControl("constant", "bust_x", scale = 0.1)
            self.walkModules["keep bust_y"] = AngularControl("constant", "bust_y", scale = 0.1)                   
            self.walkModules["keep abs_z"] = AngularControl("constant", "abs_z", scale = 0.1)
            self.walkModules["keep abs_y"] = AngularControl("constant", "abs_y", scale = 0.1) 
            self.walkModules["keep abs_x"] = AngularControl("constant", "abs_x", scale = 0.1) 
            
            #~ self.walkModules["torso vertical"] = AngularControl("r_hip_x", "abs_x", inverse=True)
            
            # balancing module
            #~ self.walkModules["balancing"] = ControlZMP(self.kinematics)
            
            #logger module
            #~ self.walkModules["logger"] = LoggerModule(["l_hip_x", "l_knee_y", "r_knee_y"])
            
            #############
            ## leftStepModules : active during left step (left foot up)
            
            #left leg bend
            self.leftStepModules["l_ankle_y"] = CPGModule("l_ankle_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = 10, stopRatio = 0.5)
            self.leftStepModules["l_knee_y"] = CPGModule("l_knee_y", self.dt, cycleTime =2*CPGstepTime, amplitude = 30, offset = 10)
            self.leftStepModules["l_hip_y"] = CPGModule("l_hip_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = -30, offset = -10)
            #right leg gets straight
            self.leftStepModules["r_knee_y"] = CPGModule("r_knee_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = -10, offset = 10)
            self.leftStepModules["r_hip_y"] = CPGModule("r_hip_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = 10, offset = -10)

            
            #############
            ## rightStepModules : active during right step (right foot up)
            #right leg bent
            self.rightStepModules["r_ankle_y"] = CPGModule("r_ankle_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = 10, stopRatio = 0.5)
            self.rightStepModules["r_knee_y"] = CPGModule("r_knee_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = 30, offset = 10)
            self.rightStepModules["r_hip_y"] = CPGModule("r_hip_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = -30, offset = -10)
            #left leg gets straight
            self.rightStepModules["l_knee_y"] = CPGModule("l_knee_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = -10, offset = 10)
            self.rightStepModules["l_hip_y"] = CPGModule("l_hip_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = 10, offset = -10)
 
            
            #############
            ## leftDoubleSupportModules : active during left double support phase (both feet down, left in front)
            
            #############
            ## rightDoubleSupportModules : active during right double support phase (both feet down, right in front)
            
   
        
        
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
        
        