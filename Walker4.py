
from walkerModules.WalkerModule import WalkerModule, AngularControl, LoggerModule, MOCKWalkerModule
from walkerModules.PlayJsonModule import PlayJsonModule
from walkerModules.ControlZMP import ControlZMP, MOCKControlZMP
from walkerModules.CPGModule import CPGModule
from walkerModules.TrajectoryModule import TrajectoryModule
import Kinematics

import random, time, copy

from numpy import array, linspace

class Walker:
    def __init__(self, robot, razor=None):
        self.robot = robot
        self.dt = 0.05#seconds
        
        self.kinematics = Kinematics.Kinematics()
        self.razor = razor
        
        
        
        self.stepSide = "right"
        
        self.walkModules = {}
        self.walkModules["fullWalkModules"] = {}
        self.walkModules["right step"] = {}
        self.walkModules["right double support"] = {}
        self.walkModules["left step"] = {}
        self.walkModules["left double support"] = {}
        
        if self.robot is None:
            self.walkModules["fullWalkModules"]["fake"] = MOCKWalkerModule() #mock module that implements footLanded, canLiftLeftFoot and canLiftRightFoot
            self.leftFootLandedModule = "fake" #name of a module implementing footLanded
            self.rightFootLandedModule = "fake"#name of a module implementing footLanded
            self.canLiftLeftFootModule = "fake"#name of a module implementing canLiftLeftFoot
            self.canLiftRightFootModule ="fake"   #name of a module implementing canLiftRightFoot
        else:
            self.controlledMotors = self.robot.motors #self.robot.legs + self.robot.torso
            
            self.walkModules["fullWalkModules"]["fake"] = MOCKWalkerModule() #mock module that implements footLanded, canLiftLeftFoot and canLiftRightFoot
            self.leftFootLandedModule = "fake" #name of a module implementing footLanded
            self.rightFootLandedModule = "fake"#name of a module implementing footLanded
            self.canLiftLeftFootModule = "l_knee_y"#name of a module implementing canLiftLeftFoot
            self.canLiftRightFootModule ="fake"   #name of a module implementing canLiftRightFoot

            
            
            armTime = 1.#time of step phase
            legTime = 1. #time of double support phase
            
            leg1Time = armTime+legTime
            arm2Time = leg1Time+armTime
            leg2Time = arm2Time + legTime
            
            times = [armTime, leg1Time, arm2Time, leg2Time]
            
            self.walkModules["fullWalkModules"]["r_elbow_y"] = TrajectoryModule("r_elbow_y", [-120, -90, -90, -90], times, self.dt) 
            self.walkModules["fullWalkModules"]["r_shoulder_y"] = TrajectoryModule("r_shoulder_y", [-60, -90, -90, -90], times, self.dt) 
           
            self.walkModules["fullWalkModules"]["l_elbow_y"] = TrajectoryModule("l_elbow_y", [-90, -90, -120, -90], times, self.dt) 
            self.walkModules["fullWalkModules"]["l_shoulder_y"] = TrajectoryModule("l_shoulder_y", [-90, -90, -60, -90], times, self.dt) 
           
            self.walkModules["fullWalkModules"]["l_knee_y"] = TrajectoryModule("l_knee_y", [90, 120, 90, 70], times, self.dt) 
            self.walkModules["fullWalkModules"]["l_hip_y"] = TrajectoryModule("l_hip_y", [-90, -120, -90, -70], times, self.dt) 
  
            self.walkModules["fullWalkModules"]["r_knee_y"] = TrajectoryModule("r_knee_y", [90, 70, 90, 120], times, self.dt) 
            self.walkModules["fullWalkModules"]["r_hip_y"] = TrajectoryModule("r_hip_y", [-90, -70, -90, -120], times, self.dt) 
            
            self.walkModules["fullWalkModules"]["abs_z"] = TrajectoryModule("abs_z", [-15, -15,15, 15], times, self.dt) 
             
             
            self.walkModules["fullWalkModules"]["l_ankle_y"] = AngularControl("constant", "l_ankle_y", scale = 0.5,referenceMaster= 90) 
            self.walkModules["fullWalkModules"]["l_hip_x"] = AngularControl("constant", "l_hip_x", scale = 0.5) 
            self.walkModules["fullWalkModules"]["l_hip_z"] = AngularControl("constant", "l_hip_z", scale = 0.5,referenceMaster= 20) 
            self.walkModules["fullWalkModules"]["l_shoulder_x"] = AngularControl("constant", "l_shoulder_x", scale = 0.5) 
            self.walkModules["fullWalkModules"]["l_arm_z"] = AngularControl("constant", "l_arm_z", scale = 0.5)
            
            self.walkModules["fullWalkModules"]["r_ankle_y"] = AngularControl("constant", "r_ankle_y", scale = 0.5, referenceMaster=90) 
            self.walkModules["fullWalkModules"]["r_hip_x"] = AngularControl("constant", "r_hip_x", scale = 0.5) 
            self.walkModules["fullWalkModules"]["r_hip_z"] = AngularControl("constant", "r_hip_z", scale = 0.5,referenceMaster= -20) 
            self.walkModules["fullWalkModules"]["r_shoulder_x"] = AngularControl("constant", "r_shoulder_x", scale = 0.5) 
            self.walkModules["fullWalkModules"]["r_arm_z"] = AngularControl("constant", "r_arm_z", scale = 0.5) 
            
            self.walkModules["fullWalkModules"]["head_y"] = AngularControl("constant", "head_y", scale = 0.5) 
            self.walkModules["fullWalkModules"]["head_z"] = AngularControl("constant", "head_z", scale = 0.5) 
            self.walkModules["fullWalkModules"]["bust_x"] = AngularControl("constant", "bust_x", scale = 0.5) 
            self.walkModules["fullWalkModules"]["bust_y"] = AngularControl("constant", "bust_y", scale = 0.5) 
            self.walkModules["fullWalkModules"]["abs_x"] = AngularControl("constant", "abs_x", scale = 0.5) 
            self.walkModules["fullWalkModules"]["abs_y"] = AngularControl("constant", "abs_y", scale = 0.5) 
            #~ self.walkModules["fullWalkModules"]["abs_z"] = AngularControl("constant", "abs_z", scale = 0.5) 

            #############
            ## walkModules : active during all phases
           
            #logger module
            self.walkModules["fullWalkModules"]["logger"] = LoggerModule([ "r_elbow_y", "r_shoulder_y","r_knee_y", "r_hip_y"])
            
            #############
            ## leftStepModules : active during left step (left foot up)


            
            #############
            ## rightStepModules : active during right step (right foot up)

            #############
            ## leftDoubleSupportModules : active during left double support phase (both feet down, left in front)

            
            #############
            ## rightDoubleSupportModules : active during right double support phase (both feet down, right in front)

            
        
        self.initTime = time.time()
        self.lastTime = self.initTime
        
        
    ###
    
    def update(self):
        
        #read motors positions
        motorPositions = {}
        motorPositionsList = [0.]*25
        if self.robot is not None:
            for m in self.controlledMotors:
                motorPositions[m.name] = m.present_position
                motorPositionsList[self.kinematics.articulationNames.index(m.name)] =motorPositions[m.name]
                
        else:
            for m in self.kinematics.articulationNames:
                motorPositions[m] = random.randint(-20, 20)
                motorPositionsList[self.kinematics.articulationNames.index(m)] =motorPositions[m]
                
        #~ print "motor positions ",motorPositions.keys()
        #~ print "motor positions ",motorPositions.values()
        
        #read razor data
        razorData = [0., 0.]
        if self.razor is not None:
            try:
                razorData = self.razor.eul()
            except:
                print "error could not read razor"    
                
        #compute kinematics
        #~ self.kinematics.updateModel(array(motorPositionsList), razorData[0], razorData[1])

        return motorPositions
        
    #called at the beginning of each pahse. Phase can be "left step", "right double support", "right step" or "left double support"
    def initPhase(self, phase):
        
        print "time ",time.time()- self.initTime
        
        #change reference frame for kinematics
        if phase == "left step" or phase == "right double support":
            self.kinematics.refFrame = "RFoot"
        else:
            self.kinematics.refFrame = "LFoot"
            
        #reset modules
        for m in self.walkModules[phase].values():
            m.reset()
  
        for m in self.walkModules["fullWalkModules"].values():
            m.reset(phase=phase)
        
    def setMotorPositions(self, positions):
        if self.robot is not None:
            #~ self.robot.goto_position(positions, self.dt, wait=False) #longer. Dummy method equivalent to speed control
            for m in self.controlledMotors:
                m.goal_position = positions[m.name]
 
    def setMotorSpeeds(self, positions, positionsBefore):
        if self.robot is not None:
            for m in self.controlledMotors:
                
                speed = ( positions[m.name] - positionsBefore[m.name] )/self.dt

                #~ print m.name, speed
                max_speed = 50
                if speed > max_speed:
                    speed = max_speed
                if speed < -max_speed:
                    speed = -max_speed
                m.goal_speed = speed    
                
    #unused, hybrid method between position and speed control to enhance processing speed
    def moveMotors(self, positions, positionsBefore):
        if self.robot is not None:
            for m in self.controlledMotors:

                speed = ( positions[m.name] - positionsBefore[m.name] )/self.dt

                max_speed = 30

                if abs(speed) < max_speed:
                    #~ print m.name, " SPEED"
                    m.goal_speed = speed    
                else:
                    #~ print m.name, " POSITION"
                    m.goal_speed = 2*max_speed
                    m.goal_position = positions[m.name]
                    #~ m.goto_position(positions[m.name], self.dt, wait=False)

    def executeModules(self, phase):
        #~ print "----"
        
        #read motor positions
        motorPositions = self.update()
        motorNextPositions = copy.deepcopy(motorPositions)
        
        #modify motor next positions by each module
        for m in self.walkModules[phase].values():
            motorNextPositions = m.execute(motorPositions, motorNextPositions, phase=phase)
            #~ print m#, " ",motorNextPositions.keys()
            
        for m in self.walkModules["fullWalkModules"].values():
            motorNextPositions = m.execute(motorPositions, motorNextPositions, phase=phase)
            #~ print m#, " ",motorNextPositions.keys()
            
            
        #~ print "next position ",motorNextPositions.keys()
        
        #~ print "next position ",motorNextPositions.values()
        #Apply modified values
        #~ self.setMotorPositions(motorNextPositions)
        self.setMotorSpeeds(motorNextPositions, motorPositions)
        #~ self.moveMotors(motorNextPositions, motorPositions)
        
#make sure we have self.dt time between each step. Print a warning if overtime
    def waitForStepEnd(self):

        now = time.time()
        #~ print "dt ",now - self.lastTime
        #~ print "time ",now - self.initTime
        if now - self.lastTime < self.dt:
            time.sleep(self.dt - now + self.lastTime)
        else:
            print "Warning, overtime! ",now - self.lastTime," instead of ",self.dt
        self.lastTime =  time.time()
        
        
    ###
        
    def stepRight(self):
        print "### starting step right ###"

        self.initPhase("right step")
  
        while not self.walkModules["fullWalkModules"][self.rightFootLandedModule ].footLanded():
            
            self.executeModules("right step")
            self.waitForStepEnd()
            

    def stepLeft(self):
        print "### starting step left ###"

        self.initPhase("left step")
  
        while not self.walkModules["fullWalkModules"][self.leftFootLandedModule ].footLanded():
            
            self.executeModules("left step")
            self.waitForStepEnd()

            
    def doubleSupportRight(self):
        print "### starting double support right ###"
        
        self.initPhase("right double support")
        
        while not self.walkModules["fullWalkModules"][self.canLiftLeftFootModule].canLiftLeftFoot():
            
            self.executeModules("right double support")
            self.waitForStepEnd()       
        
  
    def doubleSupportLeft(self):
        print "### starting double support left ###"

        self.initPhase("left double support")

        while not self.walkModules["fullWalkModules"][self.canLiftRightFootModule].canLiftRightFoot():
            
            self.executeModules("left double support")
            self.waitForStepEnd()     
        
    ###
    
    #when true, walk starts
    #wait for arms to be lifted
    def init(self):
        
        return True

        
    def startWalk(self):
        initPos = {}
        initPos["head_y"] = 0.0
        initPos["head_z"] = -10.0
        initPos["r_shoulder_y"] = -90.0
        initPos["r_shoulder_x"] = 0.0
        initPos["r_arm_z"] = 0.0
        initPos["r_elbow_y"] = -90.0
        initPos["l_shoulder_y"] = -90.0
        initPos["l_shoulder_x"] = 0.0
        initPos["l_arm_z"] = 0.0
        initPos["l_elbow_y"] = -90.0
        initPos["r_hip_z"] = 0.0
        initPos["r_hip_y"] = -90.0
        initPos["r_hip_x"] = 0.0
        initPos["r_knee_y"] = 90.0
        initPos["r_ankle_y"] = 90.0
        initPos["l_hip_z"] = 0.0
        initPos["l_hip_y"] = -90.0
        initPos["l_hip_x"] = 0.0
        initPos["l_knee_y"] = 90.0
        initPos["l_ankle_y"] = 90.0
        initPos["bust_x"] = 0.0
        initPos["bust_y"] = 0.0
        initPos["abs_x"] = 0.0
        initPos["abs_y"] = 0.0
        initPos["abs_z"] = 0.0
        
        self.robot.goto_position(initPos, 3., wait=True)
        
        
            
    def oneStep(self):
        if self.stepSide == "right":
            self.doubleSupportRight()
            self.stepSide = "left"
            self.stepLeft()
            
        else:
            self.doubleSupportLeft()
            self.stepSide = "right"
            self.stepRight()
            
    #when true, walk stops
    # true after 40 seconds (timeout) or when arms are lowered
    def mustStopWalk(self):
        if self.robot is not None:
            #safety : don't walk more than X seconds
            if time.time() - self.initTime > 40:
                print "walk timeout"
                self.robot.head_y.goal_position = 10
                return True

            if self.robot.l_shoulder_y.present_position < -20 or self.robot.r_shoulder_y.present_position < -20 :
                return False

        print "asked to stop walk"
        self.robot.head_y.goal_position = 10
        return True
        
    def stopWalk(self):
        print "### stopping walk ###"
        if self.robot is not None:
            #set goal speed to 0
            for m in self.robot.motors:
                m.goal_speed = 0.

            time.sleep(0.5)
            
            #slowly return to zero position
            for m in self.robot.motors:
                m.goto_position(0.0,2., wait=False)
            print "%%% Warning, I will remove compliance! %%%"                
            time.sleep(2)
            
            #remove compliance
            for m in self.robot.motors:
                m.compliant = True
            
        #plot walk data
        self.plot()

    #use pylab to create a graph of walk data
    def plot(self):
        import pylab
        
        colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
        colorsIndex = 0
        
        hasPlot = False
        
        for modules in self.walkModules.values():
            for module in modules.values():
                #for all modules, if there is log, plot it
                toPlot = module.logs
                for dataName in toPlot.keys():
                    pos = toPlot[dataName]
                    t = linspace(0., 1.*len(pos)*self.dt, len(pos))
        
                    pylab.plot(t, pos, '-'+colors[colorsIndex], label=dataName)
                    hasPlot = True
                    colorsIndex += 1
                    if colorsIndex >= len(colors):
                        colorsIndex -= len(colors)
        if hasPlot:
            print "### plotting ###"
            pylab.legend(loc='upper right')
            pylab.grid(True)
            pylab.savefig("walk_data.png")
        
    def clean(self):
        pass
        
        