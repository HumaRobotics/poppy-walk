
from walkerModules.WalkerModule import WalkerModule, AngularControl, LoggerModule, MOCKWalkerModule
from walkerModules.PlayJsonModule import PlayJsonModule
from walkerModules.ControlZMP import ControlZMP, MOCKControlZMP
from walkerModules.CPGModule import CPGModule
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
            
            self.leftFootLandedModule = "l_knee_y" #from left step module !
            self.rightFootLandedModule = "r_knee_y"
            self.canLiftLeftFootModule = "l_hip_x"
            self.canLiftRightFootModule = "r_hip_x"    

            #~ self.rightStepModules["swingFoot"] = PlayJsonModule("json/rlegstep2.json")
            #~ self.leftStepModules["swingFoot"] = PlayJsonModule("json/llegstep2.json")    
            
            
            CPGstepTime = 1.5#time of step phase
            CPGDSTime = 0.5 #time of double support phase
            twoStepsTime = 2*(CPGstepTime  + CPGDSTime)
            
            #############
            ## walkModules : active during all phases
            
            #transfer weight : full sinus, start at double support
            self.walkModules["fullWalkModules"]["l_hip_x"] = CPGModule("l_hip_x", self.dt, cycleTime = twoStepsTime, startRatio = (2*CPGstepTime+CPGDSTime) /twoStepsTime, amplitude = 20., offset = 5., reset="never")
            self.walkModules["fullWalkModules"]["r_hip_x"] = CPGModule("r_hip_x", self.dt, cycleTime = twoStepsTime, startRatio = (CPGstepTime) /twoStepsTime, amplitude = -20., offset = -5., reset="never")
           
           #keep bust straight
            self.walkModules["fullWalkModules"]["keep bust_y"] = AngularControl("constant", "bust_y", scale = 0.5)                   
            #~ self.walkModules["fullWalkModules"]["keep abs_z"] = AngularControl("constant", "abs_z", scale = 0.5)
            self.walkModules["fullWalkModules"]["keep abs_z"] = CPGModule("abs_z", self.dt, cycleTime = twoStepsTime, startRatio = 0., amplitude = 10., reset="never")
            self.walkModules["fullWalkModules"]["keep abs_y"] = AngularControl("constant", "abs_y", scale = 0.8) 
            
            #bust oscillate to compensate moves from weight transfer
            self.walkModules["fullWalkModules"]["abs_x"] = AngularControl("l_hip_x", "abs_x", scale = 0.5, referenceMaster= 5) 
            self.walkModules["fullWalkModules"]["bust_x"] = AngularControl("l_hip_x", "bust_x", scale = 0.5, referenceMaster= 5) 
            
            #move the leg on y to avoid passing through other leg. Keep constant otherwise
            self.walkModules["fullWalkModules"]["l_hip_z"] = CPGModule("l_hip_z", self.dt, cycleTime = twoStepsTime+2*CPGDSTime, startRatio = 0.5, amplitude = -30., disabledPhases=["right step", "left step"], reset="right double support")
            self.walkModules["left step"]["l_hip_z"] = CPGModule("l_hip_z", self.dt, cycleTime = twoStepsTime+2*CPGDSTime, amplitude = 0., offset = 15)
            self.walkModules["right step"]["l_hip_z"] = CPGModule("l_hip_z", self.dt, cycleTime = twoStepsTime+2*CPGDSTime, amplitude = 0., offset = 0)
            
            self.walkModules["fullWalkModules"]["r_hip_z"] = CPGModule("r_hip_z", self.dt, cycleTime = twoStepsTime+2*CPGDSTime , startRatio =0., amplitude = -30., disabledPhases=["left step", "right step"], reset="left double support")
            self.walkModules["left step"]["r_hip_z"] = CPGModule("r_hip_z", self.dt, cycleTime = twoStepsTime+2*CPGDSTime, amplitude = 0., offset = 0)
            self.walkModules["right step"]["r_hip_z"] = CPGModule("r_hip_z", self.dt, cycleTime = twoStepsTime+2*CPGDSTime, amplitude = 0., offset = -15)

            #when not lifted, leg goes straight 
            self.walkModules["fullWalkModules"]["r_knee_y"] = CPGModule("r_knee_y", self.dt, cycleTime =twoStepsTime+2*CPGDSTime, startRatio= 0., amplitude = -20, offset = 20, disabledPhases=["right step"], reset="right double support")
            self.walkModules["fullWalkModules"]["r_hip_y"] = CPGModule("r_hip_y", self.dt, cycleTime = twoStepsTime+2*CPGDSTime , startRatio= 0., amplitude =10, offset = -10, disabledPhases=["right step"], reset="right double support")
            
            self.walkModules["fullWalkModules"]["l_knee_y"] = CPGModule("l_knee_y", self.dt, cycleTime =twoStepsTime+2*CPGDSTime ,startRatio= 0.,  amplitude = -20, offset = 20, disabledPhases=["left step"], reset="left double support")
            self.walkModules["fullWalkModules"]["l_hip_y"] = CPGModule("l_hip_y", self.dt, cycleTime = twoStepsTime+2*CPGDSTime , startRatio= 0., amplitude = 10, offset = -10, disabledPhases=["left step"], reset="left double support")           
            
            #ankle move: flat when landing, max bent when other foot is landing
            self.walkModules["fullWalkModules"]["r_ankle_y"] = CPGModule("r_ankle_y", self.dt, cycleTime = twoStepsTime , startRatio= 0.25, amplitude =-5, offset= -5., reset="never")
            self.walkModules["fullWalkModules"]["l_ankle_y"] = CPGModule("l_ankle_y", self.dt, cycleTime = twoStepsTime , startRatio= 0.25, amplitude =5, offset= -5., reset="never")
            
            # balancing module
            #~ self.walkModules["fullWalkModules"]["ZMPbalancing"] = ControlZMP(self.kinematics)
            
            #logger module
            #~ self.walkModules["fullWalkModules"]["logger"] = LoggerModule([ "r_hip_x", "l_hip_x"])
            #~ self.walkModules["fullWalkModules"]["logger"] = LoggerModule([ "l_ankle_y", "l_knee_y","l_hip_x", "l_hip_y", "l_hip_z"])
            
            #############
            ## leftStepModules : active during left step (left foot up)
            
            #left leg lift
            self.walkModules["left step"]["l_knee_y"] = CPGModule("l_knee_y", self.dt, cycleTime =0.8*2*CPGstepTime, amplitude = 50, offset = 20, stopRatio=0.5)
            self.walkModules["left step"]["l_hip_y"] = CPGModule("l_hip_y", self.dt, cycleTime = 2*CPGstepTime, amplitude = -40, offset = -0)

            
            #############
            ## rightStepModules : active during right step (right foot up)
            #right leg lift
            self.walkModules["right step"]["r_knee_y"] = CPGModule("r_knee_y", self.dt, cycleTime = 0.8*2*CPGstepTime, amplitude = 50, offset = 20, stopRatio=0.5)
            self.walkModules["right step"]["r_hip_y"] = CPGModule("r_hip_y", self.dt, cycleTime = 2*CPGstepTime,amplitude = -40, offset = -0)

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
                
        #~ print motorPositions
        
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
        pass
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
            
        #Apply modified values
        self.setMotorPositions(motorNextPositions)
        #~ self.setMotorSpeeds(motorNextPositions, motorPositions)
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
  
        while not self.walkModules["right step"][self.rightFootLandedModule ].footLanded():
            
            self.executeModules("right step")
            self.waitForStepEnd()
            

    def stepLeft(self):
        print "### starting step left ###"

        self.initPhase("left step")
  
        while not self.walkModules["left step"][self.leftFootLandedModule ].footLanded():
            
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

        if self.robot is not None:
            
            #free arms
            for m in self.robot.arms:
                m.compliant = True
            time.sleep(0.5)
            
            #look up
            self.robot.head_y.goal_position = -10
            
            #init time to avoid overtime warning
            self.lastTime = time.time()
            
            print "%%% lift robot's arms to start walk %%%"
            
            counter = int(30./self.dt) #wait max 30 seconds
            while counter > 0:
                counter -= 1
                #~ print self.robot.l_shoulder_y.present_position,  " ", self.robot.r_shoulder_y.present_position
                if self.robot.l_shoulder_y.present_position < -80 and self.robot.r_shoulder_y.present_position < -80 :
                    print "%%% starting walk %%%"
                    
                    #arms not compliants and 
                    for m in self.robot.arms:
                        m.compliant = False
                    #except for shoulder_y
                    self.robot.l_shoulder_y.compliant = True
                    self.robot.r_shoulder_y.compliant =True

                    #init time
                    self.initTime = time.time()
                    self.lastTime = self.initTime
                    print "%%% put robot's arms down to stop walk %%%"
                    return True
                 
                #if arms not in position, wait self.dt and retry
                self.waitForStepEnd()  
                
        print "arms have not been raised"
        return False
        
    def startWalk(self):
        pass
        
        
            
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

            if self.robot.l_shoulder_y.present_position < -30 or self.robot.r_shoulder_y.present_position < -30 :
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
        
        