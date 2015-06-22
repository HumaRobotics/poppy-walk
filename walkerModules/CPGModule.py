
import WalkerModule
import math
        
class CPGModule(WalkerModule.WalkerModule):
    def __init__(self, motorName, dt, amplitude = 1., cycleTime=1., startRatio = 0., stopRatio = 1., offset = 0., disabledPhases=[], reset="always"):
        WalkerModule.WalkerModule.__init__(self, reset)
        self.motorName = motorName
        self.amplitude = amplitude
        self.cycleTime = cycleTime
        self.startRatio = startRatio
        self.currentTime = startRatio*cycleTime
        self.finishedTime = stopRatio*cycleTime
        self.offset = offset
        self.dt = dt
        self.finished = False
        self.disabledPhases = disabledPhases

        
    def execute(self, motorPositions, motorNextPositions, phase=""):

        
        #~ print self.currentTime
        if self.finished:
            #~ print "CPG finished"
            self.finished = False
            #~ return motorNextPositions

        if self.motorName not in motorNextPositions.keys():
            raise Exception,"No motor named: " +self.motorName
           
        if phase not in self.disabledPhases:
            motorNextPositions[self.motorName] += self.amplitude*math.sin(2*math.pi*self.currentTime/self.cycleTime) + self.offset - motorPositions[self.motorName]
        #~ print self.motorName, " ",motorNextPositions[self.motorName]

        self.currentTime += self.dt
        if self.currentTime >= self.finishedTime:
            #~ print self.motorName," foot landed ",self.currentTime
            self.finished = True
            #~ print self.motorName, " ","finished"
        if self.currentTime >= self.cycleTime:
            self.currentTime -= self.cycleTime
            #~ print self.motorName, " ","time reset"

        return motorNextPositions
        
    def reset(self, phase=""):
        
        if self.resetCondition == "always" or self.resetCondition==phase or (self.resetCondition == "on finished" and self.finished):
            print "RESET ", self.resetCondition
            self.currentTime = self.startRatio*self.cycleTime
            self.finished = False
        
        
    def footLanded(self):
        if self.finished:
            #~ print self.motorName," foot landed ",self.currentTime
            #~ self.finished = False
            return True
        return False
        
    def canLiftLeftFoot(self):
        return self.finished
        
    def canLiftRightFoot(self):
        return self.finished
              #~ #MOCK
        #~ r = random.randint(0, 10)
        #~ if r == 0:
            #~ print "foot landed"
            #~ return True

        #~ return False