
import WalkerModule
import math
        
class CPGModule(WalkerModule.WalkerModule):
    def __init__(self, motorName, dt, amplitude = 1., cycleTime=1., ratio = 0., offset = 0.):
        WalkerModule.WalkerModule.__init__(self)
        self.motorName = motorName
        self.amplitude = amplitude
        self.cycleTime = cycleTime
        self.currentTime = ratio*cycleTime
        self.offset = offset
        self.dt = dt
        self.finished = False

        
    def execute(self, motorPositions, motorNextPositions, phase=""):
        
        
        if self.finished:
            #~ print "CPG finished"
            self.finished = False
            return motorNextPositions

        if self.motorName not in motorNextPositions.keys():
            raise Exception,"No motor named: " +self.motorName
            
        motorNextPositions[self.motorName] = self.amplitude*math.sin(2*math.pi*self.currentTime/self.cycleTime) + self.offset
        print self.motorName, " ",motorNextPositions[self.motorName]
        
        
        #play one step of the file
        #~ nextPositions = self.move.positions()[self.index]
        #~ for m in nextPositions.keys():
            #~ motorNextPositions[m] = nextPositions[m]
        
        self.currentTime += self.dt
        if self.currentTime >= self.cycleTime:
            self.currentTime -= self.cycleTime
            self.finished = True
        return motorNextPositions
        
    def footLanded(self):
        if self.finished:
            self.finished = False
            return True
        return False
        
        #~ #MOCK
        #~ r = random.randint(0, 10)
        #~ if r == 0:
            #~ print "foot landed"
            #~ return True

        #~ return False