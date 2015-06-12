        
from pypot.primitive.move import Move,  MovePlayer
import WalkerModule
        
class CPGModule(WalkerModule.WalkerModule, motorName, dt, amplitude = 1., cycleTime=1., ratio = 0.):
    def __init__(self, file):
        WalkerModule.WalkerModule.__init__(self)
        self.motorName = motorName
        self.amplitude = amplitude
        self.cycleTime = cycleTime
        self.currentTime = ratio*cycleTime
        self.dt = dt
        self.finished = False

        
    def execute(self, motorPositions, motorNextPositions, phase=""):
        
        
        if self.finished:
            print "CPG finished"
            return motorNextPositions

        
        
        
        #play one step of the file
        nextPositions = self.move.positions()[self.index]
        for m in nextPositions.keys():
            motorNextPositions[m] = nextPositions[m]
        
        self.currentTime += dt
        if self.currentTime >= self.cycleTime:
            self.currentTime -= self.cycleTime
            self.finished = true
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