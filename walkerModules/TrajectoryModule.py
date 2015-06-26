import WalkerModule
        
class TrajectoryModule(WalkerModule.WalkerModule):
    def __init__(self, motorName, positions, times, dt, startTime=0.):
        WalkerModule.WalkerModule.__init__(self)
        self.motorName = motorName
        self.positions = positions
        self.times = times
        self.dt = dt
        self.currentTime = startTime
        self.finished = False
        
    def computeTangeants(self):
        self.tangeants = []
        for i in range(1, len(positions -1)):
            t = 0.5*((positions[i]- positions[i-1])(times[i+1]- times[i]) + (positions[i] - positions[i-1])(times[i+1] - times[i]))
            self.tangeants.append(t)
        
    def computePolynom(self):
        pass

        
    def execute(self, motorPositions, motorNextPositions, phase=""):
        if self.finished:
            self.finished = False
            self.currentTime = 0
            #~ print "restarting"
            #~ print "move ",self.filename," finished"
            #~ return motorNextPositions
        nextTime = -1
        nextPos = -1
            
        #~ for p in self.positions:
        for i in range(0, len(self.positions)):
            if self.times[i] > self.currentTime and (nextTime == -1 or  self.times[i]  - self.currentTime <  nextTime ):
                nextTime = self.times[i] - self.currentTime 
                nextPos = self.positions[i]
        
        if nextTime == -1:
            self.finished = True
            return motorNextPositions
            
            
        #~ print "target ",nextPos
        #~ print "time to go there ",nextTime
        #~ print "pos ",motorPositions[self.motorName]
        dpos =(nextPos - motorPositions[self.motorName])*self.dt/(nextTime)
        #~ print "dpos ", dpos
        max_dpos = 5
        if dpos > max_dpos:
            dpos = max_dpos
        if dpos < -max_dpos:
            dpos = -max_dpos
        
        
        motorNextPositions[self.motorName] += dpos
        
        self.currentTime +=self.dt
        
        
        #play one step of the file
        #~ nextPositions = self.move.positions()[self.index]
        #~ for m in nextPositions.keys():
            #~ motorNextPositions[m] = nextPositions[m]
        
        #~ self.index +=1
        #~ if self.index >= len(self.move.positions()):
            #~ self.finished = True
        return motorNextPositions
       

    def canLiftLeftFoot(self):
        return self.finished
    #~ def footLanded(self):
        #~ if self.finished:
            #~ self.init()
            #~ return True
        #~ return False
        
        #~ #MOCK
        #~ r = random.randint(0, 10)
        #~ if r == 0:
            #~ print "foot landed"
            #~ return True

        #~ return False