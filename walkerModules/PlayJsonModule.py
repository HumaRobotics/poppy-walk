        
from pypot.primitive.move import Move,  MovePlayer
import WalkerModule
        
class PlayJsonModule(WalkerModule.WalkerModule):
    def __init__(self, file):
        WalkerModule.WalkerModule.__init__(self)
        self.filename = file
        self.move = None

        #file should be recorded with correct frequency
        with open(self.filename ) as f:
            self.move  = Move.load(f)
            
        self.init()
        
    def init(self):
        self.index = 0
        self.finished = False
        
    def execute(self, motorPositions, motorNextPositions, phase=""):
        if self.finished:
            print "move ",self.filename," finished"
            return motorNextPositions
        
        #play one step of the file
        nextPositions = self.move.positions()[self.index]
        for m in nextPositions.keys():
            motorNextPositions[m] = nextPositions[m]
        
        self.index +=1
        if self.index >= len(self.move.positions()):
            self.finished = True
        return motorNextPositions
        
    def footLanded(self):
        if self.finished:
            self.init()
            return True
        return False
        
        #~ #MOCK
        #~ r = random.randint(0, 10)
        #~ if r == 0:
            #~ print "foot landed"
            #~ return True

        #~ return False