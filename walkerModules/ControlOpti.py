import WalkerModule
import random



class ControlZMP(WalkerModule.WalkerModule):
    def __init__(self, kinematics, footContact):
        WalkerModule.WalkerModule.__init__(self)
        self.kinematics = kinematics
        self.footContact = footContact
        # initial state (stand up)
        self.state = 0
        # speed of the algorithm (in deg/s)
        self.speed = 10
        # number of degrees of freedom
        self.ndof = 25
    
    def stepRightExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #position under left foot
        return self.controlOpti( motorPositions, motorNextPositions)
        
    def stepLeftExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #position under right foot
        return self.controlOpti( motorPositions, motorNextPositions)
        
    def doubleSupportRightExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #between feet
        return self.controlOpti( motorPositions, motorNextPositions)
        
    def doubleSupportLeftExecute(self, motorPositions, motorNextPositions):
        ZMPpos = [] #between feet
        return self.controlOpti( motorPositions, motorNextPositions)
        
        
    def controlOpti(self, motorPositions, motorNextPositions):
        #temporary costs before differenciation
        c0 = zeros((self.ndof,))
        cp = zeros((self.ndof,))
        cm = zeros((self.ndof,))
        # differenciation
        dc = zeros((self.ndof,))
        
        # computation of robot joint positions
        l_ankle = dot(kinematics.points["l_ankle"]["position"].reshape(kinematic["l_ankle"]["position"].shape+(1,)),ones((1,self.ndof)))
        l_ankle_p = l_ankle+self.dq*kinematics.points["l_ankle"]["jacobian"]
        l_ankle_m = l_ankle-self.dq*kinematics.points["l_ankle"]["jacobian"]
        l_toe = dot(kinematics.points["l_toe"]["position"].reshape(kinematic["l_toe"]["position"].shape+(1,)),ones((1,self.ndof)))
        l_toe_p = l_toe+self.dq*kinematics.points["l_toe"]["jacobian"]
        l_toe_m = l_toe-self.dq*kinematics.points["l_toe"]["jacobian"]
        
        
        #~ print "controlling ZMP"
        return motorNextPositions
       
    def canLiftLeftFoot(self):
        
        #MOCK
        r = random.randint(0, 10)
        if r == 0:
            print "can lift left foot"
            return True
            
        #~ print "can't lift left foot"
        return False
        
         
    def canLiftRightFoot(self):
        
        #MOCK
        r = random.randint(0, 10)
        if r == 0:
            print "can lift right foot"
            return True
            
        #~ print "can't lift right foot"
        return False       
 