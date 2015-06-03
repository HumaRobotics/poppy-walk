from numpy import *

class Kinematics:
    def __init__(self):
        pass
        #TODO define model
        
    def updateModel(self, q):
        #q: liste of motor angles, order TO DEFINE
        print "compute model"
        pass
        
    def getPosition(self, fromRef, toRef, point = array([0.,0.,0.])):
        #exemple razorPos = getPosition("pelvis", "r_foot", point = array([0.2,-0.3,0.])), with [0.2,-0.3,0.] position of razor in pelvis referential
        return array([0.,0.,0.])
        
    def getSpeed(self, fromRef, toRef, point = array([0.,0.,0.])):
        #exemple razorSpeed = getSpeed("pelvis", "r_foot", point = array([0.2,-0.3,0.])), with [0.2,-0.3,0.] position of razor in pelvis referential
        return array([0.,0.,0.])