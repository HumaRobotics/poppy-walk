from numpy import *

class Kinematics:
    def __init__(self):
        self.motorsOrder = {"abs_y" : 0, "abs_x" : 1,"abs_z" : 2,"bust_y" : 3,"bust_x" : 4,
                                        "l_shoulder_y" : 5,"l_shoulder_x" : 6,"l_arm_z" : 7,"l_elbow_y" : 8,"r_shoulder_y" : 9,
                                        "r_shoulder_x" : 10,"r_arm_z" : 11,"r_elbow_y" : 12,"head_z" : 13,"head_y" : 14,
                                        "l_hip_x" : 15,"l_hip_z" : 16,"l_hip_y" : 17,"l_knee_y" : 18,"l_ankle_y" : 19,
                                        "r_hip_x" : 20,"r_hip_z" : 21,"r_hip_y" : 22,"r_knee_y" : 23,"r_ankle_y" : 24}
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