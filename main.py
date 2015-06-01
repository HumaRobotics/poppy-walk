#~ import sensors 

import time


#############
# PARAMATERS
#############

### Robot config ###
FULL_POPPY_HUMANOID = True
robot_configuration_file = "/home/poppy/poppy.json" #Needed only if FULL_POPPY_HUMANOID is False
###

### Activated modules ###

###


#TODO: when we have several functions of that type, create a utils scipt
def createPoppyCreature():
    poppy = None
    if FULL_POPPY_HUMANOID:
        from poppy_humanoid import PoppyHumanoid
        try:
            from poppy_humanoid import PoppyHumanoid
            poppy = PoppyHumanoid()
        except Exception,e:
            print "could not create poppy object"
            print e
    else:
        import pypot.robot 
        try:
            
            with open(robot_configuration_file) as f:
                poppy_config = json.load(f)  
            poppy = pypot.robot.from_config(poppy_config)
            poppy.start_sync()
        except Exception,e:
            print "could not create poppy object"
            print e
        
    return poppy
    
def initWalk(poppy):
    #any initialization (modules, put robot standing...)
    #no walk moves yet !
    pass
    
def startWalk(poppy, side):
    print "starting Walk. Foot ", side, " in front"
    if side == "left":
        #initial step with left foot on front 
        pass
    elif side=="right":
        #initial step with right foot on front 
        pass
    else:
        raise Error('poppy-walk: doubleSupport argument should be "right" or "left"')
    
def step(poppy, side):
    print "step. Foot ", side, " up"
    if side == "left":
        #do step with left foot up and right foot on the floor
        pass
    elif side=="right":
        #do step with right foot up and left foot on the floor
        pass
    else:
        raise Error('poppy-walk: step argument should be "right" or "left"')
        
def doubleSupport(poppy, side):
    print "Double support. Foot ", side, " in front"
    if side == "left":
        #double support phase with left foot on front and right foot on back
        pass
    elif side=="right":
        #double support phase with right foot on front and left foot on back
        pass
    else:
        raise Error('poppy-walk: doubleSupport argument should be "right" or "left"')

def fullStep(poppy, side):
    #starting from double support with "side" step in front
    #do full walk cycle until back to initial position
    doubleSupport(poppy, side)
    
    if side == "right":
        side="left"
    else:
        side = "right"
        
    step(poppy, side)
    doubleSupport(poppy, side)
    
    if side == "right":
        side="left"
    else:
        side = "right"
    
    step(poppy, side)

def mustStopWalk(poppy):
    #destination reached, fall...
    return False

def stopWalk(poppy, side):
    print "stopping Walk."
    if side == "left":
        #final step starting with left foot on front 
        pass
    elif side=="right":
        #final step starting with right foot on front 
        pass
    else:
        raise Error('poppy-walk: doubleSupport argument should be "right" or "left"')

def cleanWalk(poppy):
    #clean, sit the robot...
    if not FULL_POPPY_HUMANOID:
        poppy.stop_sync()
        poppy.close()

#############
## MAIN
#############

poppy = None #createPoppyCreature()

initWalk(poppy)

side = "right"

startWalk(poppy, side )

counter = 5
while counter > 0 and not mustStopWalk(poppy):
    fullStep(poppy, side )
    counter -=1
    
stopWalk(poppy, side )
cleanWalk(poppy)






