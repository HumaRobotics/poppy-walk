#~ import sensors 

import time

HAS_REAL_ROBOT = True

#############
# PARAMATERS
#############

### Robot config ###
FULL_POPPY_HUMANOID = False
robot_configuration_file = "/home/poppy/poppy.json" #Needed only if FULL_POPPY_HUMANOID is False
###


#TODO: when we have several functions of that type, create a utils scipt
def createPoppyCreature():
    poppy = None
    if FULL_POPPY_HUMANOID:
        from poppy_humanoid import PoppyHumanoid
        try:
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
    


#############
## MAIN
#############
# starts the robot
if HAS_REAL_ROBOT:
    poppy = createPoppyCreature()
    time.sleep(5)    
else:
    poppy = None

# stops the robot
if HAS_REAL_ROBOT:
    time.sleep(0.5)
    for m in poppy.motors:
        m.compliant = True
        
    time.sleep(0.5)

    if not FULL_POPPY_HUMANOID:
        poppy.stop_sync()
        poppy.close()