#~ import sensors 

import time

HAS_REAL_ROBOT = True
HAS_RAZOR = False

#############
# PARAMATERS
#############

### Robot config ###
FULL_POPPY_HUMANOID = True
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
razor = None
if HAS_RAZOR:
    from sensors import razor
    razor = razor.Razor()
    razor.start()

if HAS_REAL_ROBOT:
    poppy = createPoppyCreature()

    for m in poppy.motors:
        m.compliant = False
        #~ m.torque_limit = 80
        
    time.sleep(0.5)   
        
    for m in poppy.motors:
        m.goto_position(0.0, 1., wait=False)
      
    time.sleep(2)    
else:
    poppy = None

import Walker

walker = Walker.Walker(poppy, razor)

if walker.init():
    walker.startWalk()

    #~ for i in range(0,6):
    while not walker.mustStopWalk():
        walker.oneStep()

    walker.stopWalk()

    walker.clean()

if HAS_REAL_ROBOT:
    time.sleep(0.5)
    for m in poppy.motors:
        m.compliant = True
        
    time.sleep(0.5)

    if not FULL_POPPY_HUMANOID:
        poppy.stop_sync()
        poppy.close()

if HAS_RAZOR:
    razor.stop()





