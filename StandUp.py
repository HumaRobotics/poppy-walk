# this script is there to verify the Poppy system for walking

from sensors.razor import Razor
import sensors.footContact
import pypot.robot
import time,sys

# first plug the razor !
all_connected = True
try:
  razor = Razor()
except:
  all_connected = False
  raise
if all_connected == True:
  try:
    foot = sensors.footContact.YoctoFootContact()
  except:
    razor.stop()
    raise
    all_connected = False
if all_connected == True:
  try:
    poppy = pypot.robot.from_json("/home/poppy/poppy.json")
  except:
    razor.stop()
    foot.stop()
    raise
    all_connected = False
    
#~ if all_connected == True:
  #~ try:
    #~ from poppy_humanoid import PoppyHumanoid
    #~ poppy = PoppyHumanoid()
  #~ except:
    #~ razor.stop()
    #~ foot.stop()
    #~ raise
    #~ all_connected = False
    
if all_connected == True:
  foot.start()
  razor.start()
  poppy.start_sync()
  
  # set the robot to stand-up position
  speedmax = 10
  for m in poppy.motors:
    m.moving_speed = speedmax
    m.compliant = False
    m.goal_position = 0
    
  time.sleep(5)
  
  #~ print foot.rightFront
  #~ print foot.rightBack
  #~ print foot.leftFront
  #~ print foot.leftBack
  print razor.eul
  
  for m in poppy.motors:
    m.compliant = True
  
  razor.stop()
  foot.stop()
  poppy.close()
else:
  print "one of the connection aborted"

sys.exit()