# this script is there to verify the Poppy system for walking

import sensors
import pypot.robot
import time

# first plug the razor !
razor = sensors.razor.Razor()
razor.start()
foot = sensors.footContact.YoctoFootContact()
foot.start()
poppy = pypot.robot.from_json("/home/poppy/poppy.json")
poppy.start_sync()

# set the robot to stand-up position
speedmax = 10
for m in poppy.motors:
  m.moving_speed = speedmax
  m.compliant = False
  m.goal_position = 0
  
time.sleep(5)

print foot.rightFront
print foot.rightBack
print foot.leftFront
print foot.leftBack
print razor.eul

razor.stop()
foot.stop()
poppy.close()