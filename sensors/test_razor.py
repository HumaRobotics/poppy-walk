import razor
import pypot.robot 
import time

rz = razor.Razor()
rz.start()
rb = pypot.robot.from_json("/home/poppy/poppy.json")
rb.start_sync()

time.sleep(5)
print rb.head_z.present_position
print rz.eul

rb.stop_sync()
rb.close()
rz.stop()