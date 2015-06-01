import sensors as S
import pypot.robot as R
import time as T

rb = R.from_json("/home/poppy/poppy.json")
rb.start_sync()
rz = S.razor.Razor()
rz.start()

T.sleep(5)
print rb.head_z.present_position
print rz.eul

rb.stop_sync()
rb.close()
rz.stop()