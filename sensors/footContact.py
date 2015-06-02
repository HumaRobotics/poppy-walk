import threading as T
from yocto_api import *
from yocto_anbutton import *

class YoctoFootContact(T.Thread):
  def __init__(self):
    T.Thread.__init__(self)
    
    # initialisation of Yocto API
    errmsg = YRefParam()
    if YAPI.RegisterHub("usb", errmsg) != YAPI.SUCCESS:
      sys.exit("init error" + errmsg.value)

    YAPI.DisableExceptions()
    # Setup the API to use local USB devices
    if YAPI.RegisterHub("usb", errmsg) != YAPI.SUCCESS:
      sys.exit("init error" + errmsg.value)

    # allocate the sensor
    ch = YAnButton.FirstAnButton()
    serial = ch.get_module().get_serialNumber()
    self.channel1 = YAnButton.FindAnButton(serial + '.anButton1')
    self.channel1.registerValueCallback(button1Callback)
    self.channel2 = YAnButton.FindAnButton(serial + '.anButton2')
    self.channel2.registerValueCallback(button2Callback)
    self.channel3 = YAnButton.FindAnButton(serial + '.anButton3')
    self.channel3.registerValueCallback(button3Callback)
    self.channel4 = YAnButton.FindAnButton(serial + '.anButton4')
    self.channel4.registerValueCallback(button4Callback)
      
    self._rightFront = 0.0
    self._rightBack = 0.0
    self._leftFront = 0.0
    self._leftBack = 0.0
    self._goon = True
    print "Yocto buttons connected."
    
  def run(self):
    while self._goon:
      YAPI.Sleep(1)

  def button1Callback(fct, measure):
    self._rightFront = int(measure)

  def button2Callback(fct, measure):
    self._rightFront = int(measure)
  
  def button3Callback(fct, measure):
    self._rightFront = int(measure)
    
  def button4Callback(fct, measure):
    self._rightFront = int(measure)
  
  def stop(self):
    self.channel1.registerValueCallback(None)
    self.channel2.registerValueCallback(None)
    self.channel3.registerValueCallback(None)
    self.channel4.registerValueCallback(None)
    self._goon = False
    time.sleep(1)
    print "Yocto buttons disconnected."