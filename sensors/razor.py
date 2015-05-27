import pypot.dynamixel as D
import threading as T
import serial as S
import logging
import time

logger = logging.getLogger(__name__)
# the Razor sensor is inherited from a Thread

class Razor(T.Thread):

  def __init__(self):
    T.Thread.__init__(self)
    self.daemon = True
    # get the list of ports
    listcom = D.get_available_ports()
    razorcom = []
    for com in listcom:
      try:
        #test the port
        a = S.Serial(com, 57600,timeout = 0.1)
        time.sleep(2)
        # do not forget to strip the zeros values
        t = a.readline().strip('\x00')
        a.close()
        if len(t)>0:
          if t[0:5] == '#YPR=':
            razorcom.append(com)
      except S.SerialException:
        pass
    if len(razorcom) == 1:
      # we got the right port
      print('Razor connected to '+razorcom[0]+'.')
      self.s = S.Serial(razorcom[0],57600,timeout=0.1)
    else:
      # no port available or no razor connected
      print('no Razor connected.')
      self.s = None
      
    # initialisation of all parameters (aeronautical convention with Z downward)
    # euler angles in degrees
    self._eul = [0.0,0.0,0.0]
    # acceleration not converted
    self._acc = [0.0,0.0,0.0]
    # magnetic field not converted
    self._mag = [0.0,0.0,0.0]
    # attitude rate not converted
    self._gyr = [0.0,0.0,0.0]
    # control flag of the main loop
    self._goon = True
    # sampling period of the sensor
    self._dt = 0.01

# property definition is independant to include conversion later
  @property
  def eul(self):
    return self._eul
  
  @property
  def acc(self):
    return self._acc
    
  @property
  def mag(self):
    return self._mag
  
  @property
  def gyr(self):
    return self._gyr
    
  # main loop of the Thread
  def run(self):
    # stop the streaming
    self.s.write("#o0")
    # wash the streaming (flush does not work)
    l = self.s.readlines()
    # start to read the euler angles (a=1) a=2 matches with raw data
    a = 1
    while self._goon:
      t0 = time.time()
      if a == 1:
        # ask the euler angles
        self.s.write("#ot#f")
        # switch to raw data
        a = 2
      elif a == 2:
      # ask the raw data
        self.s.write("#osrt#f")
        # switch to euler angles
        a = 1
      while time.time()-t0<self._dt:
        # get one line (do not forget to strip the 00 characters)
        l = self.s.readline().strip('\x00')
        if len(l)>0:
        # if the line is not empty
          # euler angles
          if l[0:5]=='#YPR=':
            l = l.replace('\r\n','')
            l = l.replace('#YPR=','')
            self._eul = map(float,l.split(','))
          # accelerations
          if l[0:5]=='#A-R=':
            l = l.replace('\r\n','')
            l = l.replace('#A-R=','')
            self._acc = map(float,l.split(','))
          # magnetic field
          if l[0:5]=='#M-R=':
            l = l.replace('\r\n','')
            l = l.replace('#M-R=','')
            self._mag = map(float,l.split(','))
          # attitude rates
          if l[0:5]=='#G-R=':
            l = l.replace('\r\n','')
            l = l.replace('#G-R=','')
            self._gyr = map(float,l.split(','))
        
  def stop(self):
    self._goon = False
    # wait the main loop is finished
    time.sleep(1)
    self.s.close()
    print "razor sensor disconnected."