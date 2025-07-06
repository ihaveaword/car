from pyb import Pin

class WATERPUMP:
  def __init__(self,ipin):
    self.pump_state=0
    self.pump_key=Pin(ipin, Pin.OUT,Pin.PULL_NONE)
  def on(self):
      self.pump_key.value(1)
      self.pump_state=1
  def off(self):
      self.pump_key.value(0)
      self.pump_state=0
  def state(self):
      return self.pump_state

