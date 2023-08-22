from tkinter import *

STATUS_BOX_UPPER_LEFT = (10, 10)
STATUS_BOX_DIMENSIONS = (160, 70)
STATUS_BOX_LOWER_RIGHT = (STATUS_BOX_UPPER_LEFT[0] + STATUS_BOX_DIMENSIONS[0],
                   STATUS_BOX_UPPER_LEFT[1] + STATUS_BOX_DIMENSIONS[1])

STATUS_LIGHT_UPPER_LEFT = (20, 50)
STATUS_LIGHT_DIMENSIONS = (20, 20)
STATUS_LIGHT_LOWER_RIGHT = (STATUS_LIGHT_UPPER_LEFT[0] + STATUS_LIGHT_DIMENSIONS[0],
                            STATUS_LIGHT_UPPER_LEFT[1] + STATUS_LIGHT_DIMENSIONS[1])

STATUS_HEADING_COORDS = (80, 30)
STATUS_MSG_COORDS = (100, 60)

class StatusLight:
  def __init__(self, canvas:Canvas, reachedTargetInitVal:bool = False):
    self.canvas = canvas
    self.reachedTarget = reachedTargetInitVal

    self.initializeStatusLight()

  def initializeStatusLight(self):
    self.canvas.create_rectangle(*STATUS_BOX_UPPER_LEFT, *STATUS_BOX_LOWER_RIGHT, fill='white', outline='black')
    self.canvas.create_text(*STATUS_HEADING_COORDS, text='Robot Status', font=('TkHeadingFont', 16))

    self.status_light = self.canvas.create_oval(*STATUS_LIGHT_UPPER_LEFT, *STATUS_LIGHT_LOWER_RIGHT, fill='red', outline='black')
    self.status_msg = self.canvas.create_text(*STATUS_MSG_COORDS, text='Navigating to target')

  def setReachedTarget(self):
    self.canvas.itemconfig(self.status_light, fill='green')
    self.canvas.itemconfig(self.status_msg, text='Reached target')
    self.reachedTarget = True
  
  def setNavigatingToTarget(self):
    self.canvas.itemconfig(self.status_light, fill='red')
    self.canvas.itemconfig(self.status_msg, text='Navigating to target')
    self.reachedTarget = False
  
  def getReachedTarget(self) -> bool:
    return self.reachedTarget