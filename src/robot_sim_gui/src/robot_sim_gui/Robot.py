from tkinter import *
from PIL import Image, ImageTk
import math

from typing import Tuple

UPDATE_PERIOD = 10 # in ms

class Robot:
  def __init__(self, root:Tk, canvas:Canvas, resource_path: str, init_x:int, init_y:int):
    self.root = root
    self.canvas = canvas
    self.canvas.update()
    self.canvas_width = self.canvas.winfo_width()
    self.canvas_height = self.canvas.winfo_height()
    self.__resource_path = resource_path
    
    # Set initial robot position
    self.x_pos = init_x
    self.y_pos = init_y
    self.angle = 0

    # Initialize robot speed
    self.forward_speed = 0
    self.turn_speed = 0

    # Add robot image
    self.image = Image.open(f'{self.__resource_path}/SmallTank.png')
    self.tk_image = ImageTk.PhotoImage(self.image.rotate(self.angle))
    self.img_id = self.canvas.create_image(self.x_pos, self.y_pos, image=self.tk_image, anchor=NW)

    # Get image dimensions
    self.image_width = self.tk_image.width()
    self.image_height = self.tk_image.height()

    # Start update position loop
    self.updatePos()

  def updatePos(self):
    # Calculate robot x and y speeds
    x_speed = -self.forward_speed*math.sin(math.radians(self.angle))
    y_speed = -self.forward_speed*math.cos(math.radians(self.angle))

    # Don't update position if it would go outside the canvas
    new_x_pos = self.x_pos + x_speed
    new_y_pos = self.y_pos + y_speed
    if \
      new_x_pos + self.image_width < self.canvas_width and new_x_pos > 0 \
      and new_y_pos + self.image_height < self.canvas_height and new_y_pos > 0:
      
      self.x_pos += x_speed
      self.y_pos += y_speed
      self.angle += self.turn_speed
      self.angle %= 360

      self.canvas.delete(self.img_id)
      self.tk_image = ImageTk.PhotoImage(self.image.rotate(self.angle))
      self.img_id = self.canvas.create_image(self.x_pos, self.y_pos, image=self.tk_image, anchor=NW)

    self.root.after(UPDATE_PERIOD, lambda: self.updatePos())
  
  def updateSpeeds(self, new_speeds: Tuple[float, float]):
    '''
    Update left and right speed of robot.
    new_speeds = (<left_speed>, <right_speed>)
    '''
    left_speed, right_speed = new_speeds
    
    self.forward_speed = right_speed + left_speed
    self.turn_speed = 2*(right_speed - left_speed)

  def getPos(self) -> Tuple[int,int]:
    return (self.x_pos, self.y_pos)
  
  def getOrientation(self) -> float:
    return self.angle
