from tkinter import *
#from robot_sim_gui.Robot import Robot
from Robot import Robot
from StatusLight import StatusLight

from typing import Tuple

class RobotSimCanvas:
  def __init__(self, root:Tk, canvas_width:int, canvas_height:int, resource_path:str, robot_init_x:int = 0, robot_init_y:int = 0):
    # Initialize canvas
    self.root = root
    self.canvas = Canvas(root, width=canvas_width, height=canvas_height)
    self.canvas.pack()
    self.__resource_path = resource_path

    # Create robot
    self.robot = Robot(root, self.canvas, resource_path, init_x=robot_init_x, init_y=robot_init_y)

    # Add target image
    self.target_tk_image = None
    self.target_img_id = None

    # Add status light
    self.status_light = StatusLight(self.canvas)

  def updateRobotSpeeds(self, new_speeds: Tuple[float, float]):
    '''
    Update left and right speed of robot.
    new_speeds = (<left_speed>, <right_speed>)
    '''
    self.robot.updateSpeeds(new_speeds)

  def addTarget(self, x_pos=0, y_pos=0):
    # Open image is it hasn't been yet
    if self.target_tk_image is None:
      self.target_tk_image = PhotoImage(file=f'{self.__resource_path}/TrafficCone.png')
    
    # Generate new target position
    self.target_x_pos = x_pos
    self.target_y_pos = y_pos

    # Create the image if it isn't on canvas
    if self.target_img_id is None:
      self.target_img_id = self.canvas.create_image(self.target_x_pos, self.target_y_pos, image=self.target_tk_image, anchor=NW)
    
    # Otherwise, just move
    else:
      self.canvas.moveto(self.target_img_id, self.target_x_pos, self.target_y_pos)

  def removeTarget(self):
    # Remove target, if it exists
    if self.target_img_id is not None:
      self.canvas.delete(self.target_img_id)
      self.target_img_id = None
      self.target_x_pos = None
      self.target_y_pos = None
  
  def getRobotPos(self) -> Tuple[int, int]:
    return self.robot.getPos()
  
  def getRobotOrientation(self) -> float:
    return self.robot.getOrientation()

  def getTargetPos(self) -> Tuple[int,int]:
    return (self.target_x_pos, self.target_y_pos)

  def setReachedTarget(self):
    self.status_light.setReachedTarget()
  
  def setNavigatingToTarget(self):
    self.status_light.setNavigatingToTarget()
  
  def getReachedTarget(self):
    return self.status_light.getReachedTarget()
  