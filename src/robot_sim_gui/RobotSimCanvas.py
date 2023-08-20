from tkinter import *
from Robot import Robot

class RobotSimCanvas:
  def __init__(self, root:Tk, canvas_width:int, canvas_height:int, robot_init_x:int = 0, robot_init_y:int = 0):
    # Initialize canvas
    self.root = root
    self.canvas = Canvas(root, width=canvas_width, height=canvas_height)
    self.canvas.pack()

    # Create robot
    self.robot = Robot(root, self.canvas, init_x=robot_init_x, init_y=robot_init_y)

    # Add target image
    self.target_tk_image = None
    self.target_img_id = None

  def updateRobotSpeeds(self, new_speeds: tuple[float, float]):
    '''
    Update left and right speed of robot.
    new_speeds = (<left_speed>, <right_speed>)
    '''
    self.robot.updateSpeeds(new_speeds)

  def addTarget(self, x_pos=0, y_pos=0):
    # Open image is it hasn't been yet
    if self.target_tk_image is None:
      self.target_tk_image = PhotoImage(file='resources/TrafficCone.png')
    
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
  
  def getRobotPos(self) -> tuple[int, int]:
    return self.robot.getPos()

  def getTargetPos(self) -> tuple[int,int]:
    return (self.target_x_pos, self.target_y_pos)
  