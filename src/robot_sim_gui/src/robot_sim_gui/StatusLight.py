from tkinter import *
from tkinter import ttk

FRAME_TOP_LEFT = (10, 10)
FRAME_DIM = (160, 70)

LIGHT_DIAM = 20
LIGHT_DIMEN = (LIGHT_DIAM, LIGHT_DIAM)

class StatusLight:
  def __init__(self, canvas:Canvas, reachedTargetInitVal:bool = False):
    self.canvas = canvas
    self.reachedTarget = reachedTargetInitVal

    # Create Style object to configure styles
    s = ttk.Style()

    # Create status light frame
    s.configure('StatusFrame.TFrame', background='white', relief='solid')
    self.frame = ttk.Frame(self.canvas, width=FRAME_DIM[0], height=FRAME_DIM[1], style='StatusFrame.TFrame')
    self.frame['padding'] = 5
    
    # Create status heading
    s.configure('StatusLabel.TLabel', background='white', anchor=NW)
    self.status_heading = ttk.Label(self.frame, text='Robot Status', font=('TkHeadingFont', 16), style='StatusLabel.TLabel')

    # Create status light
    self.light_canvas = Canvas(self.frame, width=LIGHT_DIMEN[0] + 5, height=LIGHT_DIMEN[1] + 5, bg='white')
    self.light_canvas.configure(highlightthickness=0, borderwidth=0)
    self.status_light = self.light_canvas.create_oval(0, 0, *LIGHT_DIMEN, fill='red', outline='black')

    # Create status message
    self.status_msg = ttk.Label(self.frame, text='Navigating to target', style='StatusLabel.TLabel')

    # Arrange status light contents within frame
    self.status_heading.grid(column=0, row=0, columnspan=2, sticky=NW)
    self.light_canvas.grid(column=0, row=1, sticky=NW)
    self.status_msg.grid(column=1, row=1, sticky=NW)
    self.frame.grid_columnconfigure(1, minsize=120)

    # Add frame of status light to parent Canvas
    self.canvas.create_window(*FRAME_TOP_LEFT, anchor=NW, window=self.frame)

  def setReachedTarget(self):
    self.light_canvas.itemconfig(self.status_light, fill='green')
    self.status_msg['text'] = 'Reached target'
    self.reachedTarget = True
  
  def setNavigatingToTarget(self):
    self.light_canvas.itemconfig(self.status_light, fill='red')
    self.status_msg['text'] = 'Navigating to target'
    self.reachedTarget = False
  
  def getReachedTarget(self) -> bool:
    return self.reachedTarget
  