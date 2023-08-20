from tkinter import *

from robot_sim_gui.RobotSimCanvas import RobotSimCanvas

# Set up canvas
window = Tk()
window.geometry(f'{window.winfo_screenwidth()}x{window.winfo_screenheight()}')

robotSimCanvas = RobotSimCanvas(window, window.winfo_screenwidth(), window.winfo_screenheight(), robot_init_x=100, robot_init_y=300)

# Test RobotSimCanvas
window.after(1, lambda: robotSimCanvas.updateRobotSpeeds( (2, 1) ))
window.after(200, lambda: print(f'Robot position:\t{robotSimCanvas.getRobotPos()}'))
window.after(300, lambda: print(f'Robot position:\t{robotSimCanvas.getRobotPos()}'))
window.after(400, lambda: print(f'Robot position:\t{robotSimCanvas.getRobotPos()}'))

window.after(1000, lambda: robotSimCanvas.addTarget())
window.after(1001, lambda: print(f'Target position:\t{robotSimCanvas.getTargetPos()}'))

window.after(2000, lambda: robotSimCanvas.removeTarget())
window.after(2001, lambda: print(f'Target position:\t{robotSimCanvas.getTargetPos()}'))

window.after(3000, lambda: robotSimCanvas.addTarget(x_pos=100, y_pos=100))
window.after(3001, lambda: print(f'Target position:\t{robotSimCanvas.getTargetPos()}'))

window.after(6000, lambda: robotSimCanvas.updateRobotSpeeds( (1.5, 2) ))
window.after(8000, lambda: robotSimCanvas.updateRobotSpeeds( (-1, -1) ))

window.mainloop()