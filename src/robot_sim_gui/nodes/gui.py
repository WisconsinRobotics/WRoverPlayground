#!/usr/bin/env python3
from tkinter import *
import rospy
from robot_sim_gui.RobotSimCanvas import RobotSimCanvas
from robot_sim_gui.msg import DrivePower

import math
import random

if __name__=='__main__':

    rospy.init_node(name='robot_sim_gui')

    # Set up canvas
    window = Tk()
    window.geometry(f'{window.winfo_screenwidth()}x{window.winfo_screenheight()}')

    robotSimCanvas = RobotSimCanvas(window, window.winfo_screenwidth(), window.winfo_screenheight(), rospy.get_param('~resource_path'), robot_init_x=100, robot_init_y=300)

    def setRobotPower(msg: DrivePower):
        robotSimCanvas.updateRobotSpeeds([msg.leftPower, msg.rightPower])
    powerSubscriber = rospy.Subscriber('/robot/drive_power', DrivePower, setRobotPower)

    def processTargets():
        if robotSimCanvas.getTargetPos() == [None, None]:
            return
        
        currTargetPos = robotSimCanvas.getTargetPos()
        currRobotPos = robotSimCanvas.getRobotPos()
        distToTarget = math.sqrt((currTargetPos[0] - currRobotPos[0]) ** 2 + (currTargetPos[1] - currRobotPos[1]) ** 2)
        if distToTarget < 5:
            robotSimCanvas.removeTarget()
            robotSimCanvas.addTarget(random.randint(50,robotSimCanvas.canvas.winfo_width()-50), random.randint(50,robotSimCanvas.canvas.winfo_height()-50))
    targetTimer = rospy.Timer(rospy.Duration(0.1), lambda _: processTargets())
    robotSimCanvas.addTarget(random.randint(50,robotSimCanvas.canvas.winfo_width()-50), random.randint(50,robotSimCanvas.canvas.winfo_height()-50))

    # Stop Tk on ROS shutsdown
    def stopTk():
        window.quit()
        window.destroy()
    rospy.on_shutdown(stopTk)

    window.mainloop()