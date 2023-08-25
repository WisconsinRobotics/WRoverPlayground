#!/usr/bin/env python3
from tkinter import *
import rospy
from robot_sim_gui.RobotSimCanvas import RobotSimCanvas
from robot_sim_gui.msg import DrivePower, IRSensorData

import math
import random
from copy import deepcopy

if __name__=='__main__':

    rospy.init_node(name='robot_sim_gui')

    # Set up canvas
    window = Tk()
    window.geometry(f'{window.winfo_screenwidth()}x{window.winfo_screenheight()}')

    robotSimCanvas = RobotSimCanvas(window, window.winfo_screenwidth(), window.winfo_screenheight(), rospy.get_param('~resource_path'), robot_init_x=300, robot_init_y=500)

    # Set drive power
    def setRobotPower(msg: DrivePower):
        robotSimCanvas.updateRobotSpeeds([msg.leftPower, msg.rightPower])
    powerSubscriber = rospy.Subscriber('/robot/drive_power', DrivePower, setRobotPower, queue_size=1)


    # Process placing/hitting targets
    def placeTargetRandom():
        robotSimCanvas.addTarget(random.randint(150,robotSimCanvas.canvas.winfo_width()-150), random.randint(150,robotSimCanvas.canvas.winfo_height()-150))

    def processTargets():
        currTargetPos = deepcopy(robotSimCanvas.getTargetPos()) # copy-on-read to avoid race condition
        currRobotPos = robotSimCanvas.getRobotPos()

        if currTargetPos == [None, None]:
            return
        
        distToTarget = math.sqrt((currTargetPos[0] - currRobotPos[0]) ** 2 + (currTargetPos[1] - currRobotPos[1]) ** 2)
        if distToTarget < 100:
            robotSimCanvas.removeTarget()
            placeTargetRandom()
    targetTimer = rospy.Timer(rospy.Duration(0.1), lambda _: processTargets())
    placeTargetRandom()

    # Publish sensing messages
    def publishIRMessage(publisher: rospy.Publisher):
        startingMessage = [math.inf] * 180
        currTargetPos = deepcopy(robotSimCanvas.getTargetPos()) # copy-on-read to avoid race condition
        currRobotPos = robotSimCanvas.getRobotPos()

        # If there's no target, publish empty message
        if None in currTargetPos:
            publisher.publish(IRSensorData(distances=startingMessage))
            return

        dY = currRobotPos[0] - currTargetPos[0]
        dX = currRobotPos[1] - currTargetPos[1]
        absAngle = math.degrees(math.atan2(dY, dX))
        relAngle = (((absAngle - (robotSimCanvas.getRobotOrientation()))+ 540) % 360) - 180

        # Get distance
        if abs(relAngle) < 90:
            ind = 90 - int(relAngle)
            startingMessage[ind] = math.sqrt((currTargetPos[0] - currRobotPos[0]) ** 2 + (currTargetPos[1] - currRobotPos[1]) ** 2)
        # Publish final message
        publisher.publish(IRSensorData(distances=startingMessage))
    irPublisher = rospy.Publisher('/robot/ir_sensor', IRSensorData, queue_size=1)
    irTimer = rospy.Timer(rospy.Duration(0.1), lambda _: publishIRMessage(irPublisher))

    # Stop Tk on ROS shutsdown
    def stopTk():
        window.quit()
    rospy.on_shutdown(stopTk)

    window.mainloop()
