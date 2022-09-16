#!/usr/bin/env python3

import rospy, std_msgs, math
import tkinter as tk


def update_gui(msg):
    global ticks

    theta = msg.data[2]
    x = msg.data[0]
    y = msg.data[1]

    center = (x + width/2, y + height/2)
    coord = rotate_rect((25*botScale, 25*botScale), -theta)

    w.delete('all')

    w.create_polygon(
        coord[0] + center[0], coord[1] + center[1],
        -coord[1] + center[0], coord[0] + center[1],
        -coord[0] + center[0], -coord[1] + center[1],
        coord[1] + center[0], -coord[0] + center[1],
        fill="red"
    )

    w.create_line(
        center[0], center[1],
        center[0]+math.sin(theta)*50*botScale,
        center[1]+math.cos(theta)*50*botScale,
        fill="red",
        width=10
    )

    ticks += 1

    if ticks == ticks_per_point:
        trail.append((center[0], center[1]))
        ticks = 0

        if len(trail) > 100:
            trail.pop(0)
    
    for i in range(1,len(trail)):
        w.create_line(trail[i-1][0], trail[i-1][1], trail[i][0], trail[i][1], fill="red",  width=5)

    #ganatt


def rotate_rect(coord, theta):
    return (
        coord[0]*math.cos(theta) - coord[1]*math.sin(theta),
        coord[0]*math.sin(theta) + coord[1]*math.cos(theta)
        # coord[2]*math.cos(theta) - coord[3]*math.sin(theta),
        # coord[2]*math.sin(theta) + coord[3]*math.cos(theta),
    )

x = 0.0
y = 0.0
theta = 0.0
status = 0
botScale = 1

trail = []
max_trail = 100
ticks_per_point = 10
ticks = 0


root = tk.Tk()
width= root.winfo_screenwidth()/2             
height= root.winfo_screenheight()/2               
root.geometry("%dx%d" % (width, height))

w = tk.Canvas (root, bg="white", height=height, width=width)
w.pack()

rospy.init_node('rover')
rospy.on_shutdown(root.destroy)
pos_subscriber = rospy.Subscriber('/rover/pose', std_msgs.msg.Float64MultiArray, update_gui)

root.mainloop()
rospy.spin()