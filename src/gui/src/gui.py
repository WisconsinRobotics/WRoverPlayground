#!/usr/bin/env python3

from curses import nonl
import rospy, std_msgs, math, os
import tkinter as tk
from geometry_msgs.msg import Pose2D



def update_gui(msg):
    global ticks, x, y, theta

    theta = msg.theta
    x = msg.x
    y = msg.y
    print(get_status_str())

    center = (x + width/2, y + height/2)
    coord = rotate_rect((25*botScale, 25*botScale), -theta)

    w.delete('all')

    w.create_polygon(
        coord[0] + center[0], coord[1] + center[1],
        -coord[1] + center[0], coord[0] + center[1],
        -coord[0] + center[0], -coord[1] + center[1],
        coord[1] + center[0], -coord[0] + center[1],
        fill="red", outline='black'
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
    
    w.create_text(50,60,text=get_status_str(), anchor=tk.NW, font=30)

def get_status_str():
    return "x: {0}\ny: {1}\n\u03B8: {2}".format(x,y,theta)

def rotate_rect(coord, theta):
    return (
        coord[0]*math.cos(theta) - coord[1]*math.sin(theta),
        coord[0]*math.sin(theta) + coord[1]*math.cos(theta)
        # coord[2]*math.cos(theta) - coord[3]*math.sin(theta),
        # coord[2]*math.sin(theta) + coord[3]*math.cos(theta),
    )

def key_down(e):
    global keys
    print('down: ' + str(e.keycode))

    if e.keycode == 111: #up
        keys |= 8
    elif e.keycode == 116: #down
        keys |= 4
    elif e.keycode == 113: #left
        keys |= 2
    elif e.keycode == 114: #right
        keys |= 1
    
    print(keys)
    key_publisher.publish(keys)
    


def key_up(e):
    global keys
    print('up: ' + str(e.keycode))
    if e.keycode == 111: # up
        keys &= 15-8
    elif e.keycode == 116: #down
        keys &= 15-4
    elif e.keycode == 113: #left
        keys &= 15-2
    elif e.keycode == 114: #right
        keys &= 15-1
    
    print(keys)
    key_publisher.publish(keys)

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
root.protocol("WM_DELETE_WINDOW", lambda: rospy.signal_shutdown("Window closed"))

keys = 0 #0000,up,down,left,right
os.system('xset r off')
root.bind("<KeyPress>", key_down)
root.bind("<KeyRelease>",key_up)

pos_subscriber = rospy.Subscriber('/rover/pose', Pose2D, update_gui)
key_publisher = rospy.Publisher('/gui/keys', std_msgs.msg.Int8, queue_size=10)

root.mainloop()
rospy.spin()