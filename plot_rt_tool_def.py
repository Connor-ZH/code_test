#!/usr/bin/env python
# -*- coding: utf-8 -*-

from matplotlib import pyplot as plt
import rospy
from std_msgs.msg import Float64
from matplotlib import animation

#setting for plotting
length = 50 #length of the rod_line
fig, ax = plt.subplots(figsize=(100, 100))
x = [50, 50]
y = [0, length]
rod_line = ax.plot(x, y, linewidth=50.0, color='#808080')#seting for the rod_line
point = ax.plot(50, 1, 'yx', markersize=40, lw=80, markeredgewidth=10)#setting for the point
artists = [rod_line[0], point[0]]
plt.axis('off')#hiding the axes

#setting variable
time = 0#indicating the time the point has shown
x = -1#the value of the signal
period = 20000#indicating the period the point will show

#updating the global variable when receiving new message from tool_loc topic
def plotx(msg):
    global x
    x = msg.data
    global time
    time = 0
    return

#init function
def init():
    global artists
    return artists

#update function
def animate(frame):
    global artists
    global time
    time = time + frame #time becomes greater while update function is called
    if time <= period and x != -1:#when time < the given value and x is not equal to initial value, show the point
        artists[1].set_ydata(length - x)#set and show the point
    else:#when time > the given value, hide the point
        artists[1].set_ydata(length - 100000)#hide the point
    return artists

if __name__ == '__main__':
    rospy.init_node("plotter")

    rospy.Subscriber("tool_loc", Float64, plotx)

    anim = animation.FuncAnimation(fig, animate, init_func=init, frames=200, interval=10, blit=True)

    plt.show()

    rospy.spin()

