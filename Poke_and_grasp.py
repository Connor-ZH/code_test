#!/usr/bin/env python
# -*- coding: utf-8 -*-

from matplotlib import pyplot as plt
import rospy
from std_msgs.msg import Float64
from matplotlib import animation
from std_msgs.msg import String
from matplotlib.widgets import Button
import matplotlib.image as mpimg

fig, ax = plt.subplots()
plt.title('Title',fontsize=40)
ax.set_xlim([-250,250])
ax.set_ylim([0,100])
line1 =ax.plot([-20,-20],[-0,-10],color='#808080',linewidth=5)
line2 =ax.plot([-20,-20],[-0,-10],color='#808080',linewidth=5)
line3 =ax.plot([-20,-20],[-0,-10],color='#808080',linewidth=5)
line4 =ax.plot([-20,-20],[-0,-10],color='#808080',linewidth=5)
artists = [line1[0],line2[0],line3[0],line4[0]]
ax.axis('off')

banana = mpimg.imread('/home/zhangheng/catkin_ws/src/plot/src/banana.jpg')
left, bottom, width, height = 0.12,0.2,0.2,0.2
ax1 = fig.add_axes([left,bottom,width,height])
ax1.imshow(banana)
ax1.axis('off')

watermelon = mpimg.imread('/home/zhangheng/catkin_ws/src/plot/src/watermelon.jpg')
left, bottom, width, height = 0.32,0.2,0.2,0.2
ax2 = fig.add_axes([left,bottom,width,height])
ax2.imshow(watermelon)
ax2.axis('off')

tofu = mpimg.imread('/home/zhangheng/catkin_ws/src/plot/src/tofu.jpg')
left, bottom, width, height = 0.52,0.2,0.2,0.2
ax3 = fig.add_axes([left,bottom,width,height])
ax3.imshow(tofu)
ax3.axis('off')

water = mpimg.imread('/home/zhangheng/catkin_ws/src/plot/src/water.jpg')
left, bottom, width, height = 0.71,0.2,0.2,0.2
ax4 = fig.add_axes([left,bottom,width,height])
ax4.imshow(water)
ax4.axis('off')

#setting variable
time = 0#indicating the time the point has shown
x = -1#the value of the signal
period = 20000#indicating the period the point will show



axcut = plt.axes([0.27, 0.65, 0.15, 0.15])
axcut1 = plt.axes([0.6, 0.65, 0.15, 0.15])

bcut = Button(axcut, 'Grasp' )
bcut.label.set_fontsize(20)
bcut1 = Button(axcut1, 'Poke')
bcut1.label.set_fontsize(20)
def grasp():
    Grasp.publish(40)

def poke():
    Poke.publish(35)


def _yes(event):
	print("Grasp")
	try:
		grasp()
	except rospy.ROSInterruptException:
		pass

def _yes1(event):
	print("Poke")
	try:
		poke()
	except rospy.ROSInterruptException:
		pass

bcut.on_clicked(_yes)
bcut1.on_clicked(_yes1)


def plotx(msg):
	global x
	if msg.data == "A":
		x = -248
	if msg.data == "B":
		x = -118
	if msg.data == "C":
		x = 12
	if msg.data == "D":
		x = 139
	global time
	time = 0
	return


def init():
    global artists
    return artists

def animate(frame):
	global artists
	global time
	time = time + frame #time becomes greater while update function is called
	height_of_box = 34
	width_of_box = 110
	if time <= period and x != -1:
		artists[0].set_xdata([x, x])
		artists[0].set_ydata([10, 10+height_of_box])
		artists[1].set_xdata([x, x + width_of_box])
		artists[1].set_ydata([10+height_of_box, 10+height_of_box])
		artists[2].set_xdata([x + width_of_box, x + width_of_box])
		artists[2].set_ydata([10+height_of_box, 10])
		artists[3].set_xdata([x, x + width_of_box])
		artists[3].set_ydata([10, 10])
	else:
		artists[0].set_xdata([-100, -100])
		artists[0].set_ydata([-100, -100])
		artists[1].set_xdata([-100, -100])
		artists[1].set_ydata([-100, -100])
		artists[2].set_xdata([-100, -100])
		artists[2].set_ydata([-100, -100])
		artists[3].set_xdata([-100, -100])
		artists[3].set_ydata([-100, -100])
	return artists

if __name__ == '__main__':
	rospy.init_node("plotter")

	rospy.Subscriber("tool_loc", String, plotx)

	anim = animation.FuncAnimation(fig, animate, init_func=init, frames=200, interval=10, blit=True)
	Grasp = rospy.Publisher("grasp", Float64, queue_size=10)
	Poke = rospy.Publisher("poke", Float64, queue_size=10)

	plt.show()

	rospy.spin()
