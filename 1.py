#!/usr/bin/env python 

# PROPER TESTED RUN:
# python3 /home/crslab/clear_lab_stable/new_neutouch/plot.py

# usage:
# 1. set WHITE_BACKGROUND=True to have white background for plot. If False, background becomes blue. 
# The intensities for taxels automatically adjust by this parameter.
# 2. Set SUM_POLARITIES=True if only want to visualize total spike counts (regardless of polarity). If False,
# the plot is 3 channel RGB (R=negative, B=positive) and WHITE_BACKGROUND parameter is not appliable

# settings for plotting
WHITE_BACKGROUND = True # sets white background on plotting (only work for total spike count)
SUM_POLARITIES = True # negative and positive polarities are summed together



import numpy as np
from matplotlib import pyplot as plt
import rospy
from std_msgs.msg import String, Int16MultiArray
from matplotlib import animation
import os, sys

FILE_DIR = os.path.dirname(__file__)
ASSETS_DIR = os.path.join(FILE_DIR, '../../new_neutouch/assets')

left_map = np.load(ASSETS_DIR + '/left_map_rectangular.npy')
right_map = np.load(ASSETS_DIR +  '/right_map_rectangular.npy')

height, width = left_map.shape

# if SUM_POLARITIES:
left_image = np.zeros((height,width))
right_image = np.zeros((height,width))
# else:
    # left_image = np.zeros((height,width,3))
    # right_image = np.zeros((height,width,3))

# adc_values = np.zeros([80]) 
adc_values = np.zeros([81]) # edited by hh
fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(25, 25))
fig.canvas.set_window_title('Tactile Spikes')
fig.patch.set_facecolor('white')

#ax = plt.axes()
cmap = 'PiYG'


im1 = ax[1].imshow(left_image, vmin=-40, vmax=40, cmap=cmap)
im2 = ax[0].imshow(right_image, vmin=-40, vmax=40, cmap=cmap)
plt.colorbar(im2, ax=ax[1])
#plt.colorbar(im, ax=ax)
ax[1].set_xticks([])
ax[1].set_yticks([])
ax[1].set_title('Left', fontsize=30)
ax[0].set_xticks([])
ax[0].set_yticks([])
ax[0].set_title('Right', fontsize=30)

# add text
all_artists = [im1, im2]
# for taxel in range(1, 80):
for taxel in range(1, 81): #hh
    if taxel <= 40:
        x,y = np.where(left_map == taxel)
        x_c = np.mean(x)
        y_c = np.mean(y)
        all_artists.append( ax[1].text(y_c, x_c, '0', fontsize=20, ha='center', va='center') )
    else:
        x,y = np.where(right_map == taxel)
        x_c = np.mean(x)
        y_c = np.mean(y)
        all_artists.append( ax[0].text(y_c, x_c, '0', fontsize=20, ha='center', va='center') )

# initialization function: plot the background of each frame
def init():
    global left_image, right_image, adc_values, all_artists
    all_artists[0].set_array(left_image)
    all_artists[1].set_array(right_image)

    # for taxel in range(80): 
    for taxel in range(1,81): #edited HH
        if int(adc_values[taxel]) == 0:
                        # all_artists[taxel+2].set_text('')
            all_artists[taxel+1].set_text('') #hh
        else:
            # all_artists[taxel+2].set_text(int(adc_values[taxel]))
             all_artists[taxel+1].set_text(int(adc_values[taxel])) #hh

    return all_artists

# animation function.  This is called sequentially
def animate(i):
    global  left_image, right_image, adc_values, all_artists
    all_artists[0].set_array(left_image)
    all_artists[1].set_array(right_image)

    # set text
    # for taxel in range(80): 
    for taxel in range(1,81): #edited by hh    
        if int(adc_values[taxel]) == 0:
            # all_artists[taxel+2].set_text('')
            all_artists[taxel+1].set_text('') #hh
        else:
            # all_artists[taxel+2].set_text(int(adc_values[taxel])
             all_artists[taxel+1].set_text(int(adc_values[taxel])) #hh


    return all_artists

def plot_x(msg):
    global left_image, right_image, adc_values

    left_image = np.zeros((height,width))
    right_image = np.zeros((height,width))
    # adc_values = np.zeros([80,1]) 
    adc_values = np.zeros([81,1]) #edited by hh 


    
    if msg.data == 'no data':    
        return

    # parse the data 1-80 
    arr = msg.data.split(',')
    _pos_loc = np.fromstring(arr[0][1:-1], dtype=int, sep=' ') #hh
    _pos_count = np.fromstring(arr[1][1:-1], dtype=int, sep=' ')

    _neg_loc = np.fromstring(arr[2][1:-1], dtype=int, sep=' ') #hh
    _neg_count = np.fromstring(arr[3][1:-1], dtype=int, sep=' ')

    _loc_adc = np.fromstring(arr[4][1:-1], dtype=int, sep=' ')
    _adc_value = np.fromstring(arr[5][1:-1], dtype=int, sep=' ')

    
    # parse the data 0-79


    # _pos_loc = np.fromstring(arr[0][1:-1], dtype=int, sep=' ')+1
    # _pos_count = np.fromstring(arr[1][1:-1], dtype=int, sep=' ')

    # _neg_loc = np.fromstring(arr[2][1:-1], dtype=int, sep=' ')+1
    # _neg_count = np.fromstring(arr[3][1:-1], dtype=int, sep=' ')

    # _loc_adc = np.fromstring(arr[4][1:-1], dtype=int, sep=' ')
    # _adc_value = np.fromstring(arr[5][1:-1], dtype=int, sep=' ')



    for i, taxel in enumerate(_loc_adc):
        adc_values[taxel] = _adc_value[i]

    print('right', _adc_value[_loc_adc > 40])
    print('right index', _loc_adc[_loc_adc > 40])

    for m in range(len(_pos_loc)):
        left_image[left_map ==_pos_loc[m]] = -_pos_count[m]
        right_image[right_map ==_pos_loc[ m]] = -_pos_count[m]
                                      
    for m in range(len(_neg_loc)):
        left_image[left_map ==_neg_loc[m]] += _neg_count[m]
        right_image[right_map ==_neg_loc[m]] += _neg_count[m]




if __name__ == '__main__':

    rospy.init_node("plotter")
    rospy.Subscriber("sensor_real_time", String, plot_x)

    anim = animation.FuncAnimation(fig, animate, init_func=init, interval=1, blit=True)
    plt.show()
    rospy.spin()