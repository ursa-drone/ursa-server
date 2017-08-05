#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import time
import nav_msgs.msg
from std_msgs.msg import String
from multiprocessing import Process, Array

import ctypes as c
import multiprocessing as mp

arr = Array('d', 120*120)

def CB(data):
    if len(data.data) < 120*120:
        arr[:] = np.random.rand(120*120)
    else:
        arr[:] = np.array(data.data)

class Formatter(object):
    def __init__(self, im):
        self.im = im
    def __call__(self, x, y):
        z = self.im.get_array()[int(y), int(x)]
        return 'x={:.01f}, y={:.01f}, z={:.01f}'.format(x, y, z)

class ClickToDrawPoints(object):
    def __init__(self, ax):
        self.ax = ax
        self.fig = ax.figure
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        plt.show()

    def on_click(self, event):
        if event.inaxes is None:
            return
        im = self.ax.imshow(np.array(arr).reshape(120,120), interpolation='None', origin='lower')
        ax.format_coord = Formatter(im)
        print im.get_array()[0, 0]
        self.ax.draw_artist(self.ax)
        self.fig.canvas.blit(self.ax.bbox)

    def show(self):
        plt.show()


if __name__ == '__main__':
    print("Initialising live_plot_costmap node...")

    rospy.init_node('live_plot_costmap', anonymous=True)

    fig, ax = plt.subplots()
    ax.imshow(np.random.random((120, 120)), interpolation='None')
    p = Process(target=ClickToDrawPoints, args=[ax])
    p.start()

    costmap = rospy.Subscriber('/move_base/local_costmap/costmap', nav_msgs.msg.OccupancyGrid, CB, queue_size=10)
    rospy.spin()