#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import time
import nav_msgs.msg
from std_msgs.msg import String
from multiprocessing import Process

heatmap_data = None;

def CB(data):
    global heatmap_data;
    heatmap_data = data;
    print heatmap_data

class ClickToDrawPoints(object):
    def __init__(self, ax):
        self.ax = ax
        self.fig = ax.figure
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        plt.show()

    def on_click(self, event):
        if event.inaxes is None:
            return
        print heatmap_data
        im = self.ax.imshow(np.random.random((120, 120)), interpolation='None')
        print im.get_array()[0, 0]
        self.ax.draw_artist(self.ax)
        self.fig.canvas.blit(self.ax.bbox)

    def show(self):
        plt.show()


if __name__ == '__main__':
    print("!!!!!!!!!!!!!!!!!!!!!Initialising live_plot_costmap node...")
    rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!Initialising live_plot_costmap node...")

    rospy.init_node('live_plot_costmap', anonymous=True)

    fig, ax = plt.subplots()
    ax.imshow(np.random.random((120, 120)), interpolation='None')
    p = Process(target=ClickToDrawPoints, args=[ax])
    p.start()

    # costmap = rospy.Subscriber('/move_base/local_costmap/costmap', nav_msgs.msg.OccupancyGrid, plotHeatmap.CB(), queue_size=10)
    costmap = rospy.Subscriber('/chatter', String, CB, queue_size=10)
    rospy.spin()
