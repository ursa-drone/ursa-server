#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import time
import nav_msgs.msg
from std_msgs.msg import String
from multiprocessing import Process
import multiprocessing

heatmap_data = None;

def CB(data):
    global heatmap_data;
    heatmap_data = data

def worker(ax, q, heatmap_data):
    print "hi"
    obj = ClickToDrawPoints(ax, q, heatmap_data)
    obj.update_heatmap(heatmap_data)

class ClickToDrawPoints(object):
    def __init__(self, ax, q, heatmap_data):
        print "init"
        self.ax = ax
        self.fig = ax.figure
        self.heatmap_data = heatmap_data
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        plt.show()

    def update_heatmap(self, heatmap_data):
        self.heatmap_data = heatmap_data

    def on_click(self, event):
        if event.inaxes is None:
            return
        global heatmap_data
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
    queue = multiprocessing.Queue()
    p = Process(target=worker, args=[ax, queue, heatmap_data])
    p.start()
    heatmap_data = 3
    queue.put(heatmap_data)

    # costmap = rospy.Subscriber('/move_base/local_costmap/costmap', nav_msgs.msg.OccupancyGrid, plotHeatmap.CB(), queue_size=10)
    costmap = rospy.Subscriber('/chatter', String, CB, queue_size=10)
    rospy.spin()
