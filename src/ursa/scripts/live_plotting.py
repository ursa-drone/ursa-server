#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import time
import nav_msgs.msg

fig, ax = plt.subplots()
fig.show()

class Formatter(object):
    def __init__(self, im):
        self.im = im
    def __call__(self, x, y):
        z = self.im.get_array()[int(y), int(x)]
        return 'x={:.01f}, y={:.01f}, z={:.01f}'.format(x, y, z)

def plotDataCB(data):
    if len(data.data) < 120*120:
        data = np.random.rand(120,120)
    else:
        data = np.array(data.data).reshape(120,120)
    fig.clear()
    im = plt.imshow(data)
    ax.format_coord = Formatter(im)
    fig.canvas.draw()

if __name__ == '__main__':
    rospy.init_node('live_plot_costmap', anonymous=True)
    print("!!!!!!!!!!!!!!!!!!!!!Initialising live_plot_costmap node...")
    rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!Initialising live_plot_costmap node...")
    costmap = rospy.Subscriber('/move_base/local_costmap/costmap', nav_msgs.msg.OccupancyGrid, plotDataCB, queue_size=10)
    rospy.spin()
