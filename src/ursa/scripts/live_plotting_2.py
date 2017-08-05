#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import time
import nav_msgs.msg
from std_msgs.msg import String
from multiprocessing import Process


class Formatter(object):
    def __init__(self, im):
        self.im = im
    def __call__(self, x, y):
        z = self.im.get_array()[int(y), int(x)]
        return 'x={:.01f}, y={:.01f}, z={:.01f}'.format(x, y, z)

# def plotDataCB(data):
#     if len(data.data) < 120*120:
#         data = np.random.rand(120,120)
#     else:
#         data = np.array(data.data).reshape(120,120)
#     fig.clear()
#     im = plt.imshow(data)
#     ax.format_coord = Formatter(im)
#     fig.canvas.draw()

def CB(data, args):
    fig = args[0]
    ax = args[1]
    im = ax.imshow(np.random.random((120, 120)), interpolation='None')
    print im.get_array()[0, 0]
    ax.draw_artist(ax)
    fig.canvas.blit(ax.bbox)

def test(data, ax):
    print data

def main():
    print("!!!!!!!!!!!!!!!!!!!!!Initialising live_plot_costmap node...")
    rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!Initialising live_plot_costmap node...")

    rospy.init_node('live_plot_costmap', anonymous=True)
    fig, ax = plt.subplots()
    ax.imshow(np.random.random((120, 120)), interpolation='None')
    fig.show()
    # costmap = rospy.Subscriber('/move_base/local_costmap/costmap', nav_msgs.msg.OccupancyGrid, CB(ax), queue_size=10)
    costmap = rospy.Subscriber('/chatter', String, CB, (fig, ax), queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()

