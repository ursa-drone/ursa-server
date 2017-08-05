#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import show, ion
import time
import nav_msgs.msg
from std_msgs.msg import String

class Formatter(object):
    def __init__(self, im):
        self.im = im
    def __call__(self, x, y):
        z = self.im.get_array()[int(y), int(x)]
        return 'x={:.01f}, y={:.01f}, z={:.01f}'.format(x, y, z)

def CB(data):
    plt.close()
    fig,ax = plt.subplots()

    if len(data.data) < 120*120:
        hm = np.random.rand(120,120)
    else:
        hm = np.array(data.data).reshape(120,120)
    im = ax.imshow(hm, interpolation='None')
    ax.format_coord = Formatter(im)
    plt.show(block=False)
    time.sleep(1)

if __name__ == '__main__':
    print("!!!!!!!!!!!!!!!!!!!!!Initialising live_plot_costmap node...")
    rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!Initialising live_plot_costmap node...")

    rospy.init_node('live_plot_costmap', anonymous=True)

    costmap = rospy.Subscriber('/move_base/local_costmap/costmap', nav_msgs.msg.OccupancyGrid, CB, queue_size=10)
    # costmap = rospy.Subscriber('/chatter', String, CB, queue_size=10)
    rospy.spin()

