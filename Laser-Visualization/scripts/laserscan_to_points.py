#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Point
import math
# In this example, we are using `visualization_msgs.Marker`, which has
# an array of `geometry_msgs.Point` member named `points`.
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan

class LaserScanToPointsNode:

    def __init__(self):
        #node name initialization
        rospy.init_node('laserscan_to_points')
        #publisher and subscriber initialization
        self.subscriber = rospy.Subscriber("scan", LaserScan , self.callback)
        self.pub = rospy.Publisher('point_positions', Marker, queue_size=1)
        #marker initialization
        self.marker = Marker()
        # These values will affect how the Marker message is visualized in RViz.
        self.marker.type = Marker.POINTS
        # Alpha value: 0 = invisible (fully transparent), 1 = visible (opaque)
        self.marker.color.a = 1.
        # Red, green and blue
        self.marker.color.r, self.marker.color.g, self.marker.color.b = (1., 0., 0.)
        # Width and height
        self.marker.scale.x = .5
        self.marker.scale.y = .5

    def callback(self,msg):
        #set marker header
        self.marker.header= msg.header
        #calculate carthesian coordinates from msg data
        for i in range(len(msg.ranges)):
            p=Point()
            angle = msg.angle_min+i * msg.angle_increment
            p.x=msg.ranges[i]*math.cos(angle)
            p.y=msg.ranges[i]*math.sin(angle)
            self.marker.points.append(p)
        #publish new marker
        self.pub.publish(self.marker)
        self.marker.points.clear()

    def run(self):
            rospy.spin()

if __name__ == '__main__':
        scan_to_points= LaserScanToPointsNode()
        scan_to_points.run()




