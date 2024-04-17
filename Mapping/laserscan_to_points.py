#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Point
import math
# In this example, we are using `visualization_msgs.Marker`, which has
# an array of `geometry_msgs.Point` member named `points`.
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
import tf2_ros
import math
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
import numpy as np

class LaserScanToPointsNode:

    def __init__(self):
        #node name initialization
        rospy.init_node('laserscan_to_points')
        #set private parameters
        self.global_frame_id= rospy.get_param('~global_frame', 'map')
        self.accumulate_points=rospy.get_param('~accumulate_points', False)
        self.accumulate_every_n=rospy.get_param('~accumulate_every_n', 50)
        #print parameters values
        print("Starting the laser scan visualizer node.")
        print("Global frame: "+self.global_frame_id+" accumulate every n: "+str(self.accumulate_every_n))
        #publisher,subscriber and listener initialization
        self.subscriber = rospy.Subscriber("/pioneer/scan", LaserScan , self.callback)
        self.publisher = rospy.Publisher('point_positions', Marker, queue_size=1)
        #buffer instance
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        #marker initialization
        self.marker = Marker()
        self.marker.type = Marker.POINTS
        self.marker.color.a = 1.
        # Red color
        self.marker.color.r, self.marker.color.g, self.marker.color.b = (1., 0., 0.)
        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05
        self.index=0
        #publisher for map
        self.pub_map = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        self.grid_msg = OccupancyGrid()
        self.grid_msg.header.stamp = rospy.Time()
        self.grid_msg.header.frame_id = self.global_frame_id
        self.grid_msg.info.resolution = 0.05
        self.grid_msg.info.width = 1200  # in pixels
        self.grid_msg.info.height = 1200  # in pixels
        self.grid_msg.info.origin = Pose(Point(-10, -50, 0), Quaternion(0, 0, 0, 1))
        self.grid = np.ones((self.grid_msg.info.height, self.grid_msg.info.width), dtype=np.int32) *-1

    def callback(self,msg):
        #for the bag loop
        if msg.header.stamp < self.marker.header.stamp:
            print('Timestamp has jumped backwards, clearing the buffer.')
            self.marker.header.stamp = msg.header.stamp
            self.marker.points.clear()
            # Clear the occupancy grid from subtask k) here as well!
            self.buffer.clear()
            self.grid = np.ones((self.grid_msg.info.height, self.grid_msg.info.width), dtype=np.int32) * -1
            return

        if not self.accumulate_points:
            if self.index<self.accumulate_every_n:
                self.index = self.index+1
                return
            elif self.accumulate_points==50:
                self.index = 0

        #set marker and grid header
        self.marker.header.stamp = msg.header.stamp
        self.marker.header.frame_id = self.global_frame_id
        self.grid_msg.header.stamp = msg.header.stamp
        #frame transform
        try:
            transform = self.buffer.lookup_transform(self.global_frame_id, msg.header.frame_id, msg.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            #print(e)
            return
        #recover orientation
        theta = 2*math.atan2(transform.transform.rotation.z, transform.transform.rotation.w)
        #calculate coordinates from msg data
        for i in range(len(msg.ranges)):
            #filter for the invalid range readings
            if msg.ranges[i]==0:
                continue
            p=Point()
            angle = msg.angle_min+i * msg.angle_increment+theta
            p.x=msg.ranges[i]*math.cos(angle)+transform.transform.translation.x
            p.y=msg.ranges[i]*math.sin(angle)+transform.transform.translation.y
            self.marker.points.append(p)
            self.grid[int((p.y-10)/0.05),int((p.x-50)/0.05)] = 100

        self.publisher.publish(self.marker)
        flat_grid = self.grid.reshape((self.grid.size,))
        self.grid_msg.data = list(flat_grid)
        self.pub_map.publish(self.grid_msg)



    def run(self):
            rospy.spin()

if __name__ == '__main__':
        scan_to_points= LaserScanToPointsNode()
        scan_to_points.run()




