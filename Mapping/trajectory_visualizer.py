#!/usr/bin/env python3
import rospy
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class TrajectoryVisualizerNode:
    def __init__(self):
        #ros private parameters
        self.frame_id = rospy.get_param('~frame_id','map')
        self.child_frame_id = rospy.get_param('~child_frame_id','pioneer/base_link')
        #print values of the parameters
        print("Starting the trajectory visualizer node.")
        print("frame_id: "+self.frame_id+" child_frame_id: "+self.child_frame_id)
        #subscriber and publisher
        self.subscriber= rospy.Subscriber('/tf',TFMessage,self.scan_callback)
        self.publisher= rospy.Publisher('/robot_positions', Marker, queue_size=1)
        #marker for visualization
        self.marker= Marker()
        self.marker.header.stamp = rospy.Time(0)
        #publish marker as LINE_STRIP
        self.marker.type = Marker.LINE_STRIP
        self.marker.color.a = 1
        #navy blue color
        self.marker.color.r, self.marker.color.g, self.marker.color.b = (0, 0, 128)
        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05
        self.marker.pose.orientation.w = 1



    def scan_callback(self, scan):
        if len(scan.transforms)>0 and scan.transforms[0].child_frame_id == self.child_frame_id:
            #algoritm for checking current time stamp and bag restart
            if self.marker.header.stamp > scan.transforms[0].header.stamp:
                print("Timestamp has jumped backwards, clearing the trajectory")
                self.marker.points = []

        if len(scan.transforms)>0 and self.child_frame_id==scan.transforms[0].child_frame_id and self.frame_id==scan.transforms[0].header.frame_id:
            #for matching frames
            # using header from received transform
            self.marker.header = scan.transforms[0].header
            new_point = Point()
            new_point.x = scan.transforms[0].transform.translation.x
            new_point.y = scan.transforms[0].transform.translation.y
            self.marker.points.append(new_point)
            self.publisher.publish(self.marker)

    def run(self):
        rospy.spin()


if __name__=='__main__':
    rospy.init_node('trajectory_visualizer')
    obs= TrajectoryVisualizerNode()
    obs.run()




