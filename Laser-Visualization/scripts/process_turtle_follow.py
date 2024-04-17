#!/usr/bin/env python3
import rosbag
import sys
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from geometry_msgs.msg import Point, Twist

if __name__ == "__main__":

    if len(sys.argv) != 2:
        sys.exit()

    #declaration
    inbag_filename = sys.argv[1]
    outbag_filename = "processed_follow.bag"
    time = []
    pose = Pose()
    pixel = Point()
    msg_counter = 0
    distance= 0


    def get_distance(msg_x, msg_y):
        #compute euclidean distance between msg coordinate and last saved coordinate in Pose()
        euclidean_distance = sqrt(pow((msg_x-pose.x), 2)+pow((msg_y-pose.y), 2))
        return euclidean_distance

    with rosbag.Bag(outbag_filename, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(inbag_filename, 'r').read_messages():
            if topic == "/turtle1/pose":
                #sum euclidian distances and update pose
                # write in rosbag output file
                if msg_counter == 0:
                    # for the first coordinate; set first pose values and append time
                    msg_counter += 1
                    time.append(t.to_sec())
                    # update pose
                    pose.x = round(msg.x, 4)
                    pose.y = round(msg.y, 4)
                    outbag.write('/follower/pose', msg, t)
                else:
                    msg_counter += 1
                    time.append(t.to_sec())
                    #sum distances
                    distance += get_distance(msg.x,msg.y)
                    #update pose
                    pose.x = round(msg.x, 4)
                    pose.y = round(msg.y, 4)
                    outbag.write('/follower/pose',msg, t)

            elif topic == "/mouse_position":
                # transform 1920*1080 resolution to 800*600
                # write in rosbag output file
                msg_counter += 1
                pixel.x = round((msg.x/1920)*800)
                pixel.y = round((msg.y/1080)*600)
                outbag.write('/mouse_positions_on_grandparents_computer',pixel,t)

    #calculate duration between first and last coordinate
    duration = time[-1]-time[0]
    #calculate velocity
    average_velocity = distance / duration

    #formate and print output
    output= f"Processing input bagfile: {inbag_filename}\n" \
            f"Follower turtle\n\tCovered distance: {round(distance,2)} m" \
            f"\n\tAverage velocity: {round(average_velocity,2)} m/s\nFollow session duration: {round(duration,2)} s\n" \
            f"Wrote {msg_counter} messages to {outbag_filename}"
    print(output)

