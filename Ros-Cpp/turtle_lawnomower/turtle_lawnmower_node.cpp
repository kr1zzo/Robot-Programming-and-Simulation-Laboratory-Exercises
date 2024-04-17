#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h" 
#include "turtlesim/Spawn.h" 
#include <iostream>
#include <boost/math/constants/constants.hpp>


class TurtlebotLawnmower
{
    ros::NodeHandle nh_; 
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher pubpose_;
    public:
    TurtlebotLawnmower(); // Class constructor
    ~TurtlebotLawnmower(); // Class destructor
    void turtleCallback(const turtlesim::Pose::ConstPtr& msg);
};

TurtlebotLawnmower::TurtlebotLawnmower()
{
    ros::service::waitForService("spawn");
    ros::ServiceClient add_turtle = nh_.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn srv;
    srv.request.x = 1;
    srv.request.y = 0.5;
    srv.request.theta = 0;
    add_turtle.call(srv);
    sub_ = nh_.subscribe("turtle2/pose", 1,
        &TurtlebotLawnmower::turtleCallback, this);

    pubpose_= nh_.advertise<turtlesim::Pose>("turtle2/pose", 1);
    pub_ = nh_.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1);
    
}


float sgn(float i) {
    return i < 0? -1 : 1;
}

void TurtlebotLawnmower::turtleCallback(const turtlesim::Pose::ConstPtr& msg)
{   
    const double pi = boost::math::constants::pi<double>();

    static bool right = true;
    geometry_msgs::Twist turtle_cmd_vel;
    static float speed = 1;
    static float direction = 0;

    if ((right && msg->x > 10) || (!right && msg->x < 1)) {
            right = !right;
    }

    if (!(msg->y > 10.6f && !right)) {
        turtle_cmd_vel.linear.x = 1;

        float theta = msg->theta;
        if (!right && theta < 0) {
            theta += 2 * pi;
        }

        float goal = right? 0 : pi;

        if (theta != goal) {
            if (abs(goal - theta) < pi / 2) {
                float prev = direction;
                direction = sgn(goal - theta);

                if (prev != direction) {
                    speed /= 2;
                }

            } else {
                direction = right? -1 : 1;
                speed = 1;
            }
        }
            
        turtle_cmd_vel.angular.z = speed * pi * direction;
    }

    pub_.publish(turtle_cmd_vel);
}

TurtlebotLawnmower::~TurtlebotLawnmower(){}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_lawnmower_node");
    TurtlebotLawnmower TtMower;
    system("rosservice call /kill turtle1");
    ros::spin();
    return 0;
}



