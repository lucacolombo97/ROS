#include "ros/ros.h"
#include "project1/ComputeDistance.h"
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

float distance = -1; //set to -1 until obs starts and the distance is computed

void callback(const nav_msgs::Odometry::ConstPtr& msg_car, const nav_msgs::Odometry::ConstPtr& msg_obs) {
    distance = sqrt(pow(msg_car->pose.pose.position.x - msg_obs->pose.pose.position.x, 2) + pow(msg_car->pose.pose.position.y - msg_obs->pose.pose.position.y, 2) +
            pow(msg_car->pose.pose.position.z - msg_obs->pose.pose.position.z, 2));
    ROS_INFO("Distance: %f", distance);
}

bool calc_dist(project1::ComputeDistance::Request &req, project1::ComputeDistance::Response &res) {
    res.distance = distance;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "compute_distance_service");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("compute_distance", calc_dist);
    
    message_filters::Subscriber<nav_msgs::Odometry> sub_car(n, "car_odom", 1);
    message_filters::Subscriber<nav_msgs::Odometry> sub_obs(n, "obs_odom", 1);
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_car, sub_obs);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    
    ros::spin();
    return 0;
}
