#include "ros/ros.h"
#include "project1/Flag.h"
#include "project1/ComputeDistance.h"
#include <dynamic_reconfigure/server.h>
#include <project1/thresholds_paramConfig.h>
#include <csignal>

float crash;
float unsafe;

static void on_close(int signal) {
    puts(" Closing node...");
    ros::shutdown();
}

void callback(project1::thresholds_paramConfig &config, uint32_t level) {
    if(config.unsafe_param > config.crash_param && config.unsafe_param > 0 && config.crash_param > 0){
        crash = config.crash_param;
        unsafe = config.unsafe_param;
    }else
        ROS_INFO("Wrong thresholds");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "custom_msg");
    ros::NodeHandle n;

    ros::Publisher custom_pub = n.advertise<project1::Flag>("custom_msg", 1000);

    ros::ServiceClient client = n.serviceClient<project1::ComputeDistance>("compute_distance");
    project1::ComputeDistance srv;

    dynamic_reconfigure::Server<project1::thresholds_paramConfig> server;
    dynamic_reconfigure::Server<project1::thresholds_paramConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    while (ros::ok()) {
        if (client.call(srv)) {
            project1::Flag msg;
            msg.dist = srv.response.distance;
            if (std::isnan(msg.dist)) {
                msg.flag = "GPS signal lost";
            } else if (msg.dist <= crash && msg.dist > 0)
                msg.flag = "Crash";
            else if (msg.dist > crash && msg.dist <= unsafe)
                msg.flag = "Unsafe";
            else if (msg.dist > unsafe)
                msg.flag = "Safe";
            if(msg.dist != -1) //if dist is -1 obs is not yet started so the message is not published
                custom_pub.publish(msg);
        } else
            ROS_ERROR("Failed to call service compute_distance");
        signal(SIGINT, on_close);
        ros::spinOnce();
    }
    return 0;
}
