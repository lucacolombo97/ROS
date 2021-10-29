#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include <math.h>  
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"

class pub_sub {
private:
    float latitude_init;
    float longitude_init;
    float h0;
    int flag;
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    tf::Transform transform;
    tf::TransformBroadcaster br;
    nav_msgs::Odometry odom;

public:

    pub_sub(int flag) {
        this->flag = flag;
        if (flag == 0) {
            sub = n.subscribe("/swiftnav/front/gps_pose", 1000, &pub_sub::chatterCallback, this);
            pub = n.advertise<nav_msgs::Odometry>("car_odom", 1000);
        } else if (flag == 1) {
            sub = n.subscribe("/swiftnav/obs/gps_pose", 1000, &pub_sub::chatterCallback, this);
            pub = n.advertise<nav_msgs::Odometry>("obs_odom", 1000);
        }
    }

    void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        ROS_INFO("Input position: [%f,%f, %f]", msg->latitude, msg->longitude, msg->altitude);

        float xEast;
        float yNorth;
        float zUp;
        if (msg->latitude == 0 && msg->longitude == 0 && msg->altitude == 0) {
            xEast = std::nan("");
            yNorth = std::nan("");
            zUp = std::nan("");
            ROS_INFO("GPS signal lost");
        } else {
            // fixed values
            double a = 6378137;
            double b = 6356752.3142;
            double f = (a - b) / a;
            double e_sq = f * (2 - f);
            float deg_to_rad = 0.0174533;

            // input data from msg
            float latitude = msg->latitude;
            float longitude = msg->longitude;
            float h = msg->altitude;

            // fixed position from launch as param
            n.getParam("/latitude_sp", latitude_init);
            n.getParam("/longitude_sp", longitude_init);
            n.getParam("/altitude_sp", h0);

            //lla to ecef
            float lamb = deg_to_rad * (latitude);
            float phi = deg_to_rad * (longitude);
            float s = sin(lamb);
            float N = a / sqrt(1 - e_sq * s * s);

            float sin_lambda = sin(lamb);
            float cos_lambda = cos(lamb);
            float sin_phi = sin(phi);
            float cos_phi = cos(phi);

            float x = (h + N) * cos_lambda * cos_phi;
            float y = (h + N) * cos_lambda * sin_phi;
            float z = (h + (1 - e_sq) * N) * sin_lambda;

            ROS_INFO("ECEF position: [%f,%f, %f]", x, y, z);

            // ecef to enu
            lamb = deg_to_rad * (latitude_init);
            phi = deg_to_rad * (longitude_init);
            s = sin(lamb);
            N = a / sqrt(1 - e_sq * s * s);

            sin_lambda = sin(lamb);
            cos_lambda = cos(lamb);
            sin_phi = sin(phi);
            cos_phi = cos(phi);

            float x0 = (h0 + N) * cos_lambda * cos_phi;
            float y0 = (h0 + N) * cos_lambda * sin_phi;
            float z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

            float xd = x - x0;
            float yd = y - y0;
            float zd = z - z0;

            xEast = -sin_phi * xd + cos_phi * yd;
            yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
            zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;
        }

        ROS_INFO("ENU position: [%f,%f, %f]", xEast, yNorth, zUp);

        //pub tf
        tf::Quaternion tf_q;
        tf_q.setRPY(0, 0, 0);
        transform.setOrigin(tf::Vector3(xEast, yNorth, zUp));
        transform.setRotation(tf_q);
        if (flag == 0)
            br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "world", "car"));
        else if (flag == 1)
            br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "world", "obs"));

        //pub odometry
        geometry_msgs::Quaternion odom_q;
        quaternionTFToMsg(tf_q, odom_q);
        odom.header.stamp = msg->header.stamp;
        odom.header.frame_id = "world";
        odom.pose.pose.position.x = xEast;
        odom.pose.pose.position.y = yNorth;
        odom.pose.pose.position.z = zUp;
        odom.pose.pose.orientation = odom_q;
        pub.publish(odom);
    }

};

int main(int argc, char **argv) {
    
    //argv[1] contains the parameter for car(0) or obs(1)
    if (strcmp(argv[1], "0") == 0) {
        ros::init(argc, argv, "pub_sub_car");
    } else if (strcmp(argv[1], "1") == 0) {
        ros::init(argc, argv, "pub_sub_obs");
    } else {
        return -1;
    }
    if (argc != 2) {
        return -1;
    }
    pub_sub pub_sub(atoi(argv[1]));
    ros::spin();
    return 0;

}


