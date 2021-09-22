#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "marco_demo");
    ros::NodeHandle marco_nh;
    ros::Publisher joint1_pub = marco_nh.advertise<std_msgs::Float64>("/marco/joint1_position_controller/command", 1000);
    ros::Publisher joint2_pub = marco_nh.advertise<std_msgs::Float64>("/marco/joint2_position_controller/command", 1000);
    ros::Publisher joint3_pub = marco_nh.advertise<std_msgs::Float64>("/marco/joint3_position_controller/command", 1000);
    ros::Publisher joint4_pub = marco_nh.advertise<std_msgs::Float64>("/marco/joint4_position_controller/command", 1000);
    ros::Publisher joint5_pub = marco_nh.advertise<std_msgs::Float64>("/marco/joint5_position_controller/command", 1000);
    ros::Publisher joint6_pub = marco_nh.advertise<std_msgs::Float64>("/marco/joint6_position_controller/command", 1000);
    ros::Rate rate(100);
    int count = 0;
    while (ros::ok()) {
        std_msgs::Float64 msg1, msg2, msg3, msg4, msg5, msg6;
        msg1.data = ((double)(count % 1000) / 1000.0) * (2.0 * 3.14159265);
        msg2.data = (45.0 / 360.0) * (2.0 * 3.14159265);
        msg3.data = (-90.0 / 360.0) * (2.0 * 3.14159265);
        msg4.data = (0.0 / 360.0) * (2.0 * 3.14159265);
        msg5.data = (-45.0 / 360.0) * (2.0 * 3.14159265);
        msg6.data = ((double)count / 360.0) * (2.0 * 3.14159265);
        joint1_pub.publish(msg1);
        joint2_pub.publish(msg2);
        joint3_pub.publish(msg3);
        joint4_pub.publish(msg4);
        joint5_pub.publish(msg5);
        joint6_pub.publish(msg6);
        ros::spinOnce();
        rate.sleep();
        count++;
    }
    return 0;
}