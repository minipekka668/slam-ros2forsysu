/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"

using namespace std;

class ScanDiluteNode : public rclcpp::Node {
public:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    int multiple = 3;
    ScanDiluteNode() : Node("scan_dilute_node") {
        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(
                "scan_dilute", rclcpp::QoS(rclcpp::KeepLast(10)));
        auto lidar_info_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10,
                std::bind(&ScanDiluteNode::scanCb, this, std::placeholders::_1));
//        scan_filter_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>
//        (node_, "/scan", rclcpp::SensorDataQoS().get_rmw_qos_profile());
//        scan_filter_->registerCallback(std::bind(&ScanDiluteNode::scanCb, this, std::placeholders::_1));
    }

private:
    void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
        cout<<"publish:::"<<endl;
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan_msg->header.stamp = this->now();
        scan_msg->header.frame_id = scan->header.frame_id;
        scan_msg->angle_max = scan->angle_max;
        scan_msg->angle_min = scan->angle_min;
        scan_msg->range_max = scan->range_max;
        scan_msg->range_min = scan->range_min;
        scan_msg->scan_time = scan->scan_time;
        scan_msg->time_increment = scan->time_increment;
        scan_msg->intensities = scan->intensities;
        scan_msg->angle_increment = scan->angle_increment * (double) multiple;
        for (unsigned int i = 0; i < scan->ranges.size(); ++i) {
            if (i % multiple==0) scan_msg->ranges = scan->ranges;
        }
        cout<<"publish:::"<<endl;
        scan_pub->publish(*scan_msg);
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto sllidar_node = std::make_shared<ScanDiluteNode>();
    rclcpp::spin(sllidar_node);
    return 0;
}
