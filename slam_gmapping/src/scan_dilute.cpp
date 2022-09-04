#include <sensor_msgs/msg/laser_scan__struct.hpp>
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

class ScanDiluteNode : public rclcpp::Node {
public:
    ScanDiluteNode(std::string name) : Node(name) {
        sub_novel = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10, std::bind(&ScanDiluteNode::scanCb, this, _1));
        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(
                "/scan_dilute", rclcpp::QoS(rclcpp::KeepLast(10)));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_novel;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    int multiple = 3;

    void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan_msg->header.stamp = this->now();
        scan_msg->header.frame_id = "laser";
//        scan_msg->header.frame_id = scan->header.frame_id;
        scan_msg->angle_max = scan->angle_max;
        scan_msg->angle_min = scan->angle_min;
        scan_msg->range_max = scan->range_max;
        scan_msg->range_min = scan->range_min;
        scan_msg->scan_time = scan->scan_time;
        scan_msg->time_increment = scan->time_increment;
        scan_msg->angle_increment = scan->angle_increment * (double) multiple;
        for (unsigned int i = 1; i < scan->ranges.size(); ++i) {
            if (i % multiple == 0) {
                scan_msg->ranges.push_back(scan->ranges.at(i));
                scan_msg->intensities.push_back(scan->intensities.at(i));
            }
        }
        scan_pub->publish(*scan_msg);
    };
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanDiluteNode>("scan_dilute_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}