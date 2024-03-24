#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <chrono>
#include <memory>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

public:
    Custom() : high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
    }

    void highUdpSend()
    {
        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void highUdpRecv()
    {
        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

std::shared_ptr<Custom> custom_ptr;

class Go1ImuNode : public rclcpp::Node
{
public:
    Go1ImuNode() : Node("go1_imu")
    {
        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
        pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        loop_udpSend = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&Custom::highUdpSend, custom_ptr));
        loop_udpRecv = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&Custom::highUdpRecv, custom_ptr));

        loop_rate = std::make_shared<rclcpp::Rate>(100);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr loop_udpSend;
    rclcpp::TimerBase::SharedPtr loop_udpRecv;
    std::shared_ptr<rclcpp::Rate> loop_rate;

    long count = 0;

    void publishData()
    {
        auto current_time = this->now();

        sensor_msgs::msg::Imu msg_imu;

        msg_imu.header.stamp = current_time;
        msg_imu.header.frame_id = "imu_link";

        msg_imu.orientation.w = custom_ptr->high_state.imu.quaternion[0];
        msg_imu.orientation.x = custom_ptr->high_state.imu.quaternion[1];
        msg_imu.orientation.y = custom_ptr->high_state.imu.quaternion[2];
        msg_imu.orientation.z = custom_ptr->high_state.imu.quaternion[3];

        msg_imu.angular_velocity.x = custom_ptr->high_state.imu.gyroscope[0];
        msg_imu.angular_velocity.y = custom_ptr->high_state.imu.gyroscope[1];
        msg_imu.angular_velocity.z = custom_ptr->high_state.imu.gyroscope[2];

        msg_imu.linear_acceleration.x = custom_ptr->high_state.imu.accelerometer[0];
        msg_imu.linear_acceleration.y = custom_ptr->high_state.imu.accelerometer[1];
        msg_imu.linear_acceleration.z = custom_ptr->high_state.imu.accelerometer[2];

        pub_imu->publish(msg_imu);

        nav_msgs::msg::Odometry msg_odom;

        msg_odom.header.stamp = current_time;
        msg_odom.header.frame_id = "odom";
        msg_odom.child_frame_id = "base_link";

        msg_odom.pose.pose.position.x = custom_ptr->high_state.position[0];
        msg_odom.pose.pose.position.y = custom_ptr->high_state.position[1];
        msg_odom.pose.pose.position.z = custom_ptr->high_state.position[2];

        msg_odom.pose.pose.orientation.w = custom_ptr->high_state.imu.quaternion[0];
        msg_odom.pose.pose.orientation.x = custom_ptr->high_state.imu.quaternion[1];
        msg_odom.pose.pose.orientation.y = custom_ptr->high_state.imu.quaternion[2];
        msg_odom.pose.pose.orientation.z = custom_ptr->high_state.imu.quaternion[3];

        msg_odom.twist.twist.linear.x = custom_ptr->high_state.velocity[0];
        msg_odom.twist.twist.linear.y = custom_ptr->high_state.velocity[1];
        msg_odom.twist.twist.angular.z = custom_ptr->high_state.velocity[2];

        pub_odom->publish(msg_odom);

        // Publish tf transform
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = custom_ptr->high_state.position[0];
        odom_trans.transform.translation.y = custom_ptr->high_state.position[1];
        odom_trans.transform.translation.z = custom_ptr->high_state.position[2];
        odom_trans.transform.rotation.w = custom_ptr->high_state.imu.quaternion[0];
        odom_trans.transform.rotation.x = custom_ptr->high_state.imu.quaternion[1];
        odom_trans.transform.rotation.y = custom_ptr->high_state.imu.quaternion[2];
        odom_trans.transform.rotation.z = custom_ptr->high_state.imu.quaternion[3];

        tf_broadcaster->sendTransform(odom_trans);

        count++;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    custom_ptr = std::make_shared<Custom>();
    auto node = std::make_shared<Go1ImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
