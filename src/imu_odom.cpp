#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <chrono>
#include <memory>
#include "geometry_msgs/msg/twist.hpp"
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
        high_state.imu.quaternion[0]=0;
        high_state.imu.quaternion[1]=0;
        high_state.imu.quaternion[2]=0;
        high_state.imu.quaternion[3]=0;
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

class Go1ImuNode : public rclcpp::Node
{
public:
    Go1ImuNode() : Node("go1_imu")
    {   subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/ug1/cmd_vel", 10, std::bind(&Go1ImuNode::topic_callback, this, std::placeholders::_1));
		RCLCPP_INFO(this->get_logger(), "Initializing Go1ImuNode.");

		// Start the Go1 SDK.
		this->go1_sdk = std::make_shared<Custom>();
		RCLCPP_INFO(this->get_logger(), "Unitree Go1 SDK initialized.");

		// Create publishers and subscribers.
        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
        pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		// Create timers.
        loop_udpSend = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&Custom::highUdpSend, this->go1_sdk));
        loop_udpRecv = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&Custom::highUdpRecv, this->go1_sdk));
        loop_publishData = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Go1ImuNode::publishData, this));

        RCLCPP_INFO(this->get_logger(), "Go1ImuNode initialized.");

    }

private:
    void topic_callback(const geometry_msgs::msg::Twist & msg) const
    {
    this->go1_sdk->high_cmd.velocity[0]=msg.linear.x;
    this->go1_sdk->high_cmd.velocity[1]=msg.linear.y;
    this->go1_sdk->high_cmd.yawSpeed=msg.angular.z;
    this->go1_sdk->high_cmd.mode=2;
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

	std::shared_ptr<Custom> go1_sdk;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr loop_udpSend;
    rclcpp::TimerBase::SharedPtr loop_udpRecv;
    rclcpp::TimerBase::SharedPtr loop_publishData;

    long count = 0;

    void publishData()
    {
        if(this->go1_sdk->high_state.imu.quaternion[0] == NAN)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for valid transform.");
            return;
        }

        auto current_time = this->now();


        // IMU message
        sensor_msgs::msg::Imu msg_imu;

        msg_imu.header.stamp = current_time;
        msg_imu.header.frame_id = "imu_link";

        msg_imu.orientation.w = this->go1_sdk->high_state.imu.quaternion[0];
        msg_imu.orientation.x = this->go1_sdk->high_state.imu.quaternion[1];
        msg_imu.orientation.y = this->go1_sdk->high_state.imu.quaternion[2];
        msg_imu.orientation.z = this->go1_sdk->high_state.imu.quaternion[3];

        msg_imu.angular_velocity.x = this->go1_sdk->high_state.imu.gyroscope[0];
        msg_imu.angular_velocity.y = this->go1_sdk->high_state.imu.gyroscope[1];
        msg_imu.angular_velocity.z = this->go1_sdk->high_state.imu.gyroscope[2];

        msg_imu.linear_acceleration.x = this->go1_sdk->high_state.imu.accelerometer[0];
        msg_imu.linear_acceleration.y = this->go1_sdk->high_state.imu.accelerometer[1];
        msg_imu.linear_acceleration.z = this->go1_sdk->high_state.imu.accelerometer[2];

        pub_imu->publish(msg_imu);


        // Odometry message
        nav_msgs::msg::Odometry msg_odom;

        msg_odom.header.stamp = current_time;
        msg_odom.header.frame_id = "odom";
        msg_odom.child_frame_id = "base_link";

        msg_odom.pose.pose.position.x = this->go1_sdk->high_state.position[0];
        msg_odom.pose.pose.position.y = this->go1_sdk->high_state.position[1];
        msg_odom.pose.pose.position.z = this->go1_sdk->high_state.position[2];

        msg_odom.pose.pose.orientation.w = this->go1_sdk->high_state.imu.quaternion[0];
        msg_odom.pose.pose.orientation.x = this->go1_sdk->high_state.imu.quaternion[1];
        msg_odom.pose.pose.orientation.y = this->go1_sdk->high_state.imu.quaternion[2];
        msg_odom.pose.pose.orientation.z = this->go1_sdk->high_state.imu.quaternion[3];

        msg_odom.twist.twist.linear.x = this->go1_sdk->high_state.velocity[0];
        msg_odom.twist.twist.linear.y = this->go1_sdk->high_state.velocity[1];
        msg_odom.twist.twist.angular.z = this->go1_sdk->high_state.velocity[2];

        pub_odom->publish(msg_odom);


        // Odom -> base_link transform
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = this->go1_sdk->high_state.position[0];
        odom_trans.transform.translation.y = this->go1_sdk->high_state.position[1];
        odom_trans.transform.translation.z = this->go1_sdk->high_state.position[2];
        odom_trans.transform.rotation.w = this->go1_sdk->high_state.imu.quaternion[0];
        odom_trans.transform.rotation.x = this->go1_sdk->high_state.imu.quaternion[1];
        odom_trans.transform.rotation.y = this->go1_sdk->high_state.imu.quaternion[2];
        odom_trans.transform.rotation.z = this->go1_sdk->high_state.imu.quaternion[3];

        tf_broadcaster->sendTransform(odom_trans);

        count++;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go1ImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
