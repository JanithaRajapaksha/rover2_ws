#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class MecanumNode : public rclcpp::Node
{
public:
    MecanumNode()
    : Node("mecanum_node")
    {
        // Declare and get parameters
        this->declare_parameter<double>("wheel_radius", 0.05);
        this->declare_parameter<double>("lx", 0.15);
        this->declare_parameter<double>("ly", 0.15);

        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        lx_ = this->get_parameter("lx").as_double();
        ly_ = this->get_parameter("ly").as_double();
        L_ = lx_ + ly_;

        // Subscribe to cmd_vel
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_joy", 10,
            std::bind(&MecanumNode::cmdVelCallback, this, std::placeholders::_1)
        );

        // Timer at 50 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MecanumNode::computeWheels, this)
        );

        RCLCPP_INFO(this->get_logger(), "Mecanum node started (50 Hz loop)");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        wz_ = msg->angular.z;
    }

    void computeWheels()
    {
        double w1 = (1.0 / wheel_radius_) * (vx_ - vy_ - L_ * wz_);
        double w2 = (1.0 / wheel_radius_) * (vx_ + vy_ + L_ * wz_);
        double w3 = (1.0 / wheel_radius_) * (vx_ + vy_ - L_ * wz_);
        double w4 = (1.0 / wheel_radius_) * (vx_ - vy_ + L_ * wz_);

        // Throttled logging to avoid slowing RPi
        RCLCPP_INFO_THROTTLE(   
            this->get_logger(),
            *this->get_clock(),
            100,  // once per second
            "w1=%.3f  w2=%.3f  w3=%.3f  w4=%.3f",
            w1, w2, w3, w4
        );
    }

    // ROS Objects
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Robot parameters
    double wheel_radius_, lx_, ly_, L_;

    // Stored command velocity
    double vx_ = 0.0;
    double vy_ = 0.0;
    double wz_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanumNode>());
    rclcpp::shutdown();
    return 0;
}
