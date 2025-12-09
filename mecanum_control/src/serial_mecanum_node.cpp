#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <iostream>

class MecanumNode : public rclcpp::Node
{
public:
    MecanumNode()
    : Node("mecanum_node_serial")
    {
        // Declare parameters
        this->declare_parameter<double>("wheel_radius", 0.045);
        this->declare_parameter<double>("lx", 0.2);
        this->declare_parameter<double>("ly", 0.115);
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);

        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        lx_ = this->get_parameter("lx").as_double();
        ly_ = this->get_parameter("ly").as_double();
        L_ = lx_ + ly_;

        serial_port_name_ = this->get_parameter("serial_port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();

        if (!setupSerial()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_name_.c_str());
        }

        // Subscribe to cmd_vel
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MecanumNode::cmdVelCallback, this, std::placeholders::_1)
        );

        // Timer at 50 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MecanumNode::computeWheels, this)
        );

        RCLCPP_INFO(this->get_logger(), "Mecanum node with serial started (50 Hz loop)");
    }

    ~MecanumNode() {
        if (serial_fd_ >= 0) close(serial_fd_);
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
    // Compute wheel angular velocity in rad/s
    double w1 = (1.0 / wheel_radius_) * (vx_ - vy_ - L_ * wz_);
    double w2 = (1.0 / wheel_radius_) * (vx_ + vy_ + L_ * wz_);
    double w3 = (1.0 / wheel_radius_) * (vx_ + vy_ - L_ * wz_);
    double w4 = (1.0 / wheel_radius_) * (vx_ - vy_ + L_ * wz_);

    // Convert rad/s to RPM
    double rpm1 = w1 * 9.549296585;
    double rpm2 = w2 * 9.549296585;
    double rpm3 = w3 * 9.549296585;
    double rpm4 = w4 * 9.549296585;

    // Throttled logging in RPM
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000, // once per second
        "RPM1=%.2f  RPM2=%.2f  RPM3=%.2f  RPM4=%.2f",
        rpm1, rpm2, rpm3, rpm4
    );

    // Send RPM (integer) to ESP32
    if (serial_fd_ >= 0) {
        std::ostringstream ss;
        ss << static_cast<int>(rpm1) << " "
           << static_cast<int>(rpm2) << " "
           << static_cast<int>(rpm3) << " "
           << static_cast<int>(rpm4) << "\n";

        std::string out_str = ss.str();
        write(serial_fd_, out_str.c_str(), out_str.size());
    }
}


    bool setupSerial()
    {
        serial_fd_ = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) return false;

        termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) return false;

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
        tty.c_lflag = 0;
        tty.c_iflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 1;

        return tcsetattr(serial_fd_, TCSANOW, &tty) == 0;
    }

    // ROS objects
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Robot parameters
    double wheel_radius_, lx_, ly_, L_;

    // Stored command velocity
    double vx_ = 0.0;
    double vy_ = 0.0;
    double wz_ = 0.0;

    // Serial
    std::string serial_port_name_;
    int baudrate_;
    int serial_fd_ = -1;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanumNode>());
    rclcpp::shutdown();
    return 0;
}
