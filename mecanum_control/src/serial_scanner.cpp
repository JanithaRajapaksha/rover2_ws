#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <dirent.h>
#include <vector>
#include <string>
#include <iostream>

class SerialCheckerNode : public rclcpp::Node
{
public:
    SerialCheckerNode() : Node("serial_checker_node")
    {
        RCLCPP_INFO(this->get_logger(), "Serial Checker Node Started");

        // Check available serial ports
        auto ports = list_serial_ports();
        for (const auto &port : ports)
        {
            RCLCPP_INFO(this->get_logger(), "Found port: %s", port.c_str());

            if (test_serial_port(port))
            {
                RCLCPP_INFO(this->get_logger(), "Port %s is accessible", port.c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Port %s could NOT be opened", port.c_str());
            }
        }
    }

private:
    // List serial ports under /dev
    std::vector<std::string> list_serial_ports()
    {
        std::vector<std::string> ports;
        DIR *dir = opendir("/dev");
        if (!dir)
            return ports;

        struct dirent *entry;
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string name(entry->d_name);
            if (name.find("ttyUSB") != std::string::npos || name.find("ttyACM") != std::string::npos)
            {
                ports.push_back("/dev/" + name);
            }
        }
        closedir(dir);
        return ports;
    }

    // Try opening the serial port with basic settings
    bool test_serial_port(const std::string &port)
    {
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0)
            return false;

        termios tty;
        if (tcgetattr(fd, &tty) != 0)
        {
            close(fd);
            return false;
        }

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
        tty.c_cc[VTIME] = 5;

        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
            close(fd);
            return false;
        }

        close(fd);
        return true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialCheckerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
