#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class CmdVelPublisher : public rclcpp::Node
{
public:
    CmdVelPublisher() : Node("cmd_vel_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&CmdVelPublisher::publish_velocity, this));
    }

private:
    void publish_velocity()
    {
        auto msg = geometry_msgs::msg::Twist();

        msg.linear.x = 0.2;
        msg.angular.z = 0.0;

        RCLCPP_INFO(this->get_logger(),
                    "Publishing velocity: linear=%.2f angular=%.2f",
                    msg.linear.x,
                    msg.angular.z);

        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelPublisher>());
    rclcpp::shutdown();
    return 0;
}