#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cmath>
#include <memory>

class TeleopAckermannJoy : public rclcpp::Node
{
public:
    // Constructor
    TeleopAckermannJoy() : Node("teleop_ackermann_joy_node")
    {
        // Declare parameters with default values
        this->declare_parameter<int>("enable_button");
        this->declare_parameter<double>("axis_dead_zone");
        this->declare_parameter<int>("axis_speed");
        this->declare_parameter<double>("scale_speed");
        this->declare_parameter<int>("axis_steering");
        this->declare_parameter<double>("scale_steering");

        // Get parameters
        enable_button_ = this->get_parameter("enable_button").as_int();
        axis_dead_zone_ = this->get_parameter("axis_dead_zone").as_double();
        axis_speed_ = this->get_parameter("axis_speed").as_int();
        scale_speed_ = this->get_parameter("scale_speed").as_double();
        axis_steering_ = this->get_parameter("axis_steering").as_int();
        scale_steering_ = this->get_parameter("scale_steering").as_double();

        // Create publisher and subscriber
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&TeleopAckermannJoy::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Teleop Ackermann Joy node started.");
        RCLCPP_INFO(this->get_logger(), "--- Parameters ---");
        RCLCPP_INFO(this->get_logger(), "Enable Button: %d", enable_button_);
        RCLCPP_INFO(this->get_logger(), "Dead Zone: %.2f", axis_dead_zone_);
        RCLCPP_INFO(this->get_logger(), "Speed Axis: %d, Scale: %.2f", axis_speed_, scale_speed_);
        RCLCPP_INFO(this->get_logger(), "Steering Axis: %d, Scale: %.2f", axis_steering_, scale_steering_);
        RCLCPP_INFO(this->get_logger(), "--------------------");
    }

private:
    // Callback function for joy messages
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Check if the enable button is pressed
        if (msg->buttons.size() > enable_button_ && msg->buttons[enable_button_] == 1)
        {
            auto ackermann_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
            ackermann_msg->header.stamp = this->get_clock()->now();
            ackermann_msg->header.frame_id = "base_link";

            // Speed axis
            if (msg->axes.size() > axis_speed_ && std::abs(msg->axes[axis_speed_]) > axis_dead_zone_)
            {
                ackermann_msg->drive.speed = msg->axes[axis_speed_] * scale_speed_;
            }
            else
            {
                ackermann_msg->drive.speed = 0.0;
            }

            // Steering axis
            if (msg->axes.size() > axis_steering_ && std::abs(msg->axes[axis_steering_]) > axis_dead_zone_)
            {
                ackermann_msg->drive.steering_angle = msg->axes[axis_steering_] * scale_steering_;
            }
            else
            {
                ackermann_msg->drive.steering_angle = 0.0;
            }

            publisher_->publish(std::move(ackermann_msg));
        }
        // If enable button is not pressed, you might want to publish a zero-command message
        else
        {
            auto ackermann_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
            ackermann_msg->header.stamp = this->get_clock()->now();
            ackermann_msg->header.frame_id = "base_link";
            ackermann_msg->drive.speed = 0.0;
            ackermann_msg->drive.steering_angle = 0.0;
            publisher_->publish(std::move(ackermann_msg));
        }
    }

    // Member variables
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

    int enable_button_;
    double axis_dead_zone_;
    int axis_speed_;
    double scale_speed_;
    int axis_steering_;
    double scale_steering_;
};

// Main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopAckermannJoy>());
    rclcpp::shutdown();
    return 0;
}