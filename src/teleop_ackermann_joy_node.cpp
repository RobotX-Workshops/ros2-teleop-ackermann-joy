#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cmath>
#include <memory>

class TeleopAckermannJoy : public rclcpp::Node
{
public:
    // Constructor
    TeleopAckermannJoy() : Node("teleop_ackermann_joy_node", rclcpp::NodeOptions().enable_logger_service(true))
    {
        // Declare parameters with default values
        this->declare_parameter<bool>("require_enable_button", false); // Default to false (no enable button required)
        this->declare_parameter<int>("enable_button", 4);
        this->declare_parameter<double>("axis_dead_zone", 0.1);
        this->declare_parameter<int>("axis_speed", 1);
        this->declare_parameter<double>("scale_speed", 1.0);
        this->declare_parameter<int>("axis_steering", 0);
        this->declare_parameter<double>("scale_steering", 1.0);

        // Get parameters
        require_enable_button_ = this->get_parameter("require_enable_button").as_bool();
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
        RCLCPP_INFO(this->get_logger(), "Require Enable Button: %s", require_enable_button_ ? "true" : "false");
        if (require_enable_button_)
        {
            RCLCPP_INFO(this->get_logger(), "Enable Button: %d", enable_button_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Enable Button: DISABLED (always enabled)");
        }
        RCLCPP_INFO(this->get_logger(), "Dead Zone: %.2f", axis_dead_zone_);
        RCLCPP_INFO(this->get_logger(), "Speed Axis: %d, Scale: %.2f", axis_speed_, scale_speed_);
        RCLCPP_INFO(this->get_logger(), "Steering Axis: %d, Scale: %.2f", axis_steering_, scale_steering_);
        RCLCPP_INFO(this->get_logger(), "--------------------");
    }

private:
    // Callback function for joy messages
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Debug: Log incoming joy message details
        RCLCPP_DEBUG(this->get_logger(), "Joy callback triggered");
        RCLCPP_DEBUG(this->get_logger(), "Joy message - Axes count: %zu, Buttons count: %zu",
                     msg->axes.size(), msg->buttons.size());

        // Debug: Log all button states
        for (size_t i = 0; i < msg->buttons.size(); ++i)
        {
            if (msg->buttons[i] == 1)
            {
                RCLCPP_DEBUG(this->get_logger(), "Button %zu is pressed", i);
            }
        }

        // Debug: Log all axis values
        for (size_t i = 0; i < msg->axes.size(); ++i)
        {
            if (std::abs(msg->axes[i]) > 0.01)
            { // Only log non-zero axes
                RCLCPP_DEBUG(this->get_logger(), "Axis %zu value: %.3f", i, msg->axes[i]);
            }
        }

        // Check if we should process commands (either no enable button required, or enable button is pressed)
        bool should_process_commands = true;

        if (require_enable_button_)
        {
            // Enable button is required - check if it's pressed
            if (msg->buttons.size() > enable_button_)
            {
                should_process_commands = (msg->buttons[enable_button_] == 1);
                RCLCPP_DEBUG(this->get_logger(), "Enable button %d state: %s",
                             enable_button_, should_process_commands ? "PRESSED" : "NOT PRESSED");
            }
            else
            {
                should_process_commands = false;
                RCLCPP_WARN(this->get_logger(), "Enable button %d not available (only %zu buttons)",
                            enable_button_, msg->buttons.size());
            }
        }
        else
        {
            // No enable button required - always process commands
            RCLCPP_DEBUG(this->get_logger(), "Enable button disabled - always processing commands");
        }

        if (should_process_commands)
        {
            auto ackermann_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
            ackermann_msg->header.stamp = this->get_clock()->now();
            ackermann_msg->header.frame_id = "base_link";

            // Speed axis
            if (msg->axes.size() > axis_speed_)
            {
                double raw_speed = msg->axes[axis_speed_];
                RCLCPP_DEBUG(this->get_logger(), "Speed axis %d raw value: %.3f (dead zone: %.3f)",
                             axis_speed_, raw_speed, axis_dead_zone_);

                if (std::abs(raw_speed) > axis_dead_zone_)
                {
                    ackermann_msg->drive.speed = raw_speed * scale_speed_;
                    RCLCPP_DEBUG(this->get_logger(), "Speed calculated: %.3f (raw: %.3f * scale: %.3f)",
                                 ackermann_msg->drive.speed, raw_speed, scale_speed_);
                }
                else
                {
                    ackermann_msg->drive.speed = 0.0;
                    RCLCPP_DEBUG(this->get_logger(), "Speed set to 0.0 (within dead zone)");
                }
            }
            else
            {
                ackermann_msg->drive.speed = 0.0;
                RCLCPP_WARN(this->get_logger(), "Speed axis %d not available (only %zu axes)",
                            axis_speed_, msg->axes.size());
            }

            // Steering axis
            if (msg->axes.size() > axis_steering_)
            {
                double raw_steering = msg->axes[axis_steering_];
                RCLCPP_DEBUG(this->get_logger(), "Steering axis %d raw value: %.3f (dead zone: %.3f)",
                             axis_steering_, raw_steering, axis_dead_zone_);

                if (std::abs(raw_steering) > axis_dead_zone_)
                {
                    ackermann_msg->drive.steering_angle = raw_steering * scale_steering_;
                    RCLCPP_DEBUG(this->get_logger(), "Steering calculated: %.3f (raw: %.3f * scale: %.3f)",
                                 ackermann_msg->drive.steering_angle, raw_steering, scale_steering_);
                }
                else
                {
                    ackermann_msg->drive.steering_angle = 0.0;
                    RCLCPP_DEBUG(this->get_logger(), "Steering set to 0.0 (within dead zone)");
                }
            }
            else
            {
                ackermann_msg->drive.steering_angle = 0.0;
                RCLCPP_WARN(this->get_logger(), "Steering axis %d not available (only %zu axes)",
                            axis_steering_, msg->axes.size());
            }

            RCLCPP_DEBUG(this->get_logger(), "Publishing Ackermann command - Speed: %.3f, Steering: %.3f",
                         ackermann_msg->drive.speed, ackermann_msg->drive.steering_angle);
            publisher_->publish(std::move(ackermann_msg));
        }
        // If commands should not be processed (enable button required but not pressed)
        else
        {
            if (require_enable_button_)
            {
                RCLCPP_DEBUG(this->get_logger(), "Enable button not pressed - publishing zero command");
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "No joystick input - publishing zero command");
            }
            auto ackermann_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
            ackermann_msg->header.stamp = this->get_clock()->now();
            ackermann_msg->header.frame_id = "base_link";
            ackermann_msg->drive.speed = 0.0;
            ackermann_msg->drive.steering_angle = 0.0;
            RCLCPP_DEBUG(this->get_logger(), "Publishing zero Ackermann command");
            publisher_->publish(std::move(ackermann_msg));
        }
    }

    // Member variables
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

    bool require_enable_button_;
    int enable_button_;
    double axis_dead_zone_;
    int axis_speed_;
    double scale_speed_;
    int axis_steering_;
    double scale_steering_;
};

// Main function
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopAckermannJoy>());
    rclcpp::shutdown();
    return 0;
}