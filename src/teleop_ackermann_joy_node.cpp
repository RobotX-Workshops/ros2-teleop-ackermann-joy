#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <memory>

namespace teleop_ackermann_joy
{
    // Parameter name constants
    static const std::string PARAM_REQUIRE_ENABLE_BUTTON = "require_enable_button";
    static const std::string PARAM_ENABLE_BUTTON = "enable_button";
    static const std::string PARAM_AXIS_DEAD_ZONE = "axis_dead_zone";
    static const std::string PARAM_USE_TRIGGER_MODE = "use_trigger_mode";
    static const std::string PARAM_AXIS_SPEED = "axis_speed";
    static const std::string PARAM_AXIS_FORWARD_TRIGGER = "axis_forward_trigger";
    static const std::string PARAM_AXIS_REVERSE_TRIGGER = "axis_reverse_trigger";
    static const std::string PARAM_SCALE_SPEED = "scale_speed";
    static const std::string PARAM_AXIS_STEERING = "axis_steering";
    static const std::string PARAM_SCALE_STEERING = "scale_steering";
    static const std::string PARAM_INVERT_STEERING = "invert_steering";

    class TeleopAckermannJoy : public rclcpp::Node
    {
    public:
        // Constructor
        TeleopAckermannJoy() : Node("teleop_ackermann_joy_node", rclcpp::NodeOptions().enable_logger_service(true))
        {
            // Declare parameters with default values
            this->declare_parameter<bool>(PARAM_REQUIRE_ENABLE_BUTTON, false);
            this->declare_parameter<int>(PARAM_ENABLE_BUTTON, 4);
            this->declare_parameter<double>(PARAM_AXIS_DEAD_ZONE, 0.1);

            // Speed control parameters
            this->declare_parameter<bool>(PARAM_USE_TRIGGER_MODE, true);
            this->declare_parameter<int>(PARAM_AXIS_SPEED, 1);
            this->declare_parameter<int>(PARAM_AXIS_FORWARD_TRIGGER, 5);
            this->declare_parameter<int>(PARAM_AXIS_REVERSE_TRIGGER, 4);
            this->declare_parameter<double>(PARAM_SCALE_SPEED, 1.0);

            // Steering parameters
            this->declare_parameter<int>(PARAM_AXIS_STEERING, 0);
            this->declare_parameter<double>(PARAM_SCALE_STEERING, 1.0);
            this->declare_parameter<bool>(PARAM_INVERT_STEERING, false);

            // Update configuration
            update_config();

            // Initialize activation state (default to inactive for safety)
            is_active_ = false;

            // Create publisher and subscriber (using fixed topic names for remapping)
            publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd", 10);
            subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "/joy", 10, std::bind(&TeleopAckermannJoy::joy_callback, this, std::placeholders::_1));
            activation_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
                "/activate", 10, std::bind(&TeleopAckermannJoy::activation_callback, this, std::placeholders::_1));

            // Add parameter callback
            param_callback_handle_ = this->add_on_set_parameters_callback(
                [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult
                {
                    rcl_interfaces::msg::SetParametersResult result;
                    result.successful = true;

                    // Schedule the update to happen after a short delay
                    this->parameter_update_timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(100),
                        [this]()
                        {
                            RCLCPP_INFO(this->get_logger(), "Updating parameters after delay");
                            this->update_config();
                            // Cancel the timer to make it one-shot
                            this->parameter_update_timer_->cancel();
                        });

                    return result;
                });

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

            if (use_trigger_mode_)
            {
                RCLCPP_INFO(this->get_logger(), "Speed Control: DUAL TRIGGERS - Forward: %d, Reverse: %d, Scale: %.2f",
                            axis_forward_trigger_, axis_reverse_trigger_, scale_speed_);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Speed Control: SINGLE AXIS - Axis: %d, Scale: %.2f",
                            axis_speed_, scale_speed_);
            }

            RCLCPP_INFO(this->get_logger(), "Steering Axis: %d, Scale: %.2f, Inverted: %s",
                        axis_steering_, scale_steering_, invert_steering_ ? "true" : "false");
            RCLCPP_INFO(this->get_logger(), "--------------------");
        }

        void activation_callback(const std_msgs::msg::Bool::SharedPtr msg)
        {
            is_active_ = msg->data;
            RCLCPP_INFO(this->get_logger(), "Teleop controller %s", is_active_ ? "ACTIVATED" : "DEACTIVATED");

            // If deactivated, publish a zero command for safety
            if (!is_active_)
            {
                auto ackermann_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
                ackermann_msg->header.stamp = this->get_clock()->now();
                ackermann_msg->header.frame_id = "base_link";
                ackermann_msg->drive.speed = 0.0;
                ackermann_msg->drive.steering_angle = 0.0;
                publisher_->publish(std::move(ackermann_msg));
                RCLCPP_INFO(this->get_logger(), "Published zero command due to deactivation");
            }
        }

    private:
        void update_config()
        {
            RCLCPP_DEBUG(this->get_logger(), "Updating configuration parameters");

            // Get all parameters using constants
            require_enable_button_ = this->get_parameter(PARAM_REQUIRE_ENABLE_BUTTON).as_bool();
            enable_button_ = this->get_parameter(PARAM_ENABLE_BUTTON).as_int();
            axis_dead_zone_ = this->get_parameter(PARAM_AXIS_DEAD_ZONE).as_double();

            // Speed control parameters
            use_trigger_mode_ = this->get_parameter(PARAM_USE_TRIGGER_MODE).as_bool();
            axis_speed_ = this->get_parameter(PARAM_AXIS_SPEED).as_int();
            axis_forward_trigger_ = this->get_parameter(PARAM_AXIS_FORWARD_TRIGGER).as_int();
            axis_reverse_trigger_ = this->get_parameter(PARAM_AXIS_REVERSE_TRIGGER).as_int();
            scale_speed_ = this->get_parameter(PARAM_SCALE_SPEED).as_double();

            // Steering parameters
            axis_steering_ = this->get_parameter(PARAM_AXIS_STEERING).as_int();
            scale_steering_ = this->get_parameter(PARAM_SCALE_STEERING).as_double();
            invert_steering_ = this->get_parameter(PARAM_INVERT_STEERING).as_bool();
        }

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

            // Check if we should process commands (must be active AND either no enable button required, or enable button is pressed)
            bool should_process_commands = is_active_;

            if (is_active_ && require_enable_button_)
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
            else if (is_active_)
            {
                // No enable button required but node is active - always process commands
                RCLCPP_DEBUG(this->get_logger(), "Enable button disabled - processing commands (node active)");
            }
            else
            {
                // Node is not active - don't process commands
                RCLCPP_DEBUG(this->get_logger(), "Node is not active - not processing commands");
            }

            if (should_process_commands)
            {
                auto ackermann_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
                ackermann_msg->header.stamp = this->get_clock()->now();
                ackermann_msg->header.frame_id = "base_link";

                // Speed calculation - dual trigger mode or single axis mode
                if (use_trigger_mode_)
                {
                    // Dual trigger mode (like your Python node)
                    double forward_throttle = 0.0;
                    double reverse_throttle = 0.0;

                    if (msg->axes.size() > axis_forward_trigger_)
                    {
                        forward_throttle = msg->axes[axis_forward_trigger_];
                        RCLCPP_DEBUG(this->get_logger(), "Forward trigger (axis %d): %.3f",
                                     axis_forward_trigger_, forward_throttle);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Forward trigger axis %d not available (only %zu axes)",
                                    axis_forward_trigger_, msg->axes.size());
                    }

                    if (msg->axes.size() > axis_reverse_trigger_)
                    {
                        reverse_throttle = msg->axes[axis_reverse_trigger_];
                        RCLCPP_DEBUG(this->get_logger(), "Reverse trigger (axis %d): %.3f",
                                     axis_reverse_trigger_, reverse_throttle);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Reverse trigger axis %d not available (only %zu axes)",
                                    axis_reverse_trigger_, msg->axes.size());
                    }

                    // Calculate combined throttle (forward - reverse, then divide by 2 to normalize)
                    double combined_throttle = (forward_throttle - reverse_throttle) / 2.0;
                    RCLCPP_DEBUG(this->get_logger(), "Combined throttle: %.3f (forward: %.3f - reverse: %.3f) / 2",
                                 combined_throttle, forward_throttle, reverse_throttle);

                    if (std::abs(combined_throttle) > axis_dead_zone_)
                    {
                        ackermann_msg->drive.speed = combined_throttle * scale_speed_;
                        RCLCPP_DEBUG(this->get_logger(), "Speed calculated: %.3f (combined: %.3f * scale: %.3f)",
                                     ackermann_msg->drive.speed, combined_throttle, scale_speed_);
                    }
                    else
                    {
                        ackermann_msg->drive.speed = 0.0;
                        RCLCPP_DEBUG(this->get_logger(), "Speed set to 0.0 (within dead zone)");
                    }
                }
                else
                {
                    // Single axis mode (original behavior)
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
                }

                // Steering axis
                if (msg->axes.size() > axis_steering_)
                {
                    double raw_steering = msg->axes[axis_steering_];
                    RCLCPP_DEBUG(this->get_logger(), "Steering axis %d raw value: %.3f (dead zone: %.3f)",
                                 axis_steering_, raw_steering, axis_dead_zone_);

                    if (std::abs(raw_steering) > axis_dead_zone_)
                    {
                        // Apply steering inversion if configured
                        double processed_steering = invert_steering_ ? -raw_steering : raw_steering;
                        ackermann_msg->drive.steering_angle = processed_steering * scale_steering_;
                        RCLCPP_DEBUG(this->get_logger(), "Steering calculated: %.3f (raw: %.3f, inverted: %s, * scale: %.3f)",
                                     ackermann_msg->drive.steering_angle, raw_steering,
                                     invert_steering_ ? "true" : "false", scale_steering_);
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
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr activation_subscription_;

        // Parameter callback handle
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        // Timer for delayed parameter updates
        rclcpp::TimerBase::SharedPtr parameter_update_timer_;

        // Activation state
        bool is_active_;

        bool require_enable_button_;
        int enable_button_;
        double axis_dead_zone_;

        // Speed control variables
        bool use_trigger_mode_;
        int axis_speed_;
        int axis_forward_trigger_;
        int axis_reverse_trigger_;
        double scale_speed_;

        // Steering control variables
        int axis_steering_;
        double scale_steering_;
        bool invert_steering_;
    };

} // namespace teleop_ackermann_joy

// Main function
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<teleop_ackermann_joy::TeleopAckermannJoy>());
    rclcpp::shutdown();
    return 0;
}