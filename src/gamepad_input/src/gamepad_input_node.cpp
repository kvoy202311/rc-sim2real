#include <rclcpp/rclcpp.hpp>
#include <kvoy_msgs/msg/gamepad_cmd.hpp>
#include <SDL2/SDL.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

namespace kvoy {

// GameSir 7 Pro mapping verified by user on Linux joystick interface.
static constexpr int AXIS_LX = 0;      // left stick left/right, left negative
static constexpr int AXIS_LY = 1;      // left stick up/down, forward negative
static constexpr int AXIS_RX = 2;      // right stick left/right, left negative
static constexpr int AXIS_RY = 3;      // right stick up/down, forward negative
static constexpr int AXIS_RT = 4;      // right trigger, released negative / pressed positive
static constexpr int AXIS_LT = 5;      // left trigger, released negative / pressed positive
static constexpr int AXIS_DPAD_X = 6;  // d-pad left/right, left negative / right positive
static constexpr int AXIS_DPAD_Y = 7;  // d-pad up/down, up negative / down positive

static constexpr int BTN_A  = 0;
static constexpr int BTN_B  = 1;
static constexpr int BTN_X  = 3;
static constexpr int BTN_Y  = 4;
static constexpr int BTN_LB = 6;
static constexpr int BTN_RB = 7;
static constexpr int HAT_DPAD = 0;

static constexpr float VX_MAX = 1.1f;
static constexpr float VY_MAX = 1.0f;
static constexpr float YAW_MAX = 2.0f;
static constexpr float STICK_DEADZONE = 0.1f;
static constexpr float DIGITAL_AXIS_THRESHOLD = 0.5f;

class GamepadInputNode : public rclcpp::Node
{
public:
    explicit GamepadInputNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("gamepad_input_node", options)
    {
        declare_parameter("publish_rate_hz", 50.0);
        const double rate = get_parameter("publish_rate_hz").as_double();

        cmd_pub_ = create_publisher<kvoy_msgs::msg::GamepadCmd>("/gamepad_cmd", 10);

        if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
            throw std::runtime_error(std::string("SDL_Init failed: ") + SDL_GetError());
        }

        if (SDL_NumJoysticks() == 0) {
            RCLCPP_WARN(get_logger(), "No joystick detected — publishing zero commands");
        } else {
            joystick_ = SDL_JoystickOpen(0);
            if (!joystick_) {
                throw std::runtime_error(std::string("SDL_JoystickOpen failed: ") + SDL_GetError());
            }
            RCLCPP_INFO(
                get_logger(),
                "Gamepad: %s  axes=%d  buttons=%d  hats=%d",
                SDL_JoystickName(joystick_),
                SDL_JoystickNumAxes(joystick_),
                SDL_JoystickNumButtons(joystick_),
                SDL_JoystickNumHats(joystick_));
        }

        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate));
        timer_ = create_wall_timer(period, std::bind(&GamepadInputNode::publish, this));
    }

    ~GamepadInputNode() override
    {
        if (joystick_) {
            SDL_JoystickClose(joystick_);
        }
        SDL_Quit();
    }

private:
    using SteadyClock = std::chrono::steady_clock;

    struct ChordPulseFilter {
        bool latched{false};
        bool has_low_since{false};
        SteadyClock::time_point low_since{};
    };

    static float normalize_axis_value(Sint16 raw)
    {
        if (raw >= 0) {
            return std::min(static_cast<float>(raw) / 32767.0f, 1.0f);
        }
        return std::max(static_cast<float>(raw) / 32768.0f, -1.0f);
    }

    static float axis_value(SDL_Joystick* js, int idx)
    {
        if (!js) {
            return 0.0f;
        }
        return normalize_axis_value(SDL_JoystickGetAxis(js, idx));
    }

    static bool button_value(SDL_Joystick* js, int idx)
    {
        return js && SDL_JoystickGetButton(js, idx) != 0;
    }

    static Uint8 hat_value(SDL_Joystick* js, int idx)
    {
        if (!js || SDL_JoystickNumHats(js) <= idx) {
            return SDL_HAT_CENTERED;
        }
        return SDL_JoystickGetHat(js, idx);
    }

    static float apply_deadzone(float value)
    {
        const float magnitude = std::abs(value);
        if (magnitude < STICK_DEADZONE) {
            return 0.0f;
        }

        const float sign = value >= 0.0f ? 1.0f : -1.0f;
        return sign * (magnitude - STICK_DEADZONE) / (1.0f - STICK_DEADZONE);
    }

    static bool axis_negative_pressed(float value)
    {
        return value <= -DIGITAL_AXIS_THRESHOLD;
    }

    static bool axis_positive_pressed(float value)
    {
        return value >= DIGITAL_AXIS_THRESHOLD;
    }

    static bool chord_press_pulse(bool raw, ChordPulseFilter& filter)
    {
        const auto release_debounce = std::chrono::milliseconds(200);
        const auto now = SteadyClock::now();

        if (raw) {
            filter.has_low_since = false;
            if (!filter.latched) {
                filter.latched = true;
                return true;
            }
            return false;
        }

        if (!filter.has_low_since) {
            filter.has_low_since = true;
            filter.low_since = now;
        } else if (filter.latched &&
                   (now - filter.low_since) >= release_debounce) {
            filter.latched = false;
        }

        return false;
    }

    void publish()
    {
        SDL_JoystickUpdate();

        const float axis_lx = axis_value(joystick_, AXIS_LX);
        const float axis_ly = axis_value(joystick_, AXIS_LY);
        const float axis_rx = axis_value(joystick_, AXIS_RX);
        const float axis_ry = axis_value(joystick_, AXIS_RY);
        const float axis_rt = axis_value(joystick_, AXIS_RT);
        const float axis_lt = axis_value(joystick_, AXIS_LT);
        const float axis_dpad_x = axis_value(joystick_, AXIS_DPAD_X);
        const float axis_dpad_y = axis_value(joystick_, AXIS_DPAD_Y);
        const Uint8 hat_dpad = hat_value(joystick_, HAT_DPAD);

        auto msg = kvoy_msgs::msg::GamepadCmd();
        msg.header.stamp = now();

        msg.axis_lx = axis_lx;
        msg.axis_ly = axis_ly;
        msg.axis_rx = axis_rx;
        msg.axis_ry = axis_ry;
        msg.axis_lt = axis_lt;
        msg.axis_rt = axis_rt;
        msg.axis_dpad_x = axis_dpad_x;
        msg.axis_dpad_y = axis_dpad_y;

        msg.btn_a = button_value(joystick_, BTN_A);
        msg.btn_b = button_value(joystick_, BTN_B);
        msg.btn_x = button_value(joystick_, BTN_X);
        msg.btn_y = button_value(joystick_, BTN_Y);
        msg.btn_lb = button_value(joystick_, BTN_LB);
        msg.btn_rb = button_value(joystick_, BTN_RB);
        msg.btn_lt = axis_positive_pressed(axis_lt);
        msg.btn_rt = axis_positive_pressed(axis_rt);
        msg.btn_dpad_left = axis_negative_pressed(axis_dpad_x) || (hat_dpad & SDL_HAT_LEFT);
        msg.btn_dpad_right = axis_positive_pressed(axis_dpad_x) || (hat_dpad & SDL_HAT_RIGHT);
        msg.btn_dpad_up = axis_negative_pressed(axis_dpad_y) || (hat_dpad & SDL_HAT_UP);
        msg.btn_dpad_down = axis_positive_pressed(axis_dpad_y) || (hat_dpad & SDL_HAT_DOWN);

        // High-level safety-critical actions use chorded inputs instead of single buttons.
        msg.vx = -apply_deadzone(axis_ly) * VX_MAX;
        msg.vy = -apply_deadzone(axis_lx) * VY_MAX;
        msg.yaw_rate = -apply_deadzone(axis_rx) * YAW_MAX;

        msg.btn_standup = chord_press_pulse(msg.btn_lt && msg.btn_lb && msg.btn_y, standup_filter_);
        msg.btn_liedown = chord_press_pulse(msg.btn_rt && msg.btn_rb && msg.btn_x, liedown_filter_);
        msg.btn_estop = chord_press_pulse(msg.btn_lb && msg.btn_rb, estop_filter_);

        // Move policy switching away from the action chords.
        msg.btn_policy_prev = chord_press_pulse(msg.btn_dpad_left, policy_prev_filter_);
        msg.btn_policy_next = chord_press_pulse(msg.btn_dpad_right, policy_next_filter_);

        cmd_pub_->publish(msg);
    }

    rclcpp::Publisher<kvoy_msgs::msg::GamepadCmd>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    SDL_Joystick* joystick_{nullptr};
    ChordPulseFilter standup_filter_;
    ChordPulseFilter liedown_filter_;
    ChordPulseFilter estop_filter_;
    ChordPulseFilter policy_prev_filter_;
    ChordPulseFilter policy_next_filter_;
};

} // namespace kvoy

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kvoy::GamepadInputNode>());
    rclcpp::shutdown();
    return 0;
}
