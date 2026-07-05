#include <rclcpp/rclcpp.hpp>
#include <kvoy_msgs/msg/gamepad_cmd.hpp>
#include <SDL2/SDL.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <fstream>
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

static constexpr float DEFAULT_VX_MAX = 1.0f;
static constexpr float DEFAULT_VY_MAX = 1.0f;
static constexpr float DEFAULT_YAW_MAX = 2.0f;
static constexpr float DEFAULT_STICK_DEADZONE = 0.1f;
static constexpr float DEFAULT_DIGITAL_AXIS_THRESHOLD = 0.5f;
static constexpr float DEFAULT_POLICY_SWITCH_LONG_PRESS_S = 1.0f;

class GamepadInputNode : public rclcpp::Node
{
public:
    explicit GamepadInputNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("gamepad_input_node", options)
    {
        declare_parameter("publish_rate_hz", 50.0);
        declare_parameter("vx_max_mps", static_cast<double>(DEFAULT_VX_MAX));
        declare_parameter("vy_max_mps", static_cast<double>(DEFAULT_VY_MAX));
        declare_parameter("yaw_rate_max_rad_s", static_cast<double>(DEFAULT_YAW_MAX));
        declare_parameter("stick_deadzone", static_cast<double>(DEFAULT_STICK_DEADZONE));
        declare_parameter(
            "digital_axis_threshold",
            static_cast<double>(DEFAULT_DIGITAL_AXIS_THRESHOLD));
        declare_parameter(
            "policy_switch_long_press_s",
            static_cast<double>(DEFAULT_POLICY_SWITCH_LONG_PRESS_S));
        declare_parameter("joystick_reconnect_interval_s", 1.0);
        declare_parameter("joystick_index", -1);
        declare_parameter("preferred_joystick_name_substring", "GameSir");
        const double rate = get_parameter("publish_rate_hz").as_double();
        vx_max_mps_ = read_nonnegative_float_param("vx_max_mps", DEFAULT_VX_MAX);
        vy_max_mps_ = read_nonnegative_float_param("vy_max_mps", DEFAULT_VY_MAX);
        yaw_rate_max_rad_s_ =
            read_nonnegative_float_param("yaw_rate_max_rad_s", DEFAULT_YAW_MAX);
        stick_deadzone_ = clamp_float_param(
            "stick_deadzone", DEFAULT_STICK_DEADZONE, 0.0f, 0.95f);
        digital_axis_threshold_ = clamp_float_param(
            "digital_axis_threshold", DEFAULT_DIGITAL_AXIS_THRESHOLD, 0.0f, 1.0f);
        policy_switch_long_press_s_ = read_nonnegative_float_param(
            "policy_switch_long_press_s", DEFAULT_POLICY_SWITCH_LONG_PRESS_S);
        joystick_reconnect_interval_s_ = read_nonnegative_float_param(
            "joystick_reconnect_interval_s", 1.0f);
        joystick_index_ = static_cast<int>(get_parameter("joystick_index").as_int());
        preferred_joystick_name_substring_ =
            get_parameter("preferred_joystick_name_substring").as_string();

        cmd_pub_ = create_publisher<kvoy_msgs::msg::GamepadCmd>("/gamepad_cmd", 10);

        if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
            throw std::runtime_error(std::string("SDL_Init failed: ") + SDL_GetError());
        }
        SDL_JoystickEventState(SDL_ENABLE);

        try_open_joystick();
        RCLCPP_INFO(
            get_logger(),
            "Gamepad command limits: vx=%.2f m/s vy=%.2f m/s yaw=%.2f rad/s deadzone=%.2f policy_switch_hold=%.2fs reconnect=%.2fs",
            vx_max_mps_,
            vy_max_mps_,
            yaw_rate_max_rad_s_,
            stick_deadzone_,
            policy_switch_long_press_s_,
            joystick_reconnect_interval_s_);

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

    struct LongPressPulseFilter {
        bool active{false};
        bool fired{false};
        bool has_low_since{false};
        SteadyClock::time_point pressed_since{};
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
        if (!js || SDL_JoystickNumAxes(js) <= idx) {
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

    float read_nonnegative_float_param(const std::string& name, float fallback)
    {
        const double value = get_parameter(name).as_double();
        if (!std::isfinite(value)) {
            throw std::runtime_error(name + " must be finite");
        }
        if (value < 0.0) {
            RCLCPP_WARN(
                get_logger(),
                "%s %.3f is negative, using fallback %.3f",
                name.c_str(), value, fallback);
            return fallback;
        }
        return static_cast<float>(value);
    }

    float clamp_float_param(const std::string& name, float fallback, float min_value, float max_value)
    {
        const double value = get_parameter(name).as_double();
        if (!std::isfinite(value)) {
            throw std::runtime_error(name + " must be finite");
        }
        const float value_f = static_cast<float>(value);
        const float clamped = std::max(min_value, std::min(max_value, value_f));
        if (clamped != value_f) {
            RCLCPP_WARN(
                get_logger(),
                "%s %.3f is out of range [%.3f, %.3f], clamped to %.3f",
                name.c_str(), value_f, min_value, max_value, clamped);
        }
        (void)fallback;
        return clamped;
    }

    static float apply_deadzone(float value, float deadzone)
    {
        const float magnitude = std::abs(value);
        if (magnitude < deadzone) {
            return 0.0f;
        }

        const float sign = value >= 0.0f ? 1.0f : -1.0f;
        return sign * (magnitude - deadzone) / (1.0f - deadzone);
    }

    static bool axis_negative_pressed(float value, float threshold)
    {
        return value <= -threshold;
    }

    static bool axis_positive_pressed(float value, float threshold)
    {
        return value >= threshold;
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

    static bool long_press_pulse(
        bool raw,
        LongPressPulseFilter& filter,
        std::chrono::duration<float> hold_duration)
    {
        const auto release_debounce = std::chrono::milliseconds(200);
        const auto now = SteadyClock::now();

        if (raw) {
            filter.has_low_since = false;
            if (!filter.active) {
                filter.active = true;
                filter.fired = false;
                filter.pressed_since = now;
            }
            if (!filter.fired && (now - filter.pressed_since) >= hold_duration) {
                filter.fired = true;
                return true;
            }
            return false;
        }

        if (!filter.has_low_since) {
            filter.has_low_since = true;
            filter.low_since = now;
        } else if (filter.active &&
                   (now - filter.low_since) >= release_debounce) {
            filter.active = false;
            filter.fired = false;
        }

        return false;
    }

    static bool contains_case_insensitive(const std::string& text, const std::string& needle)
    {
        if (needle.empty()) {
            return true;
        }
        return std::search(
            text.begin(), text.end(), needle.begin(), needle.end(),
            [](char lhs, char rhs) {
                return std::tolower(static_cast<unsigned char>(lhs)) ==
                       std::tolower(static_cast<unsigned char>(rhs));
            }) != text.end();
    }

    static std::string detected_gamesir_usb_mode()
    {
        std::ifstream input_devices("/proc/bus/input/devices");
        std::string line;
        while (std::getline(input_devices, line)) {
            if (line.find("Vendor=3537") == std::string::npos) {
                continue;
            }
            if (line.find("Product=1022") != std::string::npos) {
                return "3537:1022";
            }
            if (line.find("Product=0575") != std::string::npos) {
                return "3537:0575";
            }
        }
        return "";
    }

    void close_joystick()
    {
        if (joystick_) {
            SDL_JoystickClose(joystick_);
            joystick_ = nullptr;
        }
    }

    void reinitialize_joystick_subsystem()
    {
        close_joystick();
        SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
        if (SDL_InitSubSystem(SDL_INIT_JOYSTICK) < 0) {
            RCLCPP_WARN(
                get_logger(),
                "SDL joystick subsystem reinit failed: %s",
                SDL_GetError());
            return;
        }
        SDL_JoystickEventState(SDL_ENABLE);
    }

    int select_joystick_index(int joystick_count)
    {
        if (joystick_index_ >= 0) {
            if (joystick_index_ < joystick_count) {
                return joystick_index_;
            }
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Configured joystick_index=%d is out of range for %d joystick(s)",
                joystick_index_, joystick_count);
        }

        for (int i = 0; i < joystick_count; ++i) {
            const char* name_c = SDL_JoystickNameForIndex(i);
            const std::string name = name_c ? name_c : "";
            if (contains_case_insensitive(name, preferred_joystick_name_substring_)) {
                return i;
            }
        }

        return 0;
    }

    void log_joystick_candidates(int joystick_count)
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "SDL sees %d joystick candidate(s); preferred name contains '%s', configured index=%d",
            joystick_count,
            preferred_joystick_name_substring_.c_str(),
            joystick_index_);
        for (int i = 0; i < joystick_count; ++i) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "  joystick[%d]: %s",
                i,
                SDL_JoystickNameForIndex(i) ? SDL_JoystickNameForIndex(i) : "<unknown>");
        }
    }

    void try_open_joystick()
    {
        if (joystick_) {
            return;
        }

        const auto now = SteadyClock::now();
        if (has_last_open_attempt_ &&
            (now - last_open_attempt_) < std::chrono::duration<float>(joystick_reconnect_interval_s_)) {
            return;
        }
        has_last_open_attempt_ = true;
        last_open_attempt_ = now;

        SDL_PumpEvents();
        SDL_JoystickUpdate();
        const int joystick_count = SDL_NumJoysticks();
        if (joystick_count <= 0) {
            const std::string gamesir_mode = detected_gamesir_usb_mode();
            if (gamesir_mode == "3537:0575") {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 3000,
                    "GameSir detected as USB ID 3537:0575, which exposes no Linux joystick (/dev/input/js0). "
                    "Switch the controller/receiver to 2.4G PC/XInput mode until it enumerates as 3537:1022; "
                    "publishing zero commands and retrying");
            } else if (!gamesir_mode.empty()) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 3000,
                    "GameSir detected as USB ID %s but SDL sees no joystick - publishing zero commands and retrying",
                    gamesir_mode.c_str());
            } else {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 3000,
                    "No joystick detected - publishing zero commands and retrying");
            }
            reinitialize_joystick_subsystem();
            return;
        }

        log_joystick_candidates(joystick_count);
        const int selected_index = select_joystick_index(joystick_count);
        joystick_ = SDL_JoystickOpen(selected_index);
        if (!joystick_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 3000,
                "SDL_JoystickOpen(%d) failed: %s - publishing zero commands and retrying",
                selected_index, SDL_GetError());
            reinitialize_joystick_subsystem();
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Gamepad opened joystick[%d]: %s  axes=%d  buttons=%d  hats=%d",
            selected_index,
            SDL_JoystickName(joystick_),
            SDL_JoystickNumAxes(joystick_),
            SDL_JoystickNumButtons(joystick_),
            SDL_JoystickNumHats(joystick_));
    }

    void publish()
    {
        if (!joystick_ || !SDL_JoystickGetAttached(joystick_)) {
            if (joystick_) {
                RCLCPP_WARN(get_logger(), "Joystick disconnected - publishing zero commands and retrying");
                close_joystick();
            }
            try_open_joystick();
        }

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
        msg.btn_lt = axis_positive_pressed(axis_lt, digital_axis_threshold_);
        msg.btn_rt = axis_positive_pressed(axis_rt, digital_axis_threshold_);
        msg.btn_dpad_left =
            axis_negative_pressed(axis_dpad_x, digital_axis_threshold_) || (hat_dpad & SDL_HAT_LEFT);
        msg.btn_dpad_right =
            axis_positive_pressed(axis_dpad_x, digital_axis_threshold_) || (hat_dpad & SDL_HAT_RIGHT);
        msg.btn_dpad_up =
            axis_negative_pressed(axis_dpad_y, digital_axis_threshold_) || (hat_dpad & SDL_HAT_UP);
        msg.btn_dpad_down =
            axis_positive_pressed(axis_dpad_y, digital_axis_threshold_) || (hat_dpad & SDL_HAT_DOWN);

        // High-level safety-critical actions use chorded inputs instead of single buttons.
        msg.vx = -apply_deadzone(axis_ly, stick_deadzone_) * vx_max_mps_;
        msg.vy = -apply_deadzone(axis_lx, stick_deadzone_) * vy_max_mps_;
        msg.yaw_rate = -apply_deadzone(axis_rx, stick_deadzone_) * yaw_rate_max_rad_s_;

        msg.btn_standup = chord_press_pulse(msg.btn_lt && msg.btn_lb && msg.btn_y, standup_filter_);
        msg.btn_liedown = chord_press_pulse(msg.btn_rt && msg.btn_rb && msg.btn_x, liedown_filter_);
        msg.btn_estop = chord_press_pulse(msg.btn_lb && msg.btn_rb, estop_filter_);

        // Policy switching uses a long press to avoid accidental D-pad taps.
        const auto policy_switch_hold = std::chrono::duration<float>(policy_switch_long_press_s_);
        msg.btn_policy_prev = long_press_pulse(
            msg.btn_dpad_left, policy_prev_filter_, policy_switch_hold);
        msg.btn_policy_next = long_press_pulse(
            msg.btn_dpad_right, policy_next_filter_, policy_switch_hold);

        cmd_pub_->publish(msg);
    }

    rclcpp::Publisher<kvoy_msgs::msg::GamepadCmd>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    SDL_Joystick* joystick_{nullptr};
    bool has_last_open_attempt_{false};
    SteadyClock::time_point last_open_attempt_{};
    float joystick_reconnect_interval_s_{1.0f};
    int joystick_index_{-1};
    std::string preferred_joystick_name_substring_{"GameSir"};
    float vx_max_mps_{DEFAULT_VX_MAX};
    float vy_max_mps_{DEFAULT_VY_MAX};
    float yaw_rate_max_rad_s_{DEFAULT_YAW_MAX};
    float stick_deadzone_{DEFAULT_STICK_DEADZONE};
    float digital_axis_threshold_{DEFAULT_DIGITAL_AXIS_THRESHOLD};
    float policy_switch_long_press_s_{DEFAULT_POLICY_SWITCH_LONG_PRESS_S};
    ChordPulseFilter standup_filter_;
    ChordPulseFilter liedown_filter_;
    ChordPulseFilter estop_filter_;
    LongPressPulseFilter policy_prev_filter_;
    LongPressPulseFilter policy_next_filter_;
};

} // namespace kvoy

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kvoy::GamepadInputNode>());
    rclcpp::shutdown();
    return 0;
}
