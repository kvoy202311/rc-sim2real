#include "rl_controller/rl_controller_node.hpp"

#include <NvInfer.h>
#include <NvInferVersion.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace kvoy {
namespace {

class TrtLogger : public nvinfer1::ILogger
{
public:
    explicit TrtLogger(rclcpp::Logger logger)
    : logger_(std::move(logger))
    {}

    void log(Severity severity, const char* msg) noexcept override
    {
        if (msg == nullptr) return;

        switch (severity) {
            case Severity::kINTERNAL_ERROR:
            case Severity::kERROR:
                RCLCPP_ERROR(logger_, "TensorRT: %s", msg);
                break;
            case Severity::kWARNING:
                RCLCPP_WARN(logger_, "TensorRT: %s", msg);
                break;
            case Severity::kINFO:
                RCLCPP_INFO(logger_, "TensorRT: %s", msg);
                break;
            case Severity::kVERBOSE:
                RCLCPP_DEBUG(logger_, "TensorRT: %s", msg);
                break;
        }
    }

private:
    rclcpp::Logger logger_;
};

template <typename T>
struct TrtDeleter
{
    void operator()(T* ptr) const
    {
        if (ptr) {
#if NV_TENSORRT_MAJOR >= 10
            delete ptr;
#else
            ptr->destroy();
#endif
        }
    }
};

template <typename T>
using TrtUniquePtr = std::unique_ptr<T, TrtDeleter<T>>;

inline void throw_if_cuda_failed(cudaError_t status, const char* what)
{
    if (status != cudaSuccess) {
        throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(status));
    }
}

inline std::size_t volume(const nvinfer1::Dims& dims)
{
    std::size_t v = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] < 0) {
            throw std::runtime_error("TensorRT tensor has unresolved dynamic dimension");
        }
        v *= static_cast<std::size_t>(dims.d[i]);
    }
    return v;
}

inline bool dims_match_2d(const nvinfer1::Dims& dims, int d0, int d1)
{
    return dims.nbDims == 2 && dims.d[0] == d0 && dims.d[1] == d1;
}

inline bool dims_match_1d(const nvinfer1::Dims& dims, int d0)
{
    return dims.nbDims == 1 && dims.d[0] == d0;
}

inline bool dims_compatible_2d(const nvinfer1::Dims& dims, int d0, int d1)
{
    return dims.nbDims == 2 &&
           (dims.d[0] == d0 || dims.d[0] == -1) &&
           (dims.d[1] == d1 || dims.d[1] == -1);
}

inline bool dims_compatible_output(const nvinfer1::Dims& dims, int width)
{
    return dims_match_1d(dims, width) || dims_compatible_2d(dims, 1, width);
}

inline bool has_dynamic_dim(const nvinfer1::Dims& dims)
{
    for (int i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] < 0) return true;
    }
    return false;
}

inline std::string trim_copy(const std::string& in)
{
    const auto first = in.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const auto last = in.find_last_not_of(" \t\r\n");
    return in.substr(first, last - first + 1);
}

inline float row_dot(
    const std::array<float, 9>& m, int row_a, int row_b)
{
    return m[static_cast<std::size_t>(row_a * 3 + 0)] *
               m[static_cast<std::size_t>(row_b * 3 + 0)] +
           m[static_cast<std::size_t>(row_a * 3 + 1)] *
               m[static_cast<std::size_t>(row_b * 3 + 1)] +
           m[static_cast<std::size_t>(row_a * 3 + 2)] *
               m[static_cast<std::size_t>(row_b * 3 + 2)];
}

inline float det3(const std::array<float, 9>& m)
{
    return m[0] * (m[4] * m[8] - m[5] * m[7]) -
           m[1] * (m[3] * m[8] - m[5] * m[6]) +
           m[2] * (m[3] * m[7] - m[4] * m[6]);
}

constexpr int kPolicySwitchHoldCycles = 10;

} // namespace

struct RLControllerNode::PolicyConfig
{
    int history_steps{6};
    int one_step_obs{HIMLOCO_V1_ONE_STEP_OBS};
    std::array<float, NUM_JOINTS> default_joint_pos{STAND_POS};
    float action_scale{0.25f};
    float clip_obs{100.0f};
    float clip_actions{100.0f};
    float obs_ang_vel_scale{0.25f};
    float obs_dof_pos_scale{1.0f};
    float obs_dof_vel_scale{0.05f};
    float cmd_lin_vel_scale{2.0f};
    float cmd_ang_vel_scale{0.25f};

    int total_obs() const { return history_steps * one_step_obs; }
};

struct RLControllerNode::TensorRtContext
{
    explicit TensorRtContext(
        const std::string& policy_name_in,
        rclcpp::Logger logger_in,
        PolicyConfig config_in)
    : policy_name(policy_name_in),
      logger(std::move(logger_in)),
      trt_logger(logger),
      config(std::move(config_in))
    {}

    std::string policy_name;
    rclcpp::Logger logger;
    TrtLogger trt_logger;
    PolicyConfig config;
    TrtUniquePtr<nvinfer1::IRuntime> runtime;
    TrtUniquePtr<nvinfer1::ICudaEngine> engine;
    TrtUniquePtr<nvinfer1::IExecutionContext> context;
    cudaStream_t stream{nullptr};
    void* device_input{nullptr};
    void* device_output{nullptr};
    std::vector<float> host_output;

    int input_binding{-1};
    int output_binding{-1};
    std::size_t input_bytes{0};
    std::size_t output_bytes{0};
    std::string input_name;
    std::string output_name;
    std::vector<float> obs_history;
    std::array<float, NUM_JOINTS> last_action{};

    ~TensorRtContext()
    {
        if (device_input) cudaFree(device_input);
        if (device_output) cudaFree(device_output);
        if (stream) cudaStreamDestroy(stream);
    }
};

RLControllerNode::RLControllerNode(const rclcpp::NodeOptions& options)
: Node("rl_controller_node", options)
{
    declare_parameter("policy_slot_names", std::vector<std::string>{});
    declare_parameter("default_policy",    "");
    declare_parameter("control_rate_hz",   50.0);
    declare_parameter("action_scale",      0.25);
    declare_parameter("clip_obs",          100.0);
    declare_parameter("clip_actions",      100.0);
    declare_parameter("obs_ang_vel_scale", 0.25);
    declare_parameter("obs_dof_pos_scale", 1.0);
    declare_parameter("obs_dof_vel_scale", 0.05);
    declare_parameter("cmd_lin_vel_scale", 2.0);
    declare_parameter("cmd_ang_vel_scale", 0.25);
    declare_parameter("joint_pos_min", std::vector<double>(NUM_JOINTS, -3.14));
    declare_parameter("joint_pos_max", std::vector<double>(NUM_JOINTS,  3.14));
    declare_parameter("max_imu_age_s",     0.1);
    declare_parameter("policy_switch_sound_enabled", false);
    declare_parameter("policy_switch_sound_file", std::string{});
    declare_parameter("policy_switch_sound_interval_ms", 150);
    declare_parameter("imu_to_body_rotation", std::vector<double>{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0});

    action_scale_      = static_cast<float>(get_parameter("action_scale").as_double());
    clip_obs_          = static_cast<float>(get_parameter("clip_obs").as_double());
    clip_actions_      = static_cast<float>(get_parameter("clip_actions").as_double());
    obs_ang_vel_scale_ = static_cast<float>(get_parameter("obs_ang_vel_scale").as_double());
    obs_dof_pos_scale_ = static_cast<float>(get_parameter("obs_dof_pos_scale").as_double());
    obs_dof_vel_scale_ = static_cast<float>(get_parameter("obs_dof_vel_scale").as_double());
    cmd_lin_vel_scale_ = static_cast<float>(get_parameter("cmd_lin_vel_scale").as_double());
    cmd_ang_vel_scale_ = static_cast<float>(get_parameter("cmd_ang_vel_scale").as_double());
    const auto joint_pos_min = get_parameter("joint_pos_min").as_double_array();
    const auto joint_pos_max = get_parameter("joint_pos_max").as_double_array();
    if (joint_pos_min.size() != NUM_JOINTS || joint_pos_max.size() != NUM_JOINTS) {
        throw std::runtime_error("joint_pos_min and joint_pos_max must each contain exactly 12 values");
    }
    for (int i = 0; i < NUM_JOINTS; ++i) {
        joint_pos_min_[static_cast<std::size_t>(i)] =
            static_cast<float>(joint_pos_min[static_cast<std::size_t>(i)]);
        joint_pos_max_[static_cast<std::size_t>(i)] =
            static_cast<float>(joint_pos_max[static_cast<std::size_t>(i)]);
        if (joint_pos_min_[static_cast<std::size_t>(i)] >
            joint_pos_max_[static_cast<std::size_t>(i)]) {
            throw std::runtime_error("joint_pos_min must be <= joint_pos_max for every joint");
        }
    }
    max_imu_age_s_     = std::max(0.0, get_parameter("max_imu_age_s").as_double());
    policy_switch_sound_enabled_ = get_parameter("policy_switch_sound_enabled").as_bool();
    policy_switch_sound_file_ = trim_copy(get_parameter("policy_switch_sound_file").as_string());
    policy_switch_sound_interval_ms_ = std::max(
        0, static_cast<int>(get_parameter("policy_switch_sound_interval_ms").as_int()));
    load_imu_params();
    hold_position_target_ = STAND_POS;

    try {
        load_policies_from_params();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to load TensorRT policies: %s", e.what());
    }

    state_sub_ = create_subscription<kvoy_msgs::msg::MotorState>(
        "/motor_state", 10,
        std::bind(&RLControllerNode::on_motor_state, this, std::placeholders::_1));
    imu_sub_ = create_subscription<kvoy_msgs::msg::ImuData>(
        "/imu/data", 10,
        std::bind(&RLControllerNode::on_imu, this, std::placeholders::_1));
    gamepad_sub_ = create_subscription<kvoy_msgs::msg::GamepadCmd>(
        "/gamepad_cmd", 10,
        std::bind(&RLControllerNode::on_gamepad, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<kvoy_msgs::msg::MotorCmd>("/motor_cmd", 10);

    const double rate = get_parameter("control_rate_hz").as_double();
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = create_wall_timer(period, std::bind(&RLControllerNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "rl_controller_node ready (%.0f Hz)", rate);
}

RLControllerNode::~RLControllerNode() = default;

void RLControllerNode::load_imu_params()
{
    const auto values = get_parameter("imu_to_body_rotation").as_double_array();
    if (values.size() != imu_to_body_rotation_.size()) {
        throw std::runtime_error(
            "imu_to_body_rotation must contain 9 row-major values");
    }

    for (std::size_t i = 0; i < values.size(); ++i) {
        imu_to_body_rotation_[i] = static_cast<float>(values[i]);
    }

    const float row0_norm = row_dot(imu_to_body_rotation_, 0, 0);
    const float row1_norm = row_dot(imu_to_body_rotation_, 1, 1);
    const float row2_norm = row_dot(imu_to_body_rotation_, 2, 2);
    const float row01_dot = row_dot(imu_to_body_rotation_, 0, 1);
    const float row02_dot = row_dot(imu_to_body_rotation_, 0, 2);
    const float row12_dot = row_dot(imu_to_body_rotation_, 1, 2);
    const float det = det3(imu_to_body_rotation_);

    if (std::abs(row0_norm - 1.0f) > 1.0e-3f ||
        std::abs(row1_norm - 1.0f) > 1.0e-3f ||
        std::abs(row2_norm - 1.0f) > 1.0e-3f ||
        std::abs(row01_dot) > 1.0e-3f ||
        std::abs(row02_dot) > 1.0e-3f ||
        std::abs(row12_dot) > 1.0e-3f ||
        std::abs(det - 1.0f) > 1.0e-3f) {
        RCLCPP_WARN(
            get_logger(),
            "imu_to_body_rotation is not a proper right-handed rotation "
            "(row_norms=[%.4f %.4f %.4f], row_dots=[%.4f %.4f %.4f], det=%.4f)",
            row0_norm, row1_norm, row2_norm, row01_dot, row02_dot, row12_dot, det);
    }
}

void RLControllerNode::set_active(bool active)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_ == active) {
        return;
    }

    active_ = active;
    if (active_) {
        auto it = policies_.find(active_policy_name_);
        if (it != policies_.end()) {
            reset_runtime_state_locked(*it->second);
        }
        hold_position_target_ = joint_pos_;
        hold_cycles_remaining_ = has_motor_state_ ? kPolicySwitchHoldCycles : 0;
    }
}

std::unique_ptr<RLControllerNode::TensorRtContext> RLControllerNode::load_engine(
    const std::string& policy_name,
    const std::string& engine_path,
    const PolicyConfig& config) const
{
    std::ifstream engine_file(engine_path, std::ios::binary | std::ios::ate);
    if (!engine_file.is_open()) {
        throw std::runtime_error("cannot open engine file: " + engine_path);
    }

    const auto file_size = engine_file.tellg();
    if (file_size <= 0) {
        throw std::runtime_error("engine file is empty: " + engine_path);
    }
    engine_file.seekg(0, std::ios::beg);

    std::vector<char> engine_data(static_cast<std::size_t>(file_size));
    engine_file.read(engine_data.data(), static_cast<std::streamsize>(file_size));
    if (!engine_file) {
        throw std::runtime_error("failed to read engine file: " + engine_path);
    }

    auto trt = std::make_unique<TensorRtContext>(policy_name, get_logger(), config);
    trt->runtime.reset(nvinfer1::createInferRuntime(trt->trt_logger));
    if (!trt->runtime) {
        throw std::runtime_error("createInferRuntime returned null");
    }

    trt->engine.reset(
        trt->runtime->deserializeCudaEngine(engine_data.data(), engine_data.size()));
    if (!trt->engine) {
        throw std::runtime_error("deserializeCudaEngine failed");
    }

#if NV_TENSORRT_MAJOR >= 10
    if (trt->engine->getNbIOTensors() != 2) {
        throw std::runtime_error("expected exactly 2 engine IO tensors");
    }

    for (int i = 0; i < trt->engine->getNbIOTensors(); ++i) {
        const char* name = trt->engine->getIOTensorName(i);
        if (name == nullptr) {
            throw std::runtime_error("TensorRT returned a null tensor name");
        }
        if (trt->engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
            trt->input_binding = i;
            trt->input_name = name;
        } else if (trt->engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT) {
            trt->output_binding = i;
            trt->output_name = name;
        }
    }
#else
    if (trt->engine->getNbBindings() != 2) {
        throw std::runtime_error("expected exactly 2 engine bindings");
    }

    for (int i = 0; i < trt->engine->getNbBindings(); ++i) {
        const char* name = trt->engine->getBindingName(i);
        if (name == nullptr) {
            throw std::runtime_error("TensorRT returned a null binding name");
        }
        if (trt->engine->bindingIsInput(i)) {
            trt->input_binding = i;
            trt->input_name = name;
        } else {
            trt->output_binding = i;
            trt->output_name = name;
        }
    }
#endif

    if (trt->input_binding < 0 || trt->output_binding < 0) {
        throw std::runtime_error("failed to identify input/output bindings");
    }

    trt->context.reset(trt->engine->createExecutionContext());
    if (!trt->context) {
        throw std::runtime_error("createExecutionContext failed");
    }

#if NV_TENSORRT_MAJOR >= 10
    auto input_dims = trt->engine->getTensorShape(trt->input_name.c_str());
    auto output_dims = trt->engine->getTensorShape(trt->output_name.c_str());
#else
    auto input_dims = trt->engine->getBindingDimensions(trt->input_binding);
    auto output_dims = trt->engine->getBindingDimensions(trt->output_binding);
#endif
    if (!dims_compatible_2d(input_dims, 1, trt->config.total_obs())) {
        std::ostringstream oss;
        oss << "unexpected input dims, expected [1, " << trt->config.total_obs() << "]";
        throw std::runtime_error(oss.str());
    }
    if (!dims_compatible_output(output_dims, NUM_JOINTS)) {
        throw std::runtime_error("unexpected output dims, expected [1, 12] or [12]");
    }

#if NV_TENSORRT_MAJOR >= 10
    if (trt->engine->getTensorDataType(trt->input_name.c_str()) != nvinfer1::DataType::kFLOAT ||
        trt->engine->getTensorDataType(trt->output_name.c_str()) != nvinfer1::DataType::kFLOAT) {
#else
    if (trt->engine->getBindingDataType(trt->input_binding) != nvinfer1::DataType::kFLOAT ||
        trt->engine->getBindingDataType(trt->output_binding) != nvinfer1::DataType::kFLOAT) {
#endif
        throw std::runtime_error("only float32 TensorRT bindings are supported");
    }

    if (has_dynamic_dim(input_dims)) {
        nvinfer1::Dims runtime_input_dims = input_dims;
        runtime_input_dims.d[0] = 1;
        runtime_input_dims.d[1] = trt->config.total_obs();
#if NV_TENSORRT_MAJOR >= 10
        if (!trt->context->setInputShape(trt->input_name.c_str(), runtime_input_dims)) {
            throw std::runtime_error("failed to set TensorRT runtime input shape");
        }
        input_dims = trt->context->getTensorShape(trt->input_name.c_str());
        output_dims = trt->context->getTensorShape(trt->output_name.c_str());
#else
        if (!trt->context->setBindingDimensions(trt->input_binding, runtime_input_dims)) {
            throw std::runtime_error("failed to set TensorRT binding dimensions");
        }
        input_dims = trt->context->getBindingDimensions(trt->input_binding);
        output_dims = trt->context->getBindingDimensions(trt->output_binding);
#endif
    }

    trt->input_bytes = volume(input_dims) * sizeof(float);
    trt->output_bytes = volume(output_dims) * sizeof(float);
    trt->host_output.resize(trt->output_bytes / sizeof(float));
    trt->obs_history.assign(static_cast<std::size_t>(trt->config.total_obs()), 0.0f);

    throw_if_cuda_failed(cudaStreamCreate(&trt->stream), "cudaStreamCreate failed");
    throw_if_cuda_failed(cudaMalloc(&trt->device_input, trt->input_bytes), "cudaMalloc input failed");
    throw_if_cuda_failed(cudaMalloc(&trt->device_output, trt->output_bytes), "cudaMalloc output failed");

    return trt;
}

RLControllerNode::PolicyConfig RLControllerNode::make_default_policy_config() const
{
    PolicyConfig config;
    config.history_steps = 6;
    config.one_step_obs = HIMLOCO_V1_ONE_STEP_OBS;
    config.default_joint_pos = STAND_POS;
    config.action_scale = action_scale_;
    config.clip_obs = clip_obs_;
    config.clip_actions = clip_actions_;
    config.obs_ang_vel_scale = obs_ang_vel_scale_;
    config.obs_dof_pos_scale = obs_dof_pos_scale_;
    config.obs_dof_vel_scale = obs_dof_vel_scale_;
    config.cmd_lin_vel_scale = cmd_lin_vel_scale_;
    config.cmd_ang_vel_scale = cmd_ang_vel_scale_;
    return config;
}

std::array<float, NUM_JOINTS> RLControllerNode::clamp_joint_targets(
    const std::array<float, NUM_JOINTS>& target) const
{
    std::array<float, NUM_JOINTS> clamped{};
    for (int i = 0; i < NUM_JOINTS; ++i) {
        clamped[static_cast<std::size_t>(i)] = std::max(
            joint_pos_min_[static_cast<std::size_t>(i)],
            std::min(joint_pos_max_[static_cast<std::size_t>(i)],
                     target[static_cast<std::size_t>(i)]));
    }
    return clamped;
}

void RLControllerNode::load_policies_from_params()
{
    const auto policy_slot_names = get_parameter("policy_slot_names").as_string_array();
    const auto default_policy = trim_copy(get_parameter("default_policy").as_string());
    const PolicyConfig default_config = make_default_policy_config();

    std::lock_guard<std::mutex> lock(mutex_);
    policies_.clear();
    policy_order_.clear();

    if (policy_slot_names.empty()) {
        RCLCPP_WARN(get_logger(), "No TensorRT policy slots configured - RL controller will stay inactive");
        return;
    }

    for (const auto& slot_raw : policy_slot_names) {
        const std::string slot_name = trim_copy(slot_raw);
        if (slot_name.empty()) {
            throw std::runtime_error("policy_slot_names contains an empty slot name");
        }

        declare_parameter(slot_name + ".name", slot_name);
        declare_parameter(slot_name + ".path", "");
        declare_parameter(slot_name + ".history_steps", default_config.history_steps);
        declare_parameter(slot_name + ".one_step_obs", default_config.one_step_obs);
        declare_parameter(slot_name + ".default_joint_pos", std::vector<double>(
            default_config.default_joint_pos.begin(), default_config.default_joint_pos.end()));
        declare_parameter(slot_name + ".action_scale", default_config.action_scale);
        declare_parameter(slot_name + ".clip_obs", default_config.clip_obs);
        declare_parameter(slot_name + ".clip_actions", default_config.clip_actions);
        declare_parameter(slot_name + ".obs_ang_vel_scale", default_config.obs_ang_vel_scale);
        declare_parameter(slot_name + ".obs_dof_pos_scale", default_config.obs_dof_pos_scale);
        declare_parameter(slot_name + ".obs_dof_vel_scale", default_config.obs_dof_vel_scale);
        declare_parameter(slot_name + ".cmd_lin_vel_scale", default_config.cmd_lin_vel_scale);
        declare_parameter(slot_name + ".cmd_ang_vel_scale", default_config.cmd_ang_vel_scale);

        const std::string policy_name = trim_copy(get_parameter(slot_name + ".name").as_string());
        const std::string policy_path = trim_copy(get_parameter(slot_name + ".path").as_string());
        if (policy_path.empty()) {
            continue;
        }
        if (policy_name.empty()) {
            throw std::runtime_error("policy slot '" + slot_name + "' has empty name for a non-empty path");
        }
        if (policies_.count(policy_name) > 0) {
            throw std::runtime_error("duplicate policy name: " + policy_name);
        }

        PolicyConfig config;
        config.history_steps = get_parameter(slot_name + ".history_steps").as_int();
        config.one_step_obs = get_parameter(slot_name + ".one_step_obs").as_int();
        if (config.history_steps <= 0 || config.one_step_obs <= 0) {
            throw std::runtime_error("policy slot '" + slot_name + "' must have positive history_steps and one_step_obs");
        }
        if (config.one_step_obs != HIMLOCO_V1_ONE_STEP_OBS) {
            throw std::runtime_error(
                "policy slot '" + slot_name + "' currently only supports one_step_obs = 45");
        }
        const auto default_pos = get_parameter(slot_name + ".default_joint_pos").as_double_array();
        if (default_pos.size() != NUM_JOINTS) {
            throw std::runtime_error(
                "policy slot '" + slot_name + "'.default_joint_pos must contain exactly 12 values");
        }
        for (int i = 0; i < NUM_JOINTS; ++i) {
            config.default_joint_pos[static_cast<std::size_t>(i)] =
                static_cast<float>(default_pos[static_cast<std::size_t>(i)]);
        }
        config.action_scale = static_cast<float>(get_parameter(slot_name + ".action_scale").as_double());
        config.clip_obs = static_cast<float>(get_parameter(slot_name + ".clip_obs").as_double());
        config.clip_actions = static_cast<float>(get_parameter(slot_name + ".clip_actions").as_double());
        config.obs_ang_vel_scale = static_cast<float>(get_parameter(slot_name + ".obs_ang_vel_scale").as_double());
        config.obs_dof_pos_scale = static_cast<float>(get_parameter(slot_name + ".obs_dof_pos_scale").as_double());
        config.obs_dof_vel_scale = static_cast<float>(get_parameter(slot_name + ".obs_dof_vel_scale").as_double());
        config.cmd_lin_vel_scale = static_cast<float>(get_parameter(slot_name + ".cmd_lin_vel_scale").as_double());
        config.cmd_ang_vel_scale = static_cast<float>(get_parameter(slot_name + ".cmd_ang_vel_scale").as_double());

        auto trt = load_engine(policy_name, policy_path, config);
        policies_.emplace(policy_name, std::move(trt));
        policy_order_.push_back(policy_name);
        RCLCPP_INFO(get_logger(), "Loaded policy '%s' from %s", policy_name.c_str(), policy_path.c_str());
    }

    if (policies_.empty()) {
        RCLCPP_WARN(get_logger(), "Policy slots are configured but no valid policy paths were provided");
        return;
    }

    const std::string selected_policy =
        !default_policy.empty() ? default_policy : policy_order_.front();
    if (!select_policy_locked(selected_policy)) {
        throw std::runtime_error("default_policy not found: " + selected_policy);
    }
    policies_loaded_ = true;
}

bool RLControllerNode::select_policy_locked(const std::string& policy_name)
{
    auto it = policies_.find(policy_name);
    if (it == policies_.end()) {
        return false;
    }

    if (active_policy_name_ == policy_name) {
        return true;
    }

    active_policy_name_ = policy_name;
    reset_runtime_state_locked(*it->second);
    hold_position_target_ = joint_pos_;
    hold_cycles_remaining_ = has_motor_state_ ? kPolicySwitchHoldCycles : 0;
    RCLCPP_INFO(get_logger(), "Active RL policy switched to '%s'", active_policy_name_.c_str());
    request_policy_switch_sound_locked();
    return true;
}

bool RLControllerNode::select_policy_by_offset_locked(int offset)
{
    if (policy_order_.empty() || offset == 0) {
        return false;
    }

    auto it = std::find(policy_order_.begin(), policy_order_.end(), active_policy_name_);
    if (it == policy_order_.end()) {
        return select_policy_locked(policy_order_.front());
    }

    const int current_index = static_cast<int>(std::distance(policy_order_.begin(), it));
    const int count = static_cast<int>(policy_order_.size());
    const int next_index = (current_index + offset % count + count) % count;
    return select_policy_locked(policy_order_[static_cast<std::size_t>(next_index)]);
}

void RLControllerNode::reset_runtime_state_locked(TensorRtContext& policy)
{
    std::fill(policy.obs_history.begin(), policy.obs_history.end(), 0.0f);
    policy.last_action.fill(0.0f);
}

void RLControllerNode::request_policy_switch_sound_locked()
{
    if (!policy_switch_sound_enabled_) {
        return;
    }
    if (policy_switch_sound_file_.empty()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "policy_switch_sound_enabled=true but policy_switch_sound_file is empty");
        return;
    }

    int sound_count = 0;
    auto it = std::find(policy_order_.begin(), policy_order_.end(), active_policy_name_);
    if (it != policy_order_.end()) {
        sound_count = static_cast<int>(std::distance(policy_order_.begin(), it)) + 1;
    }
    if (sound_count <= 0) {
        return;
    }

    const std::string sound_file = policy_switch_sound_file_;
    const int interval_ms = policy_switch_sound_interval_ms_;
    auto request_seq = sound_request_seq_;
    const std::uint64_t token = request_seq->fetch_add(1, std::memory_order_relaxed) + 1;

    std::thread([this, sound_file, interval_ms, sound_count, request_seq, token]() {
        for (int i = 0; i < sound_count; ++i) {
            if (request_seq->load(std::memory_order_relaxed) != token) {
                return;
            }

            const std::string cmd = "aplay -q \"" + sound_file + "\" >/dev/null 2>&1";
            const int ret = std::system(cmd.c_str());
            if (ret != 0) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Failed to play policy switch sound with aplay");
                return;
            }

            if (i + 1 < sound_count && interval_ms > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
            }
        }
    }).detach();
}

void RLControllerNode::on_motor_state(const kvoy_msgs::msg::MotorState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        joint_pos_[i] = msg->position[i];
        joint_vel_[i] = msg->velocity[i];
    }
    has_motor_state_ = true;
}

void RLControllerNode::on_imu(const kvoy_msgs::msg::ImuData::SharedPtr msg)
{
    const std::array<float, 4> sensor_quat{
        static_cast<float>(msg->orientation.x),
        static_cast<float>(msg->orientation.y),
        static_cast<float>(msg->orientation.z),
        static_cast<float>(msg->orientation.w)};
    const std::array<float, 3> sensor_ang_vel{
        static_cast<float>(msg->angular_velocity.x),
        static_cast<float>(msg->angular_velocity.y),
        static_cast<float>(msg->angular_velocity.z)};
    const std::array<float, 3> gravity_world{0.0f, 0.0f, -1.0f};

    std::lock_guard<std::mutex> lock(mutex_);
    const auto gravity_sensor = quat_rotate_inverse(sensor_quat, gravity_world);
    ang_vel_ = mat3_mul_vec3(imu_to_body_rotation_, sensor_ang_vel);
    projected_gravity_ = mat3_mul_vec3(imu_to_body_rotation_, gravity_sensor);
    last_imu_time_ = now();
    has_imu_ = true;
}

void RLControllerNode::on_gamepad(const kvoy_msgs::msg::GamepadCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    command_[0] = msg->vx;
    command_[1] = msg->vy;
    command_[2] = msg->yaw_rate;

    const bool press_prev = msg->btn_policy_prev && !prev_btn_policy_prev_;
    const bool press_next = msg->btn_policy_next && !prev_btn_policy_next_;
    prev_btn_policy_prev_ = msg->btn_policy_prev;
    prev_btn_policy_next_ = msg->btn_policy_next;

    if (press_prev && !select_policy_by_offset_locked(-1)) {
        RCLCPP_WARN(get_logger(), "Policy switch to previous failed");
    }
    if (press_next && !select_policy_by_offset_locked(1)) {
        RCLCPP_WARN(get_logger(), "Policy switch to next failed");
    }
}

std::array<float, 3> RLControllerNode::quat_rotate_inverse(
    const std::array<float, 4>& q, const std::array<float, 3>& v)
{
    const float x = q[0];
    const float y = q[1];
    const float z = q[2];
    const float w = q[3];
    const float norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (norm <= 1.0e-6f) {
        return v;
    }

    const float inv_norm = 1.0f / norm;
    const float nx = x * inv_norm;
    const float ny = y * inv_norm;
    const float nz = z * inv_norm;
    const float nw = w * inv_norm;

    // Rotate world vector into body frame using q^-1 * v * q.
    const float tx = 2.0f * (ny * v[2] - nz * v[1]);
    const float ty = 2.0f * (nz * v[0] - nx * v[2]);
    const float tz = 2.0f * (nx * v[1] - ny * v[0]);
    return {
        v[0] - nw * tx + (ny * tz - nz * ty),
        v[1] - nw * ty + (nz * tx - nx * tz),
        v[2] - nw * tz + (nx * ty - ny * tx),
    };
}

std::array<float, 3> RLControllerNode::mat3_mul_vec3(
    const std::array<float, 9>& m, const std::array<float, 3>& v)
{
    return {
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
        m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
    };
}

std::array<float, RLControllerNode::HIMLOCO_V1_ONE_STEP_OBS> RLControllerNode::build_obs(
    const PolicyConfig& config,
    const std::array<float, NUM_JOINTS>& last_action) const
{
    std::array<float, HIMLOCO_V1_ONE_STEP_OBS> obs{};
    int idx = 0;

    obs[idx++] = command_[0] * config.cmd_lin_vel_scale;
    obs[idx++] = command_[1] * config.cmd_lin_vel_scale;
    obs[idx++] = command_[2] * config.cmd_ang_vel_scale;

    obs[idx++] = ang_vel_[0] * config.obs_ang_vel_scale;
    obs[idx++] = ang_vel_[1] * config.obs_ang_vel_scale;
    obs[idx++] = ang_vel_[2] * config.obs_ang_vel_scale;

    obs[idx++] = projected_gravity_[0];
    obs[idx++] = projected_gravity_[1];
    obs[idx++] = projected_gravity_[2];

    for (int i = 0; i < NUM_JOINTS; ++i) {
        obs[idx++] = (joint_pos_[i] - config.default_joint_pos[i]) * config.obs_dof_pos_scale;
    }
    for (int i = 0; i < NUM_JOINTS; ++i) {
        obs[idx++] = joint_vel_[i] * config.obs_dof_vel_scale;
    }
    for (int i = 0; i < NUM_JOINTS; ++i) {
        obs[idx++] = last_action[i];
    }

    for (auto& v : obs) {
        v = std::max(-config.clip_obs, std::min(config.clip_obs, v));
    }
    return obs;
}

void RLControllerNode::run_inference(
    TensorRtContext& trt,
    const std::vector<float>& obs_history,
    std::array<float, NUM_JOINTS>& action) const
{
    throw_if_cuda_failed(
        cudaMemcpyAsync(trt.device_input,
                        obs_history.data(),
                        trt.input_bytes,
                        cudaMemcpyHostToDevice,
                        trt.stream),
        "cudaMemcpyAsync input failed");

#if NV_TENSORRT_MAJOR >= 10
    if (!trt.context->setTensorAddress(trt.input_name.c_str(), trt.device_input) ||
        !trt.context->setTensorAddress(trt.output_name.c_str(), trt.device_output)) {
        throw std::runtime_error("TensorRT setTensorAddress failed");
    }
    if (!trt.context->enqueueV3(trt.stream)) {
        throw std::runtime_error("TensorRT enqueueV3 failed");
    }
#else
    void* bindings[2]{};
    bindings[trt.input_binding] = trt.device_input;
    bindings[trt.output_binding] = trt.device_output;
    if (!trt.context->enqueueV2(bindings, trt.stream, nullptr)) {
        throw std::runtime_error("TensorRT enqueueV2 failed");
    }
#endif

    throw_if_cuda_failed(
        cudaMemcpyAsync(trt.host_output.data(),
                        trt.device_output,
                        trt.output_bytes,
                        cudaMemcpyDeviceToHost,
                        trt.stream),
        "cudaMemcpyAsync output failed");
    throw_if_cuda_failed(cudaStreamSynchronize(trt.stream), "cudaStreamSynchronize failed");

    for (int i = 0; i < NUM_JOINTS; ++i) {
        action[i] = std::max(-trt.config.clip_actions, std::min(trt.config.clip_actions, trt.host_output[i]));
    }
}

void RLControllerNode::control_loop()
{
    if (!active_ || !policies_loaded_) return;

    TensorRtContext* active_policy = nullptr;
    std::vector<float> obs_history_snapshot;
    std::array<float, NUM_JOINTS> hold_position_snapshot{};
    bool publish_hold_position = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_motor_state_ || !has_imu_) {
            return;
        }
        if (max_imu_age_s_ > 0.0 && (now() - last_imu_time_).seconds() > max_imu_age_s_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Skipping RL inference: IMU data is stale");
            return;
        }
        if (active_policy_name_.empty()) {
            return;
        }

        auto it = policies_.find(active_policy_name_);
        if (it == policies_.end()) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000, "Active policy '%s' is not loaded",
                active_policy_name_.c_str());
            return;
        }

        if (hold_cycles_remaining_ > 0) {
            hold_position_snapshot = hold_position_target_;
            --hold_cycles_remaining_;
            publish_hold_position = true;
        }

        active_policy = it->second.get();
        const auto current_obs = build_obs(active_policy->config, active_policy->last_action);
        std::memmove(active_policy->obs_history.data() + active_policy->config.one_step_obs,
                     active_policy->obs_history.data(),
                     static_cast<std::size_t>(
                         active_policy->config.total_obs() - active_policy->config.one_step_obs) * sizeof(float));
        std::memcpy(active_policy->obs_history.data(),
                    current_obs.data(),
                    static_cast<std::size_t>(active_policy->config.one_step_obs) * sizeof(float));
        obs_history_snapshot = active_policy->obs_history;
    }

    if (publish_hold_position) {
        auto msg = kvoy_msgs::msg::MotorCmd();
        msg.header.stamp = now();
        for (int i = 0; i < NUM_JOINTS; ++i) {
            msg.position[i] = hold_position_snapshot[i];
            msg.velocity[i] = 0.0f;
            msg.torque[i]   = 0.0f;
        }
        cmd_pub_->publish(msg);
        return;
    }

    std::array<float, NUM_JOINTS> action{};
    try {
        run_inference(*active_policy, obs_history_snapshot, action);
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000, "TensorRT inference failed: %s", e.what());
        return;
    }

    auto msg = kvoy_msgs::msg::MotorCmd();
    msg.header.stamp = now();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = policies_.find(active_policy_name_);
        if (it != policies_.end()) {
            std::array<float, NUM_JOINTS> joint_target{};
            for (int i = 0; i < NUM_JOINTS; ++i) {
                joint_target[static_cast<std::size_t>(i)] =
                    it->second->config.default_joint_pos[static_cast<std::size_t>(i)] +
                    action[static_cast<std::size_t>(i)] * it->second->config.action_scale;
            }
            joint_target = clamp_joint_targets(joint_target);
            it->second->last_action = action;
            for (int i = 0; i < NUM_JOINTS; ++i) {
                msg.position[i] = joint_target[static_cast<std::size_t>(i)];
                msg.velocity[i] = 0.0f;
                msg.torque[i]   = 0.0f;
            }
            cmd_pub_->publish(msg);
            return;
        }
        const auto& fallback_default_pos = STAND_POS;
        std::array<float, NUM_JOINTS> joint_target{};
        for (int i = 0; i < NUM_JOINTS; ++i) {
            joint_target[static_cast<std::size_t>(i)] =
                fallback_default_pos[static_cast<std::size_t>(i)] +
                action[static_cast<std::size_t>(i)] * action_scale_;
        }
        joint_target = clamp_joint_targets(joint_target);
        for (int i = 0; i < NUM_JOINTS; ++i) {
            msg.position[i] = joint_target[static_cast<std::size_t>(i)];
            msg.velocity[i] = 0.0f;
            msg.torque[i]   = 0.0f;
        }
    }
    cmd_pub_->publish(msg);
}

} // namespace kvoy
