#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace {

using LowState = unitree_go::msg::dds_::LowState_;
using Clock = std::chrono::steady_clock;

constexpr std::array<const char *, 12> kJointNames = {
    "FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf",
};
constexpr std::array<float, 12> kDefaultPosition = {
    0.0F, 0.8F, -1.5F, 0.0F, 0.8F, -1.5F, 0.0F, 0.8F, -1.5F, 0.0F, 0.8F, -1.5F,
};
constexpr std::array<float, 12> kActionScale = {
    0.125F, 0.25F, 0.25F, 0.125F, 0.25F, 0.25F,
    0.125F, 0.25F, 0.25F, 0.125F, 0.25F, 0.25F,
};
constexpr std::array<float, 12> kJointLower = {
    -1.0472F, -1.5708F, -2.7227F, -1.0472F, -1.5708F, -2.7227F,
    -1.0472F, -0.5236F, -2.7227F, -1.0472F, -0.5236F, -2.7227F,
};
constexpr std::array<float, 12> kJointUpper = {
    1.0472F, 3.4907F, -0.83776F, 1.0472F, 3.4907F, -0.83776F,
    1.0472F, 4.5379F, -0.83776F, 1.0472F, 4.5379F, -0.83776F,
};

uint32_t Crc32Core(const uint32_t *data, uint32_t length) {
  uint32_t crc = 0xFFFFFFFFU;
  constexpr uint32_t polynomial = 0x04c11db7U;
  for (uint32_t i = 0; i < length; ++i) {
    uint32_t bit = 1U << 31U;
    for (uint32_t j = 0; j < 32; ++j) {
      crc = (crc & 0x80000000U) ? (crc << 1U) ^ polynomial : crc << 1U;
      if (data[i] & bit)
        crc ^= polynomial;
      bit >>= 1U;
    }
  }
  return crc;
}

bool IsFinite(const LowState &state) {
  const auto &imu = state.imu_state();
  for (float value : imu.quaternion())
    if (!std::isfinite(value))
      return false;
  for (float value : imu.gyroscope())
    if (!std::isfinite(value))
      return false;
  for (std::size_t i = 0; i < kJointNames.size(); ++i) {
    const auto &motor = state.motor_state()[i];
    if (!std::isfinite(motor.q()) || !std::isfinite(motor.dq()) ||
        !std::isfinite(motor.tau_est()))
      return false;
  }
  return true;
}

struct StateRanges {
  std::array<float, 12> position_min{};
  std::array<float, 12> position_max{};
  std::array<float, 12> velocity_min{};
  std::array<float, 12> velocity_max{};
  std::array<float, 12> torque_min{};
  std::array<float, 12> torque_max{};
  std::array<float, 3> gyroscope_min{};
  std::array<float, 3> gyroscope_max{};
  std::array<float, 3> rpy_min{};
  std::array<float, 3> rpy_max{};
  float temperature_min = std::numeric_limits<float>::infinity();
  float temperature_max = -std::numeric_limits<float>::infinity();
  double quaternion_norm_min = std::numeric_limits<double>::infinity();
  double quaternion_norm_max = -std::numeric_limits<double>::infinity();
  bool initialized = false;

  StateRanges() {
    position_min.fill(std::numeric_limits<float>::infinity());
    position_max.fill(-std::numeric_limits<float>::infinity());
    velocity_min.fill(std::numeric_limits<float>::infinity());
    velocity_max.fill(-std::numeric_limits<float>::infinity());
    torque_min.fill(std::numeric_limits<float>::infinity());
    torque_max.fill(-std::numeric_limits<float>::infinity());
    gyroscope_min.fill(std::numeric_limits<float>::infinity());
    gyroscope_max.fill(-std::numeric_limits<float>::infinity());
    rpy_min.fill(std::numeric_limits<float>::infinity());
    rpy_max.fill(-std::numeric_limits<float>::infinity());
  }
};

class StateReceiver {
public:
  void OnState(const void *message) {
    LowState candidate = *static_cast<const LowState *>(message);
    received_.fetch_add(1, std::memory_order_relaxed);
    const uint32_t calculated =
        Crc32Core(reinterpret_cast<const uint32_t *>(&candidate),
                  static_cast<uint32_t>((sizeof(LowState) >> 2U) - 1U));
    if (candidate.crc() != calculated) {
      crc_errors_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    if (!IsFinite(candidate)) {
      invalid_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = std::move(candidate);
      UpdateRanges(state_);
      ++sequence_;
    }
    condition_.notify_one();
  }

  bool WaitForFirst(std::chrono::seconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this] { return sequence_ > 0; });
  }

  std::pair<LowState, uint64_t> Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {state_, sequence_};
  }

  uint64_t Received() const {
    return received_.load(std::memory_order_relaxed);
  }
  uint64_t CrcErrors() const {
    return crc_errors_.load(std::memory_order_relaxed);
  }
  uint64_t Invalid() const { return invalid_.load(std::memory_order_relaxed); }

  StateRanges Ranges() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ranges_;
  }

private:
  void UpdateRanges(const LowState &state) {
    const auto &imu = state.imu_state();
    for (std::size_t i = 0; i < 3; ++i) {
      ranges_.gyroscope_min[i] =
          std::min(ranges_.gyroscope_min[i], imu.gyroscope()[i]);
      ranges_.gyroscope_max[i] =
          std::max(ranges_.gyroscope_max[i], imu.gyroscope()[i]);
      ranges_.rpy_min[i] = std::min(ranges_.rpy_min[i], imu.rpy()[i]);
      ranges_.rpy_max[i] = std::max(ranges_.rpy_max[i], imu.rpy()[i]);
    }
    const double quaternion_norm = std::sqrt(
        std::inner_product(imu.quaternion().begin(), imu.quaternion().end(),
                           imu.quaternion().begin(), 0.0));
    ranges_.quaternion_norm_min =
        std::min(ranges_.quaternion_norm_min, quaternion_norm);
    ranges_.quaternion_norm_max =
        std::max(ranges_.quaternion_norm_max, quaternion_norm);
    const float temperature = static_cast<float>(imu.temperature());
    ranges_.temperature_min = std::min(ranges_.temperature_min, temperature);
    ranges_.temperature_max = std::max(ranges_.temperature_max, temperature);
    for (std::size_t i = 0; i < 12; ++i) {
      const auto &motor = state.motor_state()[i];
      ranges_.position_min[i] = std::min(ranges_.position_min[i], motor.q());
      ranges_.position_max[i] = std::max(ranges_.position_max[i], motor.q());
      ranges_.velocity_min[i] = std::min(ranges_.velocity_min[i], motor.dq());
      ranges_.velocity_max[i] = std::max(ranges_.velocity_max[i], motor.dq());
      ranges_.torque_min[i] = std::min(ranges_.torque_min[i], motor.tau_est());
      ranges_.torque_max[i] = std::max(ranges_.torque_max[i], motor.tau_est());
    }
    ranges_.initialized = true;
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  LowState state_{};
  StateRanges ranges_;
  uint64_t sequence_ = 0;
  std::atomic<uint64_t> received_{0};
  std::atomic<uint64_t> crc_errors_{0};
  std::atomic<uint64_t> invalid_{0};
};

class Policy {
public:
  explicit Policy(const std::filesystem::path &model_path)
      : environment_(ORT_LOGGING_LEVEL_WARNING, "go2_shadow"),
        session_(nullptr) {
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetInterOpNumThreads(1);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = Ort::Session(environment_, model_path.c_str(), options);

    input_shape_ =
        session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    output_shape_ =
        session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (input_shape_ != std::vector<int64_t>({1, 45}) ||
        output_shape_ != std::vector<int64_t>({1, 12})) {
      throw std::runtime_error(
          "unexpected policy shape; expected [1,45] -> [1,12]");
    }
    auto input_name = session_.GetInputNameAllocated(0, allocator_);
    auto output_name = session_.GetOutputNameAllocated(0, allocator_);
    input_name_ = input_name.get();
    output_name_ = output_name.get();
  }

  std::array<float, 12> Infer(std::array<float, 45> &observation) {
    auto memory =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto tensor = Ort::Value::CreateTensor<float>(
        memory, observation.data(), observation.size(), input_shape_.data(),
        input_shape_.size());
    const char *input_names[] = {input_name_.c_str()};
    const char *output_names[] = {output_name_.c_str()};
    auto output = session_.Run(Ort::RunOptions{nullptr}, input_names, &tensor,
                               1, output_names, 1);
    const float *values = output.front().GetTensorData<float>();
    std::array<float, 12> action{};
    std::copy_n(values, action.size(), action.begin());
    for (float value : action) {
      if (!std::isfinite(value))
        throw std::runtime_error("policy returned NaN or infinity");
    }
    return action;
  }

private:
  Ort::Env environment_;
  Ort::Session session_;
  Ort::AllocatorWithDefaultOptions allocator_;
  std::string input_name_;
  std::string output_name_;
  std::vector<int64_t> input_shape_;
  std::vector<int64_t> output_shape_;
};

std::array<float, 3> ProjectedGravity(const std::array<float, 4> &quaternion) {
  const double norm = std::sqrt(std::inner_product(
      quaternion.begin(), quaternion.end(), quaternion.begin(), 0.0));
  if (!std::isfinite(norm) || norm < 1e-8) {
    throw std::runtime_error("invalid IMU quaternion");
  }
  const double w = quaternion[0] / norm;
  const double x = quaternion[1] / norm;
  const double y = quaternion[2] / norm;
  const double z = quaternion[3] / norm;
  return {
      static_cast<float>(2.0 * (w * y - x * z)),
      static_cast<float>(-2.0 * (w * x + y * z)),
      static_cast<float>(-(1.0 - 2.0 * (x * x + y * y))),
  };
}

std::array<float, 45>
BuildObservation(const LowState &state, const std::array<float, 3> &command,
                 const std::array<float, 12> &previous_action) {
  std::array<float, 45> observation{};
  const auto &imu = state.imu_state();
  for (std::size_t i = 0; i < 3; ++i)
    observation[i] = imu.gyroscope()[i] * 0.25F;
  const auto gravity = ProjectedGravity(imu.quaternion());
  std::copy(gravity.begin(), gravity.end(), observation.begin() + 3);
  std::copy(command.begin(), command.end(), observation.begin() + 6);
  for (std::size_t i = 0; i < 12; ++i) {
    observation[9 + i] = state.motor_state()[i].q() - kDefaultPosition[i];
    observation[21 + i] = state.motor_state()[i].dq() * 0.05F;
    observation[33 + i] = previous_action[i];
  }
  for (float &value : observation)
    value = std::clamp(value, -100.0F, 100.0F);
  return observation;
}

double Percentile(std::vector<double> values, double quantile) {
  std::sort(values.begin(), values.end());
  const auto index = std::min(
      values.size() - 1,
      static_cast<std::size_t>(std::ceil(quantile * values.size()) - 1));
  return values[index];
}

std::array<float, 12> ActionsToTargets(const std::array<float, 12> &action,
                                       std::size_t *clamped_count = nullptr) {
  std::array<float, 12> targets{};
  std::size_t clamped = 0;
  for (std::size_t i = 0; i < targets.size(); ++i) {
    const float raw = kDefaultPosition[i] + action[i] * kActionScale[i];
    targets[i] = std::clamp(raw, kJointLower[i], kJointUpper[i]);
    if (targets[i] != raw)
      ++clamped;
  }
  if (clamped_count != nullptr)
    *clamped_count = clamped;
  return targets;
}

template <typename Container>
void PrintValues(const std::string &name, const Container &values) {
  std::cout << name << '=';
  for (const auto &value : values)
    std::cout << value << ' ';
  std::cout << '\n';
}

void PrintRanges(const std::string &name, const std::array<float, 12> &minimum,
                 const std::array<float, 12> &maximum) {
  std::cout << name << '=';
  for (std::size_t i = 0; i < minimum.size(); ++i) {
    std::cout << kJointNames[i] << "[" << minimum[i] << ',' << maximum[i]
              << "] ";
  }
  std::cout << '\n';
}

void PrintRanges(const std::string &name, const std::array<float, 3> &minimum,
                 const std::array<float, 3> &maximum) {
  std::cout << name << "=" << '[' << minimum[0] << ',' << maximum[0] << "] ["
            << minimum[1] << ',' << maximum[1] << "] [" << minimum[2] << ','
            << maximum[2] << "]\n";
}

} // namespace

int main(int argc, char **argv) try {
  if (argc < 3 || argc == 5 || argc == 6 || argc > 7) {
    throw std::runtime_error("usage: go2_shadow_inference INTERFACE MODEL.onnx "
                             "[SECONDS] [VX VY YAW]");
  }
  const std::string interface = argv[1];
  const std::filesystem::path model_path = argv[2];
  const double duration_seconds = argc >= 4 ? std::stod(argv[3]) : 3.0;
  const std::array<float, 3> command =
      argc == 7 ? std::array<float, 3>{std::stof(argv[4]), std::stof(argv[5]),
                                       std::stof(argv[6])}
                : std::array<float, 3>{0.0F, 0.0F, 0.0F};
  if (!std::filesystem::is_regular_file(model_path) ||
      !std::isfinite(duration_seconds) || duration_seconds <= 0.0 ||
      duration_seconds > 60.0) {
    throw std::runtime_error("model must exist and SECONDS must be in (0, 60]");
  }

  Policy policy(model_path);
  std::array<float, 45> warmup_observation{};
  warmup_observation[5] = -1.0F;
  for (int i = 0; i < 100; ++i)
    policy.Infer(warmup_observation);

  std::cout << "mode=READ_ONLY_SHADOW publisher_created=false "
               "control_commands_sent=0\n";
  std::cout << "interface=" << interface << " topic=rt/lowstate model="
            << model_path << '\n';
  std::cout << "waiting_for_lowstate=true timeout_s=5\n" << std::flush;

  StateReceiver receiver;
  unitree::robot::ChannelFactory::Instance()->Init(0, interface);
  unitree::robot::ChannelSubscriber<LowState> subscriber("rt/lowstate");
  subscriber.InitChannel(
      [&receiver](const void *message) { receiver.OnState(message); }, 1);

  if (!receiver.WaitForFirst(std::chrono::seconds(5))) {
    throw std::runtime_error("no CRC-valid rt/lowstate received in 5 seconds "
                             "(check DDS interface/firmware)");
  }

  const uint64_t messages_at_start = receiver.Received();
  const auto start = Clock::now();
  const auto end = start + std::chrono::duration<double>(duration_seconds);
  auto next_run = start;
  uint64_t last_sequence = 0;
  uint64_t stale_policy_frames = 0;
  std::array<float, 12> previous_action{};
  std::array<float, 12> last_action{};
  std::vector<double> latency_ms;
  double maximum_absolute_action = 0.0;
  double maximum_action_delta = 0.0;
  double maximum_target_delta = 0.0;
  bool have_previous_action = false;
  std::array<float, 12> previous_targets{};

  while (Clock::now() < end) {
    const auto [state, sequence] = receiver.Snapshot();
    if (sequence == last_sequence)
      ++stale_policy_frames;
    last_sequence = sequence;
    auto observation = BuildObservation(state, command, previous_action);
    const auto inference_start = Clock::now();
    last_action = policy.Infer(observation);
    latency_ms.push_back(std::chrono::duration<double, std::milli>(
                             Clock::now() - inference_start)
                             .count());
    for (float value : last_action) {
      maximum_absolute_action = std::max(maximum_absolute_action,
                                         std::abs(static_cast<double>(value)));
    }
    const auto current_targets = ActionsToTargets(last_action);
    if (have_previous_action) {
      for (std::size_t i = 0; i < last_action.size(); ++i) {
        maximum_action_delta = std::max(
            maximum_action_delta,
            std::abs(static_cast<double>(last_action[i] - previous_action[i])));
        maximum_target_delta =
            std::max(maximum_target_delta,
                     std::abs(static_cast<double>(current_targets[i] -
                                                  previous_targets[i])));
      }
    }
    previous_targets = current_targets;
    have_previous_action = true;
    previous_action = last_action;
    next_run += std::chrono::milliseconds(20);
    std::this_thread::sleep_until(next_run);
  }

  const auto [last_state, final_sequence] = receiver.Snapshot();
  const auto state_ranges = receiver.Ranges();
  const double elapsed =
      std::chrono::duration<double>(Clock::now() - start).count();
  const uint64_t messages = receiver.Received() - messages_at_start;
  const double mean_latency =
      std::accumulate(latency_ms.begin(), latency_ms.end(), 0.0) /
      latency_ms.size();
  std::size_t clamped_targets = 0;
  const auto targets = ActionsToTargets(last_action, &clamped_targets);

  const auto &imu = last_state.imu_state();
  const double quaternion_norm = std::sqrt(
      std::inner_product(imu.quaternion().begin(), imu.quaternion().end(),
                         imu.quaternion().begin(), 0.0));
  std::array<float, 12> joint_position{};
  std::array<float, 12> joint_velocity{};
  std::array<float, 12> joint_torque{};
  for (std::size_t i = 0; i < 12; ++i) {
    joint_position[i] = last_state.motor_state()[i].q();
    joint_velocity[i] = last_state.motor_state()[i].dq();
    joint_torque[i] = last_state.motor_state()[i].tau_est();
  }

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "result=PASS policy_frames=" << latency_ms.size()
            << " lowstate_messages=" << messages
            << " lowstate_hz=" << messages / elapsed
            << " stale_policy_frames=" << stale_policy_frames << '\n';
  std::cout << "crc_errors=" << receiver.CrcErrors()
            << " invalid_state_frames=" << receiver.Invalid()
            << " final_sequence=" << final_sequence << '\n';
  std::cout << "inference_mean_ms=" << mean_latency
            << " inference_p95_ms=" << Percentile(latency_ms, 0.95)
            << " inference_max_ms="
            << *std::max_element(latency_ms.begin(), latency_ms.end()) << '\n';
  std::cout << "quaternion_norm=" << quaternion_norm
            << " imu_temperature_raw=" << static_cast<int>(imu.temperature())
            << " max_abs_action=" << maximum_absolute_action
            << " clamped_targets=" << clamped_targets
            << " action_delta_max=" << maximum_action_delta
            << " target_delta_max=" << maximum_target_delta << '\n';
  std::cout << "state_ranges_initialized=" << state_ranges.initialized
            << " quaternion_norm_range=[" << state_ranges.quaternion_norm_min
            << ',' << state_ranges.quaternion_norm_max
            << "] temperature_raw_range=[" << state_ranges.temperature_min
            << ',' << state_ranges.temperature_max << "]\n";
  PrintRanges("imu_gyroscope_range_rad_s", state_ranges.gyroscope_min,
              state_ranges.gyroscope_max);
  PrintRanges("imu_rpy_range_rad", state_ranges.rpy_min, state_ranges.rpy_max);
  PrintRanges("joint_position_range_rad", state_ranges.position_min,
              state_ranges.position_max);
  PrintRanges("joint_velocity_range_rad_s", state_ranges.velocity_min,
              state_ranges.velocity_max);
  PrintRanges("joint_torque_range_nm", state_ranges.torque_min,
              state_ranges.torque_max);
  PrintValues("imu_quaternion_wxyz", imu.quaternion());
  PrintValues("imu_rpy_rad", imu.rpy());
  PrintValues("imu_gyroscope_rad_s", imu.gyroscope());
  PrintValues("joint_position_rad", joint_position);
  PrintValues("joint_velocity_rad_s", joint_velocity);
  PrintValues("joint_torque_nm", joint_torque);
  PrintValues("shadow_action", last_action);
  PrintValues("shadow_joint_targets_rad", targets);
  PrintValues("command_vx_vy_yaw", command);
  std::cout << "mode=READ_ONLY_SHADOW publisher_created=false "
               "control_commands_sent=0\n";
  return 0;
} catch (const std::exception &error) {
  std::cerr << "ERROR: " << error.what() << '\n';
  return 1;
}
