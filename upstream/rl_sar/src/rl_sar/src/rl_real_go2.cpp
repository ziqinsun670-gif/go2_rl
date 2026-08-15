/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_go2.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace {

std::string DescribeUnitreeJoyKeys(uint16_t keys) {
  xKeySwitchUnion joy{};
  joy.value = keys;
  std::vector<std::string> names;

  if (joy.components.R1)
    names.emplace_back("R1");
  if (joy.components.L1)
    names.emplace_back("L1");
  if (joy.components.start)
    names.emplace_back("start");
  if (joy.components.select)
    names.emplace_back("select");
  if (joy.components.R2)
    names.emplace_back("R2");
  if (joy.components.L2)
    names.emplace_back("L2");
  if (joy.components.F1)
    names.emplace_back("F1");
  if (joy.components.F2)
    names.emplace_back("F2");
  if (joy.components.A)
    names.emplace_back("A");
  if (joy.components.B)
    names.emplace_back("B");
  if (joy.components.X)
    names.emplace_back("X");
  if (joy.components.Y)
    names.emplace_back("Y");
  if (joy.components.up)
    names.emplace_back("up");
  if (joy.components.right)
    names.emplace_back("right");
  if (joy.components.down)
    names.emplace_back("down");
  if (joy.components.left)
    names.emplace_back("left");

  if (names.empty()) {
    return "none";
  }

  std::ostringstream oss;
  for (size_t i = 0; i < names.size(); ++i) {
    if (i > 0) {
      oss << '+';
    }
    oss << names[i];
  }
  return oss.str();
}

} // namespace

RL_Real::RL_Real(int argc, char **argv) {
  bool wheel_mode = (argc > 2 && std::string(argv[2]) == "wheel");

#if defined(USE_ROS1) && defined(USE_ROS)
  ros::NodeHandle nh;
  this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 10, &RL_Real::CmdvelCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
  ros2_node = std::make_shared<rclcpp::Node>("rl_real_node");
  this->cmd_vel_subscriber =
      ros2_node->create_subscription<geometry_msgs::msg::Twist>(
          "/cmd_vel", rclcpp::SystemDefaultsQoS(),
          [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->CmdvelCallback(msg);
          });
#endif

  // read params from yaml
  this->ang_vel_axis = "body";
  this->robot_name = wheel_mode ? "go2w" : "go2";
  this->ReadYaml(this->robot_name, "base.yaml");

  // auto load FSM by robot_name
  if (FSMManager::GetInstance().IsTypeSupported(this->robot_name)) {
    auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
    if (fsm_ptr) {
      this->fsm = *fsm_ptr;
    }
  } else {
    std::cout << LOGGER::ERROR
              << "[FSM] No FSM registered for robot: " << this->robot_name
              << std::endl;
  }

  // init robot
  this->InitLowCmd();
  this->InitJointNum(this->params.Get<int>("num_of_dofs"));
  this->InitOutputs();
  this->InitControl();
  this->InitRealTelemetryLog();
  // create lowcmd publisher
  this->lowcmd_publisher.reset(
      new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
  this->lowcmd_publisher->InitChannel();
  // create lowstate subscriber
  this->lowstate_subscriber.reset(
      new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
  this->lowstate_subscriber->InitChannel(
      std::bind(&RL_Real::LowStateMessageHandler, this, std::placeholders::_1),
      1);
  // create joystick subscriber
  this->joystick_subscriber.reset(
      new ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(
          TOPIC_JOYSTICK));
  this->joystick_subscriber->InitChannel(
      std::bind(&RL_Real::JoystickHandler, this, std::placeholders::_1), 1);
  // init MotionSwitcherClient
  this->msc.SetTimeout(10.0f);
  this->msc.Init();
  // Shut down motion control-related service
  while (this->QueryMotionStatus()) {
    std::cout << "Try to deactivate the motion control-related service."
              << std::endl;
    int32_t ret = this->msc.ReleaseMode();
    if (ret == 0) {
      std::cout << "ReleaseMode succeeded." << std::endl;
    } else {
      std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
    }
    sleep(1);
  }

  // loop
  this->loop_keyboard = std::make_shared<LoopFunc>(
      "loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
  this->loop_control =
      std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"),
                                 std::bind(&RL_Real::RobotControl, this));
  this->loop_rl = std::make_shared<LoopFunc>(
      "loop_rl",
      this->params.Get<float>("dt") * this->params.Get<int>("decimation"),
      std::bind(&RL_Real::RunModel, this));
  this->loop_keyboard->start();
  this->loop_control->start();
  this->loop_rl->start();

#ifdef PLOT
  this->plot_t = std::vector<int>(this->plot_size, 0);
  this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
  this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
  for (auto &vector : this->plot_real_joint_pos) {
    vector = std::vector<float>(this->plot_size, 0);
  }
  for (auto &vector : this->plot_target_joint_pos) {
    vector = std::vector<float>(this->plot_size, 0);
  }
  this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002,
                                               std::bind(&RL_Real::Plot, this));
  this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
  this->CSVInit(this->robot_name);
#endif
}

RL_Real::~RL_Real() {
  this->loop_keyboard->shutdown();
  this->loop_control->shutdown();
  this->loop_rl->shutdown();
#ifdef PLOT
  this->loop_plot->shutdown();
#endif
  if (this->real_telemetry_file.is_open()) {
    this->real_telemetry_file.close();
  }
  std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<float> *state) {
  if (this->unitree_joy.components.A)
    this->control.SetGamepad(Input::Gamepad::A);
  if (this->unitree_joy.components.B)
    this->control.SetGamepad(Input::Gamepad::B);
  if (this->unitree_joy.components.X)
    this->control.SetGamepad(Input::Gamepad::X);
  if (this->unitree_joy.components.Y)
    this->control.SetGamepad(Input::Gamepad::Y);
  if (this->unitree_joy.components.L1)
    this->control.SetGamepad(Input::Gamepad::LB);
  if (this->unitree_joy.components.R1)
    this->control.SetGamepad(Input::Gamepad::RB);
  if (this->unitree_joy.components.F1)
    this->control.SetGamepad(Input::Gamepad::LStick);
  if (this->unitree_joy.components.F2)
    this->control.SetGamepad(Input::Gamepad::RStick);
  if (this->unitree_joy.components.up)
    this->control.SetGamepad(Input::Gamepad::DPadUp);
  if (this->unitree_joy.components.down)
    this->control.SetGamepad(Input::Gamepad::DPadDown);
  if (this->unitree_joy.components.left)
    this->control.SetGamepad(Input::Gamepad::DPadLeft);
  if (this->unitree_joy.components.right)
    this->control.SetGamepad(Input::Gamepad::DPadRight);
  if (this->unitree_joy.components.L1 && this->unitree_joy.components.A)
    this->control.SetGamepad(Input::Gamepad::LB_A);
  if (this->unitree_joy.components.L1 && this->unitree_joy.components.B)
    this->control.SetGamepad(Input::Gamepad::LB_B);
  if (this->unitree_joy.components.L1 && this->unitree_joy.components.X)
    this->control.SetGamepad(Input::Gamepad::LB_X);
  if (this->unitree_joy.components.L1 && this->unitree_joy.components.Y)
    this->control.SetGamepad(Input::Gamepad::LB_Y);
  if (this->unitree_joy.components.L1 && this->unitree_joy.components.F1)
    this->control.SetGamepad(Input::Gamepad::LB_LStick);
  if (this->unitree_joy.components.L1 && this->unitree_joy.components.F2)
    this->control.SetGamepad(Input::Gamepad::LB_RStick);
  if (this->unitree_joy.components.L1 && this->unitree_joy.components.up)
    this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
  if (this->unitree_joy.components.L1 && this->unitree_joy.components.down)
    this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
  if (this->unitree_joy.components.L1 && this->unitree_joy.components.left)
    this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
  if (this->unitree_joy.components.L1 && this->unitree_joy.components.right)
    this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
  if (this->unitree_joy.components.R1 && this->unitree_joy.components.A)
    this->control.SetGamepad(Input::Gamepad::RB_A);
  if (this->unitree_joy.components.R1 && this->unitree_joy.components.B)
    this->control.SetGamepad(Input::Gamepad::RB_B);
  if (this->unitree_joy.components.R1 && this->unitree_joy.components.X)
    this->control.SetGamepad(Input::Gamepad::RB_X);
  if (this->unitree_joy.components.R1 && this->unitree_joy.components.Y)
    this->control.SetGamepad(Input::Gamepad::RB_Y);
  if (this->unitree_joy.components.R1 && this->unitree_joy.components.F1)
    this->control.SetGamepad(Input::Gamepad::RB_LStick);
  if (this->unitree_joy.components.R1 && this->unitree_joy.components.F2)
    this->control.SetGamepad(Input::Gamepad::RB_RStick);
  if (this->unitree_joy.components.R1 && this->unitree_joy.components.up)
    this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
  if (this->unitree_joy.components.R1 && this->unitree_joy.components.down)
    this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
  if (this->unitree_joy.components.R1 && this->unitree_joy.components.left)
    this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
  if (this->unitree_joy.components.R1 && this->unitree_joy.components.right)
    this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
  if (this->unitree_joy.components.L1 && this->unitree_joy.components.R1)
    this->control.SetGamepad(Input::Gamepad::LB_RB);

  this->control.x = this->joystick.ly();
  this->control.y = -this->joystick.lx();
  this->control.yaw = -this->joystick.rx();

  state->imu.quaternion[0] =
      this->unitree_low_state.imu_state().quaternion()[0]; // w
  state->imu.quaternion[1] =
      this->unitree_low_state.imu_state().quaternion()[1]; // x
  state->imu.quaternion[2] =
      this->unitree_low_state.imu_state().quaternion()[2]; // y
  state->imu.quaternion[3] =
      this->unitree_low_state.imu_state().quaternion()[3]; // z

  for (int i = 0; i < 3; ++i) {
    state->imu.gyroscope[i] =
        this->unitree_low_state.imu_state().gyroscope()[i];
  }
  for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) {
    state->motor_state.q[i] =
        this->unitree_low_state
            .motor_state()[this->params.Get<std::vector<int>>(
                "joint_mapping")[i]]
            .q();
    state->motor_state.dq[i] =
        this->unitree_low_state
            .motor_state()[this->params.Get<std::vector<int>>(
                "joint_mapping")[i]]
            .dq();
    state->motor_state.tau_est[i] =
        this->unitree_low_state
            .motor_state()[this->params.Get<std::vector<int>>(
                "joint_mapping")[i]]
            .tau_est();
  }
}

void RL_Real::SetCommand(const RobotCommand<float> *command) {
  unitree_go::msg::dds_::LowCmd_ dds_low_command;
  dds_low_command.head()[0] = 0xFE;
  dds_low_command.head()[1] = 0xEF;
  dds_low_command.level_flag() = 0xFF;
  dds_low_command.gpio() = 0;

  for (int i = 0; i < 20; ++i) {
    dds_low_command.motor_cmd()[i].mode() = 0x01;
    dds_low_command.motor_cmd()[i].q() = PosStopF;
    dds_low_command.motor_cmd()[i].kp() = 0;
    dds_low_command.motor_cmd()[i].dq() = VelStopF;
    dds_low_command.motor_cmd()[i].kd() = 0;
    dds_low_command.motor_cmd()[i].tau() = 0;
  }

  for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) {
    dds_low_command
        .motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]]
        .mode() = 0x01;
    dds_low_command
        .motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]]
        .q() = command->motor_command.q[i];
    dds_low_command
        .motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]]
        .dq() = command->motor_command.dq[i];
    dds_low_command
        .motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]]
        .kp() = command->motor_command.kp[i];
    dds_low_command
        .motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]]
        .kd() = command->motor_command.kd[i];
    dds_low_command
        .motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]]
        .tau() = command->motor_command.tau[i];
  }

  dds_low_command.crc() =
      Crc32Core((uint32_t *)&dds_low_command,
                (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
  lowcmd_publisher->Write(dds_low_command);
  this->LogRealTelemetry(command);

#ifdef PLOT
  this->unitree_low_command = dds_low_command;
#endif
}

void RL_Real::RobotControl() {
  this->GetState(&this->robot_state);

  this->TorqueProtect(this->robot_state.motor_state.tau_est);
  this->AttitudeProtect(
      this->robot_state.imu.quaternion,
      this->params.Get<float>("real_hard_pitch_deg", 60.0f),
      this->params.Get<float>("real_hard_roll_deg", 60.0f));

  this->StateController(&this->robot_state, &this->robot_command);

  this->control.ClearInput();

  this->SetCommand(&this->robot_command);
}

void RL_Real::RunModel() {
  const std::string current_state =
      this->fsm.current_state_ ? this->fsm.current_state_->GetStateName() : "";
  if (current_state != "RLFSMStateRLLocomotion") {
    this->real_rl_safe_stand_requested = false;
    this->real_rl_imu_soft_guard_violation_count = 0;
  }

  if (this->rl_init_done) {
    this->episode_length_buf += 1;
    this->obs.ang_vel = this->robot_state.imu.gyroscope;
    this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
#if !defined(USE_CMAKE) && defined(USE_ROS)
    if (this->control.navigation_mode) {
      this->obs.commands = {(float)this->cmd_vel.linear.x,
                            (float)this->cmd_vel.linear.y,
                            (float)this->cmd_vel.angular.z};
    }
#endif
    this->obs.base_quat = this->robot_state.imu.quaternion;
    this->obs.dof_pos = this->robot_state.motor_state.q;
    this->obs.dof_vel = this->robot_state.motor_state.dq;

    this->obs.actions = this->Forward();
    this->ComputeOutput(this->obs.actions, this->output_dof_pos,
                        this->output_dof_vel, this->output_dof_tau);
    this->output_dof_pos =
        clamp(this->output_dof_pos,
              this->params.Get<std::vector<float>>("joint_limit_lower"),
              this->params.Get<std::vector<float>>("joint_limit_upper"));
    {
      std::lock_guard<std::mutex> lock(this->real_raw_policy_target_mutex);
      this->real_raw_policy_target = this->output_dof_pos;
    }

    if (this->protection_stop_requested.load(std::memory_order_acquire)) {
      return;
    }

    if (this->CheckRealRLSafetyGuard()) {
      return;
    }

    if (!this->output_dof_pos.empty()) {
      output_dof_pos_queue.push(this->output_dof_pos);
    }
    if (!this->output_dof_vel.empty()) {
      output_dof_vel_queue.push(this->output_dof_vel);
    }
    if (!this->output_dof_tau.empty()) {
      output_dof_tau_queue.push(this->output_dof_tau);
    }

#ifdef CSV_LOGGER
    std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
    this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos,
                    this->output_dof_pos, this->obs.dof_vel);
#endif
  }
}

void RL_Real::RequestRealRLSafeStand(const std::string &reason) {
  if (this->real_rl_safe_stand_requested) {
    return;
  }
  this->real_rl_safe_stand_requested = true;
  this->recovery_stand_low_gain_requested.store(true,
                                                std::memory_order_release);
  this->control.x = 0.0f;
  this->control.y = 0.0f;
  this->control.yaw = 0.0f;
  this->output_dof_pos_queue.clear();
  this->output_dof_vel_queue.clear();
  this->output_dof_tau_queue.clear();
  std::cout << std::endl
            << LOGGER::WARNING << "Real RL soft guard: " << reason
            << ". Requesting default stand (RLFSMStateGetUp)." << std::endl;
  this->safe_stand_requested.store(true, std::memory_order_release);
}

bool RL_Real::CheckRealRLSafetyGuard() {
  const std::string current_state =
      this->fsm.current_state_ ? this->fsm.current_state_->GetStateName() : "";
  if (current_state != "RLFSMStateRLLocomotion") {
    return false;
  }

  const std::vector<std::string> joint_names = {
      "FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
      "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"};
  const int num = this->params.Get<int>("num_of_dofs", 12);

  const float imu_roll_warning =
      this->params.Get<float>("real_rl_imu_soft_guard_roll_deg", 25.0f);
  const float imu_pitch_warning =
      this->params.Get<float>("real_rl_imu_soft_guard_pitch_deg", 18.0f);
  const int imu_required_frames = std::max(
      1, this->params.Get<int>("real_rl_imu_soft_guard_frames", 3));
  if ((imu_roll_warning > 0.0f || imu_pitch_warning > 0.0f) &&
      this->robot_state.imu.quaternion.size() >= 4) {
    const std::vector<float> euler =
        QuaternionToEuler(this->robot_state.imu.quaternion);
    constexpr float rad_to_deg = 57.29577951308232f;
    const float roll = euler[0] * rad_to_deg;
    const float pitch = euler[1] * rad_to_deg;
    const bool roll_violation =
        imu_roll_warning > 0.0f && std::fabs(roll) > imu_roll_warning;
    const bool pitch_violation =
        imu_pitch_warning > 0.0f && std::fabs(pitch) > imu_pitch_warning;

    if (roll_violation || pitch_violation) {
      this->real_rl_imu_soft_guard_violation_count += 1;
    } else {
      this->real_rl_imu_soft_guard_violation_count = 0;
    }

    if (this->real_rl_imu_soft_guard_violation_count >= imu_required_frames) {
      std::ostringstream reason;
      reason << "imu attitude roll=" << roll << " deg pitch=" << pitch
             << " deg exceeds soft guard roll=" << imu_roll_warning
             << " deg or pitch=" << imu_pitch_warning << " deg for "
             << this->real_rl_imu_soft_guard_violation_count
             << " consecutive policy frames";
      this->RequestRealRLSafeStand(reason.str());
      this->real_rl_imu_soft_guard_violation_count = 0;
      return true;
    }
  } else {
    this->real_rl_imu_soft_guard_violation_count = 0;
  }

  const float torque_warning =
      this->params.Get<float>("real_rl_torque_warning_nm", 16.0f);
  if (torque_warning > 0.0f) {
    for (int i = 0; i < num; ++i) {
      if (i >= static_cast<int>(this->robot_state.motor_state.tau_est.size())) {
        continue;
      }
      const float tau = this->robot_state.motor_state.tau_est[i];
      if (std::fabs(tau) > torque_warning) {
        const std::string joint_name = i < static_cast<int>(joint_names.size())
                                           ? joint_names[i]
                                           : ("joint_" + std::to_string(i));
        std::ostringstream reason;
        reason << "tau_est " << joint_name << "=" << tau << " Nm exceeds "
               << torque_warning << " Nm warning";
        this->RequestRealRLSafeStand(reason.str());
        return true;
      }
    }
  }

  return false;
}

std::vector<float> RL_Real::Forward() {
  std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

  // If model is being reinitialized, return previous actions to avoid blocking
  if (!lock.owns_lock()) {
    std::cout << LOGGER::WARNING
              << "Model is being reinitialized, using previous actions"
              << std::endl;
    return this->obs.actions;
  }

  std::vector<float> clamped_obs = this->ComputeObservation();

  std::vector<float> actions;
  if (!this->params.Get<std::vector<int>>("observations_history").empty()) {
    this->history_obs_buf.insert(clamped_obs);
    this->history_obs = this->history_obs_buf.get_obs_vec(
        this->params.Get<std::vector<int>>("observations_history"));
    actions = this->model->forward({this->history_obs});
  } else {
    actions = this->model->forward({clamped_obs});
  }

  if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() &&
      !this->params.Get<std::vector<float>>("clip_actions_lower").empty()) {
    return clamp(actions,
                 this->params.Get<std::vector<float>>("clip_actions_lower"),
                 this->params.Get<std::vector<float>>("clip_actions_upper"));
  } else {
    return actions;
  }
}

void RL_Real::Plot() {
  this->plot_t.erase(this->plot_t.begin());
  this->plot_t.push_back(this->motiontime);
  plt::cla();
  plt::clf();
  for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) {
    this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
    this->plot_target_joint_pos[i].erase(
        this->plot_target_joint_pos[i].begin());
    this->plot_real_joint_pos[i].push_back(
        this->unitree_low_state.motor_state()[i].q());
    this->plot_target_joint_pos[i].push_back(
        this->unitree_low_command.motor_cmd()[i].q());
    plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
    plt::named_plot("_real_joint_pos", this->plot_t,
                    this->plot_real_joint_pos[i], "r");
    plt::named_plot("_target_joint_pos", this->plot_t,
                    this->plot_target_joint_pos[i], "b");
    plt::xlim(this->plot_t.front(), this->plot_t.back());
  }
  // plt::legend();
  plt::pause(0.0001);
}

uint32_t RL_Real::Crc32Core(uint32_t *ptr, uint32_t len) {
  unsigned int xbit = 0;
  unsigned int data = 0;
  unsigned int CRC32 = 0xFFFFFFFF;
  const unsigned int dwPolynomial = 0x04c11db7;

  for (unsigned int i = 0; i < len; ++i) {
    xbit = 1 << 31;
    data = ptr[i];
    for (unsigned int bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else {
        CRC32 <<= 1;
      }

      if (data & xbit) {
        CRC32 ^= dwPolynomial;
      }
      xbit >>= 1;
    }
  }

  return CRC32;
}

void RL_Real::InitLowCmd() {
  this->unitree_low_command.head()[0] = 0xFE;
  this->unitree_low_command.head()[1] = 0xEF;
  this->unitree_low_command.level_flag() = 0xFF;
  this->unitree_low_command.gpio() = 0;

  for (int i = 0; i < 20; ++i) {
    this->unitree_low_command.motor_cmd()[i].mode() =
        (0x01); // motor switch to servo (PMSM) mode
    this->unitree_low_command.motor_cmd()[i].q() = (PosStopF);
    this->unitree_low_command.motor_cmd()[i].kp() = (0);
    this->unitree_low_command.motor_cmd()[i].dq() = (VelStopF);
    this->unitree_low_command.motor_cmd()[i].kd() = (0);
    this->unitree_low_command.motor_cmd()[i].tau() = (0);
  }
}

int RL_Real::QueryMotionStatus() {
  std::string robotForm, motionName;
  int motionStatus;
  int32_t ret = this->msc.CheckMode(robotForm, motionName);
  if (ret == 0) {
    std::cout << "CheckMode succeeded." << std::endl;
  } else {
    std::cout << "CheckMode failed. Error code: " << ret << std::endl;
  }
  if (motionName.empty()) {
    std::cout << "The motion control-related service is deactivated."
              << std::endl;
    motionStatus = 0;
  } else {
    std::string serviceName = QueryServiceName(robotForm, motionName);
    std::cout << "Service: " << serviceName << " is activate" << std::endl;
    motionStatus = 1;
  }
  return motionStatus;
}

std::string RL_Real::QueryServiceName(std::string form, std::string name) {
  if (form == "0") {
    if (name == "normal")
      return "sport_mode";
    if (name == "ai")
      return "ai_sport";
    if (name == "advanced")
      return "advanced_sport";
  } else {
    if (name == "ai-w")
      return "wheeled_sport(go2W)";
    if (name == "normal-w")
      return "wheeled_sport(b2W)";
  }
  return "";
}

void RL_Real::LowStateMessageHandler(const void *message) {
  this->unitree_low_state = *(unitree_go::msg::dds_::LowState_ *)message;
}

void RL_Real::JoystickHandler(const void *message) {
  joystick = *(unitree_go::msg::dds_::WirelessController_ *)message;
  const uint16_t keys = static_cast<uint16_t>(joystick.keys());
  if (keys != this->real_last_logged_joystick_keys) {
    const std::string state_name =
        this->fsm.current_state_ ? this->fsm.current_state_->GetStateName()
                                 : "UNKNOWN";
    std::ostringstream key_hex;
    key_hex << "0x" << std::hex << std::setw(4) << std::setfill('0') << keys;
    std::cout << std::endl
              << LOGGER::INFO << "Joystick keys: "
              << DescribeUnitreeJoyKeys(keys) << " raw=" << key_hex.str()
              << " motiontime=" << this->motiontime
              << " state=" << state_name << std::endl;
    this->real_last_logged_joystick_keys = keys;
  }
  this->unitree_joy.value = keys;
}

void RL_Real::InitRealTelemetryLog() {
  this->real_telemetry_enabled =
      this->params.Get<bool>("real_telemetry_log_enabled", true);
  if (!this->real_telemetry_enabled) {
    return;
  }

  std::filesystem::create_directories("logs");
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::tm tm_snapshot{};
  localtime_r(&now_c, &tm_snapshot);

  std::ostringstream filename;
  filename << "logs/real_q_target_tau_"
           << std::put_time(&tm_snapshot, "%Y%m%d_%H%M%S") << ".csv";
  this->real_telemetry_filename = filename.str();
  this->real_telemetry_file.open(this->real_telemetry_filename);
  if (!this->real_telemetry_file.is_open()) {
    std::cout << LOGGER::WARNING << "Failed to open real telemetry log: "
              << this->real_telemetry_filename << std::endl;
    this->real_telemetry_enabled = false;
    return;
  }

  const std::vector<std::string> joint_names = {
      "FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
      "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"};
  this->real_telemetry_file
      << "motiontime,state,command_x,command_y,command_yaw";
  this->real_telemetry_file << ",imu_roll_rad,imu_pitch_rad,imu_yaw_rad";
  this->real_telemetry_file << ",imu_roll_deg,imu_pitch_deg,imu_yaw_deg";
  for (const auto &joint_name : joint_names)
    this->real_telemetry_file << ",q_" << joint_name;
  for (const auto &joint_name : joint_names)
    this->real_telemetry_file << ",dq_" << joint_name;
  for (const auto &joint_name : joint_names)
    this->real_telemetry_file << ",tau_est_" << joint_name;
  for (const auto &joint_name : joint_names)
    this->real_telemetry_file << ",q_target_" << joint_name;
  for (const auto &joint_name : joint_names)
    this->real_telemetry_file << ",raw_policy_q_target_" << joint_name;
  for (const auto &joint_name : joint_names)
    this->real_telemetry_file << ",dq_target_" << joint_name;
  for (const auto &joint_name : joint_names)
    this->real_telemetry_file << ",kp_" << joint_name;
  for (const auto &joint_name : joint_names)
    this->real_telemetry_file << ",kd_" << joint_name;
  for (const auto &joint_name : joint_names)
    this->real_telemetry_file << ",tau_cmd_" << joint_name;
  this->real_telemetry_file << '\n';
  this->real_telemetry_file.flush();

  std::cout << LOGGER::INFO
            << "Real telemetry log: " << this->real_telemetry_filename
            << std::endl;
}

void RL_Real::LogRealTelemetry(const RobotCommand<float> *command) {
  if (!this->real_telemetry_enabled || !this->real_telemetry_file.is_open() ||
      command == nullptr) {
    return;
  }

  const int decimation =
      std::max(1, this->params.Get<int>("real_telemetry_log_decimation", 4));
  if ((this->real_telemetry_counter++ % decimation) != 0) {
    return;
  }

  const std::string state_name = this->fsm.current_state_
                                     ? this->fsm.current_state_->GetStateName()
                                     : "UNKNOWN";
  this->real_telemetry_file << this->motiontime << ',' << state_name << ','
                            << this->control.x << ',' << this->control.y << ','
                            << this->control.yaw;

  std::vector<float> imu_rpy = {0.0f, 0.0f, 0.0f};
  if (this->robot_state.imu.quaternion.size() >= 4) {
    imu_rpy = QuaternionToEuler(this->robot_state.imu.quaternion);
  }
  constexpr float rad_to_deg = 57.29577951308232f;
  this->real_telemetry_file << ',' << imu_rpy[0] << ',' << imu_rpy[1] << ','
                            << imu_rpy[2] << ',' << imu_rpy[0] * rad_to_deg
                            << ',' << imu_rpy[1] * rad_to_deg << ','
                            << imu_rpy[2] * rad_to_deg;

  const int num = this->params.Get<int>("num_of_dofs", 12);
  auto write_values = [this, num](const std::vector<float> &values) {
    for (int i = 0; i < num; ++i) {
      this->real_telemetry_file << ',';
      if (i < static_cast<int>(values.size())) {
        this->real_telemetry_file << values[i];
      }
    }
  };

  write_values(this->robot_state.motor_state.q);
  write_values(this->robot_state.motor_state.dq);
  write_values(this->robot_state.motor_state.tau_est);
  write_values(command->motor_command.q);
  std::vector<float> raw_policy_target;
  if (state_name == "RLFSMStateRLLocomotion") {
    std::lock_guard<std::mutex> lock(this->real_raw_policy_target_mutex);
    raw_policy_target = this->real_raw_policy_target;
  }
  write_values(raw_policy_target);
  write_values(command->motor_command.dq);
  write_values(command->motor_command.kp);
  write_values(command->motor_command.kd);
  write_values(command->motor_command.tau);
  this->real_telemetry_file << '\n';
  this->real_telemetry_file.flush();
}

#if !defined(USE_CMAKE) && defined(USE_ROS)
void RL_Real::CmdvelCallback(
#if defined(USE_ROS1) && defined(USE_ROS)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2) && defined(USE_ROS)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
) {
  this->cmd_vel = *msg;
}
#endif

#if defined(USE_ROS1) && defined(USE_ROS)
void signalHandler(int signum) {
  ros::shutdown();
  exit(0);
}
#elif defined(USE_CMAKE) || !defined(USE_ROS)
// Signal handler for CMAKE mode
volatile sig_atomic_t g_shutdown_requested = 0;
void signalHandler(int signum) {
  std::cout << LOGGER::INFO << "Received signal " << signum
            << ", shutting down..." << std::endl;
  g_shutdown_requested = 1;
}
#endif

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cout << LOGGER::ERROR << "Usage: " << argv[0]
              << " networkInterface [wheel]" << std::endl;
    throw std::runtime_error("Invalid arguments");
  }
  ChannelFactory::Instance()->Init(0, argv[1]);

#if defined(USE_ROS1) && defined(USE_ROS)
  signal(SIGINT, signalHandler);
  ros::init(argc, argv, "rl_sar");
  RL_Real rl_sar(argc, argv);
  ros::spin();
#elif defined(USE_ROS2) && defined(USE_ROS)
  rclcpp::init(argc, argv);
  auto rl_sar = std::make_shared<RL_Real>(argc, argv);
  rclcpp::spin(rl_sar->ros2_node);
  rclcpp::shutdown();
#elif defined(USE_CMAKE) || !defined(USE_ROS)
  signal(SIGINT, signalHandler);
  RL_Real rl_sar(argc, argv);
  while (!g_shutdown_requested) {
    sleep(1);
  }
  std::cout << LOGGER::INFO << "Exiting..." << std::endl;
#endif

  return 0;
}
