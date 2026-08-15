/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GO2_FSM_HPP
#define GO2_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"

#include <cmath>

namespace go2_fsm {

class RLFSMStatePassive : public RLFSMState {
public:
  RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

  void Enter() override {
    std::cout << LOGGER::NOTE
              << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) "
                 "to switch to RLFSMStateGetUp."
              << std::endl;
  }

  void Run() override {
    for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i) {
      // fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
      fsm_command->motor_command.dq[i] = 0;
      fsm_command->motor_command.kp[i] = 0;
      fsm_command->motor_command.kd[i] = 8;
      fsm_command->motor_command.tau[i] = 0;
    }
  }

  void Exit() override {}

  std::string CheckChange() override {
    if (rl.protection_stop_requested.load(std::memory_order_acquire)) {
      return state_name_;
    }
    if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
        rl.control.current_gamepad == Input::Gamepad::A) {
      return "RLFSMStateGetUp";
    }
    return state_name_;
  }
};

class RLFSMStateGetUp : public RLFSMState {
public:
  RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

  float percent_pre_getup = 0.0f;
  float percent_getup = 0.0f;
  std::vector<float> pre_running_pos = {0.00, 1.36, -2.65, 0.00, 1.36, -2.65,
                                        0.00, 1.36, -2.65, 0.00, 1.36, -2.65,
                                        0.00, 0.00, 0.00,  0.00};
  bool stand_from_passive = true;
  bool low_gain_recovery = false;
  bool getup_attitude_abort = false;
  bool getup_attitude_abort_logged = false;

  void Enter() override {
    rl.safe_stand_requested.store(false, std::memory_order_release);
    low_gain_recovery =
        rl.recovery_stand_low_gain_requested.exchange(false,
                                                      std::memory_order_acq_rel);
    getup_attitude_abort = false;
    getup_attitude_abort_logged = false;
    percent_pre_getup = 0.0f;
    percent_getup = 0.0f;
    if (rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive") {
      stand_from_passive = true;
    } else {
      stand_from_passive = false;
    }
    rl.now_state = *fsm_state;
    rl.start_state = rl.now_state;

    if (low_gain_recovery) {
      std::cout << LOGGER::WARNING
                << "GetUp entered from RL soft guard; using low recovery gains."
                << std::endl;
    }
  }

  void Run() override {
    if (CheckGetUpAttitudeAbort()) {
      SendPassiveCommand();
      return;
    }

    if (stand_from_passive) {

      if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q,
                      pre_running_pos, 1.0f, "Pre Getting up", true))
        return;
      if (Interpolate(percent_getup, pre_running_pos,
                      rl.params.Get<std::vector<float>>("default_dof_pos"),
                      2.0f, "Getting up", true))
        return;
    } else {
      const bool use_fixed_gains = !low_gain_recovery;
      if (Interpolate(percent_getup, rl.now_state.motor_state.q,
                      rl.params.Get<std::vector<float>>("default_dof_pos"),
                      1.0f, "Getting up", use_fixed_gains))
        return;
    }
  }

  void Exit() override {}

  std::string CheckChange() override {
    if (rl.control.current_keyboard == Input::Keyboard::P ||
        rl.control.current_gamepad == Input::Gamepad::LB_X) {
      return "RLFSMStatePassive";
    }
    if (getup_attitude_abort) {
      return "RLFSMStatePassive";
    }
    if (percent_getup >= 1.0f) {
      if (rl.control.current_keyboard == Input::Keyboard::Num1 ||
          rl.control.current_gamepad == Input::Gamepad::RB_DPadUp) {
        return "RLFSMStateRLLocomotion";
      } else if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
                 rl.control.current_gamepad == Input::Gamepad::B) {
        return "RLFSMStateGetDown";
      }
      return "RLFSMStateStandHold";
    }
    return state_name_;
  }

private:
  bool CheckGetUpAttitudeAbort() {
    const float roll_threshold =
        rl.params.Get<float>("real_getup_attitude_passive_roll_deg", 45.0f);
    const float pitch_threshold =
        rl.params.Get<float>("real_getup_attitude_passive_pitch_deg", 30.0f);
    if (roll_threshold <= 0.0f && pitch_threshold <= 0.0f) {
      return false;
    }
    if (fsm_state == nullptr || fsm_state->imu.quaternion.size() < 4) {
      return false;
    }

    const std::vector<float> euler =
        QuaternionToEuler(fsm_state->imu.quaternion);
    constexpr float rad_to_deg = 57.29577951308232f;
    const float roll = euler[0] * rad_to_deg;
    const float pitch = euler[1] * rad_to_deg;
    const bool roll_violation =
        roll_threshold > 0.0f && std::fabs(roll) > roll_threshold;
    const bool pitch_violation =
        pitch_threshold > 0.0f && std::fabs(pitch) > pitch_threshold;

    if (!roll_violation && !pitch_violation) {
      return false;
    }

    getup_attitude_abort = true;
    if (!getup_attitude_abort_logged) {
      getup_attitude_abort_logged = true;
      std::cout << std::endl
                << LOGGER::WARNING
                << "GetUp attitude guard: roll=" << roll
                << " deg pitch=" << pitch
                << " deg exceeds roll=" << roll_threshold
                << " deg or pitch=" << pitch_threshold
                << " deg. Switching to Passive." << std::endl;
    }
    return true;
  }

  void SendPassiveCommand() {
    for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i) {
      fsm_command->motor_command.dq[i] = 0;
      fsm_command->motor_command.kp[i] = 0;
      fsm_command->motor_command.kd[i] = 8;
      fsm_command->motor_command.tau[i] = 0;
    }
  }
};

class RLFSMStateStandHold : public RLFSMState {
public:
  RLFSMStateStandHold(RL *rl) : RLFSMState(*rl, "RLFSMStateStandHold") {}

  void Enter() override {
    std::cout << LOGGER::NOTE
              << "Entered stand-hold mode. Holding default stance with low "
                 "gains. Press '1' to enter RL, 'P' for Passive."
              << std::endl;
  }

  void Run() override {
    const auto default_pos =
        rl.params.Get<std::vector<float>>("default_dof_pos");
    const float hold_kp = rl.params.Get<float>("real_stand_hold_kp", 20.0f);
    const float hold_kd = rl.params.Get<float>("real_stand_hold_kd", 0.5f);
    const int num = rl.params.Get<int>("num_of_dofs");

    for (int i = 0; i < num; ++i) {
      if (i < static_cast<int>(default_pos.size())) {
        fsm_command->motor_command.q[i] = default_pos[i];
      } else if (fsm_state != nullptr &&
                 i < static_cast<int>(fsm_state->motor_state.q.size())) {
        fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
      }
      fsm_command->motor_command.dq[i] = 0;
      fsm_command->motor_command.kp[i] = hold_kp;
      fsm_command->motor_command.kd[i] = hold_kd;
      fsm_command->motor_command.tau[i] = 0;
    }
  }

  void Exit() override {}

  std::string CheckChange() override {
    if (rl.control.current_keyboard == Input::Keyboard::P ||
        rl.control.current_gamepad == Input::Gamepad::LB_X) {
      return "RLFSMStatePassive";
    } else if (rl.control.current_keyboard == Input::Keyboard::Num1 ||
               rl.control.current_gamepad == Input::Gamepad::RB_DPadUp) {
      return "RLFSMStateRLLocomotion";
    } else if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
               rl.control.current_gamepad == Input::Gamepad::B) {
      return "RLFSMStateGetDown";
    } else if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
               rl.control.current_gamepad == Input::Gamepad::A) {
      return "RLFSMStateGetUp";
    }
    return state_name_;
  }
};

class RLFSMStateGetDown : public RLFSMState {
public:
  RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

  float percent_getdown = 0.0f;

  void Enter() override {
    percent_getdown = 0.0f;
    rl.now_state = *fsm_state;
  }

  void Run() override {
    Interpolate(percent_getdown, rl.now_state.motor_state.q,
                rl.start_state.motor_state.q, 2.0f, "Getting down", true);
  }

  void Exit() override {}

  std::string CheckChange() override {
    if (rl.control.current_keyboard == Input::Keyboard::P ||
        rl.control.current_gamepad == Input::Gamepad::LB_X ||
        percent_getdown >= 1.0f) {
      return "RLFSMStatePassive";
    } else if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
               rl.control.current_gamepad == Input::Gamepad::A) {
      return "RLFSMStateGetUp";
    }
    return state_name_;
  }
};

class RLFSMStateRLLocomotion : public RLFSMState {
public:
  RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}

  float percent_transition = 0.0f;

  void Enter() override {
    percent_transition = 0.0f;
    rl.episode_length_buf = 0;

    // read params from yaml
    rl.config_name = "robot_lab";
    std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
    try {
      rl.InitRL(robot_config_path);
      rl.now_state = *fsm_state;
      rl.output_dof_pos_queue.clear();
      rl.output_dof_vel_queue.clear();
      rl.output_dof_tau_queue.clear();
      ResetRLCommandFilter(rl.now_state.motor_state.q);
    } catch (const std::exception &e) {
      std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what()
                << std::endl;
      rl.rl_init_done = false;
      rl.fsm.RequestStateChange("RLFSMStatePassive");
    }
  }

  void Run() override {
    // position transition from last default_dof_pos to current default_dof_pos
    // if (Interpolate(percent_transition, rl.now_state.motor_state.q,
    // rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy
    // transition", true)) return;

    if (!rl.rl_init_done)
      rl.rl_init_done = true;

    std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller ["
              << rl.config_name << "] x:" << rl.control.x
              << " y:" << rl.control.y << " yaw:" << rl.control.yaw
              << std::flush;
    RLControl();
  }

  void Exit() override { rl.rl_init_done = false; }

  std::string CheckChange() override {
    if (rl.safe_stand_requested.load(std::memory_order_acquire)) {
      return "RLFSMStateGetUp";
    }
    if (rl.control.current_keyboard == Input::Keyboard::P ||
        rl.control.current_gamepad == Input::Gamepad::LB_X) {
      return "RLFSMStatePassive";
    } else if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
               rl.control.current_gamepad == Input::Gamepad::B) {
      return "RLFSMStateGetDown";
    } else if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
               rl.control.current_gamepad == Input::Gamepad::A) {
      return "RLFSMStateGetUp";
    }
    if (rl.control.current_keyboard == Input::Keyboard::Num1 ||
        rl.control.current_gamepad == Input::Gamepad::RB_DPadUp) {
      return "RLFSMStateRLLocomotion";
    }
    return state_name_;
  }
};

} // namespace go2_fsm

class Go2FSMFactory : public FSMFactory {
public:
  Go2FSMFactory(const std::string &initial) : initial_state_(initial) {}
  std::shared_ptr<FSMState>
  CreateState(void *context, const std::string &state_name) override {
    RL *rl = static_cast<RL *>(context);
    if (state_name == "RLFSMStatePassive")
      return std::make_shared<go2_fsm::RLFSMStatePassive>(rl);
    else if (state_name == "RLFSMStateGetUp")
      return std::make_shared<go2_fsm::RLFSMStateGetUp>(rl);
    else if (state_name == "RLFSMStateStandHold")
      return std::make_shared<go2_fsm::RLFSMStateStandHold>(rl);
    else if (state_name == "RLFSMStateGetDown")
      return std::make_shared<go2_fsm::RLFSMStateGetDown>(rl);
    else if (state_name == "RLFSMStateRLLocomotion")
      return std::make_shared<go2_fsm::RLFSMStateRLLocomotion>(rl);
    return nullptr;
  }
  std::string GetType() const override { return "go2"; }
  std::vector<std::string> GetSupportedStates() const override {
    return {"RLFSMStatePassive", "RLFSMStateGetUp", "RLFSMStateStandHold",
            "RLFSMStateGetDown", "RLFSMStateRLLocomotion"};
  }
  std::string GetInitialState() const override { return initial_state_; }

private:
  std::string initial_state_;
};

REGISTER_FSM_FACTORY(Go2FSMFactory, "RLFSMStatePassive")

#endif // GO2_FSM_HPP
