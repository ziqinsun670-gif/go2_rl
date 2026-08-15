# Unitree GO2 轻量 RL 训练、CPU 推理与实机部署

本项目用于 Unitree GO2 的轻量化 RL locomotion 策略整理、CPU 推理验证、
`rt/lowstate` 只读影子推理，以及基于 `rl_sar + Unitree SDK2` 的低层实机部署。

当前设备按无 GPU 部署准备：ONNX Runtime CPU 负责推理，实机控制程序只针对
GO2 编译。实机安全检查和完整现场流程见
[`README_REAL_ROBOT_PRECHECK.md`](README_REAL_ROBOT_PRECHECK.md)。

## 目录说明

```text
go2_rl/
├── models/go2_robot_lab/             # 已发布的 GO2 ONNX/TorchScript 策略
├── runtime/                          # Python CPU 推理入口
├── cpp/                              # 只读 shadow inference / C++ ONNX 验证
├── tools/                            # 构建、离线验证、网络与只读实机测试脚本
├── train/                            # GPU 工作站训练/导出脚本
├── third_party/onnxruntime/          # ONNX Runtime CPU 运行时
└── upstream/rl_sar/                  # 实机部署端源码，本项目内已加 GO2 安全补丁
```

本机 `/home/hyq/unitree_go2/unitree_sdk2` 是 Unitree 官方 SDK2 目录，用于
对照和依赖来源说明；它不是本项目自研代码，也不作为本仓库核心源码维护。

## 固定版本

- 训练上游：`fan-ziqi/robot_lab` 2.3.2，提交
  `500399ed75f510aeaff28705a8ce736c514dbec3`
- 部署上游：`fan-ziqi/rl_sar`，提交
  `96cec1a886671b1583aa992bd97bd0438af04020`
- Unitree SDK2：`65e19c625b76b697f94e0b953e296ecc0ce7b4a1`
- ONNX Runtime：1.22.0，CPU x86_64
- 训练任务：`RobotLab-Isaac-Velocity-Rough-Unitree-Go2-v0`

`upstream/robot_lab/source/robot_lab/data/Robots/` 是 Robot Lab 大型仿真
mesh/资产目录，仓库发布时默认排除。GO2 CPU 推理、只读验证和实机部署不依赖
该大型资产目录；需要完整 Isaac/Robot Lab 训练资产时，按固定提交重新同步。

## 策略输入输出

当前策略位于 `models/go2_robot_lab/`，网络结构为：

```text
45 -> 512 -> 256 -> 128 -> 12
```

输入拼接顺序：

```text
body angular velocity * 0.25                       3
projected gravity                                 3
command vx, vy, yaw                               3
joint position - default position                12
joint velocity * 0.05                            12
previous action                                  12
                                                   --
                                                   45
```

输出为 12 个关节位置目标偏移，关节顺序为：

```text
FR_hip FR_thigh FR_calf
FL_hip FL_thigh FL_calf
RR_hip RR_thigh RR_calf
RL_hip RL_thigh RL_calf
```

策略周期为 `0.005 * 4 = 20 ms`。动作换算：

```text
hip   q_target = 0.0  + action * 0.125
thigh q_target = 0.8  + action * 0.25
calf  q_target = -1.5 + action * 0.25
```

这是速度跟踪型 locomotion policy，不是专用静态站立控制器，也不是“走到某个
距离”的位置任务控制器。

## 离线 CPU 验证

```bash
cd /home/hyq/unitree_go2/go2_rl
./tools/verify_offline.sh
```

只测一次 Python ONNX 推理：

```bash
PYTHONNOUSERSITE=1 ./.venv/bin/python runtime/go2_policy.py \
  --backend onnx --command 0.3 0 0
```

## GPU 工作站训练

训练需要 Linux、NVIDIA GPU、驱动、Isaac Sim 4.5/5.0/5.1、Isaac Lab
2.3.2 和兼容 Python 环境。

安装 Robot Lab：

```bash
cd /home/hyq/unitree_go2/go2_rl/upstream/robot_lab
python3 -m pip install -e source/robot_lab
```

检查训练环境并训练：

```bash
cd /home/hyq/unitree_go2/go2_rl
ISAACLAB_PYTHON=python3 ./train/check_training_env.sh
ISAACLAB_PYTHON=python3 NUM_ENVS=4096 ./train/train_go2.sh
```

显存不足时降低 `NUM_ENVS`。

导出并发布模型：

```bash
ISAACLAB_PYTHON=python3 ./train/export_go2.sh \
  upstream/robot_lab/logs/rsl_rl/unitree_go2_rough/RUN/model_20000.pt

./train/publish_policy.sh \
  upstream/robot_lab/logs/rsl_rl/unitree_go2_rough/RUN/exported

./tools/verify_offline.sh
./tools/build_go2_deploy.sh
```

## 构建 GO2 实机部署程序

```bash
cd /home/hyq/unitree_go2/go2_rl
./tools/build_go2_deploy.sh
```

输出：

```text
upstream/rl_sar/cmake_build/bin/rl_real_go2
```

也可以直接重新编译：

```bash
cmake --build /home/hyq/unitree_go2/go2_rl/upstream/rl_sar/cmake_build \
  --target rl_real_go2 -j2
```

## GO2 测试完整命令

先建立日志目录：

```bash
cd /home/hyq/unitree_go2/go2_rl
mkdir -p logs
```

### 1. 检查网络

该步骤只检查网口、IP 和 ping，不发送控制命令。

```bash
./tools/check_connection.sh enp99s0 192.168.123.161
```

PC 有线口应配置为 `192.168.123.x/24`，例如 `192.168.123.222/24`。

### 2. 只读 `rt/lowstate` + 影子推理

该步骤只订阅状态并执行本机 ONNX 推理，不创建 `rt/lowcmd` 发布器，不释放
原厂运动服务，不发送电机命令。

零速度 30 秒：

```bash
./tools/verify_robot_readonly.sh enp99s0 30 0 0 0 \
  | tee "logs/readonly_zero_$(date +%Y%m%d_%H%M%S).log"
```

影子速度命令 `vx=0.2`，仍然不发送给机器人：

```bash
./tools/verify_robot_readonly.sh enp99s0 30 0.2 0 0 \
  | tee "logs/readonly_vx02_$(date +%Y%m%d_%H%M%S).log"
```

合格条件：

```text
result=PASS
lowstate_hz 接近 500
crc_errors=0
stale_policy_frames=0
publisher_created=false
control_commands_sent=0
```

### 3. 只读关节顺序触碰

每次运行时只轻微扰动对应那条腿，检查哪条腿的 `joint_position_range`、
`joint_velocity_range` 变化最大。

```bash
./tools/verify_robot_readonly.sh enp99s0 5 0 0 0 \
  | tee "logs/touch_FR_$(date +%Y%m%d_%H%M%S).log"

./tools/verify_robot_readonly.sh enp99s0 5 0 0 0 \
  | tee "logs/touch_FL_$(date +%Y%m%d_%H%M%S).log"

./tools/verify_robot_readonly.sh enp99s0 5 0 0 0 \
  | tee "logs/touch_RR_$(date +%Y%m%d_%H%M%S).log"

./tools/verify_robot_readonly.sh enp99s0 5 0 0 0 \
  | tee "logs/touch_RL_$(date +%Y%m%d_%H%M%S).log"
```

### 4. 低层实机程序

`rl_real_go2` 会释放原厂运动服务、创建 `rt/lowcmd` 发布器并持续发送低层
电机命令。只允许在吊装、四足离地、遥控急停和物理断电都准备好的情况下运行。

运行前确认没有旧控制进程：

```bash
pgrep -af 'rl_real_go2|go2_low_level|go2_stand_example' || true
```

启动：

```bash
./upstream/rl_sar/cmake_build/bin/rl_real_go2 enp99s0 2>&1 \
  | tee "logs/real_standhold_lowcmd_$(date +%Y%m%d_%H%M%S).log"
```

推荐按键流程：

```text
0       插值到默认站姿，完成后自动进入 StandHold
1       从 StandHold 进入 RL Locomotion，开始 ONNX 推理并发送 12 关节目标
Space   速度命令清零
9       插值回程序启动时姿态
P       强制 Passive/卸力，异常立刻按
Ctrl+C  退出程序；正常先回 StandHold 或按 P，再 Ctrl+C
```

正常日志应出现：

```text
Keyboard input: Num0 ...
Switch from RLFSMStatePassive to RLFSMStateGetUp
Getting up completed
Switch from RLFSMStateGetUp to RLFSMStateStandHold
Entered stand-hold mode...

Keyboard input: Num1 ...
Switch from RLFSMStateStandHold to RLFSMStateRLLocomotion
```

如果 RL soft guard 触发，正常流程应为：

```text
Real RL soft guard: ...
Switch from RLFSMStateRLLocomotion to RLFSMStateGetUp
GetUp entered from RL soft guard; using low recovery gains.
Getting up completed
Switch from RLFSMStateGetUp to RLFSMStateStandHold
```

soft guard 后不应直接进入 Passive；除非日志中有 `Keyboard input: P`、
`Joystick keys: L1+X`、硬姿态/硬扭矩保护或 GetUp 姿态中止。

## 实机保护补丁

当前 GO2 部署程序包含以下保护：

- RL 接管过渡。
- 关节目标 slew-rate 限幅。
- 基于实际 lowcmd 目标的连续帧 tracking soft guard。
- `tau_est` 软预警，默认 `16 Nm`。
- RL 中 IMU soft guard，默认 roll `25°`、pitch `18°` 连续 3 帧。
- soft guard 后低增益 GetUp 恢复。
- GetUp 中大姿态直接 Passive。
- 全状态硬扭矩/硬姿态保护。
- soft guard 后 `GetUp -> StandHold`，默认站姿低增益保持，避免恢复后卸力塌下。
- 键盘和手柄输入事件日志。
- CSV 记录 `q/dq/tau_est/q_target/raw_policy_q_target/kp/kd/tau_cmd/IMU`。

主要参数位于：

```text
upstream/rl_sar/policy/go2/base.yaml
upstream/rl_sar/policy/go2/robot_lab/config.yaml
```

关键参数：

```yaml
real_rl_transition_seconds: 1.0
real_rl_target_slew_rate_rad_s: 2.0
real_rl_tracking_error_guard_rad: 0.8
real_rl_tracking_error_guard_frames: 3
real_rl_torque_warning_nm: 16.0
real_rl_imu_soft_guard_roll_deg: 25.0
real_rl_imu_soft_guard_pitch_deg: 18.0
real_rl_imu_soft_guard_frames: 3
real_getup_attitude_passive_roll_deg: 45.0
real_getup_attitude_passive_pitch_deg: 30.0
real_hard_roll_deg: 60.0
real_hard_pitch_deg: 60.0
real_stand_hold_kp: 20.0
real_stand_hold_kd: 0.5
real_telemetry_log_decimation: 4
```

## 查看最新实机日志

终端日志：

```bash
tail -n 120 "$(ls -t logs/real_*lowcmd_*.log | head -n 1)"
```

筛选状态切换、保护和输入事件：

```bash
grep -E 'Keyboard input|Joystick keys|Switch from|Real RL soft guard|stand-hold|GetUp entered|GetUp attitude|Torque\(|Roll exceeds|Pitch exceeds|Entered passive' \
  "$(ls -t logs/real_*lowcmd_*.log | head -n 1)"
```

查看最新 CSV：

```bash
csv="$(ls -t logs/real_q_target_tau_*.csv | head -n 1)"
head -n 1 "$csv"
tail -n 5 "$csv"
```

按状态统计行数：

```bash
awk -F, 'NR>1{count[$2]++} END{for (s in count) print s,count[s]}' \
  "$(ls -t logs/real_q_target_tau_*.csv | head -n 1)"
```

## 安全限制

- 不要用拔网线作为急停。
- 不要在地面直接测试新策略行走。
- 只读影子推理合格，不等于可以发送 `lowcmd`。
- `P` 是 Passive/卸力，异常急停可用；正常测试结束优先回 StandHold 或默认站姿。
- 当前策略没有“前进 0.5 m”距离闭环。若要固定距离动作，需要额外 supervisor
  用外部里程计/视觉/时间估计管理速度命令和停止条件。
