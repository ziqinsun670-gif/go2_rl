# GO2 实机前置检查记录

本文记录本项目在 Unitree GO2 上进入电机控制测试前必须完成的检查项。
当前阶段只允许执行网络检查、`rt/lowstate` 订阅和影子推理；这些测试不创建
`rt/lowcmd` 发布器，不发送电机命令，不释放原厂运动服务。

## 当前结论

- 网络链路已通过：PC `enp99s0` 为 `192.168.123.222/24`，GO2 为
  `192.168.123.161`，ping 0% 丢包，典型 RTT 约 0.3 ms。
- 30 秒只读影子推理已通过：`rt/lowstate` 约 500 Hz，策略 50 Hz，
  CRC 错误 0，状态陈旧帧 0，ONNX CPU 推理 p95 小于 0.2 ms。
- 关节顺序使用 `FR, FL, RR, RL`，每条腿内部为 `hip, thigh, calf`。
- FR、FL、RR 的只读触碰日志已留档并核对通过。
- 若 RL 已现场确认但未生成 `logs/touch_RL_*.log`，建议补录一次作为留档。
- 尚未进行低层电机控制测试；不得直接在地面执行 RL 行走或“前进 0.5 m”。

## 安全边界

只读测试允许机器人在地面保持原厂站立状态时进行，因为程序只订阅状态：

```text
mode=READ_ONLY_SHADOW publisher_created=false control_commands_sent=0
```

任何会发送 `rt/lowcmd` 的程序都不属于只读测试，包括
`upstream/rl_sar/cmake_build/bin/rl_real_go2`。进入这些测试前必须满足：

- 机器人吊装，四足离地。
- 遥控急停已验证有效，现场有物理断电人员。
- 不依赖拔网线作为急停。
- 低层控制程序已有状态新鲜度 watchdog，超时能锁存 Passive。
- 关节目标已有 slew-rate 限幅。
- 速度命令已有缓启动和缓停止。
- 已先完成 Passive、单关节顺序/方向、缓慢站姿、零速 RL 的吊装验证。

## 固定关节顺序

部署和影子推理使用 GO2 SDK 电机数组 0 至 11 号顺序：

```text
0  FR_hip
1  FR_thigh
2  FR_calf
3  FL_hip
4  FL_thigh
5  FL_calf
6  RR_hip
7  RR_thigh
8  RR_calf
9  RL_hip
10 RL_thigh
11 RL_calf
```

策略动作到关节目标的换算：

```text
hip   q_des = 0.0  + action * 0.125
thigh q_des = 0.8  + action * 0.25
calf  q_des = -1.5 + action * 0.25
```

正常站姿下，`hip` 应接近 0，`thigh` 应接近 `+0.8`，`calf` 应接近
`-1.5`。如果日志明显不满足这个形态，不能继续实机动作测试。

## 正负方向判断

以下规则用于只读扰动或吊装小幅单关节命令时核对方向：

```text
hip q 增大：
  左腿 FL/RL 往外侧打开
  右腿 FR/RR 往身体中线方向收

thigh q 增大：
  大腿往机器人后方/尾部方向摆

calf q 增大：
  小腿更伸直，数值从 -1.5 往 -1.3、-1.1 方向走

calf q 减小：
  膝盖更折叠，数值从 -1.5 往 -1.8、-2.0 方向走
```

只读触碰验证时不要大幅掰腿。动作过大时机身和其他腿会耦合，影子策略输出也
会被带到限位附近。

## 只读检查命令

从项目目录执行：

```bash
cd /home/hyq/unitree_go2/go2_rl
mkdir -p logs
```

检查网络：

```bash
./tools/check_connection.sh enp99s0 192.168.123.161
```

30 秒只读状态和影子推理稳定性：

```bash
./tools/verify_robot_readonly.sh enp99s0 30 0 0 0 \
  | tee "logs/readonly_zero_$(date +%Y%m%d_%H%M%S).log"
```

影子速度命令测试，仍然不发送给机器人：

```bash
./tools/verify_robot_readonly.sh enp99s0 30 0.2 0 0 \
  | tee "logs/readonly_vx02_$(date +%Y%m%d_%H%M%S).log"
```

四条腿触碰确认：

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

运行每条腿触碰命令时，只轻微扰动对应那条腿。结束后看 `joint_position_range`
和 `joint_velocity_range` 中哪条腿变化最大。

## 日志判读

查看最新日志：

```bash
tail -n 40 "$(ls -t logs/*.log | head -n 1)"
```

把最新日志按关节映射成表：

```bash
awk 'function load(prefix,arr,line,vals,n,i){sub(prefix,"",line); n=split(line,vals," "); for(i=1;i<=n;i++) arr[i]=vals[i]}
  /^joint_position_rad=/{line=$0; load("joint_position_rad=",q,line)}
  /^shadow_joint_targets_rad=/{line=$0; load("shadow_joint_targets_rad=",t,line)}
  END{
    split("FR_hip FR_thigh FR_calf FL_hip FL_thigh FL_calf RR_hip RR_thigh RR_calf RL_hip RL_thigh RL_calf",n," ");
    print "idx name q_now target delta";
    for(i=1;i<=12;i++) printf "%02d  %-9s %+8.4f %+8.4f %+8.4f\n", i-1,n[i],q[i],t[i],t[i]-q[i]
  }' "$(ls -t logs/*.log | head -n 1)"
```

按腿合计 position、velocity、torque 变化：

```bash
perl -ne '
my %p=("joint_position_range_rad"=>"pos","joint_velocity_range_rad_s"=>"vel","joint_torque_range_nm"=>"tau");
for my $k (keys %p) {
  if (/^$k=(.*)/) {
    my $metric=$p{$k}; my $rest=$1;
    while ($rest =~ /(\S+)\[([-0-9.]+),([-0-9.]+)\]/g) {
      my ($name,$lo,$hi)=($1,$2,$3); my ($leg)=$name=~/^(FR|FL|RR|RL)_/;
      $sum{$metric}{$leg}+=$hi-$lo;
      $max{$metric}{$leg}=($hi-$lo) if !defined($max{$metric}{$leg}) || ($hi-$lo)>$max{$metric}{$leg};
    }
  }
}
END{
for my $metric (qw(pos vel tau)) {
  for my $leg (qw(FR FL RR RL)) {
    printf "%s %s sum=%.6f max_joint=%.6f\n", $metric, $leg, $sum{$metric}{$leg}+0, $max{$metric}{$leg}+0;
  }
}
}' "$(ls -t logs/*.log | head -n 1)"
```

判定标准：

- `result=PASS`。
- `lowstate_hz` 接近 500 Hz。
- `crc_errors=0`。
- `stale_policy_frames=0`。
- `publisher_created=false control_commands_sent=0`。
- 触碰哪条腿，那条腿的 position/velocity 合计变化应最大。
- `clamped_targets` 最好为 0；若非 0，说明扰动过大或策略目标碰限位，不能作为动作测试放行依据。

## 已留档结果

30 秒 `vx=0.2` 只读影子推理：

```text
log: logs/readonly_vx02_20260815_155221.log
result=PASS
policy_frames=1500
lowstate_messages=14993
lowstate_hz=499.763991
stale_policy_frames=0
crc_errors=0
invalid_state_frames=0
inference_mean_ms=0.061309
inference_p95_ms=0.104741
inference_max_ms=0.236371
clamped_targets=0
control_commands_sent=0
```

FR 触碰确认：

```text
log: logs/touch_FR_20260815_161520.log
position_sum: FR 0.193648, FL 0.139104, RR 0.114282, RL 0.080527
velocity_sum: FR 2.247130, FL 1.335033, RR 1.105704, RL 0.883451
torque_sum:   FR 7.446223, FL 6.163957, RR 6.438138, RL 6.805090
结论：FR 主变化最大，FR 映射正确。扰动偏大，clamped_targets=2。
```

FL 触碰确认：

```text
log: logs/touch_FL_20260815_162002.log
position_sum: FL 0.519951, FR 0.067338, RR 0.077102, RL 0.082491
velocity_sum: FL 2.930738, FR 0.986068, RR 0.908388, RL 0.992301
torque_sum:   FL 14.494573, FR 5.667126, RR 6.322691, RL 7.477146
结论：FL 主变化最大，FL 映射正确。
```

RR 触碰确认：

```text
log: logs/touch_RR_20260815_162144.log
position_sum: RR 0.340883, FR 0.053460, FL 0.081614, RL 0.080266
velocity_sum: RR 4.536890, FR 0.926418, FL 1.062399, RL 0.982696
torque_sum:   RR 15.360411, FR 6.650475, FL 6.576260, RL 8.747043
结论：RR 主变化最大，RR 映射正确。扰动偏大，clamped_targets=1。
```

## 进入下一阶段前

下一阶段不是地面行走，而是吊装低层测试。推荐顺序：

1. 吊装并确认四足完全离地。
2. 保留原厂控制，确认遥控急停和物理断电。
3. 运行低层 Passive，确认能安全退出。
4. 极小幅单关节命令，逐个确认 12 个关节顺序和正负方向。
5. 缓慢插值到默认站姿 `0.0, 0.8, -1.5`。
6. 零速度 RL 悬空测试，只观察输出连续性和扭矩。
7. 通过后才考虑地面低速速度跟踪；若要“前进 0.5 m”，还必须先加入距离
   supervisor，因为当前策略只接收速度命令，不接收目标距离。

## 低层实机按键

`rl_real_go2` 会释放原厂运动服务、创建 `rt/lowcmd` 发布器并发送低层命令。
只允许在吊装、急停和物理断电都准备好的情况下运行。

```text
P       强制 Passive，发现异常立刻按
0       12 个关节插值到默认站姿 0.0, 0.8, -1.5
1       进入 RL Locomotion，开始 ONNX 推理并发送 12 关节目标
Space   速度命令清零
9       插值回程序启动时姿态
Ctrl+C  退出程序；正常先 0 回默认站姿，异常先 P 急停
```

## 当前 ONNX 策略能力边界

当前 `policy.onnx` 是速度跟踪型 locomotion policy，不是专用静态站立控制器，
也不是“走到某个距离”的位置任务控制器。

进入 `RLFSMStateRLLocomotion` 后，模型每 20 ms 根据 IMU 角速度、重力方向、
速度命令、12 个关节位置/速度和上一帧动作，输出 12 个关节位置目标偏移：

```text
q_target = default_dof_pos + action * action_scale
```

实际发给电机的是 12 个关节位置目标、`kp/kd` 和零前馈力矩。零速度命令
`vx=0, vy=0, yaw=0` 不等于严格站立；策略仍可能持续产生小步态/摆腿动作。
因此当前阶段只能把它看作“低速步态候选策略”，还不能当作长时间地面静态站立
或稳定行走控制器。

`logs/real_first_lowcmd_20260815_163518.log` 的分析结论：

- `P` 和 `0` 阶段正常，站姿插值完成。
- 按 `1` 后进入 `RLLocomotion`，ONNX 模型加载成功。
- 日志里速度命令为 `x:0.00 y:-0.00 yaw:-0.00`，不是误给速度。
- `RL Controller` 出现 3734 次，按 20 ms 策略周期估算约 74.68 秒，并非
  2 至 3 秒短测。
- 现场观察到关节抖动、轻微打滑和原地旋转，说明当前策略接管不能直接放行
  地面 RL。

根据这次结果，本项目已完成以下实机保护改动；继续测试前请重新编译并确认
运行的是新的 `rl_real_go2`：

- RL 接管过渡：从当前姿态逐步混入策略目标。
- 关节目标 slew-rate 限幅：限制每个策略周期的目标角度变化。
- 正常退出回默认站姿：进入 `RLLocomotion` 后按 `0` 回 `GetUp/default stand`；
  异常急停才按 `P` 进入 Passive。
- 非时间 soft guard：实际 lowcmd 目标 tracking 误差连续超限、RL 姿态连续超限，
  或扭矩预警，先回默认站姿，不等硬保护卸力。
- 全状态硬保护：硬扭矩/60 度姿态保护覆盖 Passive、GetUp 和 RLLocomotion。
- 异常恢复低增益：soft guard 触发后进入 `GetUp` 使用低恢复增益，不再用
  `fixed_kp=80` 硬拉。
- GetUp 姿态保护：`GetUp` 中姿态过大时直接切 Passive，避免继续插值站姿。
- q/target/tau/IMU 日志：记录当前关节、目标关节、估计扭矩、kp/kd、命令值和
  IMU roll/pitch/yaw；同时记录 raw policy target，用于区分 ONNX 原始目标和
  经过接管过渡/slew-rate 限幅后的实际发送目标。

当前默认保护参数位于 `upstream/rl_sar/policy/go2/robot_lab/config.yaml`：

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
real_telemetry_log_decimation: 4
```

下一次运行 `rl_real_go2` 时会额外生成：

```text
logs/real_q_target_tau_YYYYMMDD_HHMMSS.csv
```

该 CSV 用于回放每条腿的 `q`、`dq`、`tau_est`、`q_target`、
`raw_policy_q_target`、`dq_target`、`kp`、`kd`、`tau_cmd` 以及
`imu_roll/pitch/yaw`。其中 `q_target` 是实际 lowcmd 目标，
`raw_policy_q_target` 是 ONNX 策略经过 action scale/joint clamp 后、尚未经过
接管过渡和 slew-rate 限幅的目标。如果再次出现抖动，先分析这个 CSV，再决定
是否继续降低 `real_rl_target_slew_rate_rad_s`、调整
`real_rl_tracking_error_guard_rad` / `real_rl_tracking_error_guard_frames`，
或调整 `real_rl_torque_warning_nm` / IMU guard 阈值。

## 保护版零速度 RL 成功记录

2026-08-15 17:08 至 17:10 的吊装/零速度保护版测试已整理为：

```text
logs/success_rl_zero_cmd_2s_lowcmd_20260815_170834.log
logs/success_rl_zero_cmd_2s_q_target_tau_20260815_170834.csv
```

重点结论：

- `RLFSMStateRLLocomotion` 段为 100 帧 CSV 记录，按 20 ms/帧为 2.00 秒。
- RL 段速度命令全程为 `vx=0, vy=0, yaw=0`。
- 终端日志无 warning/error；该历史成功日志来自旧的 2 秒自动 Passive 版本。
- 现场观察为稳定关节运动；后续版本已删除自动时间限制，改为非时间 soft
  guard。正常退出按 `0` 回默认站姿，异常急停才按 `P`。

当前版本不再提供 `1` 后自动退回时间配置入口。进入 `RLFSMStateRLLocomotion`
后会持续运行，直到：

- 按 `0`：正常切回默认站姿流程。
- soft guard 触发：实际 lowcmd 目标的 tracking 误差连续 3 个 policy 帧超过
  `0.8 rad`、RL 姿态 roll/pitch 连续 3 帧超过 `25°/18°`，或扭矩超过
  `16 Nm` 时，自动请求 `RLFSMStateGetUp` 回默认站姿，并使用低恢复增益。
- `GetUp` 姿态保护触发：roll 超过 `45°` 或 pitch 超过 `30°` 时直接退回
  Passive。
- 按 `P`：异常急停，立即退回 Passive。
- 按 `9`：回到程序启动姿态流程。
- 60° 姿态/硬扭矩保护触发：锁存退回 Passive，需重启程序解除。

修改代码后重新编译：

```bash
./tools/build_go2_deploy.sh
```
