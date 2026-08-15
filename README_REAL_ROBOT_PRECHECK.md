# GO2 实机前置检查与测试流程

本文是 Unitree GO2 实机测试的现场手册。目标是把测试分成清晰等级：
只读状态检查、只读影子推理、吊装低层控制、吊装 RL、最后才是地面低速测试。

## 测试等级

### A. 只读测试

允许机器人在地面保持原厂站立状态时进行。

只读测试包括：

- 网络检查。
- `rt/lowstate` 订阅。
- ONNX shadow inference。
- 轻微触碰腿部验证关节顺序。

只读测试必须满足：

```text
publisher_created=false
control_commands_sent=0
```

### B. 低层控制测试

`rl_real_go2` 会释放原厂运动服务、创建 `rt/lowcmd` 发布器并发送电机命令。
该测试必须吊装，四足完全离地。

必须具备：

- 遥控急停已验证。
- 现场人员能物理断电。
- 不依赖拔网线作为急停。
- 操作员能随时按 `P` 进入 Passive。
- 明确知道 Passive 是卸力，不是站姿保持。

### C. 地面测试

只有在吊装低层控制和吊装 RL 均合格后才允许进入地面测试。当前策略没有
“前进 0.5 m”的距离闭环；如果需要固定距离动作，必须额外实现 supervisor。

## 固定网络配置

默认参数：

```text
PC 网口: enp99s0
PC IP:   192.168.123.222/24
GO2 IP:  192.168.123.161
```

检查命令：

```bash
cd /home/hyq/unitree_go2/go2_rl
mkdir -p logs

./tools/check_connection.sh enp99s0 192.168.123.161
```

合格条件：

```text
网口 UP / LOWER_UP
PC 有 192.168.123.x/24 地址
ping 192.168.123.161 成功
0% packet loss
```

## 关节顺序

部署、影子推理和 CSV 日志统一使用 GO2 SDK 电机数组 0 至 11：

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

正常默认站姿：

```text
hip   ≈  0.0 rad
thigh ≈ +0.8 rad
calf  ≈ -1.5 rad
```

策略动作换算：

```text
hip   q_target = 0.0  + action * 0.125
thigh q_target = 0.8  + action * 0.25
calf  q_target = -1.5 + action * 0.25
```

## 正负方向判断

只读触碰或吊装小幅单关节命令时按以下规则核对：

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

如果顺序或正负方向有任何疑问，停止进入低层控制测试。

## 只读 shadow inference

零速度 30 秒：

```bash
cd /home/hyq/unitree_go2/go2_rl
mkdir -p logs

./tools/verify_robot_readonly.sh enp99s0 30 0 0 0 \
  | tee "logs/readonly_zero_$(date +%Y%m%d_%H%M%S).log"
```

影子速度命令 `vx=0.2`，仍不发送给机器人：

```bash
./tools/verify_robot_readonly.sh enp99s0 30 0.2 0 0 \
  | tee "logs/readonly_vx02_$(date +%Y%m%d_%H%M%S).log"
```

合格条件：

```text
result=PASS
lowstate_hz 接近 500 Hz
policy_frames 符合 duration * 50 Hz
stale_policy_frames=0
crc_errors=0
invalid_state_frames=0
publisher_created=false
control_commands_sent=0
```

推理耗时参考：

```text
inference_mean_ms / p95 / max 应明显小于 20 ms 策略周期
```

## 只读触碰验证

每次只轻微扰动指定腿。不要大幅掰腿；机身耦合会影响其他腿和影子策略输出。

```bash
cd /home/hyq/unitree_go2/go2_rl
mkdir -p logs

./tools/verify_robot_readonly.sh enp99s0 5 0 0 0 \
  | tee "logs/touch_FR_$(date +%Y%m%d_%H%M%S).log"

./tools/verify_robot_readonly.sh enp99s0 5 0 0 0 \
  | tee "logs/touch_FL_$(date +%Y%m%d_%H%M%S).log"

./tools/verify_robot_readonly.sh enp99s0 5 0 0 0 \
  | tee "logs/touch_RR_$(date +%Y%m%d_%H%M%S).log"

./tools/verify_robot_readonly.sh enp99s0 5 0 0 0 \
  | tee "logs/touch_RL_$(date +%Y%m%d_%H%M%S).log"
```

查看最新日志：

```bash
tail -n 40 "$(ls -t logs/touch_*.log | head -n 1)"
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
}' "$(ls -t logs/touch_*.log | head -n 1)"
```

合格条件：

- 触碰哪条腿，哪条腿的 position/velocity 合计变化最大。
- `result=PASS`。
- `crc_errors=0`。
- `stale_policy_frames=0`。
- `clamped_targets=0` 最好；若不为 0，说明扰动过大或策略目标碰限位，该次只作参考。

## 构建低层实机程序

```bash
cd /home/hyq/unitree_go2/go2_rl
./tools/build_go2_deploy.sh
```

输出：

```text
upstream/rl_sar/cmake_build/bin/rl_real_go2
```

## 低层实机按键

```text
0       插值到默认站姿，完成后自动进入 StandHold
1       从 StandHold 进入 RL Locomotion，开始 ONNX 推理并发送 12 关节目标
Space   速度命令清零
9       插值回程序启动时姿态
P       强制 Passive/卸力，异常立刻按
Ctrl+C  退出程序；正常先回 StandHold 或按 P，再 Ctrl+C
```

注意：`P` 是卸力，不是保持站立。正常流程优先让程序停在 StandHold。

## 吊装低层控制测试

运行前：

```bash
cd /home/hyq/unitree_go2/go2_rl
mkdir -p logs
pgrep -af 'rl_real_go2|go2_low_level|go2_stand_example' || true
```

启动：

```bash
./upstream/rl_sar/cmake_build/bin/rl_real_go2 enp99s0 2>&1 \
  | tee "logs/real_standhold_lowcmd_$(date +%Y%m%d_%H%M%S).log"
```

第一阶段只验证默认站姿和 StandHold：

```text
1. 程序启动后保持 Passive。
2. 按 0。
3. 等待 GetUp 插值完成。
4. 确认日志进入 RLFSMStateStandHold。
5. 不按 1，观察 5 至 10 秒。
6. 正常结束：按 P，再 Ctrl+C。
```

合格日志：

```text
Keyboard input: Num0 ...
Switch from RLFSMStatePassive to RLFSMStateGetUp
Getting up completed
Switch from RLFSMStateGetUp to RLFSMStateStandHold
Entered stand-hold mode...
```

合格条件：

- 机器人吊装状态下姿态稳定。
- 关节没有持续发散。
- CSV 中 StandHold 段 `kp` 为 `real_stand_hold_kp`，默认 20。
- 没有硬保护、姿态保护或 DDS 连续异常。

## 吊装 RL 零速度测试

只有 StandHold 合格后才进入该阶段。

流程：

```text
1. 启动 rl_real_go2。
2. 按 0，进入默认站姿并等待 StandHold。
3. 按 Space，确认速度命令清零。
4. 按 1，进入 RL Locomotion。
5. 只观察零速度 RL，发现异常立刻按 P。
6. 如果 soft guard 触发，程序应自动回 GetUp，再进入 StandHold。
7. 正常结束时不要长时间留在 RL；回 StandHold 后再按 P/Ctrl+C。
```

期望日志：

```text
Keyboard input: Num1 ...
Switch from RLFSMStateStandHold to RLFSMStateRLLocomotion
Successfully loaded ONNX model: ...
RL Controller [robot_lab] x:0.00 y:0.00 yaw:0.00
```

soft guard 触发后的期望日志：

```text
Real RL soft guard: ...
Switch from RLFSMStateRLLocomotion to RLFSMStateGetUp
GetUp entered from RL soft guard; using low recovery gains.
Getting up completed
Switch from RLFSMStateGetUp to RLFSMStateStandHold
```

不合格条件：

- 零速度下明显原地旋转、打滑或摆腿发散。
- `tau_est` 接近或超过软阈值。
- `q_target - q` 连续接近阈值。
- roll/pitch 快速增大。
- soft guard 后直接进入 Passive，且日志中没有人工 `P` 或硬保护原因。

## 当前实机保护参数

参数文件：

```text
upstream/rl_sar/policy/go2/base.yaml
upstream/rl_sar/policy/go2/robot_lab/config.yaml
```

默认值：

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

保护行为：

- RL 接管时先从当前姿态过渡。
- 策略目标经过 slew-rate 限幅后才发送。
- tracking guard 检查的是实际 lowcmd 目标与实测 q 的误差。
- `tau_est` 超过软阈值时先回默认站姿，不等待硬扭矩保护。
- soft guard 后 `GetUp` 使用低恢复增益。
- `GetUp` 完成后进入 StandHold，不自动卸力 Passive。
- `P` 或手柄 `L1+X` 仍然可以强制 Passive。

## 实机日志查看

查看最新终端日志：

```bash
tail -n 120 "$(ls -t logs/real_*lowcmd_*.log | head -n 1)"
```

筛选关键事件：

```bash
grep -E 'Keyboard input|Joystick keys|Switch from|Real RL soft guard|stand-hold|GetUp entered|GetUp attitude|Torque\(|Roll exceeds|Pitch exceeds|Entered passive|ddsi_udp_conn_write' \
  "$(ls -t logs/real_*lowcmd_*.log | head -n 1)"
```

查看最新 CSV：

```bash
csv="$(ls -t logs/real_q_target_tau_*.csv | head -n 1)"
head -n 1 "$csv"
tail -n 5 "$csv"
```

按状态统计：

```bash
awk -F, 'NR>1{count[$2]++} END{for (s in count) print s,count[s]}' "$csv"
```

快速找最大扭矩、最大实际目标误差：

```bash
python3 - <<'PY'
import csv, math
from pathlib import Path

p = Path(sorted(Path("logs").glob("real_q_target_tau_*.csv"))[-1])
joints = [
    "FR_hip","FR_thigh","FR_calf","FL_hip","FL_thigh","FL_calf",
    "RR_hip","RR_thigh","RR_calf","RL_hip","RL_thigh","RL_calf",
]

def f(x):
    return None if x == "" else float(x)

best_tau = (0, None, None)
best_err = (0, None, None)
states = {}

with p.open(newline="") as fh:
    for row in csv.DictReader(fh):
        states[row["state"]] = states.get(row["state"], 0) + 1
        for j in joints:
            tau = f(row[f"tau_est_{j}"])
            if tau is not None and abs(tau) > best_tau[0]:
                best_tau = (abs(tau), j, row)
            q = f(row[f"q_{j}"])
            qt = f(row[f"q_target_{j}"])
            if q is not None and qt is not None and abs(qt - q) > best_err[0]:
                best_err = (abs(qt - q), j, row)

print("csv:", p)
print("states:", states)
for name, best in [("max_tau", best_tau), ("max_q_target_error", best_err)]:
    value, joint, row = best
    print(name, value, joint, "motiontime", row["motiontime"], "state", row["state"],
          "roll", row["imu_roll_deg"], "pitch", row["imu_pitch_deg"])
PY
```

## 放行标准

进入下一阶段前必须同时满足：

- 当前阶段日志可解释，无未知状态切换。
- 键盘/手柄事件与现场操作一致。
- `lowstate` 无明显掉帧、CRC 错误或长时间陈旧。
- `tau_est` 未持续逼近软阈值。
- `q_target - q` 未持续逼近阈值。
- roll/pitch 未快速发散。
- soft guard 能回到 `GetUp -> StandHold`。
- 现场人员确认急停和物理断电路径有效。

任何一项不满足，停止继续测试，先分析日志。
