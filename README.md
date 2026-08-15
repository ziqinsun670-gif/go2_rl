# Unitree GO2 轻量 RL 训练与 CPU 部署

训练，TorchScript 负责模型来源核验，ONNX Runtime 负责无 GPU 的 CPU
推理，`rl_sar + Unitree SDK2` 负责后续实机接入。

当前机器没有 NVIDIA GPU。已完成离线模型推理、原生 C++ 推理和实机部署
程序的编译；尚未进行 Isaac Sim/MuJoCo 闭环或实机电机控制测试。

实机前置检查、只读影子推理和关节顺序确认记录见
[`README_REAL_ROBOT_PRECHECK.md`](README_REAL_ROBOT_PRECHECK.md)。

## 固定版本

- 训练：`fan-ziqi/robot_lab` 2.3.2，提交
  `500399ed75f510aeaff28705a8ce736c514dbec3`
- 部署：`fan-ziqi/rl_sar`，提交
  `96cec1a886671b1583aa992bd97bd0438af04020`
- Unitree SDK2：`65e19c625b76b697f94e0b953e296ecc0ce7b4a1`
- ONNX Runtime：1.22.0，CPU x86_64
- 训练任务：`RobotLab-Isaac-Velocity-Rough-Unitree-Go2-v0`

说明：本机 `/home/hyq/unitree_go2/unitree_sdk2` 是 Unitree 官方 SDK2
目录，用于对照/依赖来源；它不是本项目自研代码，也不作为本仓库的核心源码维护。
另外，`upstream/robot_lab/source/robot_lab/data/Robots/` 是上游 Robot Lab
大型仿真 mesh/资产目录，本仓库上传时默认排除；需要完整训练/Isaac 仿真资产时，
按上面的固定 Robot Lab 提交重新同步即可。当前 GO2 CPU 推理、只读验证和实机
部署代码不依赖该大型资产目录。

项目中的真实预训练策略位于 `models/go2_robot_lab/`。网络为
`45 -> 512 -> 256 -> 128 -> 12`，共 189,324 个参数；ONNX 文件约
741 KiB。输入按以下顺序拼接：

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

关节顺序为 `FR, FL, RR, RL`，每条腿内部为 `hip, thigh, calf`，与 GO2
SDK 的 0 至 11 号电机一致。策略周期为 `0.005 * 4 = 20 ms`。

## 当前机器上的离线验证

```bash
cd /home/hyq/unitree_go2/go2_rl
./tools/verify_offline.sh
```

该命令运行 Python 观测/后处理测试，并编译、运行原生 C++ ONNX 基准。
只测一次 Python ONNX 推理可用：

```bash
PYTHONNOUSERSITE=1 ./.venv/bin/python runtime/go2_policy.py \
  --backend onnx --command 0.3 0 0
```

## GPU 工作站训练

训练源码已经下载到 `upstream/robot_lab`。工作站需要 Linux、NVIDIA GPU、
驱动、Isaac Sim 4.5/5.0/5.1、Isaac Lab 2.3.2 和兼容的 Python 环境。
先用该环境安装 RobotLab：

```bash
cd /home/hyq/unitree_go2/go2_rl/upstream/robot_lab
python3 -m pip install -e source/robot_lab
```

然后从项目目录检查环境并训练：

```bash
cd /home/hyq/unitree_go2/go2_rl
ISAACLAB_PYTHON=python3 ./train/check_training_env.sh
ISAACLAB_PYTHON=python3 NUM_ENVS=4096 ./train/train_go2.sh
```

显存不足时降低 `NUM_ENVS`。默认网络、奖励、域随机化和 20,000 次迭代来自
固定的 RobotLab GO2 rough 配置；本项目只把仿真 PD 刚度显式设为 20.0，
与 `rl_sar` 实机配置对齐。

训练结束后导出并发布模型：

```bash
ISAACLAB_PYTHON=python3 ./train/export_go2.sh \
  upstream/robot_lab/logs/rsl_rl/unitree_go2_rough/RUN/model_20000.pt

./train/publish_policy.sh \
  upstream/robot_lab/logs/rsl_rl/unitree_go2_rough/RUN/exported

./tools/verify_offline.sh
./tools/build_go2_deploy.sh
```

`export_go2.sh` 在 Isaac Sim 中建立一个环境，加载 checkpoint，导出 JIT 和
ONNX 后自动退出。`publish_policy.sh` 先校验 ONNX 输入输出，再更新 CPU 与
部署目录里的模型。

## GO2 部署程序

部署端只编译 GO2，避免依赖 A1、Lite3、D1 等无关 SDK：

```bash
cd /home/hyq/unitree_go2/go2_rl
./tools/build_go2_deploy.sh
```

输出为：

```text
upstream/rl_sar/cmake_build/bin/rl_real_go2
```

本地部署补丁包括：ONNX CPU 后端、GO2 关节目标限位、RL 接管过渡、
关节目标 slew-rate 限幅、基于实际 lowcmd 目标的连续帧 tracking soft guard、
tau/IMU 非时间 soft guard、soft guard 后低增益 GetUp 恢复、
GetUp 大姿态直接 Passive、q/target/raw-policy-target/tau/IMU CSV 日志、
全状态 60 度横滚/俯仰保护、全状态实测扭矩保护，以及锁存到进程重启的
Passive 停机请求。

## 连接机器人前后

连接前只运行只读网络检查：

```bash
./tools/check_connection.sh enp99s0 192.168.123.161
```

网络通过后，可运行只订阅 `rt/lowstate` 的实机影子推理：

```bash
./tools/verify_robot_readonly.sh enp99s0 3
```

该程序读取 IMU 和 12 个关节状态，以 50 Hz 执行 ONNX 策略并打印推理结果，
不会创建 `rt/lowcmd` 发布器、释放原厂运动服务或发送电机命令。它只验证
DDS 状态链路和实机观测上的前向推理，不代表策略已经具备落地控制条件。

做 30 秒稳定性记录时，可将终端输出保存下来：

```bash
mkdir -p logs
./tools/verify_robot_readonly.sh enp99s0 30 \
  | tee "logs/readonly_$(date +%Y%m%d_%H%M%S).log"
```

测试影子速度命令（仍不会发送给机器人）时，追加 `VX VY YAW`：

```bash
./tools/verify_robot_readonly.sh enp99s0 30 0.2 0 0
```

输出中的 `state_ranges`、`action_delta_max` 和 `target_delta_max` 用于检查
温度、关节/IMU 范围、通信期间的状态连续性和策略输出连续性。

PC 有线口需要类似 `192.168.123.222/24` 的静态地址。不要用
`rl_real_go2` 测试连通性：该程序会关闭原厂运动服务、创建低层控制发布器并
持续发送电机命令。

首次上机前必须先完成仿真闭环，核对固件/SDK、关节方向、IMU 四元数顺序、
20 ms 策略周期、KP/KD、限位和遥控急停；机器人应吊装，四足离地，现场人员
保持安全距离。当前预训练权重尚未在这台 GO2 上验证，不能直接落地运行。
