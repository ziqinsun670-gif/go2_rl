# Model provenance

- Robot: Unitree Go2, 12 DoF
- Task: velocity locomotion over rough terrain
- Training framework: `fan-ziqi/robot_lab` (Isaac Lab / RSL-RL)
- Deployment release: `fan-ziqi/rl_sar`
- Source commit: `96cec1a886671b1583aa992bd97bd0438af04020`
- Source file: `policy/go2/robot_lab/policy.pt`
- License: Apache-2.0; see `upstream/rl_sar/LICENSE`
- SHA-256 (TorchScript): `9f14cb95e74ac9e5e30954da0fc0c33eaa41e337852ebed19fcee869279ade0b`
- SHA-256 (ONNX): `f2e149fc1eb3cc2fb65cd1dd9710882152274b7a6ea193af0f30bb374765ecbf`

The actor is a 45 -> 512 -> 256 -> 128 -> 12 MLP with ELU activations and
189,324 parameters. The ONNX file in this directory is a fixed `[1,45]` CPU
export of the same actor and is verified numerically against TorchScript.
