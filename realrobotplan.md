# 双 Sharpa Wave 实机接入：最小可行计划

更新时间：2026-09-22

## 结论

当前项目可以使用已安装的 Sharpa SDK 控制两只真实 Sharpa Wave 手，不需要重构 ROS package，也不需要复制 SDK 到 ROS 目录。

当前环境已确认兼容：

- ROS 2 Jazzy 使用 Python 3.12.3。
- `/opt/sharpa-wave-sdk` 是 SDK 5.0.10/build 5.0.10.6。
- SDK 提供 Python 3.12 native extension。
- 设置 SDK 路径后，`python3` 可以成功 `import sharpa`。

运行硬件节点的 terminal 需要设置：

```bash
export PYTHONPATH="/opt/sharpa-wave-sdk/python${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="/opt/sharpa-wave-sdk/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

不创建新的 `.sh` 文件。每个新 terminal 需要重新执行这两个 `export`；`import` 检查命令只用于确认环境，不必每次执行。

## 项目现状

当前 package 已经包含：

- `dual_sharpa_wave/hand_node.py`：统一 ROS hand node。
- `dual_sharpa_wave/mock_hand.py`：离线 mock backend。
- `dual_sharpa_wave/sharpa_sdk_hand.py`：Sharpa SDK hardware adapter。
- `launch/dual_sharpa.launch.py`：通过 `backend:=mock` 或 `backend:=sharpa_sdk` 切换 mock/真实硬件。
- `config/dual_sharpa_hardware.yaml`：硬件配置模板。

左右手是两个独立 node：

```text
/sharpa/left_hand/hand_node
/sharpa/right_hand/hand_node
```

两侧各有独立的 `joint_command` 和 `joint_states` topic，ROS command 使用 radians。当前 package 控制的是 Sharpa 双手；CRX/FANUC 机械臂仍是独立工作范围。

## `sharpa_test` 的作用

`/home/msc-crx/sharpa_test/minimal_test.py` 和 `sharpa_wave_example.py` 可以作为 SDK 原生 API 参考，但不应直接作为双手 ROS 启动程序，因为它们按 discovery 顺序自动选择设备，也可能执行全零或循环动作。

本项目 adapter 使用配置中的明确 serial，因此实机应使用本项目的 hardware launch。

## 已完成的设备检查

在用户授权下，我执行了只读 SDK discovery 和状态读取：

- SDK native module 初始化成功。
- UDP discovery 服务成功启动（port 54321）。
- 两只手都能 ping 通：`192.168.10.10` 和 `192.168.10.20`。
- 左手：`Wave-L-01`，serial `CD52943BCD53`，IP `192.168.10.10`。
- 右手：`Wave-R-01`，serial `C956943BC957`，IP `192.168.10.20`。
- 两只手 firmware 都是 `3.0.10`，当前 control mode/source 都是 `POSITION` / `SDK`。
- 右手曾 `connect()` 成功并由 `get_states()` 读取成功，之后已断开；本次 discovery 也同时发现了左手。
- 没有调用 `start()`、`set_control_mode()`、`set_joint_position()`、`stop()` 或其他运动 API。

两个 serial 现在可以填入 hardware config；不需要修改 ROS 代码。

随后实际启动 hardware launch 已成功：左右两个 node 都完成 SDK 连接、firmware check，并打印 `left/right hand ready: backend=sharpa_sdk, ... 22 joints in radians`。启动后出现的 `Command timeout` warning 是因为尚未发布第一笔 command；按当前 node 设计，尚无 command 时只告警，不执行 hardware timeout stop。

用户随后确认左右两侧 `/sharpa/*_hand/joint_states` 都能持续看到 22 维数值；因此 ROS/SDK 实机接入和 feedback 阶段已完成。运动 command 仍保持为人工触发，不由本计划自动发送。

用户随后确认可以通过 `gui_control.py` 控制真实双手；因此 GUI → ROS command topic → hardware node → Sharpa SDK → 实机动作的完整链路已验证。

用户随后确认 `script/sharpa_wave_example.py` 的 sample-style 双手运动也能正常完成。

## 最短实施步骤

### 1. Mock 验证

```bash
cd ~/ws_fanuc/dual_sharpa_wave_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select dual_sharpa_wave
source install/setup.bash
python3 -m pytest -q test/test_mock_hand.py test/test_joint_validation.py \
  test/test_control_model.py test/test_sdk_hand.py test/test_hand_node.py
```

本次核心 mock/adapter 测试结果为 **89 passed**。完整测试在当前执行环境中受到 DDS UDP 和图形显示权限影响。

### 2. 检查 SDK 环境

```bash
source /opt/ros/jazzy/setup.bash
source <同一构建位置>/install/setup.bash
export PYTHONPATH="/opt/sharpa-wave-sdk/python${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="/opt/sharpa-wave-sdk/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
python3 -c 'import sharpa; print(sharpa.__file__)'
```

如果输出位于 `/opt/sharpa-wave-sdk/python`，环境正确。不需要使用 SDK virtual environment。

### 3. hardware config

已经把本次 SDK discovery 确认的 serial 写入 `config/dual_sharpa_hardware.yaml`：

```yaml
left:  CD52943BCD53   # 192.168.10.10
right: C956943BC957   # 192.168.10.20
```

如果换电脑或换手，只需要修改这两个 serial；serial 必须分别对应左右手且不能相同。

### 4. 启动真实双手并查看 feedback

```bash
ros2 launch dual_sharpa_wave dual_sharpa.launch.py \
  backend:=sharpa_sdk use_rviz:=false
```

另一个 terminal 查看：

```bash
ros2 topic echo /sharpa/left_hand/joint_states sensor_msgs/msg/JointState
ros2 topic echo /sharpa/right_hand/joint_states sensor_msgs/msg/JointState
```

先确认两侧都有持续的 22 维 feedback。

### 5. 最小动作测试

先只发布一侧、一个关节、很小的 radians 位移；观察对应的 `joint_states` 是否变化，再测试另一侧。确认方向和角度后，才使用现有 GUI、sine 或 step 工具。

当前真实硬件配置将 `command_timeout_sec` 设为 `0.0`。motion script 结束后 SDK session 会保持连接；关闭 launch/node 时才执行 SDK stop 和 disconnect。

## 文件变更范围

本计划不要求改变当前项目结构：

- 不添加 `.sh` 文件；
- 不添加新的 ROS node；
- 不复制 SDK；
- 不改变现有 topic、joint order 或 mock backend；
- 不修改 FANUC/CRX package。

新增的 [realrobotplan.md](realrobotplan.md) 只是调查和操作说明。
