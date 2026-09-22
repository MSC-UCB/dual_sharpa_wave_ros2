# Dual Sharpa Wave ROS 2

ROS 2 Jazzy / Ubuntu 24.04。左右各 22 個關節、兩個獨立 process，使用同一個 `hand_node` executable。第一階段可在完全沒有 Sharpa SDK 與實機的環境運作。

提供 GUI、sine、step 控制工具及可注入 Fake SDK 的 hardware adapter。Mock 與 adapter 離線測試不需要實機；官方 SDK binary、裝置連線與馬達動作尚未驗證。建置與測試結果見 [驗證紀錄](docs/validation.md)。

## 通訊架構

```text
控制程式 → ROS 2 / DDS joint_command → 共用 hand_node（驗證、排序、timeout）
                                   → HandInterface
                                       ├─ MockHand：模擬 actual/target
                                       └─ SharpaSdkHand：SDK adapter（實機待驗證）
控制程式／RViz ← ROS 2 / DDS joint_states ← 共用發布流程 ← backend 目前位置
```

Mock 與 hardware backend 共用 ROS node、topic、QoS、joint order、radians、指令驗證與發布流程。切換只改 launch/backend 與硬體設定，上層控制程式不需修改。Command 成功下發不等於到位；state 必須來自 backend 目前位置，讀取失敗時跳過發布，不拿 target 或舊資料冒充新的 feedback。

| Topic | Type | 用途 |
|---|---|---|
| `/sharpa/left_hand/joint_command` | `sensor_msgs/msg/JointState` | 左手 22 維 radians command |
| `/sharpa/left_hand/joint_states` | `sensor_msgs/msg/JointState` | 左手目前位置 |
| `/sharpa/right_hand/joint_command` | `sensor_msgs/msg/JointState` | 右手 22 維 radians command |
| `/sharpa/right_hand/joint_states` | `sensor_msgs/msg/JointState` | 右手目前位置 |

Nodes：`/sharpa/left_hand/hand_node`、`/sharpa/right_hand/hand_node`。沒有 44 維 public command/state topic。Hand node 使用 relative topic names；控制工具使用上述完整 topics。

四個 topics 共用 QoS：**RELIABLE、VOLATILE、KEEP_LAST、depth=1**。新訂閱者不會收到舊的 command；高頻下只保留最新待處理資料，不保證逐筆執行任意快速發布的軌跡。需要連續追蹤時，上層應持續發布目標。

## 建置與測試

先確認 `printenv ROS_DISTRO`。以下指令以本次確認的 Jazzy 為準，不混用其他 distro。

本 Git repository 本身就是 `ament_python` package，可放到 workspace 的 `src/dual_sharpa_wave`。本機已使用 symlink：

```text
~/ws_fanuc/src/dual_sharpa_wave → /mnt/c/Users/Howard/dual_sharpa_wave_ros2
```

```bash
cd ~/ws_fanuc
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select dual_sharpa_wave
source install/setup.bash
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=171 colcon test --packages-select dual_sharpa_wave --event-handlers console_direct+
colcon test-result --verbose
```

若改在 repository 根目錄執行 `colcon build`，產物會在該目錄的 `install/`，與 `~/ws_fanuc/install/` 是兩份獨立安裝。每個 terminal 要 source 本次 build 的那一份；`cd` 不會切換 ROS 套件來源。遇到 `No executable found` 可先執行 `ros2 pkg prefix dual_sharpa_wave` 確認來源，再用 `ros2 pkg executables dual_sharpa_wave` 檢查是否有三個控制工具。

依賴由 `package.xml` 列出；主要為 `rclpy`、`sensor_msgs`、`launch_ros`、PyYAML、Tkinter（`python3-tk`）與 pytest。未安裝 ROS 的電腦仍可執行純 Python backend/validation tests：

```bash
python3 -m pytest test/test_mock_hand.py test/test_joint_validation.py test/test_control_model.py test/test_sdk_hand.py -q
```

測試包含兩個由正式 mock launch 啟動的獨立 process、實際 DDS 訊息往返、左右隔離、name reorder、錯誤指令、timeout、回讀失敗與 cleanup。整合測試對子程序注入 import guard：若嘗試 import 官方 `sharpa` module 就失敗。測試不啟動 hardware launch 或 CRX driver。
其中 GUI 與 `use_rviz=true` 的整合測試需要可用的圖形顯示環境（例如 WSLg）。波形整合測試會執行安裝後的 `sine_control.py`／`step_control.py`，驗證左右同步輸入與未選軸保留。

## 啟動與手動 topic 測試

三個 terminals 都先執行以下環境設定；domain 172 是本機 mock 測試範例，所有 terminals 必須相同：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_fanuc/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=172
```

Terminal 1：

```bash
ros2 launch dual_sharpa_wave dual_sharpa_mock.launch.py
```

Terminal 2：

```bash
ros2 topic list
ros2 topic echo /sharpa/left_hand/joint_states sensor_msgs/msg/JointState
# 在另一個 terminal 觀察右手：
ros2 topic echo /sharpa/right_hand/joint_states sensor_msgs/msg/JointState
```

Terminal 3：一次發布 22 個角度；第一個關節為拇指 CMC FE：

```bash
ros2 topic pub --once /sharpa/left_hand/joint_command sensor_msgs/msg/JointState "{position: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
ros2 topic pub --once /sharpa/right_hand/joint_command sensor_msgs/msg/JointState "{position: [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

預期左手到 0.1、右手到 0.2，互不影響。只測一側時，另一側維持初始 zero。停止 launch 使用 Ctrl-C；每側都呼叫 backend `stop()`。

本機 DDS graph discovery 可能需數秒；剛啟動時 `topic list`/`node list` 可能暫時不完整。`echo` 明確指定 message type，可直接等待 state，避免自動判定 type 時的 discovery 競態。

## GUI 與波形控制工具

先依前述設定 source ROS／workspace，所有 terminals 使用同一個 ROS domain。Terminal 1 啟動：

```bash
ros2 launch dual_sharpa_wave dual_sharpa_mock.launch.py use_rviz:=true
```

Terminal 2 **擇一**執行以下控制工具，不要同時讓多個工具寫同一個 command topic：

```bash
ros2 run dual_sharpa_wave gui_control.py

ros2 run dual_sharpa_wave sine_control.py \
  --axes thumb_CMC_FE,index_MCP_FE,middle_MCP_FE \
  --amplitude 0.1 --frequency 0.25 --cycles 2 --repeat 1

ros2 run dual_sharpa_wave step_control.py \
  --axes thumb_CMC_FE,index_MCP_FE,middle_MCP_FE \
  --step-size 0.1 --base-seconds 1 --high-seconds 1 --cycles 2 --repeat 1
```

入口檔案位於 `script/`。GUI 左右各有 22 個 slider，單位 rad；收到該側有效回授後才可操作，按「Start Sending」才發布目標。「Load Current Pose」會先停止傳送，再從回授重設 slider。拖動一軸不改其他軸，回授數值獨立顯示。「Stop Sending」或關閉視窗不會回零，也不是實體急停。

Sine／step 啟動時保存兩側各自的目前姿勢 `q0`，一次只改一對同名關節：

- Sine：`q0 + amplitude * sin(2π * frequency * t)`，每軸跑 `cycles` 個整數週期。
- Step：`q0 → q0 + step-size → q0`，前後各保持 `base-seconds`，高段保持 `high-seconds`，重複 `cycles` 次。
- 左右共享時間起點／相位和 command stamp，依序發布到兩個 topics。同步指輸入波形，並非兩隻實體手硬體級同步，也不保證角度同號即視覺鏡像。
- 未選軸保留起始姿勢；換軸及結束前持續發送 baseline，等待兩側所有關節回到 `tolerance` 內。Step 是目標階躍，實際回授仍受 mock 限速或 SDK 插值影響。

三個工具都讀取 vendored URDF 的 joint limits，越界明確報錯。Sine 在起始姿勢附近雙向擺動；例如 PIP 在 0 rad 時無法接受負半波，需先用 GUI 調到範圍內的中心姿勢，或改用正向 step。這些限制不代表已完成實機校正。

| 共用 CLI 參數 | 預設 | 用途 |
|---|---|---|
| `--rate` | `30` | Command 發布 Hz；需足以維持 hand node 的 command timeout |
| `--state-timeout` | `1` | 回授最久可中斷秒數 |
| `--wait-timeout` | `10` | Sine／step 等兩側 ROS 就緒的期限 |
| `--gap` | `0.25` | Sine／step 換軸前 baseline 最短保持秒數 |
| `--settle-timeout` | `10` | Sine／step 等回 baseline 的最長秒數，須大於 gap |
| `--tolerance` | `0.02` | Sine／step 判定回 baseline 的 rad 容差 |

完整參數可用各腳本的 `--help` 查看。回授無效／逾時時 GUI 停止傳送且不自動恢復，波形工具報錯退出。Ctrl-C 不額外送回零命令。Hardware 在輸入結束後達到 command timeout 會關閉 SDK session，下一次控制前需重啟 hardware hand nodes；Mock 可直接繼續下一次測試。

## Command 與時間語意

- `position` 必須是 22 個有限數值；NaN/±Inf、21/23 維一律整筆拒絕。
- `name` 空陣列表示下表的固定順序；非空時必須完整、不重複且恰好對應該側 22 個名稱，node 會 reorder。
- `velocity`、`effort` 與 command timestamp 不參與控制；不推測 joint limits，也不 silent clamp。
- State 含完整 names、22 維 radians、node ROS clock 的 `header.stamp`；velocity/effort 留空。
- Timer 使用 steady clock，`dt` 與 command timeout 使用 `time.monotonic()`。ROS clock 調整不影響 mock 運動或 timeout；若設定 `use_sim_time`，只影響訊息 stamp，這不是物理模擬器的 lockstep 時間模式。
- 預設 100 Hz 發布；實際 DDS 延遲與排程不保證精確 10 ms。
- Timeout 預設 0.5 秒，`<=0` 停用。僅合法且 backend 接受的 command 重設時間；尚無 command 時從 node 啟動計時。
- Mock timeout 保留最後 target、繼續追蹤並發布目前位置，不回 zero。新合法 command 解除 timeout。每類 warning 最多每 5 秒輸出一次。
- Hardware 在第一筆接受的 command 之後才啟用 timeout 處置：逾時呼叫 SDK `stop()` 並斷開該 serial，鎖定失敗狀態，需重新啟動 hand node。尚無 command 時只警告，不因等候 GUI 而關閉連線。SDK stop 的實際馬達效果尚待實機驗證。

## 參數

編輯 `config/dual_sharpa_wave.yaml`，key 必須是完整 node 名稱。自訂檔可透過 `config_file:=/absolute/path/config.yaml` 傳入。參數為啟動時設定，運行中拒絕修改，避免 backend 與參數不一致。

| 參數 | 預設 | 說明 |
|---|---|---|
| `side` | `left` / `right` | launch 依 namespace 固定指定 |
| `backend` | `mock` | mock launch 固定為 mock，即使 YAML 填 hardware |
| `publish_rate_hz` | `100.0` | 有限正數，launch 支援 override |
| `command_timeout_sec` | `0.5` | `<=0` 停用 |
| `mock_mode` | `first_order` | `instant` 或限速追蹤 |
| `mock_max_velocity_rad_s` | `1.0` | 有限正數 |
| `serial_number` | 空字串 | mock 不使用，hardware 必須明確指定 |
| `speed_coeff` | `0.3` | SDK 速度係數，adapter 接受 `(0, 1]`；mock 不模擬 |
| `current_coeff` | `0.6` | SDK 電流係數，adapter 接受 `(0, 1]`；mock 不模擬 |
| `interpolation` | `true` | 傳給 SDK position API；mock 不模擬 SDK 插值 |
| `sdk_discovery_timeout_sec` | `10.0` | 指定 serial 的 discovery 等待期限；不限制 native API 單次呼叫時間 |

```bash
ros2 launch dual_sharpa_wave dual_sharpa_mock.launch.py publish_rate_hz:=50.0
```

`instant` 立即把 actual 設成 target。`first_order` 每次最多移動 `mock_max_velocity_rad_s * dt`，到達後不 overshoot，不加 random noise。這個模型沒有碰撞、摩擦、負載或觸覺，不代表實機速度／電流／插值響應。

## Joint order 與 SDK mapping

依據 SDK **5.0.10 / build 5.0.10.6** 的 `sample/python/sharpa_wave_example.py` `JOINT_NAMES[0..21]`，比對官方模型 revision **`0d19cac602f46456b819e4b6a2c09a74982c9a3e`**：

- `wave_01/left_sharpa_wave/left_sharpa_wave_with_flange.urdf`
- `wave_01/right_sharpa_wave/right_sharpa_wave_with_flange.urdf`

兩側都恰好 22 個 revolute joints、沒有 mimic joints。下表是完整 public canonical order，也是 SDK index order。SDK 的 thumb DIP 對應 URDF `thumb_IP`；SDK 的 Pinky CMC FE 對應 `pinky_CMC`。名稱採 URDF 原名。

| Index | Left canonical name | Right canonical name |
|---:|---|---|
| 0 | `left_thumb_CMC_FE` | `right_thumb_CMC_FE` |
| 1 | `left_thumb_CMC_AA` | `right_thumb_CMC_AA` |
| 2 | `left_thumb_MCP_FE` | `right_thumb_MCP_FE` |
| 3 | `left_thumb_MCP_AA` | `right_thumb_MCP_AA` |
| 4 | `left_thumb_IP` | `right_thumb_IP` |
| 5 | `left_index_MCP_FE` | `right_index_MCP_FE` |
| 6 | `left_index_MCP_AA` | `right_index_MCP_AA` |
| 7 | `left_index_PIP` | `right_index_PIP` |
| 8 | `left_index_DIP` | `right_index_DIP` |
| 9 | `left_middle_MCP_FE` | `right_middle_MCP_FE` |
| 10 | `left_middle_MCP_AA` | `right_middle_MCP_AA` |
| 11 | `left_middle_PIP` | `right_middle_PIP` |
| 12 | `left_middle_DIP` | `right_middle_DIP` |
| 13 | `left_ring_MCP_FE` | `right_ring_MCP_FE` |
| 14 | `left_ring_MCP_AA` | `right_ring_MCP_AA` |
| 15 | `left_ring_PIP` | `right_ring_PIP` |
| 16 | `left_ring_DIP` | `right_ring_DIP` |
| 17 | `left_pinky_CMC` | `right_pinky_CMC` |
| 18 | `left_pinky_MCP_FE` | `right_pinky_MCP_FE` |
| 19 | `left_pinky_MCP_AA` | `right_pinky_MCP_AA` |
| 20 | `left_pinky_PIP` | `right_pinky_PIP` |
| 21 | `left_pinky_DIP` | `right_pinky_DIP` |

`joint_names.py` 是唯一 authoritative order。`SDK_TO_URDF_INDEX = (0, 1, ..., 21)`，定義為 `sdk[i] → canonical[SDK_TO_URDF_INDEX[i]]`；`URDF_TO_SDK_INDEX` 是反向 mapping，本版本同為 identity。這是文件與模型順序核對，尚未以實機確認每個關節的方向／角度零位。

SDK adapter 在邊界處轉換 order；`set_joint_position()` 輸入 radians，`get_joint_position_degree()` 回讀轉 radians。採用官方 Python sample 的 degree API，ROS callback 不包含 SDK 細節。

## Hardware 邊界

`SharpaSdkHand` 已實作指定 serial 連線、POSITION mode、速度／電流係數、SDK control source、命令下發、回授轉換與資源清理。SDK 只在 hardware `start()` 才載入，錯誤不會 fallback 到 mock。

Hardware launch 預設讀取 `config/dual_sharpa_hardware.yaml`，先檢查左右 serial 非空且不重複，再啟動兩個 hand nodes。預設空 serial 會直接報錯。SDK 環境設定、操作方式、API 證據與限制見 [Hardware 使用說明](docs/hardware.md)。

Fake SDK 測試驗證的是同一份 adapter 的呼叫、mapping、單位、錯誤與 cleanup，不驗證官方 SDK binary、網路或實體馬達。自動測試不連接裝置。SDK 的 `stop()`／斷線不能宣稱是已驗證的實體急停；native API 阻塞時，同一執行緒中的 ROS timer 也無法保證準時處理 timeout。

本 package 不修改 `dual_crx_control`，不啟動任何 FANUC driver，也不實作 MIT、torque/velocity command、teleop 或物理模擬。

## 可選 RViz 預覽

在上述相同環境下執行：

```bash
ros2 launch dual_sharpa_wave dual_sharpa_mock.launch.py use_rviz:=true
```

預設 `use_rviz=false`，只啟動兩個 hand nodes；仍可接收 `joint_command` 並發布 `joint_states`。
`use_rviz=true` 另外啟動兩個 `robot_state_publisher` 與 RViz，將關節角度轉成預覽 TF 並顯示模型姿勢。
預覽統一由 `use_rviz` 控制。

每個 state publisher 只訂閱自己 namespace 的 `joint_states`，不使用 aggregator，不增加 44 維 public topic。
Robot descriptions 分別是 `/sharpa/left_hand/robot_description`、`/sharpa/right_hand/robot_description`。
預覽 TF 全部加上 `sharpa_preview/`，共同固定 frame 是 `sharpa_preview/preview_world`；每個 child frame 只有一個 publisher。
兩手 flange 在顯示座標中的 Y 偏移為 ±0.15 m，僅供並排顯示，**不是 CRX flange calibration**。
若接入正式 CRX combined model，停用此獨立預覽，由正式模型負責 TF；不要在相同 ROS domain 重複啟動兩份預覽。

已 vendor 官方左右 `with_flange` URDF 與全部引用 meshes（約 14 MB），只替換 mesh package URI。
License、NOTICE、固定 revision 與 checksum 見 [模型來源紀錄](third_party/sharpa_models/README.md)。
模型 joint order／mesh 完整性以及左右獨立 TF 更新都納入測試。RViz 本身只有視覺化，不包含物理模擬。

## 來源

- [SDK 5.0.10 release](https://github.com/sharpa-robotics/sharpa-wave-sdk/releases/tag/v5.0.10)
- [官方模型固定版本](https://github.com/sharpa-robotics/sharpa-urdf-usd-xml/tree/0d19cac602f46456b819e4b6a2c09a74982c9a3e)
