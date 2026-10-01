# ROS MIT 位置介面

本介面使用 Pilot 已保存的 MIT 增益，以 ROS 位置目標呼叫 SDK
`set_mit_control(q, dq, tau)`。第一版固定 `dq=0 rad/s`、`tau=0 N·m`，
由裝置的 Kp／Kd 產生位置剛性與速度阻尼。這不是零力模式。

2026-09-30：已實作並做離線驗證；雙手唯讀連線確認目前模式為 MIT、
`torque_source=0`，且能讀回 22 維 Kp／Kd。使用者已在 Pilot 調參並回報效果良好。
本次開發未透過 ROS 啟動實機控制、切換模式或發送運動命令；ROS 的實體測試由使用者執行。

## 啟動（由使用者執行）

在 `/home/msc-crx/ws_fanuc` 建置並載入環境：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to dual_sharpa_wave
source install/setup.bash
export PYTHONPATH="/opt/sharpa-wave-sdk/python${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="/opt/sharpa-wave-sdk/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROS_LOG_DIR=/tmp/dual_sharpa_wave_logs
```

先退出 Pilot 對同一隻手的控制，再啟動：

```bash
ros2 launch dual_sharpa_wave dual_sharpa.launch.py \
  backend:=sharpa_sdk use_rviz:=false
```

真機現在預設使用 `config/dual_sharpa_mit.yaml`，不需另加 `control_mode:=mit`。
如需 POSITION，可明確加 `control_mode:=position`。**啟動會設定 MIT 模式、
速度／電流係數與 SDK 控制來源，並啟動 SDK session；SDK 的模式設定會使能馬達。**
程式啟動時不呼叫位置／MIT 目標 API，但尚未實測模式切換時韌體如何處理既有目標，
因此不能把啟動視為純讀取或保證完全不動。Kp／Kd 與力矩來源只讀不寫。

可加 `hands:=left` 或 `hands:=right`，只啟動單側 hand node。
現有 GUI 需要雙側回授才開放操作；單側 launch 適用於回授檢查或使用者自己的單側 publisher。
`use_rviz` 的預覽仍顯示兩個手模型。

雙側啟動後，於另一個已載入 ROS／workspace 的終端開啟原 GUI：

```bash
ros2 run dual_sharpa_wave gui_control.py --rate 100
```

先按 **Load Current Pose**，以目前姿態開始傳送，再小幅調整一個關節。
第一筆有效命令送出當次回授裁切到 URDF 範圍後的姿態；後續位置目標同樣裁切後送入 MIT API。
需要持續發送才能完成移動；單次發布不保證到達目標。100 Hz 是沿用官方 MIT
範例的起始測試頻率，並非已量測的即時性或韌體頻率要求。GUI 排程及 Python/ROS 不保證硬即時。

## 命令與限制

| 項目 | 行為 |
|---|---|
| Topics | 沿用 `/sharpa/{left_hand,right_hand}/joint_command` 及 `joint_states` |
| 命令 | 22 維位置，rad；具名時重排到 canonical order，再映射至 SDK order |
| 速度／力矩欄位 | MIT 位置介面要求 `velocity`／`effort` 為空；非空整筆拒絕 |
| 首筆／暫停後恢復命令 | 普通控制以裁切後的目標與裁切後的當下回授比較，距離不可超過 `mit_start_tolerance_rad`，預設 0.1 rad；首筆送出裁切後回授 |
| 命令角度範圍 | MIT adapter 將目標裁切到 URDF 上下限；目標超限本身不鎖定或斷線 |
| 實際角度範圍 | 原始回授超出 URDF 上下限加 `joint_limit_tolerance_deg:0.5` 時鎖定普通命令，保留連線與回授，等待明確歸零恢復 |
| 目標傳遞 | 普通控制首筆接管後，裁切目標直接送入 MIT API，不額外施加目標速度限制 |
| interpolation | MIT 必須設 `false`；不呼叫位置模式插值 API，位置插值由 retargeting 等上游 publisher 負責 |
| 命令發送節奏 | 普通控制收到有效 ROS 命令才呼叫 MIT API；明確歸零 action 期間由 driver timer 發送恢復軌跡。SDK 內部傳送策略不由此規則保證 |
| 回授 | 保留真實位置回授；速度與力矩仍未發布，不以 target 假造回授 |

MIT adapter 沒有額外的目標速度／加速度限制。配置中的 SDK `speed_coeff:0.3`
仍會在啟動時寫入；其在 MIT 下的具體作用尚未核實，不能解讀為已驗證的物理速度上限。
實際響應仍受 Kp／Kd、電流係數與韌體內部行為影響。

`control_mode` 僅在啟動時選擇，支援 `position`、`mit`，不支援執行中切換。
未指定模式與配置檔時，真機預設 MIT，mock 預設 position。提供 `config_file` 時使用該 YAML；launch
`control_mode` 若非空，會覆寫 YAML 的模式，但不會替自訂 YAML 更改 interpolation 或 timeout。
MIT 設定若為 `interpolation=true`、`command_timeout_sec<=0`、非法模式或非法接管距離，
會在 SDK 連線前拒絕。

歸零軌跡保留原始回授起點；送出時裁切到 URDF 上下限，不修改回授、編碼器或零點。
首筆裁切可能產生目標步差，quintic 速度／加速度界限不代表該步差或實機速度的保證。
22 維有限數值、關節名稱與 SDK 錯誤處理仍保留。
GUI／波形工具及外部 retargeting 的自身位置限制不在此次修改範圍，SDK／韌體內部限制也未修改。

## 增益與故障處理

啟動 MIT 控制前讀取 `force_feedback_source`，不支援時嘗試舊欄位 `torque_source`。
本次兩手韌體 3.0.10 都採舊欄位，因此 SDK 印出新欄位不支援的 warning 是可預期的，
之後必須成功讀回舊欄位。接受純量或 22 維一致來源：0 為電流來源，1 為感測器來源；
其他或混合來源目前拒絕，避免猜測 FS 增益的適用範圍。

基本增益接受純量或 22 維陣列；感測器來源另要求 `mit_kp_fs`、`mit_kd_fs`
為純量、10 維或 22 維，對應 Pilot 的 FS 關節資料。所有增益須為有限非負數。
這是結構檢查，不是穩定性驗證。ROS 不覆寫、統一或重新調整任一側保存的增益。
必要讀取、模式設定或模式讀回失敗時關閉 session 並鎖定錯誤，不退回 position。
模式與增益會記錄在啟動 log。

MIT YAML 的 `command_timeout_sec:0.5` 現在是 **命令閒置判定時間**。沒有新的有效命令
超過期限，保留 SDK session、馬達設定與回授，不再因正常暫停 stop/disconnect 或鎖定。
普通控制閒置時 timer 不發送目標，也不寫模式或增益；首次命令前只等待。閒置時只記錄一次
`MIT command stream idle; next command will reacquire measured pose`。

正常狀態恢復發布時重新讀取當下回授，沿用首筆 0.1 rad 接管門檻，首筆送出裁切後回授姿態，
後續有效目標裁切後傳遞。即使新命令先於 timer 到達，也執行相同重新接管流程。
若提示 `load current pose first`，讓 publisher 以最新回授重新初始化即可，不需重啟 driver。
歸零腳本與重新啟動的 retargeting 都以回授建立起點；GUI 使用 Load Current Pose。

**停止發布不是停止馬達或卸力**：最後送出的 MIT 目標仍可能持續作用，實際保持行為由
SDK／韌體決定。本次未做實機暫停／恢復測試。SDK 讀寫等真正故障仍關閉並鎖定 session，
關閉 node 也會 stop/disconnect；閒置處理不會自動清除這類故障。

## 超限鎖定與明確歸零恢復

每隻手在 `/sharpa/{left_hand,right_hand}/control_state` 發布保留最新值的 JSON 狀態：
`normal`、`limit_locked`、`recovering` 或 `faulted`，包含原因。實際超過 0.5° 容差
時進入 `limit_locked`，沒有自動歸零、斷線或使能切換。即使回授自行返回範圍內，
仍需明確歸零成功才能解除鎖定。這是手部控制狀態，CRX 手臂的控制流程不受此狀態機管理。

使用原本指令即可：

```bash
ros2 run bimanual_manipulation move_to_default_pose.py --execute --arm-rate 100 --hand-rate 100
# 若只需要雙手歸零：
ros2 run dual_sharpa_wave move_to_default_pose.py --execute --rate 100
```

手部腳本先讀兩側 `control_mode`；MIT 使用新的
`/sharpa/{left_hand,right_hand}/recover_default` action（`sharpa_control_interfaces/action/RecoverDefault`），
position 沿用原本的 joint_command 流程。两侧需使用相同模式。action 接受軌跡速度、加速度、
最短時間、發送率與到位參數，只能執行零姿態恢復，不接受任意目標。

兩側 action 都接受後，腳本才發送帶 goal UUID 的 `/recovery_heartbeat`；driver 收到
匹配心跳後重新讀取當下姿態並開始移動，避免其中一側拒絕時另一側先開始。恢復中
普通 `joint_command` 與第二個恢復 goal 都被拒絕。目標逐步朝零移動並裁切；
完成必須同時滿足零位容差（預設 2°）與實際限位容差（0.5°），連續保持預設 1 秒。

Ctrl+C、心跳中斷超過 `recovery_client_timeout_sec:0.5`、或到位逾時會取消恢復並維持
`limit_locked`，連線與回授保留；可重新執行歸零，不需重啟。任一側拒絕或失敗時，
腳本取消另一側尚未完成的 goal。雙手沒有硬體同步或原子提交保證；已成功完成的一側
保持正常，失敗／取消的一側保持鎖定。正常完成後不用重啟，可重新啟動 teleop。

恢復中的實際超限距離相對於該關節本次恢復的最佳值，加重超過
`recovery_worsening_deg:0.5` 並連續達 `recovery_worsening_sec:0.2`，才判定持續惡化；
短暫尖峰會清除計時。持續惡化、SDK 讀寫錯誤或恢復控制迴圈停頓超過 0.5 秒會
stop/disconnect 並進入 `faulted`，這類故障需要重啟與檢查原因，不能由恢復 action 清除。
回授新鮮度目前仍以主機收訊／SDK getter 成功為依據，不是韌體序列號的新鮮度驗證。

取消或鎖定不等於物理停止，最後的 MIT 目標可能仍生效。此功能已做 Fake SDK／隔離 DDS
離線驗證，沒有由開發程式發送任何實機動作。新增介面包需要一起建置並重新 source 環境；
本節指令已使用 `--packages-up-to` 包含該依賴。

## 純讀取檢查

安裝的 `read_mit_settings.py` 使用指定 serial 連線、讀取模式及保存參數後斷線，
不呼叫 `start()`、`set_control_mode()`、`set_control_source()`、增益寫入或運動 API：

```bash
ros2 run dual_sharpa_wave read_mit_settings.py --serial CD52943BCD53
ros2 run dual_sharpa_wave read_mit_settings.py --serial C956943BC957
```

可加 `--output /tmp/mit_settings.json` 保存報告。SDK connect 仍會建立網路連線、
初始化背景收送執行緒及查詢參數；這不是離線操作。`--discovery-timeout` 只限制 discovery，
不限制單次 native SDK 呼叫。右手本次的實際讀回記錄見
[mit_right_readback_2026-09-30.json](mit_right_readback_2026-09-30.json)。

## 待使用者完成的實體驗證

確認切換／接管時是否有突跳、小幅目標是否按預期柔順追蹤，以及停止發布後的實際馬達行為。
記錄使用的發布率、增益與兩側差異，再擴大到其他關節與雙手操作。
離線 Fake SDK／DDS 測試只證明軟體路徑與錯誤處理，不證明實機阻抗響應。
