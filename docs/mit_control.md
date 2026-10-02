# ROS MIT 位置介面

本介面啟動時使用固定調參基準 × `mit_kp_ratio`／`mit_kd_ratio`（各預設 0.6，左右手共用），以 ROS 位置目標呼叫 SDK
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
因此不能把啟動視為純讀取或保證完全不動。啟動時先讀取力矩來源與增益，
最多一次寫入固定基準 × 倍率的 Kp／Kd 並讀回確認；相同值跳過寫入，力矩來源不改寫。
運行中可透過文末的增益 GUI 明確要求更新 Kp／Kd。

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
| 實際角度範圍 | 原始回授超出 URDF 上下限加 `joint_limit_tolerance_deg:5.0` 時鎖定普通命令，保留連線與回授，等待明確歸零恢復 |
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
這是結構檢查，不是穩定性驗證。啟動倍率目前僅支援電流來源 0，
來源 1 會在修改模式之前回報啟動失敗。固定 Kp／Kd 定義於 `mit_gains.py`，
是先前調參的保存值；每次乘倍率都從固定基準出發，重啟不累乘。
必要增益讀写／核對、模式設定或模式讀回失敗時關閉 session 並鎖定錯誤，不退回 position。
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
`normal`、`limit_locked`、`recovering` 或 `faulted`，包含原因。實際超過 5° 容差
時進入 `limit_locked`，沒有自動歸零、斷線或使能切換。即使回授自行返回範圍內，
需明確恢復；恢復完成或中止時，重新核對實測限位以決定是否解除鎖定。這是手部控制狀態，CRX 手臂的控制流程不受此狀態機管理。

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
完成必須同時滿足零位容差（預設 2°）與實際限位容差（5°），連續保持預設 1 秒。

Ctrl+C、心跳中斷超過 `recovery_client_timeout_sec:0.5`、或到位逾時只結束恢復，
不再因這些事件產生 `limit_locked`。結束已開始的恢復時重新讀回位置，只有實測超出
URDF 邊界大於 5° 才鎖定，否則回到 `normal`；未達零位仍回報 action 失敗。
連線與回授保留，下一筆普通命令仍需通過 0.1 rad 接管檢查。任一側拒絕或失敗時，
腳本取消另一側尚未完成的 goal。雙手沒有硬體同步或原子提交保證；已成功完成的一側
保持正常；失敗／取消的一側依實測限位判定，不再無條件鎖定。恢復未開始前若已有
真實超限鎖定，取消準備不會清除它。SDK faulted 不會被取消處理清除。

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

## 簡易 Kp／Kd 調整 GUI

已加入每手的 `mit_gains` service，使用原有 SDK 連線，僅在可寫 MIT 模式提供。
目前支援 current-based 力矩來源（`torque_source=0`）。啟動一次套用預設 0.6 倍增益；
GUI 初始目標顯示固定基準（倍率 1），按 Read device gains 後可以 Apply。
`normal` 與 `limit_locked` 都可調整增益；歸零到位失敗後仍可調參，或由使用者再次執行歸零恢復。
調參不解除普通運動命令鎖定。`recovering` 與 `faulted` 仍不接受增益寫入。

```bash
cd /home/msc-crx/ws_fanuc
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sharpa_control_interfaces dual_sharpa_wave --symlink-install
source install/setup.bash
# 先使用原本的 launch 啟動 MIT driver；更新程式後需重新啟動該 driver。
ros2 run dual_sharpa_wave adjust_mit_gains_gui.py --side left
# 右手使用 --side right
```

- `Read device gains`：只刷新裝置值，不更改固定基準、倍率或目標欄位。
- `Kp ratio`、`Kd ratio`：分開設定，各預設 `1.0`，與啟動參數共用同一份硬編碼調參基準。1 永遠代表原始值（例如首關節 Kp=20、Kd=0.5）；0.6 代表 Kp=12、Kd=0.3。不累乘，倍率只更新目標欄位，仍可手動微調。
- `Apply`：提交整組 22 關節的目標值；只有按下此鍵才會寫入增益。
- `Stop transition`：停止剩餘步驟，保留已寫入的增益；不是馬達急停。
- 固定 200 ms 排程、兩步（50% → 100%），依目前 SDK 延遲估計約 0.8–0.9 秒完成。新目標從重新讀取的目前值開始。
- 閒置不讀寫硬體增益；GUI 的 STATUS 查詢只讀軟體快取。
- 每步寫入 Kp／Kd 後讀回確認。錯誤停止 ramp，不另外關閉 driver。
- 僅驗證 22 個有限非負值及現有控制狀態；不新增增益上限或 client 心跳。
- 關閉 GUI 不取消 driver 已接受的過渡；需要停止時先按 Stop transition。

**實測限制：**2026-09-30 對兩手寫回原有增益，成功且讀回完全一致。
單次 SDK 寫入分別為 300.3 / 308.5 ms，讀回為 1.4 / 2.6 ms。
這個最小同步版本會在參數呼叫期间暫停 ROS 回呼，不能維持 100 Hz 控制／回饋，
兩步加首次 200 ms 等待預計約 0.8–0.9 秒，SDK 延遲變動時可能超過 1 秒。
兩步的增益變化比原本 10 步更大。現階段適合靜態姿態調參；
持續 teleop 中的非阻塞更新需要另行處理 SDK 通訊，未在此版本實作。

本次實機只做相同值寫回，未呼叫 start、切換模式或發送運動命令。
尚未驗證改變增益後的物理響應、運行中效果或 Flash 持久化行為。
測量資料見 [mit_gain_io_2026-09-30.json](mit_gain_io_2026-09-30.json)。

啟動倍率可用 `mit_kp_ratio:=0.6 mit_kd_ratio:=0.6` 覆寫；`dual_sharpa.launch.py` 與 `bimanual_system.launch.py` 都支援。未傳入時沿用 YAML／節點的 0.6 預設。只讀、POSITION 與 mock 不寫入硬體增益。啟動時一次寫入，不使用 GUI 的兩步過渡；SDK 寫入延遲發生在初始化完成之前。

Kp 與 Kd 可使用不同倍率，例如 `mit_kp_ratio:=0.6 mit_kd_ratio:=0.8`；兩手接收相同的一組倍率。舊的 `mit_gain_ratio` 參數已取代。
