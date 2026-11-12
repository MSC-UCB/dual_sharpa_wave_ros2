# Sharpa 原廠 Admittance Control 接口評估

評估日期：2026-11-02。範圍：目前 `dual_sharpa_wave_ros2`、本機 Sharpa Wave SDK 與 Sharpa Pilot。本階段為介面核對與文件設計，尚未修改控制程式或新增調參工具。已完成離線檢查，另嘗試 SDK 裝置搜尋，但未搜尋到指定左右手，因此未建立手部連線、切換控制模式或寫入硬體參數。

## 1. 結論

**有合適的原廠 SDK 接口，值得接入；目前 ROS 驅動尚未支援，不能只把 YAML 改成 `admittance` 就使用。**

- 已確認 C++ 與實際載入的 Python SDK 都提供 `ControlMode.ADMITTANCE`（值為 **14**），可透過 `set_control_mode()` 選擇，並用 `get_control_mode()` 讀回。
- 本機 Pilot 2.0.10 確實有導納參數頁面與寫入流程，包含 `mass`、`damping`、`stiffness`、`torque_threshold`、`torque_source`，另有 `stiffness_coeff`。因此證據不只是一個模式列舉值。
- 現有 `SharpaSdkHand` 是合適的接入位置；若確認原廠導納模式使用關節位置作為參考輸入，上層的 22 關節位置命令與回授 topic 可以沿用。
- **尚未確認的關鍵契約**：導納模式下應使用哪個位置指令、參考位置的實際意義、Python 通用參數接口是否接受 Pilot 的完整封裝、裝置韌體支援情況，以及停止送命令後的行為。

建議先以原廠既有參數完成單手驗證，再整合 ROS。後續實作規劃包含「模式選擇、參數讀回、位置參考、狀態與故障處理」，以及使用者提出的 **M／B／K 微調 Python 工具**；工具設計見第 11 節。這些目前均為設計，尚未實作。

## 2. 本次檢查的版本與證據層級

| 項目 | 本機資料 | 可證明的範圍 |
| --- | --- | --- |
| ROS 套件 | `src/dual_sharpa_wave_ros2`，檢查時 HEAD `85d02e0` | 現有原始碼與設定行為 |
| Wave SDK | `/opt/sharpa-wave-sdk/VERSION`：5.0.10，build 5.0.10.6 | 已安裝 SDK 版本 |
| Python binding | 離線 import `sharpa`，檢查 enum、類別成員與 docstring | Python 確實暴露相關 API；此項離線檢查未建立 manager 或 hand 物件 |
| Pilot | `/opt/sharpa-pilot/resources/app.asar` 內 `package.json`：2.0.10 | 原廠應用程式的參數頁面及 IPC 封裝 |
| 韌體相容資料 | SDK `sdk-release-info.json` 的 5.0.10 項目列出 firmware 3.0.10、Pilot 2.0.10 | 發行相容組合；不代表左右手目前實際韌體版本 |
| 後續裝置搜尋 | 建立 SDK manager，搜尋 5 秒；未發現 `CD52943BCD53`、`C956943BC957` | 未嘗試 connect，未取得實際 M／B／K；不能據此判定硬體不支援導納 |

本報告將「SDK 接口存在」、「Pilot 使用方式」與「實機已驗證」分開。前兩者已取得證據，第三者尚未進行。

## 3. 現有 MIT 控制路徑

```text
/sharpa/<side>_hand/joint_command（sensor_msgs/JointState.position，rad）
  → HandNode 驗證名稱與 22 關節數值
  → SharpaSdkHand.set_joint_positions()
  → MIT 接管檢查、關節限制
  → set_mit_control(q_ref, [0] × 22, [0] × 22)

get_joint_position_degree()
  → SDK/ROS 關節排序轉換、degree → rad
  → /sharpa/<side>_hand/joint_states
```

目前 MIT 使用位置目標，目標速度與前饋力矩皆為零；`JointState.velocity` 和 `effort` 必須為空。啟動時套用固定基準增益乘上 Kp/Kd ratio，目前兩者預設為 0.6。

目前設定為關節回授 100 Hz、觸覺發布 30 Hz、命令逾時 0.5 秒、`interpolation: false`。100 Hz 是 ROS 回授 timer，位置命令是在 subscription callback 送出，並非由此 timer 固定重送，也不能據此推定韌體導納迴圈頻率。

模式限制分布於雙手 launch、單手 launch、`HandNode` 與 `SharpaSdkHand`，都只接受 `position`、`mit`。即使略過驗證，adapter 現有的模式選擇仍是「MIT，否則 POSITION」，必須改成明確對應。

## 4. 可用的原廠接口

| 接口 | 已確認用途 | 對本次接入的意義 |
| --- | --- | --- |
| `ControlMode.ADMITTANCE` | Python/C++ 值皆為 14 | 原廠導納模式入口 |
| `hand.set_control_mode(mode)` | 設定控制模式，回傳 `Error` | 可新增 admittance 模式分支 |
| `hand.get_control_mode()` | 回傳 `(Error, ControlMode)` | 切換後必須讀回確認 |
| `hand.set_control_source(ControlSource.SDK)` | 選擇 SDK 控制來源 | 可沿用既有控制來源管理 |
| `hand.set_joint_position(angles_rad, interpolate=False)` | 傳送關節位置，單位 rad | 導納參考輸入的候選接口；文件沒有明確保證導納語意 |
| `hand.get_joint_position_degree()` / `get_joint_position_rad()` | 讀取關節位置 | 可沿用位置回授及排序轉換 |
| `hand.get_parameter(names)` | 回傳 `(Error, JSON string)` | 候選參數讀回接口，實際可讀 key 與格式須核對 |
| `hand.set_parameter(json_str)` | 通用參數寫入 | 候選導納調參接口；不可將 Pilot IPC 格式直接視為已驗證 Python 契約 |
| `hand.get_states()` | 提供 SDK State；文件描述角度、速度及力矩 | 未來診斷候選；有效性、來源與量測意義仍需確認 |

在已檢查的 C++ header、Python `SharpaWave` 成員及 SDK 範例中，沒有找到獨立的 `set_admittance_control()` 或導納專用 sample。這不表示 SDK 不支援導納，而是目前看到的入口為「模式選擇＋通用接口」。`set_mit_control()` 的文件明確指定 MIT，不應直接拿來當導納命令。

**切換模式會影響馬達使能。** Header 對 `set_control_mode()` 明確說明：函式會處理 enable state，並在返回前設為 True。整合時必須確認舊目標、導納內部狀態與參考位置如何初始化，不能把它當成沒有動作影響的設定讀寫。

## 5. Pilot 提供的導納參數證據

本機 Pilot 的 `AdmittanceParams` 頁面讀取既有參數，保存時先寫 `stiffness_coeff`，再透過 `set_device_admittance_params` 寫入下列內容。

| 參數 | Pilot 介面／程式證據 | 接入注意事項 |
| --- | --- | --- |
| `mass` | 每關節陣列；UI 標示 kg·m²，範圍 0.001–10 | 是旋轉慣量量綱，不應按平移質量 kg 理解 |
| `damping` | 每關節陣列；UI 標示 N·m·s/rad，範圍 0–10 | 先保留裝置已有值，再評估調整 |
| `stiffness` | 每關節陣列；UI 範圍 0–10，標題未標物理單位 | 不可直接假定與現有 MIT Kp 等價 |
| `torque_threshold` | 每關節陣列；UI 標示 N·m，範圍 0–10 | 是導納參數；不能當作最大允許接觸力或硬體急停閾值 |
| `torque_source` | IPC 使用此欄位；可由 scalar 展開成陣列 | Pilot 相容舊欄位 `force_feedback_source`；需確認裝置的來源及感測器覆蓋 |
| `stiffness_coeff` | Pilot 另發一筆通用參數寫入 | 與 `stiffness` 的作用關係、是否持久化仍需確認 |

上述數值是 **Pilot UI 接受的範圍，不是本專案建議的實機調參值**。目前沒有足夠證據給出適合這兩隻手的 M/D/K 起始值。

Pilot `out/main/index.js` 的實際封裝結構如下（結構說明，不是可直接執行的 Python SDK 寫入範例）：

```text
SET_PARAMETER payload
├─ control_parameter_type: 0       # 導納參數類型；不是控制模式 14
├─ sn: 選定手部序號
└─ params
   ├─ mass: array
   ├─ damping: array
   ├─ stiffness: array
   ├─ torque_threshold: array
   └─ torque_source: array        # 如來源存在，scalar 依 mass 長度展開

另一筆 SET_PARAMETER payload
└─ stiffness_coeff: scalar
```

Pilot 前端取來源時使用 `force_feedback_source ?? torque_source ?? 0`，並可能只取陣列第一個值，再由 IPC 展開。新 ROS 實作不宜照抄此預設及展開策略：應檢查實際完整陣列，不應在讀取失敗時自行選定來源。

Pilot 的傳輸是其內部 `request(SET_PARAMETER, ...)`，不是直接呼叫 Python `SharpaWave.set_parameter()`。因此這份 schema 可作為原廠實作線索；是否可原封不動交給 Python SDK、或應傳平鋪欄位，仍需要原廠文件／契約驗證。

## 6. 建議的 ROS 接入方式

優先接入手部原廠導納模式。從模式列舉與 Pilot 參數設計判斷，這是裝置控制模式的設定路徑，並不需要先在 ROS 重寫導納積分器；其內部控制方程、迴圈位置與頻率仍需原廠確認。

| 檔案／模組 | 建議改動 |
| --- | --- |
| `launch/dual_sharpa.launch.py`、`single_sharpa.launch.py` | 加入 `admittance` 選項，明確選擇對應設定檔 |
| `config/dual_sharpa_admittance.yaml`（建議新增） | 分離導納模式設定；第一版保留裝置既有導納參數，不填入猜測增益 |
| `dual_sharpa_wave/hand_node.py` | 新模式驗證、模式適用的命令欄位檢查、診斷與逾時處理；保持參數在啟動時選定 |
| `dual_sharpa_wave/sharpa_sdk_hand.py` | 明確 mode mapping、模式讀回、已確認的導納命令路徑、參數讀取與錯誤處理 |
| `dual_sharpa_wave/hand_interface.py` | 若確認輸入為關節位置參考，可沿用 `set_joint_positions()`，不必先改上層 topic |
| `mit_limits.py` 與 recovery/state 流程 | 抽出可共用的限制與接管檢查；導納停止／恢復策略另定，不直接套用 MIT |
| 導納參數讀寫模組（後續新增） | 集中處理 schema、單位、陣列長度、SDK 排序、讀回比對與版本差異 |

建議維持的接口是 `/sharpa/<side>_hand/joint_command` 和 `joint_states`。前提是確認它們在導納模式代表「位置參考」；若原廠要求力矩參考或其他命令，才新增有明確單位的專用 message。

目前 SDK/ROS 排序表都是 identity，但仍應經過 `SDK_TO_URDF_INDEX`、`URDF_TO_SDK_INDEX`，不要把此巧合寫死。Pilot 參數陣列也必須核對關節順序與長度，不能只因這隻手有 22 DOF 就視為已驗證。

現有 `/mit_gains`、MIT Kp/Kd baseline 與 MIT recovery action 都不能直接作為導納調參／恢復接口。MIT 的 0.5 秒逾時只將命令視為 idle，不會解除馬達使能或清除目標；導納模式在沒有新命令時仍可能因外力產生位移，必須另訂並驗證行為。現在非 MIT 的 timeout 分支則會 stop/disconnect，也不應未經確認就當成導納的完整停止策略。

## 7. 不建議直接拿現有觸覺或 FANUC 導納程式替代

現有 `/sharpa/<side>_hand/tactile/<finger>/f6` 是六分量 `Float64MultiArray`。`tactile.py` 明確只保留 SDK 數值，不賦予物理單位／軸定義；這個 message 本身也沒有時間戳與 frame。觸覺發布 30 Hz，資料不是已確認的 22 關節外力矩。

因此，若另外做 ROS 外迴路導納，還要補齊量測單位、座標、接觸點、Jacobian／力矩映射、時間同步、濾波及過期資料處理。這是另一項控制器開發工作，不能視為「啟用 Sharpa 原廠導納」。原廠的 `torque_source` 也不能未經確認就等同上述觸覺 F6。

`src/dual_crx_control/scripts/admittance_hand_guiding.py` 雖然名稱含 hand guiding，使用的是 FANUC flange 與手臂導引流程，並非 Sharpa 22 關節的原廠導納 backend。

## 8. 進入實機測試前要釐清的項目

1. **韌體與型號**：讀回左右手實際版本、hardware variant、模式及控制來源；確認該組合支援導納。
2. **指令契約**：確認 `set_joint_position()` 在模式 14 下是否有效、位置是平衡位置還是其他參考、插值是否可用、建議命令頻率。
3. **參數契約**：確認 `get_parameter()` 支援的 key、回傳格式，以及 `set_parameter()` 要用 Pilot 封裝還是平鋪格式；先以讀取與原廠文件確認，避免盲試寫入。
4. **參數語意**：確認 stiffness 單位、`stiffness_coeff` 關係、來源 0/1 的實際支援關節、校正要求、threshold 行為與參數持久化。
5. **切換與停止**：確認切換模式時舊目標如何處理、是否重設積分狀態，以及命令中斷、外力持續、SDK stop／disconnect 時的動作。
6. **原廠驗證流程**：取得適用此硬體的既有導納設定或 sample；不要用 MIT Kp/Kd 換算成 M/D/K。

## 9. 建議驗證順序與驗收條件

1. 保存實際模式、控制來源、MIT 設定與導納參數讀回，形成可比對的 baseline；實機操作時避免 Pilot 與 ROS 同時控制同一隻手。
2. 先用 Pilot 原廠功能在單手進行小幅度驗證，使用已確認的既有設定，確認外力方向、回復行為與停止流程。
3. ROS 新增模式後，先做離線 fake SDK 測試：模式 14 對應與讀回失敗、禁止誤送 MIT 指令、參數不完整／排序錯誤、命令接管與逾時分支。
4. 實機從單手、單關節的小幅位置參考開始；確認另一隻手未被改動，啟動不出現目標跳變，位置回授可持續取得。
5. 在受控條件下確認外力造成的柔順行為、命令中斷與解除外力後的結果、限制與故障處理，以及回到原 MIT 設定後的讀回。

本次評估完成的是接口可行性與接入設計，尚未實作／執行以上測試，也未宣稱導納實機控制已可用。

## 10. 使用導納控制要準備的輸入

| 類別 | 應準備的資料 | 目前確認程度 |
| --- | --- | --- |
| 裝置設定 | 指定手部、控制模式、控制來源 | SDK 有對應接口；實際韌體支援待讀回 |
| 導納參數 | M=`mass`、B=`damping`、K=`stiffness`，以及既有 `torque_threshold`、`torque_source`、`stiffness_coeff` | Pilot 有設定流程；Python 讀寫契約仍待驗證 |
| 運動參考 | 目前姿態與預計送出的 22 關節參考位置，單位 rad | 可沿用現有位置產生器的候選方案；須確認模式 14 的位置命令語意 |
| 外力回授 | 原廠 `torque_source` 對應的量測來源及適用關節 | 需確認來源、校正與有效性；不是已確認要從 ROS 傳入 F6 |

導納參數預計在初始化或調參時設定，不需要在每筆位置命令重送。ROS 位置發布頻率與裝置內部導納迴圈頻率須分別確認。

目前沒有證據顯示啟用原廠導納必須另外開發 ROS 外力輸入程式；應先釐清原廠力矩來源。也尚未確認此模式能接受目標接觸力，因此不把「抓取力目標」加入目前的位置命令介面。

## 11. M／B／K 微調 Python 工具設計（尚未實作）

### 11.1 目的與命名

規劃提供與現有 MIT 調參工具用途相似的 Python 入口，例如 `script/adjust_admittance_params.py`。操作方式可參考現有 GUI，呈現逐關節參數與比例微調；GUI 或命令列形式尚未定稿。此處的檔名、服務名稱與操作流程均為提案，不是目前可執行的功能。

介面使用 **M／B／K**，對應 SDK／Pilot 的 **mass／damping／stiffness**。先前文件中的 M／D／K，D 同樣代表阻尼，並非另一組參數。

### 11.2 以哪一組「原廠設定」為基準

優先使用 Sharpa 提供、適用此手部型號及韌體版本的設定，並與實際裝置讀回比對。若目前只能取得裝置保存值，應標為「裝置初始讀回基準」，**不能直接稱為出廠值**：裝置可能已被 Pilot 或其他程式調整過。

建立基準時保存手部序號、左右手、韌體／SDK 版本、來源說明、讀取時間、完整關節順序，以及 M／B／K 和相關參數的原始讀回。左右手各有自己的基準，不假定數值相同。

首次取得完整且有效的讀回後，才允許保存為基準。一般 Refresh／Read 僅更新目前值；套用微調後不得自動覆蓋基準。重新啟動工具時可載入同一份基準，並檢查裝置身份、版本與關節順序。

### 11.3 預計提供的操作

| 操作 | 設計行為 |
| --- | --- |
| Read | 讀取目前 M／B／K 及相關設定，顯示序號、模式、來源與讀取狀態 |
| Save／Load baseline | 保存／載入具來源資訊的基準檔；不因重新讀取而默默重設 |
| 比例微調 | M、B、K 各有獨立 ratio，預設 1.0；可作用於全手或選定關節 |
| 逐關節編輯 | 在已核對的 22 關節順序下編輯絕對目標值，顯示目前值、基準值與目標差異 |
| Preview | 檢查數值、單位及範圍，顯示預計修改內容；編輯與預覽不寫硬體 |
| Apply | 經同一個控制擁有者執行參數寫入，再讀回比對結果 |
| Restore baseline | 將目標欄位恢復為保存的基準；仍須 Apply 才寫入裝置 |
| 記錄結果 | 保存修改前、預計目標、實際讀回及錯誤；不要將失敗的目標顯示成目前值 |

比例一律以固定基準計算，對第 i 個選定關節：

```text
M_target[i] = M_baseline[i] × M_ratio
B_target[i] = B_baseline[i] × B_ratio
K_target[i] = K_baseline[i] × K_ratio
```

例如 `B_ratio=1.05` 只是「基準阻尼增加 5%」的操作示例，不是實機建議值。連續套用兩次仍應是基準的 1.05 倍，不能變成 1.1025 倍；ratio=1.0 回到保存基準。若基準 B 或 K 為零，乘法無法改變它，須用明確的絕對值編輯。

選定關節以外的值應取自套用前的新鮮讀回並保持原值，不能用舊基準覆蓋其他關節。選擇比例預覽時要顯示哪些目標欄位將被重新計算，避免悄悄覆蓋逐關節手動編輯。

### 11.4 原廠範圍與精調原則

- M 使用 Pilot 的 0.001–10 範圍，單位 kg·m²；B 使用 0–10，單位 N·m·s/rad；K 使用 0–10，但目前不替原廠補上未確認的單位。
- 這些是 UI 範圍，還需核對韌體限制。遇到範圍外、NaN／Inf、長度或關節順序錯誤，回報具體問題，不靜默裁切或補預設值。
- 第一次以已確認的原廠／裝置基準開始，先針對單手、選定關節、單一參數做小幅變更，再觀察柔順、回復及振盪行為；本文件不指定通用最佳 M／B／K。
- 一般導納概念下，M 影響加速反應，B 影響阻尼，K 影響偏離參考位置後的回復；實際行為仍受 Sharpa 的 `stiffness_coeff`、門檻、力矩來源及內部控制設計影響，不能只憑三個數值推算接觸力。
- 初版只讓使用者微調 M／B／K。`torque_threshold`、`torque_source`、`stiffness_coeff` 作為背景設定讀取與顯示；若原廠寫入契約要求整組 payload，須帶入新鮮讀回值並確認它們未被改動。

### 11.5 與 ROS／SDK 的整合方式

建議導納 backend 完成後，調參工具走專用 ROS service，例如 `/sharpa/<side>_hand/admittance_params`，由現有 hand node 序列化參數讀寫；避免 GUI 與控制節點各自建立一條 SDK 連線。這可參考目前 MIT 工具「GUI → service → adapter」的架構，但要使用獨立的參數契約與基準管理。

實作前若需獨立的 SDK 讀取工具，應限定序號，先確認該手沒有其他控制程式佔用。這種診斷連線與正式在線調參介面分開設計，不能假定兩者可同時操作。

調參操作不隱含模式切換、馬達 enable 或位置命令。模式由控制啟動流程負責，Apply 前須確認模式及狀態允許修改，並重新檢查參數是否已被其他操作改動。寫入成功須以讀回比對為準；若逾時或讀回不一致，狀態標為未確認，停止後續寫入並要求重新讀取，不盲目重試或自動切換另一種 JSON 格式。

目前 MIT 工具的兩步增益過渡，不能直接套用到 M／B／K。導納是否需要漸變、每次寫入是否重設內部狀態、允許的調整速率，均需原廠確認；初版以靜止調參為目標，不先承諾運動中連續調參。

### 11.6 後續實作與驗證順序

1. 取得可用的裝置讀回／原廠文件，確認 M／B／K 陣列、關節映射、寫入 schema、參數持久化及模式要求。
2. 完成基準檔、比例計算與預覽，再接上參數 service／SDK 寫入；不在契約未明時嘗試寫入。
3. 離線測試固定基準不累乘、重啟載入、選定關節外不變、錯誤裝置／排序／數值拒絕、讀寫失敗及讀回不符。
4. 實機先核對保存值，再執行單關節小幅調整與讀回；確認相關非 M／B／K 設定未變，並驗證恢復保存基準。

**本階段完成到文件設計。調參 Python 檔、ROS service、導納 backend 及以上驗證尚未實作；此次搜尋也未取得手部實際 M／B／K。**

## 12. 來源索引

- [目前 SDK adapter](../dual_sharpa_wave/sharpa_sdk_hand.py)：模式限制、啟動、MIT 指令及 timeout 行為。
- [ROS hand node](../dual_sharpa_wave/hand_node.py)：命令 topic、欄位驗證、回授及模式專用 service/action。
- [後端接口](../dual_sharpa_wave/hand_interface.py)、[關節排序](../dual_sharpa_wave/joint_names.py)。
- [MIT 預設設定](../config/dual_sharpa_mit.yaml)、[雙手 launch](../launch/dual_sharpa.launch.py)、[單手 launch](../launch/single_sharpa.launch.py)。
- [MIT 參數讀取](../dual_sharpa_wave/mit_control.py)、[觸覺資料處理](../dual_sharpa_wave/tactile.py)。
- [現有 MIT 調參 GUI](../dual_sharpa_wave/mit_gains_gui.py)、[MIT 調參 service](../dual_sharpa_wave/mit_gain_server.py)、[MIT 增益基準與過渡](../dual_sharpa_wave/mit_gains.py)：供後續工具架構參考，不代表導納可沿用相同參數或過渡策略。
- `/opt/sharpa-wave-sdk/include/SharpaWaveSDK.h`：enum 約第 84–99 行；位置／MIT 命令約第 711–741 行；模式切換使能語意約第 782–794 行；通用參數約第 966–982 行。
- `/opt/sharpa-wave-sdk/VERSION`、`sdk-release-info.json`：版本與發行相容資料。
- Python 離線檢查：`sharpa.ControlMode.__members__`、`SharpaWave` methods/docstrings，確認模式 14 與參數／位置接口。
- `/opt/sharpa-pilot/resources/app.asar`（以唯讀方式解析 archive，未啟動 Pilot）：`package.json`；`out/main/index.js` 的 `set_device_admittance_params`、`set_device_finger_stiffness_coefficient`、`get_device_config_by_type`；`out/renderer/assets/slaveHandView-gnXA2cr-.js` 的 `AdmittanceParams`；`out/renderer/assets/client-DQlRT4hZ.js` 的參數標題／單位。
