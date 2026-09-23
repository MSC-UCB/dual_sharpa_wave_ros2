# Sharpa hardware adapter

目前可離線驗證 adapter，尚未用官方 SDK binary／實機完成連線或運動測試。Mock 與控制工具不需要 SDK；Fake SDK 僅存在於 tests，沒有可切換成 fake 的 hardware runtime 模式。

## SDK 來源與核對範圍

靜態核對 [官方 SDK v5.0.10 release](https://github.com/sharpa-robotics/sharpa-wave-sdk/releases/tag/v5.0.10) 的 `sharpa-wave-sdk_5.0.10_amd64.deb`：

- `VERSION`：5.0.10，`BUILD_ID`：5.0.10.6。
- SHA-256：`e3fcc7554e9666919887b793450bb5be4f4c37c6c877c0d0078731dd5c51b54d`。
- `sample/python/sharpa_wave_example.py`：manager、serial 連線、設定順序、radian command、degree feedback。
- `include/SharpaWaveSDK.h`：`Error.code == 0` 為成功；`start()`／`stop()` 回傳 bool；`disconnect(serial)` 回傳 void。
- Python native extension 的匯出名稱包含上述方法。SDK 僅解壓到 `/tmp` 供閱讀，未安裝或 import／初始化。

採用 sample 的 mode → speed → current → control source → start 順序，但不複製 sample 的自動選第一隻裝置及自動全零姿勢命令。使用 `manager.disconnect(serial)` 清理自己的連線，不呼叫 `disconnect_all()`。

SDK header 記錄 Wave SE 會將 speed/current 分別限制到 0.5／0.6。Adapter 對輸入做 `(0, 1]` 驗證；實際裝置接受值與行為須按手型核對，不能將設定值視為已驗證的實際馬達響應。

## 在接有 Sharpa 手的主機上使用

依官方套件說明安裝相符 SDK 和其系統依賴；以下假設安裝位置為 `/opt/sharpa-wave-sdk`，Python ABI 與 ROS 的 Python 相容。套件內的 Python loader 提供多個版本的 native extension。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_fanuc/install/setup.bash
export PYTHONPATH="/opt/sharpa-wave-sdk/python${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="/opt/sharpa-wave-sdk/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

複製 `config/dual_sharpa_hardware.yaml`，填入左右手實際 serial（以字串儲存）。不可用 discovery 順序或假 serial 代替。兩側 serial 空白或相同，launch 會在建立兩個 hand node actions 前拒絕。

```bash
ros2 launch dual_sharpa_wave dual_sharpa.launch.py \
  backend:=sharpa_sdk config_file:=/absolute/path/my_sharpa_hardware.yaml use_rviz:=true
```

這個指令會連接並啟動 SDK session；目前機器沒有實機時請使用 mock launch。`use_rviz` 是獨立手模型預覽，並不代表手已校正／安裝到 CRX；正式 combined model 應關閉獨立預覽。

先確認兩側 `joint_states` 是可用回授，再用 GUI 單側單關節小幅測試角度／方向，最後使用同一組 sine／step 工具。不需修改控制腳本的 topics。

## 失敗與 timeout

- SDK 只在 `SharpaSdkHand.start()` lazy import；缺少 SDK 或 shared library 時回報載入失敗，不 fallback。
- `sdk_discovery_timeout_sec` 預設 10 秒，只等待指定 serial 出現在 discovery。`connect()` 等 native API 的單次呼叫沒有已核對可用的 timeout 參數，因此目前不提供 native 呼叫的硬性時間上限；同一 ROS 執行緒遇到 native 阻塞時，command watchdog 也可能延遲。
- 啟動完成前讀取一次 feedback；初始化中途失敗會嘗試 stop／disconnect，清理錯誤一起回報。
- 寫入 status 失敗、讀取 status 失敗或 feedback 非 22 維有限數值時，adapter 關閉該 SDK session 並鎖定錯誤；後續命令拒絕，需重啟 hand node。
- 第一筆接受的 command 之後，若 `command_timeout_sec` 大於 0 且超時沒有新 command，會呼叫 SDK stop 並斷線。開始控制前的等候只發出 warning，不觸發此關閉流程。`<=0` 停用 command timeout；目前提供的真实硬件配置使用 `0.0`，因此 session 只在 launch/node 關閉時停止。
- GUI 停止傳送、waveform 結束或 Ctrl-C 後不自動回零。Hardware timeout 關閉 session 後，需要重啟 hand node 再進行下一次控制；重新連線不會自動重播最後 target。
- `stop()` 即使失敗也會嘗試 disconnect；SDK bool/status 失敗會被記錄。SDK 的停止／斷線不等於經過驗證的實體急停，實際馬達效果與斷線處置仍需現場核對。

## 離線測試能證明什麼

`SharpaSdkHand(..., sdk_factory=...)` 可注入 module-shaped Fake SDK。測試 double 實作官方 sample/header 中使用到的 API，回傳獨立的 feedback，也可注入錯誤；它不會呼叫網路或官方 SDK。

測試涵蓋指定 serial、設定順序、start/stop bool、Error status、命令 interpolation、非 identity joint mapping、degree/radian 轉換、discovery 逾時、初始化／讀寫／cleanup 失敗、command timeout 鎖定。這些驗證 adapter 邏輯，不證明 native library 相容、SDK discovery、雙程序連線、韌體、實際角度零位、馬達速度或停止效果。
