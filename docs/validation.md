# 驗證紀錄

日期：2026-09-21。環境：Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12.3。

## 2026-09-22 預覽開關簡化

預覽改為只由 `use_rviz` 控制；`ros2 launch ... --show-args` 已確認只列出 `config_file`、`publish_rate_hz`、`use_rviz`。
43 項非整合測試通過；兩項 DDS 整合測試在允許 localhost 通訊的環境下分別通過，共 45 項。
`use_rviz=true` 測試透過 WSLg 啟動實際 RViz，驗證左右獨立 TF 更新與程序正常退出。測試需要可用的圖形顯示環境；Qt offscreen 模式在本機無法建立 RViz 的 OGRE render window。

## 結果

| 檢查 | 結果 |
|---|---|
| `colcon build --symlink-install --packages-select dual_sharpa_wave` | 成功 |
| `colcon test --packages-select dual_sharpa_wave --event-handlers console_direct+` | 45 passed |
| `colcon test-result --verbose` | 45 tests, 0 errors, 0 failures, 0 skipped |
| 四個 public topics / types / QoS | 通過 DDS 整合測試 |
| 左右兩個獨立 hand process | 通過；停止左手後右手仍可接受新 command |
| Command validation、name reorder、timeout、feedback、cleanup | 通過 |
| Mock 不載入官方 SDK | 子程序 import guard 通過 |
| 模型 joint order、meshes、左右獨立 TF | 通過；共 70 個 preview child frames |
| 手動 CLI pub/echo | 左手第一關節 0.1 rad、右手第一關節 0.2 rad，互不影響 |
| RViz GUI smoke test | 程序與兩個 state publishers 正常啟動、正常退出 |

測試使用 localhost discovery 與獨立 ROS domain；沒有對任何實機送 command。
`fanuc_driver`、`fanuc_description` git status 在實作前後皆乾淨。本 workspace 未找到既有 `dual_crx_control` package，未建立或修改它。

## 測試範圍與限制

- SDK 5.0.10.6 僅下載解壓並靜態閱讀；未安裝或執行。`SharpaSdkHand` 仍為 fail-fast placeholder。
- 沒有驗證 SDK binary、實機網路、serial 綁定、馬達方向／零位、動態、負載或硬體 timeout 處置。
- RViz 的模型 assets 與 TF 已自動驗證，GUI 已啟動，但 WSLg 的 X11 截圖失敗，未完成畫面的人工視覺檢查。
- 本機 DDS graph discovery 需數秒，CLI `echo` 指定 `sensor_msgs/msg/JointState` 避免自動型別判斷競態。
- GUI smoke test 中曾出現 ROS 系統時間倒退的 state publisher warning；mock 的運動與 timeout 使用 monotonic/steady time。State timestamp 仍依規格使用 ROS clock。

## 本機證據位置

- 自動測試：`~/ws_fanuc/build/dual_sharpa_wave/pytest.xml`
- CLI 結果：`/tmp/dual_sharpa_manual_results.txt`
- GUI/launch log：`/tmp/dual_sharpa_manual_launch.log`

`/tmp` 檔案屬暫存，清理後可按 README 與自動測試重新產生結果。所有本次啟動的 mock、state publisher 與 RViz 程序已結束。
