# TODO_NEXT.md

## 2026-09-13 特殊RID立即重绑定

### 已完成
- [x] current SORT保持4秒粘性，失效后默认不再等待额外45秒。
- [x] 失效槽位只按当前RID方位/俯仰二维角度从新鲜、确认、未归属SORT中执行全局一对一重绑定。
- [x] 单候选直接绑定，多候选选择当前RID最近者；已归属另一特殊RID的SORT不可抢占。
- [x] 保留默认0秒、可配置的 `SPECIAL_RID_REACQUIRE_DELAY_SECONDS`。
- [x] 覆盖误绑虚警消失、旧SORT几何干扰、4秒粘性、所有权保护、同周期UI输出和非零附加延迟测试。

### 下一步现场验证
1. 先只启用XDB RID，确认虚警短暂绑定并消失后，真实SORT在其达到 `ui_confirmed` 的同周期接管UI1。
2. XDB稳定悬停后再启用ST22Q，核对UI1/UI2不交换且摄像头来源随各自current SORT更新。
3. 保持打击发送关闭，统计 `IMMEDIATE_REACQUIRE`、`CURRENT_SORT_SWITCH` 和 `UI_SKIP`，确认不再出现额外45秒UI空挡。

## 2026-07-31 状态更新

### 已完成
- [x] 固化第三层 `logic_id=11~15` 的现场手动theta，并禁止再次叠加统一修正。
- [x] 其余未测试摄像头统一应用水平 `-2.0°`、垂直 `-1.85°` 修正。
- [x] 所有摄像头的SORT投影Y像素补偿统一设为 `0px`，撤销logic12/13旧专用补偿。
- [x] 视觉关联继续使用Y轴绝对残差的匈牙利一对一分配，不设置硬门限；X轴只记录、不参与匹配。
- [x] 关联日志增加来源摄像头、补偿前后Y、原始X/Y偏差和实际Y轴代价。

### 下一步现场验证
1. 有条件时抽测一至两个非第三层摄像头，确认统一theta修正后目标仍在云台视场内。
2. 核对启动日志中的第三层跳过列表、未测试摄像头统一角度修正和 `vision_y_compensation=0px`。
3. 按 `sort_source_logic_id` 继续统计原始 `association_dx/dy`，只在出现跨摄像头一致偏差时调整统一值。
4. 有两个Y位置明显分离的目标时检查匈牙利选择；Y位置接近的错绑风险本轮不作为验收项。

## 2026-07-27 状态更新

### 已完成
- [x] 新增视觉SORT投影、检测目标、完整候选代价矩阵和匈牙利选择专用日志。
- [x] 新增RID/视觉候选到最终SORT距离的裁决专用日志。
- [x] 删除通用事件中重复的视觉检测、抖动、匹配摘要、未匹配框和裁决记录。
- [x] 保留RID原始数据、四点候选及最终配对日志。
- [x] 将视觉结果默认新鲜期调整为3秒。

### 下一步现场验证
1. 有真实目标后按 `vision_frame_ts` 核对每帧 `SORT_PROJECTION/DETECTION/CANDIDATE` 数量和选中关系。
2. RID产生数据后核对四点候选、最终配对和 `distance_arbitration` 中的RID优先结果。
3. 测试多目标数量不相等和交叉场景，确认未选择候选及最终错绑情况可以从日志直接还原。

## 2026-07-26 状态更新

### 已完成
- [x] 在板端RID基线之上建立并跟踪 `cam_rid_distance` 分支。
- [x] RID四点匹配和云台视觉测距并行生成候选，候选范围统一为 `ui_tracks`。
- [x] SORT主线程实现固定裁决顺序：6秒内RID优先，否则当前云台目标使用新鲜视觉，否则输出 `NaN`。
- [x] 视觉回填改为无像素硬门限的匈牙利全局一对一分配；当前代价已更新为按来源摄像头补偿后的Y轴绝对残差。
- [x] RID四点配对样本按30秒历史窗口逐项过期。
- [x] RID/视觉来源切换时重置距离卡尔曼状态，避免跨来源突变被50米门控拒绝。
- [x] 保持UI UDP格式、云台控制、视觉模型和打击安全条件不变。

### 下一步现场验证
1. 单目标分别覆盖RID+视觉、仅RID、仅视觉、两者都无效，确认UI依次呈现RID、RID、视觉和 `NaN`。
2. 双目标静态场景核对匈牙利一对一结果及 `association_cost_y_px`，确认距离没有重复回填到同一SORT轨迹。
3. 统计 `DISTANCE_ARBITRATION` 中RID/视觉切换、RID抑制视觉以及无距离状态，核对6秒新鲜期。
4. 在真实云台、相机和RID同时运行时验证UI ID/距离一致性；测试期间保持打击发送关闭。

## 2026-07-24 状态更新

### 已完成
- [x] 板端仓库切换到 `RID` 分支，切换前未提交配置已安全保存。
- [x] 新增4个不同 `rid_measurement_seq` 的SORT–RID方位轨迹窗口。
- [x] 接入固定综合偏差 `b0=3.1198935°`、`Ebias<=2.0°`、`Eshape95<=2.5°` 两级硬门控。
- [x] 仅让通过门控的候选进入匈牙利一对一分配，禁止全拒绝时强制匹配。
- [x] 匹配候选仅使用 `ui_tracks/ui_ids`。
- [x] UI字段归属调整为SORT提供ID/方位/俯仰/相机来源，RID只提供距离，外部包格式不变。
- [x] 未完成RID匹配的SORT目标继续发送UI，距离字段明确置为 `NaN`。
- [x] UI状态包携带距离分档威胁评分：`<100m/100~300m/>300m` 对应 `100/50/0`，NaN距离对应NaN评分。

### 下一步现场验证
1. 服务启动后确认每个候选依次出现 `four_point_warmup_1/4` 到 `3/4`，第4个不同RID测量序号后才可能出现匹配。
2. 单机测试确认真实RID与对应SORT稳定输出 `four_point_gate_pass_selected`，周围其他RID保持拒绝。
3. 两机简单不交叉场景确认匈牙利结果是一一绑定；交叉场景暂不作为本版验收范围。
4. 统计现场新增日志中的 `curve_error_deg(Ebias)`、`shape_error_deg(Eshape95)` 和安全拒绝比例，再决定是否调整2°门限。

## 2026-07-04 状态更新

### 已完成
- [x] 新增云台相机 CPU-only YOLO 测距服务，默认设备名为 `000000008`。
- [x] 移植 `distance model` 的物理距离、MLP、运动门控和 25 帧 GRU。
- [x] 云台运动和画面模糊时禁止 YOLO 推理。
- [x] 全部有效 SORT 轨迹投影到云台画面，并通过动态 ROI 检测可见目标。
- [x] 使用匈牙利算法完成 YOLO 框与 SORT `track_id` 的一对一绑定。
- [x] 每个 `track_id` 使用独立测距时间序列，模型权重共享。
- [x] 增加双向最近邻、距离门限和歧义差值门限；歧义帧不写入距离序列。
- [x] 保持 UI 和打击端协议不变，云台控制仍只跟随 master。
- [x] `c40d_gimbal` 已配置 CPU 版 PyTorch/torchvision、YOLO 依赖和 pytest。
- [x] `tools/mock_udp_sender.py` 的生成坐标已从 4K 调整为当前检测端使用的 2K `2560x1440`。
- [x] 支持将 `20~250米.mkv` 作为云台相机源按31 FPS实时播放，并从指定帧开始与检测框UDP同步。
- [x] 视频全链路验证已覆盖 SORT、master、动态ROI、CPU YOLO、25帧GRU、UI距离和打击端数据包。

### 下一步现场验证
1. 标定固定摄像头统一角度与云台光轴之间的零偏，检查 SORT 投影点与真实云台目标中心的像素误差。
2. 用两个及以上真实目标验证 `track_id -> bbox -> distance` 日志，特别覆盖靠近、交叉、漏检和重新出现。
3. 根据标定误差调整 `GIMBAL_VISION_ASSOCIATION_MAX_PX`；门限应尽量小，但必须覆盖正常投影误差。
4. 根据目标密度调整 `GIMBAL_VISION_AMBIGUITY_MARGIN_PX`；现场优先避免错绑，不要为了提高测距率取消歧义拒绝。
5. 在 RK3588 CPU 上实测每帧 ROI 数、推理耗时和25帧预热时间，必要时限制候选轨迹范围或降低测距更新频率。
6. 检查真实清晰画面的拉普拉斯值，再确定 `GIMBAL_VISION_MIN_SHARPNESS`，避免阈值过高导致一直处于 `SETTLING`。
7. Linux 部署时设置 `GIMBAL_CAMERA_SOURCE=/dev/videoX`，不要沿用 Windows 设备名 `000000008`。

## 2026-05-30 状态更新

### 已完成
- [x] 新增 `DEVICE_HEADING_DEG`，用于描述设备自身 `0°` 方向在地图中的绝对方位。
- [x] UI 状态包 `azimuth` 发送前已转换为地图绝对方位角。
- [x] 转换只作用于 UI 发送，不影响追踪、卡尔曼、云台控制和抢占逻辑。
- [x] `UI_STATUS_SEND` 日志新增 `map_az` 字段，便于同时核对相对角 `pred_az` 和地图角 `map_az`。

### 下一步优先级
1. 现场部署时按设备实际朝向设置 `DEVICE_HEADING_DEG`，例如设备正方向朝正南时设为 `180`。
2. 用 UI 或 `udp_ui_receiver.py` 验证 `azimuth` 是否符合地图正北为 `0°` 的显示约定。

## 2026-04-22 状态更新

### 已完成
- [x] 新增 `gps.py` 的 UM980 NMEA 读取逻辑，支持持续等待有效 GGA 定位。
- [x] `gps.py` 已支持 `GPS_PORT` 环境变量覆盖，并按平台提供默认 GPS 串口。
- [x] `main_tracking_v9.py` 已在系统启动阶段启动 GPS 后台线程，获取一次经纬度后发送给 UI。
- [x] GPS 真实定位成功时发送真实经纬度；60 秒超时仍无有效定位时，发送 `gps.py` 中的默认经纬度。
- [x] GPS 线程发送一次 GPS 包后退出，不持续占用主流程决策逻辑。
- [x] UI 新增 GPS 包格式：`struct.pack('!Bff', 0x03, latitude, longitude)`。
- [x] 新增 `udp_ui_receiver.py`，用于模拟 UI 端监听 `9999` 并解析 `0x02` / `0x03` 数据包。
- [x] Linux 串口默认值已纳入 GPS：当前为云台 `/dev/ttyUSB0`、激光 `/dev/ttyUSB1`、GPS `/dev/ttyUSB2`、IMU `/dev/ttyUSB3`。

### 当前结论
- UM980 在室内无卫星时仍会持续输出 NMEA，但典型日志为：
  - `$GNGGA,,,,,,0,...`：无有效定位
  - `$GPGSA,,1,...`：未定位
  - `$GNRMC,,V,...`：定位无效
  - `$GPGSV,1,1,00,...`：可见 GPS 卫星为 0
- 该状态下主程序不会把无效 GPS 数据当成真实经纬度；启动等待超时后会发默认经纬度给 UI。
- 如果看到 `source=COM8` 或 `source=/dev/ttyUSB*` 且日志显示 `GGA fix_quality != 0`，才表示真实 GPS 坐标已获取。

### 下一步优先级
1. 部署前把 `gps.py` 中 `DEFAULT_LONGITUDE` / `DEFAULT_LATITUDE` 改成设备实际安装点坐标。
2. 在 Linux 目标机上确认 `GPS_PORT` 对应的确是 UM980，建议后续固定为 `/dev/serial/by-id/...`。
3. 在 UI 调试机运行 `python udp_ui_receiver.py --host 0.0.0.0 --port 9999`，验证 Linux 端启动后是否收到 `0x03` GPS 包。
4. 在室外或天线条件较好的位置复测真实 GPS fix，确认 UI 收到的是真实经纬度而非默认兜底值。

## 2026-04-16 状态更新

### 已完成
- [x] 将云台抢占阈值从单一 `GIMBAL_PREEMPT_DEG` 拆分为：
  - `AZ_PREEMPT_DEG = 0.5`
  - `EL_PREEMPT_DEG = 0.8`
- [x] 将云台抢占执行逻辑改为：
  - 同一 `track_id` 时按轴更新目标值
  - `track_id` 变化时整条命令替换
- [x] 补充抢占日志字段 `mode=track_switch/axis_update` 与 `axes=Az/El/Az+El`

### 当前结论
- 现在的“双轴”不只是分轴判定，而是分轴更新。
- 低方位阈值不再必然把俯仰目标一起带着刷新。
- 目标切换时仍保持整条命令替换，避免混合指向。

### 下一步优先级
1. 复测 `main_tracking_v9_20260414_162745.log` 对应的 `linear` 场景，重点看方位误差是否仍然每隔几条 `[GimbalAtt]` 明显放大。
2. 记录新的抢占日志，确认更多抢占是否表现为 `mode=axis_update, axes=Az`，而不是频繁整条替换。
3. 若方位锯齿误差明显缓解，再评估是否需要继续微调 `AZ_PREEMPT_DEG`；若俯仰仍被频繁更新，再审视 `EL_PREEMPT_DEG=0.8` 是否需要上调。

## 2026-04-14 状态更新

### 已完成
- [x] 将真实激光 `sddm_laser.py` 接入 `main_tracking_v9.py` 主流程
- [x] 新增 `USE_MOCK_LASER`，使真实/模拟激光切换方式与 `USE_MOCK_GIMBAL` 保持一致
- [x] 明确当前激光模式应使用“连续测量”，而不是“单次测量”
- [x] 按手册检查并补强 SDDM 激光驱动的数据帧校验
- [x] 完成真实硬件联调：云台 `COM3`，激光 `COM8`
- [x] 完成 15 秒 `linear` 长测：真实激光成功触发 19 次，`[Laser] No valid laser distance` 为 0
- [x] 完成 Linux 接收端 `10.72.2.28:8888` 的多轮 `linear` 实机复测，发送参数保持 `15FPS / 15s / cx=700 -> 3686.67 / cy=1080 / mono=320m`
- [x] 通过 `logs/main_tracking_v9_20260414_161730.log` 确认：出现“全程 No valid laser distance”时，并不等价于主流程没触发激光；该次运行中激光线程和连续测量命令都已正常启动
- [x] 通过 `logs/main_tracking_v9_20260414_162745.log` 确认：将激光指向更远目标后恢复稳定真实测距，统计为 `GimbalSettled=22`、`GimbalTimeout=0`、`Laser Triggered=22`、`Laser No valid=0`

### 当前结论
- 主程序已经真正使用真实激光，而不是继续只用 mono distance。
- 当激光回包有效时，`[Laser] Triggered ... dist=...m` 会在 `GimbalSettled` 后打印。
- 真实激光返回的是当前物理环境中的反射目标距离，最近一轮长测约为 `4.7m ~ 6.5m`；这与发送脚本中模拟的 `320m` 不矛盾。
- 室内联调时，“持续 `[Laser] No valid laser distance`” 当前优先解释为目标距离/反射条件不满足；在接收端初始化正常且目标改为更远目标后，激光已恢复稳定返回 `6.2m ~ 7.9m` 的真实值。

### 下一步优先级
1. 固定一个明确的真实反射目标，验证激光距离是否与该实物距离一致，而不是仅确认“有值”。
2. 系统化记录“无效 -> 有效”的目标条件边界：距离、材质、反射面角度、室内/室外光照。
3. 若激光偶发回到无效帧，补抓原始激光回包，确认是盲区、反射条件，还是串口噪声。
4. 继续复测 `15FPS linear` 场景，记录 `GimbalSettled / GimbalTimeout / Laser Triggered` 的长期稳定性。
5. 若需要让激光距离更严格地对应当前追踪目标，再评估是否增加更强的“云台稳定窗口”或多次激光采样确认逻辑。

## 本文件用途
给下一位接手者快速了解“已经做了什么、下一步要先验证什么、哪些任务还没开始”。如果需要理解为什么做这些改动，请看 `DECISIONS.md`；如果需要理解系统整体架构，请看 `PROJECT_CONTEXT.md`。

## 当前阶段 (Current phase)
基于真实动态场景（如 320m 距离，5.1m/s 速度）的卡尔曼速度估算与云台闭环控制专项调优。

## 下一步优先级
1. 先完成目标运行平台的串口配置复核：Windows 下确认 `COMx` 端口，Linux 下确认 `/dev/ttyUSB*` / `/dev/ttyACM*` 设备节点，并同步到 `GIMBAL_PORT` / `LASER_PORT` / `IMU_PORT`。
2. 再复测已改参数：15FPS `linear` 场景，确认速度估计、抢占次数、到位/超时比例、俯仰首捕获时间。
3. 如果速度估计正常但跟随仍滞后，再做 `PREDICT_DELAY` 双轴拆分。
4. 如果低速/高速目标需求冲突明显，再实现 Dual-Phase Tracking，把首次捕获和稳定跟踪参数分离。
5. 激光测距集成排在控制闭环稳定之后，除非当前测试必须依赖真实激光距离。

## 已完成 (Done)
- [x] 完成主运行程序及多模块集成的基础搭建。
- [x] 通过 `udp_sender_tracking_scenarios.py` 完成了 `linear` 场景的高速测试 (320m, 5.1m/s)。
- [x] 生成并分析了专项测试日志 `main_tracking_v9_20260409_154819.log`。
- [x] 确诊了卡尔曼速度低估的根因：`MIN_DT=0.08` 与实际 15 FPS 不匹配。
- [x] 确诊了云台震荡的根因：`PREEMPT` (0.7) 与 `SETTLE` (0.8) 阈值过于接近。
- [x] 已在 `main_tracking_v9.py` 将 `MIN_DT` 调整为 `0.001`，仅保留异常极小 `dt` 保护。
- [x] 已将迟滞参数从单一 `GIMBAL_PREEMPT_DEG=1.5` 演进为分轴基线 `AZ_PREEMPT_DEG=0.5`、`EL_PREEMPT_DEG=0.8`，`GIMBAL_SETTLE_THRESHOLD=0.3` 保持不变。
- [x] 已新增启动位姿初始化：系统启动后先通过云台控制队列下发 `Az=GIMBAL_AZ_BASE`、`El=0.0°`，保持单一控制线程所有权。
- [x] 文档和代码运行背景已调整为双平台；串口默认值和校验逻辑已支持 Windows `COMx` 与 Linux `/dev/...`。

## 当前关注的痛点/问题 (Current concerns)
1. **俯仰初始化慢**：已通过启动位姿初始化部分缓解，但需要真实硬件复测确认首次锁定时间是否明显缩短。
2. **速度估算保守**：代码已移除 `MIN_DT=0.08` 的实际帧率抬高问题，需用 15FPS `linear` 场景复测估算速度是否接近理论 `~0.91°/s`。
3. **参数一刀切**：“快速拉近”和“抑制抖动”仍未完全分离，后续仍需要 Dual-Phase Tracking 参数化。
4. **边缘横跳 (Thrashing)**：迟滞参数已初步调整，需复测 `preempt` 是否不再在 `0.71~0.74°` 附近频繁触发。
5. **动力学未解耦**：单一的 `PREDICT_DELAY=0.25` 仍未拆分为方位/俯仰独立提前量。
6. **激光有效条件边界未量化**：当前已确认同一套代码下，室内目标条件变化可导致“整场无效帧”和“整场稳定有效帧”两种结果，后续需把距离/材质/瞄准条件记录清楚。

## 推荐的后续核心技术任务 (Recommended next technical tasks)
- [x] **自适应时间步 (Adaptive Kalman DT)**：已将固定 `MIN_DT=0.08` 改为异常保护值 `0.001`，让 Kalman 使用实际收包间隔；待复测确认速度估算。
- [ ] **双轴独立前馈 (Decoupled Feedforward)**：将 `PREDICT_DELAY` 拆分为 `AZ_PREDICT_DELAY` 和 `EL_PREDICT_DELAY`，为较慢的俯仰轴设定更大的预测提前量。
- [ ] **追踪状态机 (Dual-Phase Tracking)**：区分“首次跟踪 (Acquisition)”与“稳定跟踪 (Stable)”，两阶段应用不同的预测权重和死区参数。
- [x] **扩大控制迟滞 (Widen Hysteresis)**：已设为 `SETTLE_THRESHOLD=0.3`，并在 2026-04-16 将抢占阈值更新为 `AZ_PREEMPT_DEG=0.5`、`EL_PREEMPT_DEG=0.8`；待硬件复测观察 `GimbalSettled` 与 `GimbalTimeout` 比例。
- [x] **初始化位姿优化 (Init Posture)**：已在启动时预设云台到 `Az=GIMBAL_AZ_BASE`、`El=0.0°`；待硬件复测确认首捕获延迟。
- [x] **集成激光测距 (`sddm_laser.py`)**：后台线程、连续测量、校验与主流程绑定已跑通。
- [ ] **量化激光有效工作区间**：补测近距离、远距离、不同反射目标下的 `Laser Triggered / No valid` 比例。

## 下一次启动检查清单 (Next startup checklist)
1. 阅读 `AGENTS`, `PROJECT_CONTEXT`, `TODO_NEXT`, `DECISIONS`。
2. 打开 `main_tracking_v9.py`。
3. 在目标平台确认 GT06Z、激光、IMU 的串口设备名，并同步到 `GIMBAL_PORT` / `LASER_PORT` / `IMU_PORT` 配置。
4. 检查当前基线：`MIN_DT=0.001`、`AZ_PREEMPT_DEG=0.5`、`EL_PREEMPT_DEG=0.8`、`GIMBAL_SETTLE_THRESHOLD=0.3`、启动归位 `Az=GIMBAL_AZ_BASE` / `El=0.0°`。
5. 运行 `linear` 测试 (320m, 5.1m/s, 15FPS, `cx=700 -> 3686.67`, `cy=1080`)。
6. 重点观察日志中的：预估速度是否接近 `~0.91°/s`、是否消除了 `0.71~0.74°` 附近的边缘抢占、`GimbalSettled` 是否稳定触发、是否出现更多 `GimbalTimeout`、俯仰角首捕获是否更快。
7. 若日志已出现 `[Init] Real laser enabled ...`、`[LaserThread]` 与 `[Laser] start_measurement mode=continuous`，但 settled 后仍持续 `No valid laser distance`，先调整激光朝向更远或更强反射目标，再怀疑代码路径问题。
