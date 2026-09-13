# AGENTS.md

## 2026-09-13 打击端增加站点GPS经纬度
- 打击端UDP包由13字节扩展为21字节，网络字节序主体固定为 `!2sBBHHhii`：`AA55`、长度 `0x15`、目标ID、距离、方位、俯仰、经度 `int32`、纬度 `int32`、完整主体逐字节异或校验、`55AA`。
- 经纬度编码严格为 `int(coordinate * 100) * 3600`，先向0截断到两位小数对应整数，再乘3600；经度在前、纬度在后，两字段均计入原异或校验。
- 坐标只能来自板端GPS线程维护的站点 `SharedPositionState`，不是目标RID坐标。要求站点位置有效、经纬度有限且范围合法，明确拒绝 `default/configured_default` 预设来源；否则清空周期打击快照并停止发送，禁止填0。
- UI `0x02`/`0x03`、目标ID、距离/角度编码、云台控制和打击安全门控均未改变。打击时序日志与通用事件日志增加经纬度诊断字段。

## 2026-09-13 两个特殊RID固定UI身份与SORT轨迹族
- 仅 `1581F6W8W255D0020XDB`、`1581F986425C800ST22Q` 启用 `core/special_rid_identity.py`。以“RID存在且首次绑定一个现有 `ui_confirmed=True` SORT”为注册时刻，先成功者动态获得UI_ID 1，后成功者获得2；UI_ID、SORT generation所有权和轨迹族保持到进程退出。
- 特殊RID的UI `0x02` 仍为34字节 `!BB8sIffffI`，`replaced_target_id=0`。目标ID来自特殊RID槽位；方位、俯仰、距离和威胁来自RID实时预测；SORT只提供视觉有效性、`board` 和 `camera_id`。
- 特殊RID独立5Hz发送：RID最后真实测量后0~5秒实时外推，5~7秒冻结5秒位置，RID接收年龄超过7秒停止；current SORT检测年龄超过4秒也停止。预测点不写回真实测量历史。
- SORT接续复用既有 `ui_confirmed`，不新增第三层计数。同摄像头6°、相邻摄像头9°、12秒内双轨收敛1.5°；候选generation必须晚于RID出现和至少一个族内前代，禁止吸收RID出现前的旧SORT。current SORT在4秒内新鲜时保持粘性；超过4秒后不再等待旧45秒门限，也不再参考可能误绑的旧SORT几何，而是在同一周期从新鲜、确认、未归属SORT中按当前RID方位/俯仰二维角度执行全局一对一最近重绑定。附加延迟由 `SPECIAL_RID_REACQUIRE_DELAY_SECONDS` 配置，默认 `0s`。
- 已归属特殊RID的SORT generation永久从普通UI发送路径排除；特殊ID固定占用1/2，普通UI分配器在特殊注册表启用时从3开始，避免全局目标ID冲突。普通测距runtime、云台主目标、控制或打击候选保持不变。`logs/20260912_032537` 从 `1789156230.355754` 回放得到：XDB/UI1族 `{177}`，ST22Q/UI2族 `{189,196,202,205,238}`，最终SORT238仍归ST22Q且未被XDB抢占。

## 2026-09-10 RID切换UI目标时的旧轨迹删除通知
- UI `0x02` 状态包固定为网络字节序 `!BB8sIffffI`、共34字节；末尾 `replaced_target_id` 为4字节无符号整数，`0`表示无旧目标需要删除，非0表示UI应按全局 `target_id` 立即删除旧轨迹。本协议不兼容旧版30字节UI包，追踪端和UI端必须同步升级。
- 只有正数且有限的最终距离才允许发送 `0x02`。所有RID按 `rid_item["key"]` 独立记录最近绑定的UI目标ID；同一RID切换到新UI目标后，新目标连续3个成功发送的有效包都携带旧ID，未发包、距离无效或UDP发送失败均不消耗次数，快速连续切换按FIFO处理。
- 视觉距离不建立替换关系，字段始终为0。被RID确认替换的旧UI ID禁止仅凭视觉距离重新发送；若该ID重新获得任意RID正式绑定，则取消其抑制和待删除任务，避免误删当前真实RID目标。
- RID暂时无匹配、距离过期或SORT目标短时退出 `ui_tracks` 时保留替换状态；只有RID连续300秒无上报并永久删除时才清理。`UI_MAX_LOST_SECONDS=3s`、内部 `TRACK_MAX_LOST_SECONDS=12s`、RID/SORT四点关联、距离裁决、云台控制和打击安全逻辑均未改变。

## 2026-07-31 UI轨迹漏检容忍与控制有效期解耦
- 四组 `20260731_*` 日志共统计到 4460 个同轨迹成功检测间隔：94 次超过 `1s`、24 次超过 `2s`、9 次超过 `3s`；默认 `UI_MAX_LOST_SECONDS` 因此从 `1.0s` 调整为 `3.0s`，可覆盖约 99.8% 的日志内可恢复漏检间隔。
- `ui_tracks` 改为直接从已确认的内部 `active_tracks` 中按 UI 新鲜度筛选，不再先经过 `MAX_LOCK_LOST_SECONDS`；否则把 UI 常量设置为大于 `1.6s` 实际不会生效。
- `MAX_LOCK_LOST_SECONDS=1.6s` 保持不变。目标丢失 `1.6~3.0s` 期间只允许继续向 UI 发送 SORT 预测，不参与主目标选择、云台控制或打击端有效目标；超过 `3.0s` 停止向 UI 发送，内部仍可保留到 `12.0s` 等待同 ID 重关联。

## 2026-07-31 轨迹生命周期回放优化
- 对 `logs/20260731_154234`、`164932`、`170259`、`170919` 的结构化日志做了逐周期回放；原参数可精确复现 `13/20/12/8`、合计 53 次 `NEW_TRACK`。
- 53 次建轨中：5 次为测试初始检测，21 次发生在旧轨迹因 `lost_seconds>=5s` 删除后，18 次由关联门控拒绝触发，9 次为同周期额外检测。
- 默认 `TRACK_MAX_LOST_SECONDS` 从 `5.0s` 调整为 `12.0s`，只延长内部 ID 保留；UI显示与控制/打击有效期继续由各自更短的独立门限约束。
- 默认 `TRACK_REACQUIRE_MAX_DEG` 从 `3.8°` 微调为 `4.0°`，与基础关联尺度对齐；实现仍使用 `cost >= gate` 阻断，因此恰好 `4.0°` 及更大偏差仍拒绝，`TRACK_ASSOCIATION_MAX_DEG=6.0°` 全局硬上限不变。
- 四组日志用新参数回放后建轨数为 `5/17/11/8`、合计 41，较原来减少 12 次；没有放开日志中 `7°~36°` 的大角度跳变。

## 2026-07-31 第三层实测theta与未测试摄像头统一修正
- 第三层 `logic_id=11~15` 使用现场手动调整后的theta原值，不再叠加统一修正：水平分别为 `29.9870/18.6921/359.0000/337.9173/319.7531°`，垂直均为 `13.0°`。
- 其余未测试摄像头在各自 `DEVICE_THETA` 原值上统一叠加水平 `-2.0°`、垂直 `-1.85°`；原始表不批量改写，统一修正在 `calculate_angles()` 中执行。
- SORT投影Y像素补偿统一为 `0px`，撤销旧的 `logic_id=12:+46.5px`、`logic_id=13:+306.5px`，避免与第三层theta调整重复补偿。
- 视觉关联仍以Y轴绝对残差执行无硬门限的一对一匈牙利分配；X轴不参与匹配，仅保留在日志中用于诊断。
- `vision_association_*.csv` 同时记录来源 `board/cam/logic_id`、原始SORT投影、Y补偿量、补偿后Y、原始 `dx/dy`、原始二维误差和实际匹配代价 `association_cost_y_px`，用于逐摄像头停留测试和后续补偿量复算。
- 本次不改变视觉测距模型、RID优先裁决、云台控制、安全条件或外部协议。

## 2026-07-27 双距离链路复盘日志
- 新增 `vision_association_*.csv`：每个新视觉帧分别记录全部 `SORT_PROJECTION`、全部 `DETECTION` 和完整 `CANDIDATE` 代价矩阵；候选行包含SORT投影像素、检测中心、`simple_id`、距离、有效性、像素代价和匈牙利选择结果。
- 新增 `distance_arbitration_*.csv`：当RID测量、视觉帧或最终来源变化时，记录两路候选的距离/年龄/序号、视觉匹配误差、最终来源、滤波写入、来源切换及RID抑制视觉状态。
- 通用 `events_*.csv` 不再重复写入 `GIMBAL_VISION_DETECTION`、`GIMBAL_VISION_BBOX_JITTER`、`GIMBAL_VISION_ASSOC`、`GIMBAL_VISION_UNMATCHED_DETECTION` 和 `DISTANCE_ARBITRATION`；RID原始串口、RID payload、原始UDP和四点匹配日志保持不变。
- `GIMBAL_VISION_RESULT_TTL` 默认值现为3秒。该TTL同时影响视觉距离与既有视觉安全/打击新鲜度判断，打击发送仍默认关闭。

## 2026-07-26 RID优先的双距离来源
- 当前距离架构以SORT为唯一轨迹和UI身份主线；RID四点匹配与云台视觉测距相互独立运行，二者的候选范围都严格限定为已通过二次过滤的 `ui_tracks`。
- RID与视觉只生成距离候选，不在各自提供方流程中直接改写最终距离。主线程按每条SORT轨迹统一裁决：6秒内有效RID优先；否则仅当前云台目标可使用新鲜视觉距离；两者都无效时UI输出 `NaN`。
- 云台视觉结果与SORT投影采用无硬门限的匈牙利全局一对一分配；当前代价已更新为按摄像头补偿后的Y轴绝对残差，原始X/Y及二维欧氏误差仅作诊断。
- RID四点匹配的单个同步残差样本保留30秒，超过该时长即从对应配对历史中删除；RID轨迹和距离新鲜期均为6秒。
- RID/视觉来源发生切换时必须重置距离卡尔曼滤波的距离、径向速度和协方差，避免大距离差被既有50米异常门控拒绝。
- 本轮未改变外部UDP/UI协议、云台控制线程、抢占/到位逻辑、视觉距离模型或打击安全条件；真实云台、相机和RID端到端回归仍待现场完成。

## 2026-07-13 首帧 MLP 有效距离与诊断关联门限
- 当前 `_DistanceRuntime` 的有效距离策略已明确改为：只要物理测距、运动门控和 MLP 输出有效，第一帧就允许返回 `distance_valid=True`，距离来源记为 `mlp_warmup`，不再等待 25 帧后才首次输出距离。
- `warmup_count=1~24` 表示 GRU 时序窗口仍在积累，不表示距离无效；累计到 25 帧后切换为 `gimbal_yolo_gru`，之后继续使用滚动 25 帧窗口。
- 这样做是为了降低首次测距延迟。物理测距无效或运动门控拒绝时仍返回无效，不因“首帧可用”而绕过既有质量检查。
- 当前 `GIMBAL_VISION_SIMPLE_ASSOC_MAX_PX` 默认值为 `3000px`，是为了先验证云台相机能否看到目标及测距链路能否工作而设置的高容差、低严格度诊断配置。匈牙利一对一分配仍会执行，但该配置几乎不具备错误匹配拒绝能力，不能把“成功分配”直接解释为目标身份已可靠匹配。
- 进行正式多目标距离绑定前，应基于真实投影误差收紧最终 SORT 回填门限，并恢复歧义拒绝验证；首帧 MLP 有效策略与关联门限是两个相互独立的设计维度。

## 2026-07-12 云台按轴重发与物理静止判定
- 新增 `GIMBAL_VISION_DETECTION` 事件：云台YOLO每个新帧、每个原始检测框记录一条，发生在SORT回填之前；包含2K bbox/中心、相对 `(1280,720)` 的像素与归一化偏差、按 `17.5°×9.9°` 换算的方位/俯仰偏差、置信度、类别、simple ID和warmup状态。
- `logs/20260711_222934` 已证实部分命令中俯仰到位、方位完全停留在上一位置；当前恢复 `GIMBAL_COMMAND_RETRY_INTERVAL=0.90s`，但只强制重发误差仍不小于 `GIMBAL_SETTLE_THRESHOLD` 的轴，不再无差别重发双轴。
- `GT06ZGimbal.set_angles()` 与适配层现在允许某一轴传入 `None`；返回值包含每轴 requested/sent/ack 状态，`GIMBAL_CMD_RETRY` 日志记录重发轴及驱动结果。
- 新增独立物理静止状态：编码器相邻反馈的 Az/El 变化均不超过 `GIMBAL_STATIONARY_DELTA_DEG=0.11°`，持续 `GIMBAL_STATIONARY_DWELL_SECONDS=0.35s` 后置为 stationary。
- `is_settled` 仍表示两轴均到达命令误差 `<0.3°`，继续作为打击端安全条件；stationary 只表示画面稳定，供云台 YOLO/测距、UI距离新鲜度和下一次安全视场重定位使用。
- 云台不会为目标每次小幅移动而动作：目标位于中央60%安全视场内时保持不动；连续3帧越出安全区才允许重定位，物理运动期间同目标不连续抢占，目标切换仍可替换命令。
- 已补齐 `MockGimbalAdapter` 的独立轴/`force` 兼容接口。端到端640坐标模拟中4个阶段角度误差均为0，中央安全区内没有新增命令，初始化与3条目标命令全部到位且0次超时；故障注入丢弃首次Az后在0.906秒只重发Az并最终 settled/stationary。
- 真实GT06Z小范围回归已完成：5阶段640坐标误差为0（日志舍入上限 `5e-7°`）；初始化与3条目标命令全部 `GIMBAL_SETTLED`，0次超时，每次随后均出现 `GIMBAL_STATIONARY`；安全区内阶段没有新增命令。垂直阶段出现1次仅El重发并最终到位。多条Az/El命令虽未收到ACK但姿态仍执行，故ACK只作诊断，重发继续依据实际姿态误差。测试结束已回到初始化容差内，服务保持 `inactive`。

## 2026-07-11 增量说明
- 当前板端仓库为 `/home/linaro/gimbal`，板端地址已变更为 `linaro@192.168.40.154`；开发机的 `~/.ssh/id_ed25519.pub` 已追加到板端 `~/.ssh/authorized_keys`，可直接执行 `ssh linaro@192.168.40.154` 免密登录。仓库文档中不保存口令。
- SORT 关联与 UI 状态安全性已补强：
  - UI 状态包从每条轨迹自身的最后检测记录读取 `board/cam/logic_id`，不再复用融合窗口最后一个 UDP 包的来源。
  - 关联门限增加 `TRACK_ASSOCIATION_MAX_DEG` 硬上限，协方差增长不能把匹配范围放大到 `10°~20°`。
  - 丢失超过 `TRACK_REACQUIRE_STRICT_AFTER_SECONDS` 的轨迹使用 `TRACK_REACQUIRE_MAX_DEG` 严格重关联门限；内部仍可由 `TRACK_MAX_LOST_SECONDS` 保留，但不会无限放宽匹配。
  - 匈牙利算法前先做门控，不可能的轨迹-检测组合使用 `ASSOCIATION_BLOCKED_COST` 标记为不可匹配。
  - UI 使用更严格的 `UI_MAX_LOST_SECONDS` 隐藏陈旧预测；内部轨迹保留和外部 UI/控制有效期不再混为一体。
- 云台视觉测距流程已从“先绑定 SORT、再积累 25 帧”改为“先独立测距、再回填 SORT”：
  1. 云台运动期间不执行 YOLO；云台停稳并通过原有延迟/清晰度条件后开始检测。
  2. simple 模式检测当前画面内的全部 YOLO 目标，不再只选画面中心最近目标。
  3. YOLO 目标先通过轻量画面内临时 ID 维持连续性；该 ID 只服务于区分各目标的 25 帧缓冲区，不读取或依赖 SORT 轨迹。
  4. 每个临时目标拥有独立 `_DistanceRuntime`，分别执行既有物理距离、MLP 和 25 帧 GRU 链路；距离模型内部算法没有修改。
  5. 每帧测距 runtime 更新完成后，`main_tracking_v9.py` 才将测距结果的画面位置与当前 SORT 预测位置做带硬门限的一对一匹配。
  6. 只有 `distance_valid=True` 的新鲜结果会通过 `set_mono_distance()` 写入对应 SORT 轨迹；UI 与打击端继续从统一轨迹结果读取距离。
- 关键语义：测距后的 SORT 回填失败只影响“本次距离写给哪条轨迹”，不会阻止或清空该画面目标的 25 帧测距积累；后续帧仍会继续测距并再次尝试回填。
- 当前验证结果：
  - `python3 -m py_compile core/gimbal_vision_ranging.py core/main_tracking_v9.py` 已通过。
  - 两目标模拟验证中，两个独立缓冲区均累计到 `25/25`，未向测距缓冲流程提供任何 SORT 输入。
  - 两个 `_DistanceRuntime` 的 `sequence_length` 均为 `25`，共享只读模型资产，但各自保留独立时序状态。
  - `tests/test_tracker_safety.py` 中 6 个测试已通过直接调用验证；板端当前未安装 `pytest`。
- 尚待完成：使用真实云台、相机和悬停目标做端到端回归，重点核查多目标 YOLO 稳定性、25 帧预热耗时、测距结果到 SORT 的回填成功率、UI/打击端距离与目标 ID 一致性。当前没有启动真实硬件主程序进行本轮验证。
- 本轮修改没有改变外部 UDP/UI 协议、`distance_model` 内部算法、云台控制线程所有权、抢占/到位逻辑或打击安全条件。

## 2026-05-30 增量说明
- UI 目标状态包中的 `azimuth` 已从“设备自身坐标系方位角”改为“地图绝对方位角”。
- 新增 `DEVICE_HEADING_DEG`，表示设备自身 `0°` 方向在地图上的绝对方位：正北 `0°`、正东 `90°`、正南 `180°`、正西 `270°`。
- UI 发送前转换公式为：`map_az = (relative_az + DEVICE_HEADING_DEG) % 360.0`。
- 示例：设备正方向朝正南时设置 `DEVICE_HEADING_DEG=180`，相对设备方位 `20°` 会发送为地图方位 `200°`，即南偏西 `20°`。
- 本次只转换发给 UI 的 `0x02` 状态包 `azimuth`；追踪、卡尔曼、云台控制和抢占逻辑仍继续使用设备自身坐标系角度，避免影响云台指向。

## 2026-04-22 增量说明
- 已新增 GPS 启动定位链路：`main_tracking_v9.py` 启动后会创建 GPS 后台线程，在系统刚启动时尝试获取一次有效经纬度。
- GPS 数据源为 `gps.py`，当前使用 Unicore UM980 输出的 NMEA GGA 语句；只有 `gps_qual != 0` 且经纬度字段有效时，才认为是真实定位。
- GPS 搜星等待策略：
  - 最多等待 `GPS_FIX_TIMEOUT_SECONDS = 60`
  - 等待期间打印 GGA 状态摘要，必要时可通过 `GPS_DEBUG_RAW=1` 打印每条原始 NMEA
  - 60 秒内获取真实定位，则发送真实经纬度给 UI
  - 60 秒内仍无有效定位，则发送 `gps.py` 中的 `DEFAULT_LATITUDE` / `DEFAULT_LONGITUDE` 作为兜底位置
  - 无论真实定位还是默认兜底，GPS 线程都会向 UI 发送一次经纬度信息后退出
- UI 新增 GPS 数据包格式：
  - 包头 `0x03`
  - 紧跟 `float latitude`
  - 紧跟 `float longitude`
  - 当前实现为 `struct.pack('!Bff', 0x03, latitude, longitude)`，网络字节序，与现有 UI 状态包保持一致
- 串口配置已扩展 GPS：
  - Windows 默认：云台 `COM3`、激光 `COM4`、IMU `COM5`、GPS `COM8`
  - Linux 当前默认：云台 `/dev/ttyUSB0`、激光 `/dev/ttyUSB1`、GPS `/dev/ttyUSB2`、IMU `/dev/ttyUSB3`
  - 仍可通过 `GIMBAL_PORT` / `LASER_PORT` / `GPS_PORT` / `IMU_PORT` 环境变量覆盖
- 已新增 `udp_ui_receiver.py` 用于模拟 UI 端监听 UDP `9999`，可解析并打印：
  - `0x02` 目标状态包
  - `0x03` GPS 经纬度包
- GPS 相关变更不改动云台控制线程、激光流程、目标追踪逻辑或现有 `0x02` 状态包格式。

## 2026-04-16 增量说明
- 云台抢占逻辑已从“单一总阈值 + 整条命令替换”升级为“分轴阈值 + 同目标按轴更新”。
- 当前抢占基线：
  - `AZ_PREEMPT_DEG = 0.5`
  - `EL_PREEMPT_DEG = 0.8`
- 当前执行规则：
  - **同一 `track_id` 连续跟踪**：哪个轴超过各自阈值，就只更新哪个轴的目标值。
  - **`track_id` 发生切换**：必须整条命令替换，`Az/El` 一起更新，避免出现“新方位 + 旧俯仰”的混合指向。
- 当前抢占日志已补充 `mode=track_switch/axis_update` 与 `axes=Az/El/Az+El`，便于从日志直接判断是目标切换，还是同目标下的单轴更新。
- 本次更新只涉及控制线程内的抢占更新逻辑，没有改动 `PREDICT_DELAY`、`SETTLE_THRESHOLD`、激光流程或外部协议。

## 2026-04-14 增量说明
- 激光链路已从“只有接口、未接主流程”升级为“真实可运行”状态：`main_tracking_v9.py` 现会在 `USE_MOCK_LASER=False` 时启动 `SDDMLaser` 后台线程，持续读取真实激光并写入 `SharedHardwareState.raw_laser_dist`。
- 已新增 `USE_MOCK_LASER`，语义与 `USE_MOCK_GIMBAL` 一致：
  - `True`: 激光通道使用当前主目标的 mono 距离做模拟输入
  - `False`: 激光通道使用真实 SDDM 激光串口
- 当前真实硬件基线端口：云台 `COM3`，激光 `COM8`。部署到 Linux 时仍优先通过 `GIMBAL_PORT` / `LASER_PORT` / `IMU_PORT` 覆盖。
- SDDM 激光当前采用“连续测量”而不是“单次测量”。原因是主流程是在 `GimbalSettled` 后读取一份最新缓存值，连续测量更符合现有非阻塞控制结构。
- `sddm_laser.py` 已按手册补强帧校验：除 CRC 外，还校验了数据帧 `MsgCode=0x03` 与 `PayloadLen=0x04`。
- 最新实测结论：
  - 短时测试里曾出现只有无效激光帧、主流程退回 mono 距离
  - 15 秒长测中，真实激光已稳定触发 19 次，`[Laser] No valid laser distance` 为 0
  - 真实激光返回的是实物环境距离（约 `4.7m ~ 6.5m`），不是发送脚本里的模拟 `320m`
- 2026-04-14 Linux 联调补充：
  - 接收端运行在 `10.72.2.28:8888`，串口为云台 `/dev/ttyUSB0`、激光 `/dev/ttyUSB1`
  - `logs/main_tracking_v9_20260414_161730.log` 显示：激光线程已启动、连续测量命令已发，但整场测试都只有 `[Laser] No valid laser distance`；该现象当前优先解释为室内目标距离/反射条件不满足，而不是主流程未触发
  - `logs/main_tracking_v9_20260414_162745.log` 显示：将激光指向更远目标后恢复稳定真实测距，`GimbalSettled=22`、`GimbalTimeout=0`、`Laser Triggered=22`、`Laser No valid=0`，实测距离约 `6.2m ~ 7.9m`

## 本文件用途
这是给 AI/新接手工程师的项目守则。先读本文件了解硬约束和安全边界，再读 `PROJECT_CONTEXT.md` 建立架构全貌，读 `TODO_NEXT.md` 看当前进度，读 `DECISIONS.md` 理解决策原因。

## 项目目标 (Purpose)
本仓库是一个支持 Windows 调试与 Linux 部署的实时云台追踪与测距系统。

主要职责：
- 接收来自 RK3588 或发送端脚本的 UDP 目标检测数据
- 解析检测数据并将其转换为归一化的目标观测值
- 使用基于卡尔曼滤波（Kalman-based）的逻辑对目标进行连续追踪，并根据帧率自适应时间步长
- 选择一个活动目标进行云台追踪
- 将 UI/世界坐标系下的角度转换为云台控制角度
- 通过串口控制 GT06Z 云台，应用前馈预测以补偿机械延迟
- 读取激光/IMU 相关的硬件状态（若启用）
- 通过 UDP 将目标状态发送回 UI

主入口点：
- `main_tracking_v9.py`

测试发送脚本：
- `udp_sender_tracking_scenarios.py`

---

## 开发优先级 (Development priorities)
1. 运行稳定性与真实硬件的安全性
2. 串口通信的正确性
3. 动态帧率下的追踪连续性与速度估算准确性
4. 初始捕获速度 (Acquisition Speed) 与稳定追踪精度 (Tracking Smoothness) 的平衡
5. 低延迟表现与向后兼容性

---

## 强制规则 (Hard rules)

### 1. 严禁随意重写整个项目
这是一个与硬件深度耦合的项目。优先选择局部、最小化且可控的代码修改。

### 2. 除非明确要求，否则不要更改外部协议格式
UI 输出格式、GT06Z 串口协议与坐标系约定必须保持一致。

### 3. 尊重云台多轴动力学的不对称性
水平方位轴 (Pan) 与俯仰轴 (Pitch) 的响应速度不同。在修改预测提前量 (Predictive Lead) 时，必须支持两轴解耦。

### 4. 保留双平台串口假设与控制线程所有权
默认串口设备名按平台切换：Windows 使用 `COMx`，Linux 使用 `/dev/ttyUSB*` / `/dev/ttyACM*`；也可通过 `GIMBAL_PORT` / `LASER_PORT` / `IMU_PORT` 显式指定。云台指令由单一控制线程统一下发。

### 5. 任何影响控制行为的更改都必须说明：
- 更改了什么 (例如：调整了 `PREEMPT` 或 `SETTLE_THRESHOLD`)
- 为什么更改 (例如：拉大差距以防止指令频繁重置/反复横跳)
- 可能的硬件风险及验证方法

---

## 代码修改策略 (Code modification strategy)
优先理解现有逻辑 -> 最小必要范围修改 -> 保持现有接口 -> 必要时添加注释 -> 保留调试打印风格。

---

## 重要文件 (Important files)
- `main_tracking_v9.py`: 主运行程序，包含自适应帧率的卡尔曼滤波与双阶段(捕获/稳定)状态机。
- `gimbal_interface.py`: 云台适配接口。
- `GT06Z_gimbal.py`: 底层串口驱动。
- `sddm_laser.py` / `hwt905_driver.py`: 传感器底层驱动。
- `udp_sender_tracking_scenarios.py`: 测试发送器。

---

## 当前控制基线 (Current control baseline)
截至 2026-04-10，`main_tracking_v9.py` 的当前控制基线为：
- Kalman `MIN_DT=0.001`，仅保留异常极小 `dt` 保护，避免 15FPS 场景被固定下限抬高。
- 抢占阈值已在 2026-04-16 更新为分轴基线：`AZ_PREEMPT_DEG=0.5`、`EL_PREEMPT_DEG=0.8`；`GIMBAL_SETTLE_THRESHOLD=0.3` 保持不变，用于维持到位迟滞区间。
- 启动时通过 `gimbal_cmd_queue` 投递初始化姿态命令，目标为 `Az=GIMBAL_AZ_BASE`、`El=0.0°`，仍由单一云台控制线程下发。
- 串口默认值按平台切换：Windows 为 `COM3` / `COM4` / `COM5`，Linux 为 `/dev/ttyUSB0` / `/dev/ttyUSB1` / `/dev/ttyUSB2`。
- `PREDICT_DELAY` 尚未拆分为 `AZ_PREDICT_DELAY` / `EL_PREDICT_DELAY`；后续如修改预测提前量，仍必须保持 Pan/Pitch 解耦。

---

## 未来编辑的预期行为 (Expected behavior for future edits)
- **安全修复模式**: 处理异常时必须优雅降级，严禁导致主循环崩溃。
- **控制调优模式**: 调整控制逻辑时，必须区分“首次跟踪 (Initial Acquisition)”和“稳定跟踪 (Stable Tracking)”的不同需求；必须确保控制死区和抢占阈值之间有足够的“迟滞 (Hysteresis)”。
- **重构模式**: 仅在明确要求时才执行此操作。
- **激光联调模式**: 如果日志里已出现 `[Init] Real laser enabled ...`、`[LaserThread]`、`[Laser] start_measurement mode=continuous`，但持续没有有效距离，先检查目标距离、反射条件与瞄准方向，再怀疑主流程代码回归。

---

## AI 不应“好心”更改的事项
**严禁**自动执行以下操作：
- 将 UDP 替换为 TCP 或将所有多线程替换为异步。
- 将卡尔曼滤波器时间步长 `dt` 硬编码为固定常数（必须依赖实际帧率计算）。
- 将方位角和俯仰角的预测延迟强行统一合并。
- 移除云台控制中的抢占和到位迟滞保护逻辑。
