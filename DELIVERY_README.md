# 甲方交付版本

本目录是板端可运行的交付副本，运行入口为 run_main_tracking.sh。RID 身份关联、RID 轨迹、视觉测距和距离裁决实现编译为 CPython 扩展，源码不随交付目录提供：

- core/target_measurement_runtime*.so
- core/target_identity_core*.so（RID_laser摘要匹配，不含真实RID明文）
- core/gimbal_vision_ranging*.so
- distance model/src/uav_distance_pipeline/runtime_motion_gate*.so

distance model/ 下的 RKNN、MLP/GRU 权重和 YAML/JSON 配置保持公开。core/main_tracking_v9.py 保留 UI 状态包、UI UDP 发送、打击端变量和打击端发送链路，便于甲方集成和联调。

## 板端安装

将本目录完整放置为 /home/linaro/gimbal_release，确认 linaro 用户可以读写日志目录和串口。安装服务文件并启用开机自启：

    sudo install -m 0644 /home/linaro/gimbal_release/gimbal-tracking.service \
      /etc/systemd/system/gimbal-tracking.service
    sudo systemctl daemon-reload
    sudo systemctl enable --now gimbal-tracking.service

服务默认保留原有固定串口映射；GIMBAL_PORT、GPS_PORT、RID_PORT 和 /etc/default/gimbal-tracking 环境变量仍可覆盖。

## 验收与回滚

交付前可运行：

    python3 /home/linaro/gimbal_release_work/release_tools/verify_release.py \
      /home/linaro/gimbal_release

切换服务前建议保留旧 unit 文件。若需回滚，将 unit 中的工作目录和 ExecStart 恢复为原 /home/linaro/gimbal，然后执行 systemctl daemon-reload 和 systemctl restart gimbal-tracking.service。
