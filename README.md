# gimbal_release

Linux/RK3588板端云台多目标追踪交付代码。主入口为 `main_tracking_v9.py`，实际集成实现位于 `core/main_tracking_v9.py`。

## 当前配置基线

- 接收多板卡摄像头检测数据并维护SORT/Kalman轨迹。
- 两个以SHA-256摘要配置的 `RID_laser` 目标使用固定身份层，首次成功绑定后动态获得UI_ID 1和2；真实RID序列不保存在源码中。
- 特殊RID状态以5Hz输出，RID接收新鲜期7秒，SORT视觉来源新鲜期4秒。
- 云台视觉测距默认关闭，部署环境应设置 `ENABLE_GIMBAL_VISION=0`。
- 打击端21字节UDP包包含板端GPS站点经纬度。

## 启动

板端systemd服务通过以下脚本启动：

```bash
./run_main_tracking.sh
```

服务定义参考 `gimbal-tracking.service`。实际串口和网络参数通过 `/etc/default/gimbal-tracking` 或环境变量配置。

## 外部运行资产

Git仓库不保存运行日志、历史备份、PDF/XLSX资料以及大型ONNX/RKNN/PyTorch权重。启用视觉测距前，需要单独部署与板端版本匹配的模型文件，主要包括：

```text
distance model/bestall_2k.rknn
distance model/best.onnx
distance model/best.pt
```

`distance model/models/`、`distance model/configs/` 和编译运行模块保留在仓库中。具体架构和决策记录见 `PROJECT_CONTEXT.md`、`DECISIONS.md` 与 `AGENTS.md`。
