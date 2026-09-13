# 板端双网络联调配置说明

## 目的
本说明用于板端 Linux 联调时的双网络配置：

- `wlan0` 连接手机热点，用于 `SSH / VS Code Remote`
- `eth0` 使用静态 IP，连接交换机或对端 PC，用于和 `192.168.2.x` 网段设备通信

当前示例配置：

- 板子热点侧 IP：`10.72.2.28`
- 手机热点网关：`10.72.2.166`
- Windows 电脑热点侧 IP：`10.72.2.80`
- 板子有线侧 IP：`192.168.2.28/24`
- 业务对端主机示例 IP：`192.168.2.200/24`

## 目标网络拓扑

```text
Windows电脑  --Wi-Fi热点--  手机  --Wi-Fi热点--  板子 wlan0
                                            |
                                            | SSH / Remote 开发
                                            |
板子 eth0  --网线/交换机--  业务设备 / 对端PC (192.168.2.x)
```

原则：

- `wlan0` 负责 SSH、Remote 开发、可能的外网访问
- `eth0` 负责业务数据通信
- 默认路由保留给 `wlan0`
- `eth0` 不配置默认网关

## 1. 查看当前连接

```bash
nmcli connection show
```

示例输出中可能看到：

```text
NAME                            UUID                                  TYPE      DEVICE
Xiaomi 13                       93e79433-e730-4528-94f6-c5a75f6bf129  wifi      wlan0
有线连接 1                      aff6d94b-e34e-3889-b4a3-5a387ae63611  ethernet  --
netplan-eth0                    626dd384-8b3d-3690-9511-192b2c79b3fd  ethernet  --
```

## 2. 连接手机热点

如果 `wlan0` 还没有连上热点：

```bash
nmcli dev wifi list
nmcli dev wifi connect "你的热点名" password "你的密码" ifname wlan0
```

查看热点侧 IP：

```bash
ip addr show wlan0
nmcli device show wlan0 | grep IP4
```

本次联调示例：

- 板子 `wlan0`：`10.72.2.28`
- 手机热点网关：`10.72.2.166`
- Windows：`10.72.2.80`

## 3. 新建有线静态 IP 配置

普通用户直接执行 `nmcli connection add/modify` 可能报：

```text
Insufficient privileges
```

因此需要使用 `sudo`。

创建一个专门给 `eth0` 用的配置：

```bash
sudo nmcli connection add type ethernet ifname eth0 con-name gimbal-eth0 ipv4.method manual ipv4.addresses 192.168.2.28/24 ipv6.method ignore
```

激活该配置：

```bash
sudo nmcli connection up gimbal-eth0
```

说明：

- `gimbal-eth0` 只是 NetworkManager 中的“连接配置名”
- 实际物理网卡仍然是 `eth0`

## 4. 查看静态 IP 是否设置成功

查看配置：

```bash
nmcli connection show gimbal-eth0 | grep -E "ipv4.method|ipv4.addresses|ipv4.gateway"
```

预期输出：

```text
ipv4.method:                            manual
ipv4.addresses:                         192.168.2.28/24
ipv4.gateway:                           --
```

查看 `eth0` 实际地址：

```bash
ip addr show eth0
```

预期看到：

```text
inet 192.168.2.28/24
```

查看路由：

```bash
ip route
```

预期看到类似：

```text
default via 10.72.2.166 dev wlan0 proto dhcp metric 600
10.72.2.0/24 dev wlan0 proto kernel scope link src 10.72.2.28 metric 600
192.168.2.0/24 dev eth0 proto kernel scope link src 192.168.2.28 metric 100
```

要点：

- 默认路由应走 `wlan0`
- `192.168.2.0/24` 应走 `eth0`

## 5. 网线未连接时的表现

如果此时还没有插网线，`ip addr show eth0` 可能显示：

```text
eth0: <NO-CARRIER,BROADCAST,MULTICAST,UP> ...
state DOWN
```

`ip route` 中也可能显示：

```text
192.168.2.0/24 dev eth0 proto kernel scope link src 192.168.2.28 metric 100 linkdown
```

这说明：

- 静态 IP 已经配置成功
- Linux 已经知道 `192.168.2.x` 应该走 `eth0`
- 只是物理链路还没建立

## 6. 接上网线后的检查

网线接交换机或对端 PC 后，执行：

```bash
ip link show eth0
ip route
ip route get 192.168.2.200
```

预期：

- `NO-CARRIER` 消失
- `linkdown` 消失
- `ip route get 192.168.2.200` 显示类似：

```text
192.168.2.200 dev eth0 src 192.168.2.28
```

这表示 Linux 已经明确知道：

- 去 `192.168.2.200` 要走 `eth0`
- 源地址使用 `192.168.2.28`

## 7. 为什么 Linux 知道去哪个网段找

因为给 `eth0` 配了 `192.168.2.28/24` 后，内核会自动生成直连路由：

```text
192.168.2.0/24 dev eth0
```

所以：

- 访问 `192.168.2.x` 时，走 `eth0`
- 访问外网或其他网段时，走默认路由 `wlan0`

Linux 不是“猜”走哪个口，而是按路由表匹配。

## 8. 与对端 PC 直连测试

不用交换机，板子也可以先直接和对端 PC 直连测试。

板子侧：

- `eth0 = 192.168.2.28/24`

对端 PC 手动设置：

- IP：`192.168.2.200`
- Mask：`255.255.255.0`
- Gateway：留空

说明：

- 现代网卡通常支持 Auto MDI-X，普通网线一般就可以直连
- PC 侧建议不要给这张有线网卡配置默认网关，避免影响原有 Wi-Fi 上网

## 9. 联调常用命令

### 查看热点侧 IP

```bash
ip addr show wlan0
nmcli device show wlan0 | grep IP4
```

### 查看有线侧静态配置

```bash
nmcli connection show gimbal-eth0 | grep -E "ipv4.method|ipv4.addresses|ipv4.gateway"
ip addr show eth0
ip route
```

### 查看去业务对端的选路

```bash
ip route get 192.168.2.200
```

### 测试业务侧连通性

```bash
ping 192.168.2.200
```

### 测试热点侧远程登录

在 Windows 上：

```bash
ssh linaro@10.72.2.28
```

VS Code Remote SSH 也使用：

```text
linaro@10.72.2.28
```

## 10. 当前已验证的状态

本次已确认：

- `wlan0` 连上手机热点 `Xiaomi 13`
- 板子热点侧 IP：`10.72.2.28`
- Windows 热点侧 IP：`10.72.2.80`
- 默认路由走 `wlan0`
- 新建了 `gimbal-eth0`
- `eth0` 已配置静态 IP：`192.168.2.28/24`
- `192.168.2.0/24` 路由已经指向 `eth0`
- 当前只差接上网线建立物理链路

## 11. 下次快速配置最小步骤

如果热点已经连好，只需要确认：

```bash
nmcli connection show gimbal-eth0 | grep -E "ipv4.method|ipv4.addresses|ipv4.gateway"
ip addr show eth0
ip route
```

如果 `gimbal-eth0` 不存在，再执行：

```bash
sudo nmcli connection add type ethernet ifname eth0 con-name gimbal-eth0 ipv4.method manual ipv4.addresses 192.168.2.28/24 ipv6.method ignore
sudo nmcli connection up gimbal-eth0
```

接上网线后验证：

```bash
ip route get 192.168.2.200
ping 192.168.2.200
```

## 12. 后续建议

- 后续正式部署时，可以只保留一个主用有线 profile，避免多个 ethernet profile 混淆
- 等业务链路稳定后，再补 `systemd` 开机自启
- 项目代码层面建议后续把 `UI_IP / UI_PORT / LOCAL_PORT` 也改成环境变量，避免每次改源码
