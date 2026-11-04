# G70 GNSS 原语

可独立启动 G70 硬件及可选 NTRIP 差分的 Robonix 原语。包内携带 G70 串口/NMEA/RTCM
驱动，不依赖 ranger 原 GNSS 工程目录，也不依赖特定本体的 Soma/URDF 申报。
话题固定为 /gps/fix 和 /gps/rtk_status；新增能力约定见 [CONTRACTS.md](CONTRACTS.md)。

## 环境与构建

Linux、ROS 2、rbnx/robonix-api；系统 Python 需可导入 rclpy、serial、yaml、grpc、grpc_tools。
ROS 2 Humble 已在 ranger 上验证；其他发行版尚未实测。
运行用户须有串口读写权限，容器需映射串口设备并配置一致的 ROS 网络。

```bash
export PATH="$HOME/.cargo/bin:$PATH"
cd /path/to/primitive-g70-gnss-rbnx
rbnx validate .
bash scripts/build.sh
```

源码依据 https://github.com/syswonder/robonix 和 https://book.robonix.ai/developer-guide。
复用框架 NavSatFix/String IDL 和共享生命周期，不需要 colcon。
start.sh 使用 ROS_SETUP 指定的 setup.bash，或 /opt/ros/$ROS_DISTRO/setup.bash
（未设 ROS_DISTRO 时尝试 humble），也支持预先 source 的自定义 ROS 安装。
不强制 RMW、ROS_DOMAIN_ID 或 Zenoh 地址，由部署环境传入。

## 启动硬件

默认 manage_driver=true。部署示例（path 按新机器上的包位置修改）：

```yaml
primitive:
  - name: g70_gnss
    path: ./primitives/primitive-g70-gnss-rbnx
    config:
      serial:
        port: /dev/serial/by-id/YOUR_G70_DEVICE
        baud: 115200
      frame_id: gnss_link
```

也可在加载全局约定和本包 capabilities/ 的 Atlas 上单独运行：
`rbnx start -p /path/to/primitive-g70-gnss-rbnx -c config.yaml`。
config.yaml 内容对应上述 config 块。无需完整本体申报；运行身份由部署实例名称指定，默认 g70_gnss。

不写 serial.port 时尝试 auto：查找 /dev/gnss、/dev/gps、/dev/g70，或 by-id 中名称含
GNSS/GPS/G70/RTK 的设备。只有唯一设备时才选择，否则要求配置设备路径。
不猜测普通 ttyUSB 设备，避免误开 IMU 等设备。
默认波特率 115200，必须与接收机配置一致。frame_id 默认 gnss_link；
换本体时配置串口、ROS 环境和部署 TF 即可，消息表示天线位置，不含杆臂补偿。

## 可选 RTK / NTRIP

不配置凭据时启动普通 GNSS，差分功能禁用。需要 RTK 时加入：

```yaml
ntrip:
  server_ip: 192.0.2.1  # 替换为真实 IP 或域名
  port: 2101
  username: YOUR_ACCOUNT
  mountpoint: YOUR_MOUNTPOINT
  password_env: G70_NTRIP_PASSWORD
```

启动前在环境中设置 G70_NTRIP_PASSWORD。也支持 password 字符串，与 password_env 互斥。
直接写 password 会将密码留在部署配置中，建议使用环境变量。
密码只传给驱动子进程环境，不写入运行时 YAML、命令行或本包日志。
服务器、端口、账号和挂载点均实际传给驱动。
NTRIP 等待有效新鲜 GGA 后才连接，能否 FIX 仍取决于卫星、差分源和接收机状态。

可选 driver_config 指向已有 G70 YAML 作为基础配置，公开配置覆盖基础配置；
未设时使用包内默认值，不依赖任何外部工程路径。旧 YAML 的相对 password_file 会正确解析。
完整参数见 config.spec，受管示例见 deploy.managed.fragment.yaml。

## 已有驱动与生命周期

已有 WebUI/其他系统管理 G70 时设 manage_driver=false，见 deploy.fragment.yaml。
此模式只探测并向 Atlas 声明已有话题，不终止外部驱动；传入 ntrip 覆盖项会报错。
INIT 校验配置；ACTIVATE 在受管模式确认没有已有 GNSS 发布者后启动包内驱动，
等待串口 OPEN、NavSatFix 消息及有效 RTK JSON；无需已有定位解，默认超时 10 秒。
停用、激活失败或关闭时只清理自身子进程和临时配置。
驱动以 exclusive=True 打开串口，重新激活会重建驱动并重新探测，Atlas 声明保持幂等。
修改参数需停用、重新初始化并激活，不进行热更新。

ACTIVE 表示接口接通，不表示定位有效、NTRIP 鉴权成功或已有 RTK FIX。
消费者需按 CONTRACTS.md 检查接收超时、数据龄期和定位质量。
本包不提供地图定位、融合、导航或 MCP 工具。

