# GNSS 能力约定 v1

以下两条约定目前仅随本包加载，尚未提交 Robonix 全局目录。
不新增 RPC、生命周期状态或设备控制接口。生命周期使用框架自动提供的
`robonix/lifecycle/driver`，不另建 GNSS Driver TOML。

| 约定 | mode / transport | 类型 | 固定端点 |
| --- | --- | --- | --- |
| `robonix/primitive/gnss/fix` | topic_out / ros2 | sensor_msgs/msg/NavSatFix | /gps/fix |
| `robonix/primitive/gnss/rtk_status` | topic_out / ros2 | std_msgs/msg/String | /gps/rtk_status |

## fix

直接绑定现有驱动发布的话题，不复制或改写消息。位置为 GNSS 天线的 WGS 84
经纬度，高程为椭球高，单位米；保留原始 header、status、协方差及其类型。
不是 base_link 位姿，不包含航向、杆臂补偿、地图 datum 或 ENU 转换。
G70 驱动会将 GGA 海拔高与大地水准面差相加得到椭球高。
NavSatFix.status 不能单独区分 RTK FIX 与 FLOAT，须结合 RTK 状态。

## rtk_status

`String.data` 为 G70 驱动的 JSON 对象。下列字段是 v1 接口组成部分，原样透传；
允许未来增加字段，消费者应忽略未知字段。不可将此 JSON 直接当其他厂商的通用 RTK IDL。

| 字段 | JSON 类型 | 语义 |
| --- | --- | --- |
| state | string | 接收机解算状态；当前值包括 NO_DATA、STALE、NO_FIX、SINGLE、DGPS、RTK_FIXED、RTK_FLOAT、DEAD_RECKONING、MANUAL、SIMULATION、UNKNOWN_QUALITY；消费者应容忍未知值 |
| receiver_quality | integer 或 null | GGA quality，4 为固定解，5 为浮点解，0 为无解 |
| rtk_fixed | boolean | 驱动确认数据新鲜、有效且 quality=4 |
| fix_age_s | number 或 null | 距离最近解析 GGA 的秒数 |
| measurement_age_s | number 或 null | 距离最近变化的测量时刻的秒数，防止重复 GGA 被误认为新测量 |
| serial_state | string | 驱动串口状态，OPEN 表示已打开 |
| serial_error | string | 最近串口诊断信息 |
| bad_nmea | integer | 错误 NMEA 数量 |
| ntrip | object | 差分连接状态及接收统计；含 rtcm_age_s（number/null）和 corrections_fresh（boolean） |
| satellites | integer，可省略 | 最近 GGA 的卫星数 |
| hdop | number/null，可省略 | 最近 GGA 的 HDOP |
| differential_age_s | number/null，可省略 | GGA 差分龄期，秒 |
| utc_time | string，可省略 | GGA 原始 UTC 时间字段 |

只有收到新数据时才能更新消费者的接收时刻；两个话题分别检查接收超时，
RTK JSON 的龄期也须检查。STALE、NO_DATA、NO_FIX、未知状态或无效坐标不得
当作有效测量。FLOAT 是否可用由上层定位策略决定。
`rtk_fixed=true` 也不替代应用对差分新鲜度、坐标有效性、时间一致性和精度的判断。

## 上游标准化建议

如需其他设备和通用定位服务复用，可向 Robonix 提交
`capabilities/primitive/gnss/fix.v1.toml`，复用标准 NavSatFix IDL。
RTK 状态目前依赖 G70 JSON；成为跨厂商标准前应协商结构化 RTK 消息、
状态枚举、时间戳及缺失值表达，再新增统一约定版本和设备适配。
本包不修改框架源码或全局能力目录。
