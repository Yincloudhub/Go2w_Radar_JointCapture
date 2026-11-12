# 时间与数据质量

机器狗关机后，Jetson 内核可能先从 `rtc1` 读到 1970 年；本系统的 `nx-clock-bootstrap.service` 会优先读取带后备电源的 `rtc0`，再由 chrony/NTP 校准。`nx-clock-seed.timer` 只在 chrony 已同步时保存最近可信时间并回写 `rtc0`。没有可用 RTC/NTP/NX 时间源时，选项 4 和起采都应报告时间未同步，而不是生成看似正常的时间戳。

软件触发联采使用两站计划起采时间，并记录两站确认与首个 UDP 包到达的主机时刻。机器狗在采集前后做 SSH 时钟快照；Windows/MATLAB 后处理据此把 radar A/B、IMU、odom 放到共同时间轴。**首包时刻是接收代理，不是实际发射时刻**；摄像头时间也不是硬件曝光边沿。跨设备绝对延迟需用共同触发测量或已知目标另行标定。

每次会话请检查：

1. `fleet_manifest.json` 的联采状态与参与站点。
2. 两站的 `radar_quality_summary.json`：有效帧数、期望/实际字节数、`zero_filled_bytes`、零尾和内容质量。
3. 机器狗 `controller_imu/imu_samples.csv`、`odom_samples.csv` 的消息时间戳、覆盖区间与连续性。
4. 相机可选；失联不能使雷达空采，但 manifest 必须记录失败。视频时间精度低于有硬件时间戳的传感器。

Wi‑Fi `192.168.3.x` 适合连接与功能短测。正式采集应使用机器狗到 NX1/NX2 的有线地址 `192.168.55.1/56.1`；程序会阻止这些正式地址经 Wi‑Fi 网关绕行。数据保存在各设备自己的项目 `data/sessions/`，原始数据和日志不纳入 Git。
