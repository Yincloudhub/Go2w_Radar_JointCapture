# 雷达 NX 端

本目录是 NX1/NX2 共用的 AWR2243 + DCA1000 采集包。两台 NX 使用同一套代码，设备差异只放在 `config/radar0.json`、`config/radar1.json` 和本机的 `radar_station_config.json`。

## 目录

```text
radar_station/
├── radar_station_control.py   标准采集入口
├── run_radar_station.sh       唯一命令行包装器
├── capture_*.py                雷达、相机、IMU 采集实现
├── mmwave/dataloader/          DCA1000 数据读取代码
├── fpga_udp/                   AWR2243 控制扩展及源码
├── configFiles/                雷达波形和 DCA 配置
├── config/                     站点配置样例
├── data/                       雷达会话数据
└── logs/                       运行日志
```

## 环境

- Ubuntu 20.04/22.04，Python 3.8+，NumPy、OpenCV/GStreamer。
- AWR2243/AR-DevPack 和 DCA1000 供电正常。
- 雷达专用网口配置为 `192.168.33.30/24`，DCA 数据口为 `192.168.33.180`。
- `fpga_udp` 需要使用当前 Jetson 架构重新编译；已有可用 `.so` 时可直接放在 `fpga_udp/`。

首次部署：

```bash
cd ~/radar_station
python3 -m pip install -r requirements.txt  # 若现场提供该文件
cd fpga_udp
python3 setup.py build_ext --inplace
cd ..
cp config/examples/radar0.json radar_station_config.json
```

修改 `radar_station_config.json` 的 `station_id`、`radar_interface`、波形路径和相机/IMU 可选项。雷达必须正常，雷达侧相机和串口 IMU 可以关闭。

## 软件触发采集

当前默认软件触发配置：TX0、RX0/RX3、77 GHz、10 MSps、每 chirp 1200 点、每帧 256 chirp、650 ms 帧周期。

```bash
cd ~/radar_station
./run_radar_station.sh preflight --allow-no-camera --allow-no-imu
./run_radar_station.sh capture \
  --name scene01 \
  --duration-seconds 15 \
  --radar-config configFiles/mmwaveconfig_tx0_rx03_77g_25m_1200s_256c_650ms.txt \
  --allow-no-camera --allow-no-imu
```

采集结果在 `data/sessions/<会话名>/`。重点检查 `radar/radar_quality_summary.json`：应满足 `capture_quality_status=ok`、原始字节数匹配、补零字节为 0。

## 预配置模式

`radar_ready.py arm` 可在一次配置后保持雷达待机，随后用 `capture` 快速保存固定帧窗口。STM32 未接好时不要使用硬件触发配置；硬触发文件只作为后续双雷达同步版本保留。

## 许可证和来源

雷达 UDP 读取核心参考 [gaoweifan/pyRadar](https://github.com/gaoweifan/pyRadar)，原项目许可证保留在 `LICENSE.pyRadar`。本仓库不提交原始数据、视频、密码或设备私钥。
