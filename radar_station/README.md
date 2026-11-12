# 雷达 NX 端

NX1 和 NX2 使用同一套雷达站代码。换站时只需复制这个目录，再改站点编号、网卡和波形路径；采集流程不需要另写一份。

## 目录

```text
radar_station/
├── radar_station_control.py   标准采集入口
├── run_radar_station.sh       命令行包装器
├── capture_*.py               雷达、相机和串口 IMU 采集
├── mmwave/dataloader/         DCA1000 数据读取
├── fpga_udp/                  AWR2243 控制扩展和源码
├── configFiles/               波形和 DCA 配置
├── config/                    NX1/NX2 配置样例
├── data/                      会话数据
└── logs/                      运行日志
```

## 部署前检查

- Ubuntu 20.04/22.04、Python 3.8+、NumPy、OpenCV/GStreamer。
- AWR2243、AR-DevPack 和 DCA1000 供电正常。
- 雷达专用网口：`192.168.33.30/24`；DCA 数据口：`192.168.33.180`。
- `fpga_udp` 的扩展模块必须按当前 Jetson 架构编译；不要把别的机器上的 `.so` 直接混用。

首次部署示例：

```bash
cd ~/radar_station
cd fpga_udp
python3 setup.py build_ext --inplace
cd ..
cp config/examples/radar0.json radar_station_config.json
```

然后编辑 `radar_station_config.json` 的站点编号、雷达网卡、波形路径以及相机/IMU 开关。雷达是必需项，雷达侧相机和串口 IMU 可以关闭。

## 单站软件触发测试

当前默认波形是 TX0、RX0/RX3、77 GHz、10 MSps、每 chirp 1200 点、每帧 256 chirp、650 ms 帧周期：

```bash
cd ~/radar_station
./run_radar_station.sh preflight --allow-no-camera --allow-no-imu
./run_radar_station.sh capture \
  --name scene01 \
  --duration-seconds 15 \
  --radar-config configFiles/mmwaveconfig_tx0_rx03_77g_25m_1200s_256c_650ms.txt \
  --allow-no-camera --allow-no-imu
```

采集结果在 `data/sessions/<会话名>/`。优先查看 `radar/radar_quality_summary.json`：完整帧、文件大小应匹配，`zero_filled_bytes` 应为 0，质量状态应为 `ok`。

## 预配置和硬触发

`radar_ready.py arm` 可先完成一次配置，再让后续采集快速进入保存阶段。STM32 没有接好时不要选择硬件触发配置；硬触发文件只是为后续双雷达同步保留。

## 许可证

雷达 UDP 读取部分参考 [gaoweifan/pyRadar](https://github.com/gaoweifan/pyRadar)，许可证见 `LICENSE.pyRadar`。仓库不提交原始数据、视频、密码或设备私钥。
