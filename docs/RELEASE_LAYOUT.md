# 发布与复制说明

仓库按设备拆成两个可复制目录：`dog/` 和 `radar_station/`。

- 机器狗复制到 `~/NX_radar_fleet`，只需要补充本地 `tools/radar_fleet_env.sh` 和 `tools/radar_fleet_config.json`。
- 每台雷达 NX 复制 `radar_station/` 到用户目录，使用对应的 `config/examples/radar0.json` 或 `radar1.json` 生成本地 `radar_station_config.json`。
- `data/` 和 `logs/` 只保留目录占位文件；真实会话、视频和日志不进入 Git。
- `.so`、密码环境文件、SSH 密钥和原始数据不进入 GitHub。Jetson 上需安装依赖并构建 `fpga_udp`，或从受控的同架构发布包补充二进制。

## 当前入口

机器狗：`~/NX_radar_fleet/tools/nxui`。

雷达 NX：`~/radar_station/run_radar_station.sh`。两端都只保留一个正式入口，旧入口和临时测试脚本不参与部署。
