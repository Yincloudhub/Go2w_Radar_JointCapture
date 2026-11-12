# 复制到新设备

仓库里的 `dog/` 和 `radar_station/` 是两个可以单独复制的部署包。设备换了以后，代码不需要重新拆分；只补本机配置和依赖即可。

## 机器狗

复制到：

```text
~/NX_radar_fleet
```

部署时补充：

- `tools/radar_fleet_env.sh`：SSH 用户和密码等本机凭据。
- `tools/radar_fleet_config.json`：NX1/NX2 地址、站点开关和会话参数。
- ROS 2 工作空间依赖，以及需要启用的 systemd 服务。

## 雷达 NX

把 `radar_station/` 复制到 NX 的用户目录，例如：

```text
~/radar_station
```

然后从 `config/examples/radar0.json` 或 `radar1.json` 生成 `radar_station_config.json`，填写网卡、DCA 地址、站点编号和波形文件。

## 数据边界

每台设备的代码、日志和会话数据都在同一个项目目录下：

```text
data/       真实会话和原始文件
logs/       运行日志
```

Git 只保留目录占位文件，不上传真实会话。密码、SSH 私钥、原始 `.bin`、视频和 Jetson 平台 `.so` 也不进仓库。NX 上的 `fpga_udp` 需要按当前架构重新编译，或从受控的同架构发布包补齐。

## 入口

- 机器狗：`~/NX_radar_fleet/tools/nxui`
- 雷达站：`~/radar_station/run_radar_station.sh`
