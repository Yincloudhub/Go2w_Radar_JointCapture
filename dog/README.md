# 机器狗端

本目录是 Go2W 机器狗端的完整采集控制包：`tools/nxui` 是唯一 UI 入口，`tools/radar_fleet_control.py` 负责预检、远程控制雷达站、记录机器狗 IMU/里程计和保存时间快照，`go2w_ws` 提供机器狗侧 ROS 服务。

## 目录

```text
dog/
├── tools/       采集控制、时间检查、ROS 话题记录
├── config/      配置样例（真实凭据只放设备本地）
├── deploy/      systemd 服务模板
├── go2w_ws/     Go2W ROS 2 工作空间源代码
├── data/        机器狗侧会话数据
└── logs/        运行日志和 ROS 日志
```

## 部署

1. 将本目录复制到机器狗的 `~/NX_radar_fleet`。
2. 安装 Ubuntu/ROS 2 Foxy、Python 3 和 `rclpy`、`numpy` 等依赖。
3. 编译工作空间：

   ```bash
   cd ~/NX_radar_fleet/go2w_ws
   source /opt/ros/foxy/setup.bash
   colcon build --symlink-install
   ```

4. 复制配置样例并填写本机凭据：

   ```bash
   cp ~/NX_radar_fleet/config/radar_fleet_config.example.json \
      ~/NX_radar_fleet/tools/radar_fleet_config.json
   cp ~/NX_radar_fleet/config/radar_fleet_env.example.sh \
      ~/NX_radar_fleet/tools/radar_fleet_env.sh
   chmod 600 ~/NX_radar_fleet/tools/radar_fleet_env.sh
   ```

   真实密码只保存在 `radar_fleet_env.sh`，该文件被 `.gitignore` 排除。

5. 安装服务模板：

   ```bash
   sudo ~/NX_radar_fleet/deploy/install_services.sh
   ```

## 使用

```bash
~/NX_radar_fleet/tools/nxui
```

UI 中选择雷达 IP、雷达配置、采集模式，然后执行预检，最后开始采集。单 NX1 测试选择 `nx1`/`仅 NX1 采集`；双站采集选择 `dual`。正式采集使用机器狗到雷达 NX 的有线网口，Wi-Fi 只用于调试。

机器狗侧固定记录 `/dog_imu_raw` 和 `/unitree/slam_mapping/odom`。机器狗视频录制已停用；雷达侧相机是否录制由雷达站配置决定。

## 故障定位

- 预检会直接显示 SSH、路由、DCA、时间和 ROS 话题错误。
- 会话目录在 `data/sessions/<会话名>/`，日志在 `logs/`。
- 机器狗服务异常时查看：

  ```bash
  systemctl status Go2w_velocitySerial_Autostart.service
  journalctl -u Go2w_velocitySerial_Autostart.service -n 100 --no-pager
  ```
