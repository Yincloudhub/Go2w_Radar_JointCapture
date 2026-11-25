# 机器狗端

这一端只做三件事：启动采集、记录机器狗 ROS 话题、把两台雷达的结果汇总到一个会话里。唯一的 UI 入口是 `tools/nxui`，底层控制在 `tools/radar_fleet_control.py`。

## 目录

```text
dog/
├── tools/       nxui、采集控制、时间检查和会话导出
├── config/      配置样例；真实凭据只放在设备本地
├── deploy/      systemd 服务模板
├── go2w_ws/     Go2W ROS 2 工作空间源码
├── data/        机器狗侧会话数据
└── logs/        运行日志
```

## 第一次部署

把 `dog/` 复制到机器狗的 `~/NX_radar_fleet`，然后执行：

```bash
cd ~/NX_radar_fleet/go2w_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install

cp ~/NX_radar_fleet/config/radar_fleet_config.example.json \
   ~/NX_radar_fleet/tools/radar_fleet_config.json
cp ~/NX_radar_fleet/config/radar_fleet_env.example.sh \
   ~/NX_radar_fleet/tools/radar_fleet_env.sh
chmod 600 ~/NX_radar_fleet/tools/radar_fleet_env.sh
```

在配置文件里填写 NX1/NX2 地址、站点编号和会话目录；密码只放在 `radar_fleet_env.sh`。这个文件已被 `.gitignore` 排除。

需要安装开机服务时执行：

```bash
sudo ~/NX_radar_fleet/deploy/install_services.sh
```

## 开始采集

```bash
~/NX_radar_fleet/tools/nxui
```

UI 的使用顺序是：

1. 设置雷达站 IP。
2. 选择雷达配置。
3. 选择采集模式。
4. 检查代码版本。Git 部署会检查两台雷达的提交。旧发布模式会生成版本包并切换 `current`。
5. 运行预检和预配置。
6. 开始采集。

选项 1 会依次询问 IP、用户名和密码。密码不会写入 JSON，只保存到机器狗用户目录下的 `.radar_nx_password` 或 `.radar_nx2_password`，权限为 `600`；直接回车表示保留原值。

单站测试只启用 NX1；双站采集再启用 NX2。正式采集走有线网口，Wi-Fi 只用于调试。

机器狗固定记录 `/dog_imu_raw` 和 `/unitree/slam_mapping/odom`。机器狗视频录制已停用；雷达侧相机是否保存由雷达站配置决定。

## 出问题时先看这里

- 预检界面会直接显示 SSH、路由、DCA、时间或 ROS 话题哪一项失败。
- 机器狗会话：`data/sessions/<会话名>/`
- 机器狗日志：`logs/`
- 服务状态：

  ```bash
  systemctl status Go2w_velocitySerial_Autostart.service
  journalctl -u Go2w_velocitySerial_Autostart.service -n 100 --no-pager
  ```
