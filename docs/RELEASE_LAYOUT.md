# 复制到新设备

仓库里的 `dog/` 和 `radar_station/` 是两个可以单独复制的部署包。设备换了以后，代码不需要重新拆分；只补本机配置和依赖即可。

## Git 检出部署

机器狗检出同一私有仓库的 `dog/`，每台雷达 NX 检出该仓库的 `radar_station/`。以下命令分别在各设备上执行，`COMPONENT` 在机器狗上取 `dog`，在雷达上取 `radar_station`。

```bash
git clone --sparse "$FLEET_REPO_URL" ~/dual-radar-fleet
git -C ~/dual-radar-fleet sparse-checkout set "$COMPONENT"
git -C ~/dual-radar-fleet checkout --detach "$FLEET_REVISION"
```

- 机器狗使用 `~/dual-radar-fleet/dog/tools/nxui`。在本机 `tools/radar_fleet_config.json` 设置 `"station_code_layout": "git"`，各站填写自己的 `host`、`user`、绝对 `remote_root`。波形路径也要指向新根目录下的 `configFiles/`。新配置可由 `configure_fleet.py --station-code-layout git` 生成。迁移现有配置时保留实际采集波形和设备参数。
- 雷达站从各自的 `~/dual-radar-fleet/radar_station/run_radar_station.sh` 运行。把原站点 JSON 迁到该目录的 `radar_station_config.json`，检查 DCA 网卡、波形、数据与日志路径。需要沿用历史会话目录时，给 `output_root` 设置原有数据的绝对路径。编译 DCA1000 接收扩展，并把待机服务入口改到新目录。
- Git 模式下，选项 4 核对各站的 Git 提交和已跟踪文件状态，采前检查也会验证版本。升级时将两台雷达切到同一提交。原来的 `current/` 和 `releases/` 供旧部署回退。
- 机器狗连接雷达时，机器狗采集账号的 `~/.ssh/known_hosts` 保存核对过的两台雷达主机公钥。雷达回拉 IMU/odom 时，每台雷达账号的 `~/.ssh/fleet_controller_known_hosts` 保存它实际连接的机器狗地址和经核对的主机公钥。机器狗的主机私钥由本机 SSH 服务保管。回拉使用的客户端私钥留在各台雷达，授权公钥登记在机器狗账号下。
- 机器狗在 `~/.ssh/authorized_keys` 中为雷达回拉公钥绑定带 `restrict` 的 `export_controller_session.py`。并行部署时，可在绑定的命令中为每套控制端配置传入一个 `--config`。导出入口从这些配置读取 `output_root`，根据会话名匹配唯一会话。雷达端统一使用 `export-session NAME`。

迁移顺序为整理当前线上代码并记录正在使用的波形配置、并行建立 Git 检出目录、验证 SSH 和采前检查、验证静态短采及 IMU/odom 回拉，最后切换原有服务入口。旧目录可整体保留用于回退及访问历史数据。

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

## 雷达站版本同步

双雷达模式下，在机器狗 UI 选择“4. 同步代码”。NX1 是主版本源，会在其工作目录生成：

```text
releases/vYYYYMMDD_HHMMSS/
current -> releases/vYYYYMMDD_HHMMSS
```

发布包内带有 `VERSION` 和 `SHA256SUMS`。NX2 先解包并执行 `sha256sum -c`，通过后才原子切换 `current`，最后尝试重启 `nx-radar-ui.service`。`station/`、`data/`、`logs/` 和每台设备的 `radar_station_config.json` 不会被共享包覆盖。
