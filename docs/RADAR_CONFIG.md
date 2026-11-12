# 雷达参数和配置

当前双雷达软件触发采集使用：

`radar_station/configFiles/mmwaveconfig_tx0_rx03_77g_25m_1200s_256c_650ms.txt`

仓库里的参数 PPT 用来说明 mmWave Studio、chirp/frame 和 DCA1000 的关系；PPT 中的其他示例波形不能直接替代这份配置。

| 参数 | 当前设置 |
| --- | --- |
| 发射与接收 | TX0；RX0、RX3（设备面板常写作 RX1、RX4） |
| 起始频率、斜率 | 77 GHz；约 25 MHz/μs |
| ADC | 16 位 Complex 1x；10 MSps；每 chirp 每 RX 1200 个复数点 |
| ADC 起始、斜坡结束 | 6 μs；130 μs |
| chirp 间隔 | 2370 μs 空闲；起始间隔 2500 μs |
| 每帧 chirp | 索引 0–1，每个 chirp 循环 128 次，共 256 个 |
| 帧周期 | 650 ms；软件触发 `triggerSelect=1` |
| LVDS/DCA | 4 lane；`iqSwapSel=0`；`chInterleave=0`；16 位复数原始数据 |

单帧字节数为：

```text
1200 点 × 256 chirp × 2 RX × 2(I/Q) × 2 字节 = 2,457,600 字节
```

程序还会检查完整帧数、文件大小、UDP 补零和内容质量。文件大小正确只能说明字节数对上，不能单独证明数据有效。

## 硬件触发版本

配置文件：

`configFiles/mmwaveconfig_tx0_rx03_77g_25m_1200s_256c_hwtrig_650ms.txt`

它使用 `triggerSelect=2`，STM32 的 A/B 触发相隔 1250 μs，触发对周期 700 ms。STM32 没有接好之前不要选这份配置。

原始 `.bin` 在采集端保持 DCA 字节序，NX 侧不做 I/Q 或 RX 重排。I/Q 解码和双雷达相位融合还需要用 TI 四 lane 定义和已知目标实验复核；采集质量报告不能代替相位标定报告。
