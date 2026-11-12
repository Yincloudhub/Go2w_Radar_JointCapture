# 雷达参数与配置

本页对应当前双雷达软件触发波形。参数的来源是 `radar_station/configFiles/mmwaveconfig_tx0_rx03_77g_25m_1200s_256c_650ms.txt`；上传的[参数教程 PPT](reference/雷达参数.pptx)用于理解 mmWave Studio、chirp/frame 与 DCA1000 数据链路，其中其他示例波形不能直接代替本配置。

| 参数 | 当前值 |
| --- | --- |
| 发射与接收 | TX0；RX0、RX3（从 1 开始编号为 RX1、RX4） |
| 起始频率、斜率 | 77 GHz；约 25 MHz/μs |
| ADC | 16 位 Complex 1x；10 MSps；每 chirp 每 RX 1200 个复数点 |
| ADC 开始、斜坡结束 | 6 μs；130 μs |
| chirp 空闲时间 | 2370 μs；相邻 chirp 起始间隔 2500 μs |
| 每帧 chirp | 索引 0–1，各使用 TX0，循环 128 次，共 256 个 |
| 帧周期 | 650 ms；软件触发 `triggerSelect=1` |
| LVDS/DCA | 4 lane；`iqSwapSel=0`；`chInterleave=0`；16 位复数原始数据 |

单帧字节数：`1200 样点 × 256 chirp × 2 RX × 2(I/Q) × 2 字节 = 2,457,600 字节`。采集程序会检查完整帧数、文件大小、UDP 补零字节数和内容质量；不能只凭文件大小认定采集有效。

硬触发配置在 `configFiles/mmwaveconfig_tx0_rx03_77g_25m_1200s_256c_hwtrig_650ms.txt`，雷达帧配置使用 `triggerSelect=2`，STM32 的 A/B 触发间隔为 1250 μs、触发对周期 700 ms。**未接 STM32 时不要选择它。**

原始 `.bin` 保留采集字节序，不在 NX 侧做 I/Q 或 RX 重排。复数解码与双雷达相位融合还需要结合 TI 四 lane 读取定义和已知目标实验复核，不应把本仓库的采集成功报告当作相位标定报告。
