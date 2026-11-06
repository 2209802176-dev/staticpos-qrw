# 快速开始

先在电脑上完成合成输入检查，再准备目标板实验。所有命令从仓库根目录运行。主机检查不需要飞控、串口或真实定位数据。

## 1. 准备 Python 和编译器

需要 Python 3.11 或更新版本、可编译 C++11 的 GCC，以及显示器使用的 Tcl/Tk。Windows 安装 Python 时保留 Tcl/Tk 组件；Linux 无桌面环境可先跳过 GUI 检查。

Linux / macOS：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
export CXX=g++
python tools/run_checks.py
```

Windows PowerShell：

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
$env:CXX = 'g++.exe'
.\.venv\Scripts\python.exe tools/run_checks.py
```

`CXX` 可以指定可执行文件名或你安装的编译器路径；编译器需要与当前操作系统一致。无桌面环境使用 `python tools/run_checks.py --skip-gui`，或在配置好 Xvfb 后运行 `xvfb-run -a python tools/run_checks.py`。跳过 GUI 时，结果仅覆盖未跳过的检查。

## 2. 看懂检查结果

完整主机检查会编译 C++ 核心和实际飞控适配器的模拟边界测试，比较 NumPy 合成参考，检查适配器到 Python 聚合器的逐包协议，并验证显示器。

| 输出 | 用途 |
| --- | --- |
| `runtime/host_checks.json` | 本次主机检查的命令结果与输出 |
| `runtime/replay_validation.json` | C++ 与 NumPy 参考的回放对照 |
| `runtime/monitor_validation.json` | 显示器、记录及可选 GUI 检查 |
| `runtime/` | 本机编译得到的测试程序与临时文件 |

这些检查证明相应软件行为，不代表目标板时耗、真实串口连续性或上机定位精度已经通过验证。公开检查默认使用合成输入，不需要项目的历史采集数据。希望对自己的本地 CSV 做回放时，先运行 `python tools/validate_replay.py --help` 查看显式输入选项和字段要求；这些文件不必上传仓库。

## 3. 准备固件

已有与你的源版本对应的固件时，先核对板型和 SHA-256 清单。自己构建请按 [构建指南](BUILDING.md) 操作，目标是 **ArduCopter 4.6.3 / fmuv3 / APJ board ID 9**。

部署文件是 `.apj`；`.elf` 用于调试和检查。保存设备现有参数及可恢复固件，按设备原有流程加载自定义固件。显示器不执行刷写。

## 4. 打开显示器

Windows 双击仓库根目录的 `启动显示器.cmd`，或运行：

```powershell
.\.venv\Scripts\python.exe monitor/static_monitor.py
```

其他桌面环境运行 `python monitor/static_monitor.py`。先关闭占用同一串口的程序，再手工选择 COM、设备实际波特率和系统 ID，点击连接。UDP 示例 `udpin:127.0.0.1:14551` 需要你已经配置好转发链路。

有效 V3 遥测到来后，界面显示：

- 原始 GPS、原导航接口和自研算法三路坐标；
- 质量尺度、转向倍率、Huber 因子、最新点权重与窗口点数；
- GPS 源时刻、遥测快照序号、数据年龄和实收批次间隔；
- 计算时耗、最长时耗与超过初始预约预算的次数。

固件标记应为 `ST_VER=3`。原导航接口有效不等于完整 EKF 健康认证；各路坐标也不能仅凭同一显示行就视作同一源历元。

## 5. 记录一次定点实验

仅使用单 GPS、未解锁、固定位置与原地转向场景。算法从首个有效 GPS 开始计算，90 秒窗口逐步填充；转向保留已有历史。IMU 负责转向调权，没有加速度积分定位。

连接时勾选日志，开始和结束转向时记录操作标记。每次连接在 `monitor_logs/` 新建 CSV、`.messages.jsonl` 和 `.events.jsonl`。需要逐 GPS 更新与目标板时耗时，同时核对机载记录是否启用并保存对应日志。显示缓存最多保留 5 秒，主动失效会立即清空；缓存快照不算新的 GPS 观测。

所有公开问题报告请先使用合成输入复现。文档中的坐标 `10.0000000°N, 20.0000000°E` 只是合成示例，不能用作测点参考坐标。

下一步：[构建指南](BUILDING.md) · [故障排查](TROUBLESHOOTING.md) · [显示器说明](../monitor/README.md) · [贡献指南](../CONTRIBUTING.md)
