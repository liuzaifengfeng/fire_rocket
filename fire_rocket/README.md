# 火火箭：显示与遥测固件

本次交付是**监测固件，不是完整飞控**。已实现 AS201 解析、OLED、地面高度归零、USB 配置、数据/异常记录及回放测试。

原方案中的二级点火、开伞、起飞识别、自动待发、延时/预测控制和评分优化**未实现**。代码不包含执行器脉冲或相关启用命令，GPIO0/1/7 在 `setup()` 中配置为低电平输出并保持关闭；这不代表已验证实物上电期间的波形。不能将本固件作为发射控制程序使用。

## 硬件和协议依据

| 项目 | 配置 |
| --- | --- |
| 开发板 | ESP32-C3，Arduino，4 MB Flash |
| OLED | SSD1306，72×40；SDA GPIO5，SCL GPIO6；I²C 地址 0x3C |
| 传感器 | HLK-AS201 十轴；UART RX GPIO4，TX GPIO3；115200、8N1 |
| 箭体朝向 | 模块 +Z 指向箭头 |
| 用户按键 | BOOT/GPIO9，低电平按下；按用户约定忽略充放电 KEY 关联 |
| USB | 原生 USB CDC，GPIO18/19 |

资料来源：仓库 `DOC/HLK-AS201系列姿态传感器模块说明书V1.1.pdf`、OLED 开发板资料及 `PCB/火火箭V1.3` 原理图。

程序每 2 秒只读查询传感器的订阅配置；不会修改传感器波特率、上报频率或校准参数。初始未读取到配置时，不猜测数据格式。支持十轴、九轴、六轴及不同订阅组合；六/九轴没有高度时，显示 `--`。

解析包含帧头、长度、累加校验、帧尾、字段顺序及说明书量程检查，支持拆包、粘包和损坏长度后重新同步。未知字段、旧值、预测值不会冒充有效测量。

## 屏幕和按键

屏幕采用英文短标记以适应小尺寸：

1. 主页面：相对高度 `H`、+Z 与竖直方向夹角 `T`、数据状态、LIVE/REPLAY。
2. 诊断页面：接收数、错误数、订阅掩码、磁场质量。
3. **记录页面**：记录状态和条数，不是飞控待发页面。

短按翻页；主页面长按 2 秒尝试归零。记录页面长按进入“覆盖旧记录”确认，5 秒内短按确认开始记录。记录过程中任意页面长按停止记录。去抖时间为 40 ms；长按释放不会再触发短按。

归零取最近一段连续高度数据的平均值，要求窗口完整、数据新鲜且高度波动不超过配置范围。此检查只能确认高度读数稳定，不能证明箭体位置或飞行状态。归零只修改软件高度基准，不写入传感器，也不跨重启保存。

倾角优先从有效四元数计算；未订阅四元数时使用欧拉角。不用加速度直接代替姿态。磁干扰质量单独提示，实物轴向与角度准确性仍需验证。

## USB 命令

### Python 可视化上位机

双击项目中的 **`启动上位机.cmd`**，选择 ESP32 USB 串口并连接。本机已验证 Python 3.13 + Tk + pyserial；也可以执行 `python upper_computer.py`，或用 `--port COM19` 明确指定连接端口。

上位机提供实时 H/T 曲线、传感器字段、故障诊断、归零、参数读写、记录和导出。它兼容已烧录的基础版固件：无需先更新固件即可查看配置是否读到、有效样本数、数据新鲜度和记录区状态。连接仅进行只读查询，不自动归零、改配置或格式化。

诊断版固件在 `STATUS` 后新增 `DIAG`、`RXHEX` 和 `RAW` 行，提供 UART 接收字节数、最近字节、具体失效原因、绝对高度/气压/温度及姿态数据。旧固件未提供的字段会明确标为未知，不会补造数值。新增固件已编译，未自动烧录。

`H --` 可能是尚未归零；`T --` 与高度归零无关，表示姿态数据无效或未收到。`NEED FORMAT` 仅影响记录区，不妨碍传感器解析和显示。诊断版 OLED 也会区分 `NO UART RX`、`WAIT CONFIG`、`NO SAMPLE`。

窗口关闭或点击断开会停止轮询并释放串口。烧录前先断开上位机，避免占用同一个端口。初始化记录区会清除旧记录，需要在界面中明确确认。

### 文本命令

以换行结束，区分大小写。首次无文件系统时显示 `NEED FORMAT`，不会自动清空设备。

```text
HELP
STATUS
CFG GET
CFG SET stale_ms 500
CFG SET record_seconds 600
CFG SET zero_window_ms 2000
CFG SET zero_span_m 0.5
CFG SAVE
SENSOR QUERY
ZERO
RECORD START
RECORD STOP
EXPORT CSV
EXPORT META
EXPORT EVENTS
EXPORT END
EXPORT CANCEL
```

首次需要建立记录区时，显式发送 `FS FORMAT CONFIRM`。该命令清除记录区；记录和导出过程中禁止格式化。

| 参数 | 默认值 | 接受范围 | 用途 |
| --- | --- | --- | --- |
| `stale_ms` | 500 | 100–30000 ms | 监测数据显示新鲜度 |
| `record_seconds` | 600 | 1–600 s | 单次记录时间上限 |
| `zero_window_ms` | 2000 | 500–10000 ms | 手动归零采样窗口 |
| `zero_span_m` | 0.5 | 大于 0，且不超过 10 m | 归零窗口内高度极差 |

以上均为**监测参数，不是飞行控制阈值**。传感器以低于默认 20 Hz 的速率上报时，需相应设置数据显示新鲜度。`CFG SET` 成功会要求重新归零；`CFG SAVE` 才持久保存。记录期间锁定配置、数据源与归零，重启不自动开始记录。

### 保存到电脑

可使用 PlatformIO 自带的 Python（包含 pyserial），端口以实际设备为准：

```powershell
$pioPython = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe"
& $pioPython tools/usb_monitor.py --port COM5 command "STATUS"
& $pioPython tools/usb_monitor.py --port COM5 command "RECORD STOP"
& $pioPython tools/usb_monitor.py --port COM5 export --output output/session01
```

工具只打开明确指定的端口，不自动烧录或格式化。导出产生 `telemetry.csv`、`session.json`、`events.csv`、`end.json`；拒绝覆盖已有电脑文件。异常中断保留 `.partial`，不会把不完整数据标成成功文件。如果板卡在记录期间掉电，`end.json` 可能不存在，已刷入的其他文件仍可分别用终端导出。

## 记录格式与限制

- 使用内部 LittleFS，记录每个成功解析的数据样本；传感器默认 20 Hz，固件不强行补齐缺失样本。
- 原始数据、时间戳、字段掩码、模块类型、磁场质量、相对高度、倾角和归零基准写入记录。缺失字段导出为 `nan`，不是 0。
- `validity`：bit0 数据新鲜、bit1 相对高度有效、bit2 倾角有效。`fields` 的 bit0–7 分别是加速度、角速度、欧拉角、磁场、四元数、温度、气压、高度。
- 事件记录每 250 ms 检查一次状态变化，保留新鲜度、累计协议错误、磁场质量、订阅掩码及模块类型；它不是每次错误的完整原始字节抓包。
- 新一轮记录覆盖上一轮，单纯重启不删除记录。启动记录不要求归零，因此可以记录无相对高度的原始诊断数据。
- 样本批量写入，每 500 ms 刷新；突然掉电可能丢失最近未刷入的数据。缺少结束文件表示未正常结束，不应推断成功完成。
- 存储空间不足、写入失败或时间到达上限时停止记录，监测仍继续。分区为 1.5 MiB 应用和约 2.44 MiB 数据区，无 OTA 功能。
- Flash 写入和 OLED 总线实际耗时尚未实测，不能据此声称满足飞行实时性要求。

## 编译与验证

在 `fire_rocket` 目录执行：

```powershell
pio run
.\tools\test.ps1
python -m unittest discover -s test/host -p "test_*.py"
python tools/replay.py demo --output replay-output/demo.txt
python tools/replay.py run --exe test-results/replay_main.exe --input replay-output/demo.txt --output replay-output --check-demo
```

原生测试支持 PATH 中的 g++，本机也可自动识别 Dev-C++ 附带的编译器；其他位置使用 `tools/test.ps1 -Cxx <编译器绝对路径>`。没有 Python 命令时可使用上述 `$pioPython`。

回放工具运行与固件相同的 C++ 解析器和监测逻辑。生成数据包含稳定读数、轻微变化、断流、坏校验、损坏长度、拆包与漂移；**不是火箭动力学或点火/开伞仿真**。

需要将传感器帧送入板卡做显示测试时，可发送 `SOURCE REPLAY`，再发送 `RX <不带空格的十六进制字节>`；恢复实物传感器用 `SOURCE LIVE`。两种来源互斥，切换会清除数据与归零基准，重启默认 LIVE。回放输入须先包含 `0x19` 配置响应帧。

## 验证边界

已进行软件测试、ESP32-C3 编译和 Python 上位机通过 COM19 读取已烧录基础版固件的联调。未烧录新增诊断版，未完成 OLED 实物目视检查、传感器数据恢复校验或飞行试验。具体结果见 `docs/verification.md`。
