"""Presentation state for both the original and diagnostic-v2 monitor protocols."""
from dataclasses import dataclass, field
import math
import re
import time


def key_values(text):
    return dict(re.findall(r"(\w+)=(.*?)(?=\s+\w+=|$)", text))


def number(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (ValueError, TypeError):
        return None


REASONS = {
    "NO_RX": "未收到 UART 字节", "WAIT_CONFIG": "收到字节，但配置尚未读到",
    "NO_SAMPLE": "尚无有效样本", "STALE": "传感器数据已超时",
    "NO_ALTITUDE": "模块或订阅未提供高度", "NOT_ZEROED": "尚未设置地面高度基准",
    "NO_ATTITUDE": "未订阅姿态字段", "BAD_ATTITUDE": "姿态数据无效", "OK": "有效",
}


@dataclass
class MonitorState:
    connected: bool = False
    status: dict = field(default_factory=dict)
    sensor: dict = field(default_factory=dict)
    diagnostic: dict = field(default_factory=dict)
    raw: dict = field(default_factory=dict)
    config: dict = field(default_factory=dict)
    height: float | None = None
    tilt: float | None = None
    rx_hex: str = ""
    last_status: float | None = None

    def reset(self, connected=False):
        self.__dict__.update(MonitorState(connected=connected).__dict__)

    def ingest(self, line, now=None):
        now = time.monotonic() if now is None else now
        kind, _, body = line.partition(" ")
        target = {"STATUS": self.status, "SENSOR": self.sensor, "DIAG": self.diagnostic,
                  "RAW": self.raw, "CFG": self.config}.get(kind)
        if target is not None:
            target.clear()
            target.update(key_values(body))
            if kind == "STATUS":
                self.last_status = now
                self.height = self.tilt = None
                self.raw.clear()
        elif kind == "HEIGHT":
            self.height = number(body.split()[0]) if body else None
        elif kind == "TILT":
            self.tilt = number(body.split()[0]) if body else None
            return True
        elif kind == "RXHEX":
            self.rx_hex = body
        return False

    def responding(self, now=None):
        now = time.monotonic() if now is None else now
        return self.connected and self.last_status is not None and now-self.last_status < 3

    def fresh(self, now=None):
        return self.responding(now) and self.status.get("fresh") == "1"

    def displayed(self, key, now=None):
        if not self.fresh(now):
            return None
        if key == "height":
            return self.height if self.status.get("zeroed") == "1" else None
        if key == "tilt":
            return self.tilt
        return number(self.raw.get(key))

    def reason(self, key, now=None):
        if not self.connected:
            return "未连接"
        if not self.responding(now):
            return "等待板卡响应"
        if not self.fresh(now):
            return "无有效数据或数据超时"
        code = self.diagnostic.get(f"{key}_reason")
        if code:
            return REASONS.get(code, code)
        if key == "height" and self.status.get("zeroed") != "1":
            return "等待手动归零"
        return "有效" if self.displayed(key, now) is not None else "缺少有效字段"

    def diagnosis(self, now=None):
        if not self.connected:
            return "未连接设备", "选择 ESP32 的 USB 串口，然后点击连接。蓝牙串口不是这块开发板的 USB 接口。"
        if not self.responding(now):
            return "串口已打开，等待固件响应", "检查端口、USB 数据线和固件；本程序只发送 STATUS / CFG GET，不会自动归零或清空记录。"
        if self.sensor.get("config") != "1":
            if self.diagnostic.get("rx_bytes") == "0":
                return "板卡在线，但未收到 AS201 字节", "检查传感器供电、共地和 UART：传感器 TX → GPIO4，RX → GPIO3；默认 115200。"
            if self.diagnostic.get("rx_bytes") is not None:
                return "UART 有数据，但传感器配置未解析", "查看原始接收字节和错误计数，核对波特率及模块协议；点击“重新查询传感器”。"
            return "板卡在线，但尚未读到传感器配置", "点击“重新查询传感器”。当前固件没有 UART 字节诊断；有效样本为 0 时，归零不能解决姿态缺失。"
        if self.sensor.get("reporting") == "0":
            return "传感器配置显示主动上报关闭", "当前监测固件只查询配置，不自动改写模块。请核对传感器上报设置。"
        if self.sensor.get("samples", "0") == "0":
            return "配置已读到，但没有有效样本", "查看订阅字段、校验和及数据错误；不要把接收失败理解成实际高度为零。"
        if not self.fresh(now):
            return "传感器数据已超时", "显示值已置为无效。检查供电、连线和上报频率；曲线保留历史，但不会将旧值当成实时值。"
        try:
            mask = int(self.raw.get("fields", self.sensor.get("fields", "0")), 16)
        except ValueError:
            mask = 0
        if not mask & 128:
            return "收到数据，但没有高度字段", "六轴/九轴模块没有气压高度；十轴模块也需要订阅高度字段。"
        if self.status.get("zeroed") != "1":
            return "正在接收数据，尚未归零", "将板卡放稳，点击“高度归零”。相对高度 H 在归零前显示 --；倾角 T 不依赖归零。"
        if self.tilt is None:
            return "高度有效，姿态字段无效", "查看四元数/欧拉角的订阅及数据质量；高度归零不会修复姿态数据。"
        if self.status.get("source") == "REPLAY":
            return "当前数据来自回放", "这些是注入的测试数据，不是现场传感器读数。"
        return "传感器数据正常", "H 是相对归零点高度，T 是模块 +Z 与竖直方向的夹角。记录区状态单独显示，不影响传感器接收。"
