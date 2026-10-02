"""One background thread owns the serial port; exports suspend polling."""
from pathlib import Path
import queue
import threading
import time

from tools.usb_monitor import save_export


class SerialWorker(threading.Thread):
    def __init__(self, port_name, events):
        super().__init__(daemon=True)
        self.port_name = port_name
        self.events = events
        self.tasks = queue.Queue()
        self.stopping = threading.Event()
        self.port = None
        self.pending = b""

    def submit(self, kind, payload):
        self.tasks.put((kind, payload))

    def stop(self):
        self.stopping.set()
        if self.port:
            try:
                self.port.cancel_read()
            except (AttributeError, OSError):
                pass

    def readline(self):
        if self.stopping.is_set():
            raise InterruptedError("用户已断开")
        self.pending += self.port.readline()
        if len(self.pending) > 16384:
            self.pending = b""
            raise ValueError("串口行超过长度限制；请核对固件和波特率")
        if b"\n" not in self.pending:
            return b""
        line, self.pending = self.pending.split(b"\n",1)
        return line+b"\n"

    def write(self, data):
        return self.port.write(data)

    def request(self, command):
        self.events.put(("tx", command))
        self.port.write((command+"\n").encode("ascii"))
        deadline = time.monotonic()+2.5
        quiet = None
        lines = []
        while not self.stopping.is_set() and time.monotonic() < deadline:
            raw = self.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                lines.append(line)
                self.events.put(("line", line))
                quiet = time.monotonic()+0.12
            elif quiet is not None and time.monotonic() >= quiet:
                break
        return lines

    def run(self):
        try:
            import serial
            self.port = serial.Serial(self.port_name, 115200, timeout=0.05, write_timeout=1)
            self.port.reset_input_buffer()
            self.events.put(("connected", self.port_name))
            self.request("CFG GET")
            next_poll = 0
            while not self.stopping.is_set():
                try:
                    kind, payload = self.tasks.get_nowait()
                except queue.Empty:
                    kind = None
                if kind is not None:
                    self.events.put(("busy", True))
                    try:
                        if kind == "commands":
                            for command in payload:
                                lines = self.request(command)
                                if not lines or any(line.startswith("ERR") for line in lines):
                                    raise RuntimeError(next((x for x in lines if x.startswith("ERR")), "板卡未应答"))
                                if command not in ("STATUS", "CFG GET") and not any(x.startswith("OK") for x in lines):
                                    raise RuntimeError("未收到命令确认："+command)
                        elif kind == "export":
                            for name, filename in (("CSV","telemetry.csv"),("META","session.json"),
                                                   ("EVENTS","events.csv"),("END","end.json")):
                                save_export(self, name, Path(payload)/filename)
                                self.events.put(("log", "已保存 "+str(Path(payload)/filename)))
                            self.events.put(("notice", "导出完成："+str(payload)))
                    except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
                        self.events.put(("error", str(exc)))
                    finally:
                        self.events.put(("busy", False))
                    next_poll = 0
                elif time.monotonic() >= next_poll:
                    self.request("STATUS")
                    next_poll = time.monotonic()+0.35
                else:
                    self.stopping.wait(0.025)
        except Exception as exc:
            if not self.stopping.is_set():
                self.events.put(("error", "串口连接异常："+str(exc)))
        finally:
            if self.port:
                self.port.close()
            self.events.put(("disconnected", self.port_name))
