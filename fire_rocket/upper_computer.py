"""Run the Chinese desktop telemetry monitor: python upper_computer.py [--port COM19]."""
import argparse
from pathlib import Path
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port",help="Open an explicitly selected port after launching")
    args=parser.parse_args()
    if sys.platform=="win32":
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except OSError:
            pass
    import tkinter as tk
    from desktop.app import MonitorApp
    root=tk.Tk()
    MonitorApp(root,args.port)
    root.mainloop()


if __name__=="__main__":
    try:
        main()
    except Exception:
        import traceback
        report=traceback.format_exc()
        Path(__file__).with_name("upper_computer_error.log").write_text(report,encoding="utf-8")
        if sys.stderr:
            print(report,file=sys.stderr)
        try:
            from tkinter import messagebox
            messagebox.showerror("上位机启动失败","请使用“启动上位机.cmd”。错误详情已写入 upper_computer_error.log。\n"+report[-800:])
        except Exception:
            pass
        raise SystemExit(1)
