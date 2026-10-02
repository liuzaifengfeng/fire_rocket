"""Exercise the Tk window, optionally reading an explicitly selected board port."""
import argparse
import ctypes
import json
from pathlib import Path
import subprocess
import sys
import tkinter as tk

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from desktop.app import MonitorApp

parser=argparse.ArgumentParser()
parser.add_argument("--port")
parser.add_argument("--snapshot-python",type=Path)
parser.add_argument("--output",type=Path,required=True)
args=parser.parse_args()
if sys.platform=="win32":
    ctypes.windll.shcore.SetProcessDpiAwareness(1)
root=tk.Tk()
app=MonitorApp(root,args.port)
args.output.mkdir(parents=True,exist_ok=True)
failed=[]
root.report_callback_exception=lambda kind,value,trace:failed.append(str(value))


def verify():
    try:
        root.update_idletasks()
        assert root.winfo_width()>=960
        assert app.plot.winfo_height()>100
        assert app.footer.winfo_ismapped(),"Footer not visible"
        assert app.footer.winfo_rooty()+app.footer.winfo_height()<=root.winfo_rooty()+root.winfo_height(),"Footer clipped"
        for tab in app.notebook.tabs():
            app.notebook.select(tab)
            root.update_idletasks()
        app.notebook.select(0)
        root.update()
        if args.port:
            assert app.state.responding(),"Board did not answer STATUS"
        result={"tk_window":True,"board_responded":app.state.responding(),
                "port":args.port,"status":app.state.status,"sensor":app.state.sensor,
                "callback_errors":failed,"size":[root.winfo_width(),root.winfo_height()]}
        (args.output/"gui_smoke.json").write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
        (args.output/"gui_serial.log").write_text(app.log.get("1.0","end-1c"),encoding="utf-8")
        if args.snapshot_python:
            box=(root.winfo_rootx(),root.winfo_rooty(),root.winfo_rootx()+root.winfo_width(),root.winfo_rooty()+root.winfo_height())
            source="import ctypes,sys; ctypes.windll.shcore.SetProcessDpiAwareness(1); from PIL import ImageGrab; ImageGrab.grab(bbox=tuple(map(int,sys.argv[2:6]))).save(sys.argv[1])"
            subprocess.run([str(args.snapshot_python),"-c",source,str(args.output/"upper_computer.png"),*map(str,box)],check=True)
        assert not failed,failed
        print("GUI_SMOKE_PASS",json.dumps(result,ensure_ascii=True))
    except Exception as exc:
        failed.append(str(exc));print("GUI_SMOKE_FAIL",str(exc))
    finally:
        worker=app.worker
        app.close()
        if worker:
            worker.join(timeout=2)


root.after(4000 if args.port else 800,verify)
root.mainloop()
raise SystemExit(bool(failed))
