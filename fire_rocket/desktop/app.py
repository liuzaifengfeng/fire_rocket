from collections import deque
from datetime import datetime
from pathlib import Path
import queue
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .model import MonitorState
from .transport import SerialWorker

BG = "#eef2f7"
INK = "#18283d"
MUTED = "#62748a"
BLUE = "#187bc4"
ORANGE = "#e89528"


class Plot(tk.Canvas):
    def __init__(self, parent):
        super().__init__(parent, background="#112236", highlightthickness=0, height=240)
        self.samples = deque(maxlen=180)
        self.bind("<Configure>", lambda _: self.redraw())

    def add(self, h, tilt):
        self.samples.append((time.monotonic(), h, tilt))
        self.redraw()

    def redraw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 150 or h < 100:
            return
        now = time.monotonic()
        points = [p for p in self.samples if now-p[0] <= 90]
        for band, label, color, index in ((0,"相对高度 / m", "#55caff",1),(1,"倾角 / °", "#ffbf63",2)):
            top, bottom = 32+band*h/2, (band+1)*h/2-25
            left, right = 64, w-22
            self.create_text(left, top-15, text=label, anchor="w", fill=color, font=("Microsoft YaHei UI",10))
            values = [p[index] for p in points if p[index] is not None]
            low, high = (min(values),max(values)) if values else (0,1)
            padding = max((high-low)*0.15, 0.1 if index==1 else 1)
            low, high = low-padding, high+padding
            for i in range(4):
                y = top+(bottom-top)*i/3
                self.create_line(left,y,right,y,fill="#2a4057")
                self.create_text(left-8,y,text=f"{high-(high-low)*i/3:.1f}",anchor="e",fill="#9db0c4",font=("Segoe UI",9))
            segment=[]
            previous=None
            for point in points:
                value=point[index]
                if value is None or (previous is not None and point[0]-previous>2):
                    if len(segment)>=4:
                        self.create_line(*segment, fill=color,width=2)
                    segment=[]
                if value is not None:
                    segment.extend((right-(now-point[0])/90*(right-left), bottom-(value-low)/(high-low)*(bottom-top)))
                previous=point[0]
            if len(segment)>=4:
                self.create_line(*segment,fill=color,width=2)
            if not values:
                self.create_text((left+right)/2,(top+bottom)/2,text="等待有效数据 · 无效值不会画成 0",fill="#9db0c4",font=("Microsoft YaHei UI",11))
            self.create_text(right,bottom+13,text="最近 90 秒",anchor="e",fill="#9db0c4",font=("Microsoft YaHei UI",9))


class MonitorApp:
    def __init__(self, root, port=None):
        self.root=root
        root.title("火火箭 · 遥测监测台")
        root.geometry("1180x830")
        root.minsize(1060,800)
        root.configure(background=BG)
        root.protocol("WM_DELETE_WINDOW",self.close)
        self.state=MonitorState()
        self.events=queue.Queue()
        self.worker=None
        self.busy=False
        self.closing=False
        self.last_gap=0
        self.last_config=None
        self.action_result=None
        self.controls=[]
        self.info={}
        self.cards={}
        self.config_vars={}
        style=ttk.Style(root)
        style.theme_use("clam")
        style.configure("TFrame",background=BG)
        style.configure("TLabel",background=BG,foreground=INK,font=("Microsoft YaHei UI",10))
        style.configure("TButton",font=("Microsoft YaHei UI",10),padding=(10,6))
        style.configure("TNotebook",background=BG,borderwidth=0)
        style.configure("TNotebook.Tab",padding=(16,8),font=("Microsoft YaHei UI",10))
        style.configure("Treeview",rowheight=28,font=("Microsoft YaHei UI",10))
        style.configure("Treeview.Heading",font=("Microsoft YaHei UI",10,"bold"))
        self.build()
        self.refresh_ports()
        self.root.after(80,self.pump)
        if port:
            self.port_var.set(port)
            self.root.after(300,self.connect)

    def build(self):
        header=tk.Frame(self.root,bg="#112236",padx=24,pady=15)
        header.pack(fill="x")
        tk.Label(header,text="火火箭  /  遥测监测台",font=("Microsoft YaHei UI",20,"bold"),bg="#112236",fg="white").pack(side="left")
        tk.Label(header,text="ESP32-C3  ·  HLK-AS201  ·  USB",font=("Segoe UI",11),bg="#112236",fg="#9bb5d0").pack(side="right")
        content=ttk.Frame(self.root,padding=(20,12))
        content.pack(fill="both",expand=True)
        connection=ttk.Frame(content)
        connection.pack(fill="x",pady=(0,12))
        ttk.Label(connection,text="串口").pack(side="left")
        self.port_var=tk.StringVar()
        self.ports=ttk.Combobox(connection,textvariable=self.port_var,width=38,state="readonly")
        self.ports.pack(side="left",padx=8)
        self.refresh_button=ttk.Button(connection,text="刷新",command=self.refresh_ports)
        self.refresh_button.pack(side="left")
        self.connect_button=ttk.Button(connection,text="连接",command=self.connect)
        self.connect_button.pack(side="left",padx=6)
        self.disconnect_button=ttk.Button(connection,text="断开",command=self.disconnect,state="disabled")
        self.disconnect_button.pack(side="left")
        self.connection_label=ttk.Label(connection,text="未连接",foreground=MUTED)
        self.connection_label.pack(side="right")

        cards=ttk.Frame(content)
        cards.pack(fill="x")
        for i,(key,title,unit) in enumerate((("height","相对高度 H","m"),("tilt","倾角 T","°"),
                                            ("altitude_m","传感器海拔","m"),("samples","有效样本","帧"))):
            cards.columnconfigure(i,weight=1)
            panel=tk.Frame(cards,bg="white",padx=16,pady=12,highlightthickness=1,highlightbackground="#dce4ed")
            panel.grid(row=0,column=i,sticky="nsew",padx=(0,10 if i<3 else 0))
            tk.Label(panel,text=title,bg="white",fg=MUTED,font=("Microsoft YaHei UI",10)).pack(anchor="w")
            value=tk.Label(panel,text="--",bg="white",fg=INK,font=("Segoe UI",28,"bold"))
            value.pack(anchor="w")
            detail=tk.Label(panel,text=unit+" · 等待数据",bg="white",fg=MUTED,font=("Microsoft YaHei UI",9))
            detail.pack(anchor="w")
            self.cards[key]=(value,detail,unit)

        banner=tk.Frame(content,bg="#fff4df",padx=14,pady=9)
        banner.pack(fill="x",pady=12)
        self.diag_title=tk.Label(banner,text="等待连接",bg="#fff4df",fg="#815215",font=("Microsoft YaHei UI",11,"bold"),anchor="w")
        self.diag_title.pack(fill="x")
        self.diag_detail=tk.Label(banner,text="",bg="#fff4df",fg="#815215",font=("Microsoft YaHei UI",9),anchor="w",justify="left",wraplength=1050)
        self.diag_detail.pack(fill="x",pady=(3,0))
        banner.bind("<Configure>",lambda e:self.diag_detail.configure(wraplength=max(500,e.width-32)))

        actions=ttk.Frame(content)
        actions.pack(fill="x",pady=(0,10))
        for text,callback in (("高度归零",lambda:self.send(["ZERO","STATUS"])),
                              ("重新查询传感器",lambda:self.send(["SENSOR QUERY","STATUS"])),
                              ("开始记录（覆盖旧记录）",lambda:self.send(["RECORD START","STATUS"])),
                              ("停止记录",lambda:self.send(["RECORD STOP","STATUS"])),
                              ("导出记录",self.export)):
            b=ttk.Button(actions,text=text,command=callback,state="disabled")
            b.pack(side="left",padx=(0,6)); self.controls.append(b)

        notebook=ttk.Notebook(content)
        self.notebook=notebook
        notebook.pack(fill="both",expand=True)
        live=ttk.Frame(notebook,padding=10)
        raw=ttk.Frame(notebook,padding=10)
        config=ttk.Frame(notebook,padding=16)
        logs=ttk.Frame(notebook,padding=8)
        for page,title in ((live,"实时曲线与诊断"),(raw,"传感器字段"),(config,"监测参数"),(logs,"串口日志")):
            notebook.add(page,text=title)
        self.plot=Plot(live)
        self.plot.pack(side="left",fill="both",expand=True,padx=(0,14))
        facts=ttk.Frame(live,width=240)
        facts.pack(side="right",fill="y")
        for row,(key,title) in enumerate((("source","数据源"),("config","订阅配置"),("rx_bytes","UART 字节数"),
                          ("fields","订阅字段"),("rate","上报频率"),("errors","协议错误"),
                          ("storage","记录区"),("recording","记录状态"),("version","诊断版本"))):
            ttk.Label(facts,text=title,foreground=MUTED).grid(row=row,column=0,sticky="w",pady=4)
            value=ttk.Label(facts,text="--",font=("Microsoft YaHei UI",10,"bold"))
            value.grid(row=row,column=1,sticky="w",padx=(14,0));self.info[key]=value
        self.raw_tree=ttk.Treeview(raw,columns=("value","unit"),show="tree headings")
        self.raw_tree.heading("#0",text="字段")
        self.raw_tree.heading("value",text="当前值")
        self.raw_tree.heading("unit",text="单位 / 说明")
        self.raw_tree.pack(fill="both",expand=True)
        self.raw_fields={"altitude_m":("海拔高度","m"),"pressure_pa":("气压","Pa"),"temperature_c":("温度","°C"),
                         "ax":("加速度 X","m/s²"),"ay":("加速度 Y","m/s²"),"az":("加速度 Z","m/s²"),
                         "gx":("角速度 X","°/s"),"gy":("角速度 Y","°/s"),"gz":("角速度 Z","°/s"),
                         "roll":("横滚角","°"),"pitch":("俯仰角","°"),"yaw":("航向角","°")}
        for key,(title,unit) in self.raw_fields.items():
            self.raw_tree.insert("","end",iid=key,text=title,values=("--",unit))
        self.raw_hex=tk.StringVar(value="原始 UART：需要诊断版固件提供")
        ttk.Label(raw,textvariable=self.raw_hex,wraplength=1000).pack(anchor="w",pady=8)

        for row,(key,title,unit) in enumerate((("stale_ms","数据超时","100–30000 ms"),
                                              ("record_seconds","记录时长","1–600 s"),
                                              ("zero_window_ms","归零窗口","500–10000 ms"),
                                              ("zero_span_m","归零高度极差","大于 0，且不超过 10 m"))):
            ttk.Label(config,text=title).grid(row=row,column=0,sticky="w",pady=8)
            var=tk.StringVar(); self.config_vars[key]=var
            ttk.Entry(config,textvariable=var,width=20).grid(row=row,column=1,padx=18)
            ttk.Label(config,text=unit,foreground=MUTED).grid(row=row,column=2,sticky="w")
        buttons=ttk.Frame(config)
        buttons.grid(row=4,column=0,columnspan=3,sticky="w",pady=12)
        for text,fn in (("读取板卡参数",lambda:self.send(["CFG GET"])),("应用并保存",self.apply_config),
                        ("初始化记录区…",self.format_storage)):
            button=ttk.Button(buttons,text=text,command=fn,state="disabled")
            button.pack(side="left",padx=(0,10)); self.controls.append(button)
        ttk.Label(config,text="参数仅在收到板卡确认后读回；修改参数后需要重新归零。\n初始化记录区会删除板上旧记录，与修复传感器接收无关。",foreground=MUTED).grid(row=5,column=0,columnspan=3,sticky="w")
        self.log=tk.Text(logs,bg="#122236",fg="#c9d9eb",font=("Consolas",10),wrap="none",state="disabled")
        scroll=ttk.Scrollbar(logs,command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right",fill="y");self.log.pack(fill="both",expand=True)
        ttk.Button(logs,text="保存日志…",command=self.save_log).pack(anchor="e",pady=(6,0))
        self.footer=ttk.Label(content,text="仅监测与记录 · 连接后不会自动归零、修改配置或格式化",foreground=MUTED)
        self.footer.pack(side="bottom",anchor="w",pady=(8,0),before=notebook)

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            ports=list(list_ports.comports())
            choices=[f"{p.device}  |  {p.description}" for p in ports]
            self.ports.configure(values=choices)
            if not self.port_var.get() or self.port_var.get() not in choices:
                preferred=next((p for p in ports if p.vid==0x303a),None)
                if preferred:
                    self.port_var.set(f"{preferred.device}  |  {preferred.description}")
                elif choices:
                    self.port_var.set(choices[0])
                else:
                    self.port_var.set("")
        except ImportError:
            self.footer.configure(text="缺少 pyserial：请使用启动脚本，或运行 python -m pip install pyserial")

    def connect(self):
        if self.worker:
            return
        port=self.port_var.get().split("|")[0].strip()
        if not port:
            messagebox.showinfo("选择串口","请先连接开发板 USB 数据线，然后刷新串口。",parent=self.root)
            return
        self.state.reset()
        self.plot.samples.clear(); self.plot.redraw()
        self.last_config=None
        self.worker=SerialWorker(port,self.events)
        self.worker.start()
        self.connection_label.configure(text="正在连接 "+port)
        self.connect_button.configure(state="disabled")
        self.disconnect_button.configure(state="normal")
        self.ports.configure(state="disabled")
        self.refresh_button.configure(state="disabled")

    def disconnect(self):
        if self.worker:
            self.worker.stop()
        self.state.reset()

    def send(self,commands):
        if not self.worker or not self.state.connected or self.busy:
            return
        self.busy=True
        self.worker.submit("commands",commands)
        self.footer.configure(text="发送中；等待板卡确认…")
        self.render()

    def apply_config(self):
        limits={"stale_ms":(100,30000),"record_seconds":(1,600),"zero_window_ms":(500,10000),"zero_span_m":(0,10)}
        try:
            values={k:float(v.get()) for k,v in self.config_vars.items()}
            for key,value in values.items():
                lo,hi=limits[key]
                if not lo<=value<=hi or (key=="zero_span_m" and value==0) or (key!="zero_span_m" and value!=int(value)):
                    raise ValueError("参数超出范围："+key)
        except ValueError as exc:
            messagebox.showerror("参数错误",str(exc),parent=self.root);return
        self.last_config=None
        self.send([f"CFG SET {k} {v:g}" for k,v in values.items()]+["CFG SAVE","CFG GET","STATUS"])

    def format_storage(self):
        if messagebox.askyesno("初始化记录区","这会清除板上的旧记录。\n它不会修复传感器接收。确定继续？",parent=self.root):
            self.send(["FS FORMAT CONFIRM","STATUS"])

    def export(self):
        folder=filedialog.askdirectory(title="选择保存目录",parent=self.root)
        if folder and self.worker and not self.busy:
            target=Path(folder)/datetime.now().strftime("telemetry_%Y%m%d_%H%M%S")
            self.busy=True; self.worker.submit("export",str(target)); self.render()
            self.footer.configure(text="正在导出；已暂停状态轮询。")

    def append_log(self,text):
        self.log.configure(state="normal")
        self.log.insert("end",datetime.now().strftime("%H:%M:%S")+"  "+text+"\n")
        if int(self.log.index("end-1c").split(".")[0])>1800:
            self.log.delete("1.0","301.0")
        self.log.see("end");self.log.configure(state="disabled")

    def save_log(self):
        path=filedialog.asksaveasfilename(parent=self.root,defaultextension=".txt",filetypes=[("日志","*.txt")])
        if path:
            Path(path).write_text(self.log.get("1.0","end-1c"),encoding="utf-8")

    def render(self):
        state=self.state
        for key,(value,detail,unit) in self.cards.items():
            reading=state.displayed(key)
            if key=="samples":
                value.configure(text=state.sensor.get("samples","--") if state.responding() else "--")
                detail.configure(text="帧 · 成功解析，不是字节数")
            else:
                value.configure(text="--" if reading is None else f"{reading:.2f}",fg=INK if reading is not None else MUTED)
                subtitle=state.reason(key) if key in ("height","tilt") else ("有效实测" if reading is not None else "旧固件或无有效高度")
                detail.configure(text=unit+" · "+subtitle)
        title,detail=state.diagnosis()
        self.diag_title.configure(text=title); self.diag_detail.configure(text=detail)
        errors=" / ".join(state.sensor.get(k,"--") for k in ("checksum_errors","framing_errors","data_errors"))
        rates={"1":"0.1 Hz","2":"0.5 Hz","3":"1 Hz","4":"2 Hz","5":"5 Hz","6":"10 Hz","7":"20 Hz"}
        facts={"source":state.status.get("source","--"),"config":"已读到" if state.sensor.get("config")=="1" else "未读到",
               "rx_bytes":state.diagnostic.get("rx_bytes","旧固件未提供"),"fields":state.sensor.get("fields","--"),
               "rate":rates.get(state.sensor.get("rate_code"),"--"),"errors":errors,
               "storage":state.status.get("storage","--"),"recording":"记录中" if state.status.get("recording")=="1" else "未记录",
               "version":state.diagnostic.get("firmware","基础版（兼容）")}
        for key,label in self.info.items():
            label.configure(text=facts[key] if state.responding() else "--")
        for key,(_,unit) in self.raw_fields.items():
            value=state.displayed(key)
            self.raw_tree.item(key,values=("--" if value is None else f"{value:.5f}",unit))
        self.raw_hex.set("最近 UART 字节："+(state.rx_hex or "尚未提供"))
        cfg=tuple(sorted(state.config.items()))
        if cfg and cfg!=self.last_config:
            for key,var in self.config_vars.items():
                var.set(state.config.get(key,""))
            self.last_config=cfg
        for button in self.controls:
            button.configure(state="normal" if state.connected and not self.busy else "disabled")
        if state.connected:
            self.connection_label.configure(text=("板卡在线" if state.responding() else "等待响应")+" · "+self.worker.port_name)

    def pump(self):
        if self.closing:
            return
        for _ in range(300):
            try:
                kind,value=self.events.get_nowait()
            except queue.Empty:
                break
            if kind=="connected":
                self.state.reset(connected=True)
                self.append_log("已打开 "+value)
            elif kind=="disconnected":
                self.state.reset();self.worker=None;self.busy=False
                self.connect_button.configure(state="normal");self.disconnect_button.configure(state="disabled")
                self.ports.configure(state="readonly");self.refresh_button.configure(state="normal")
                self.connection_label.configure(text="未连接")
                self.append_log("已断开 "+value)
            elif kind=="line":
                if self.state.ingest(value):
                    self.plot.add(self.state.displayed("height"),self.state.displayed("tilt"))
                self.append_log("← "+value)
            elif kind=="tx":
                self.append_log("→ "+value)
            elif kind=="busy":
                self.busy=value
                if value:
                    self.action_result=None
                else:
                    self.footer.configure(text=self.action_result or "操作结束；以板卡回读状态为准。")
            elif kind in ("error","notice","log"):
                self.append_log(value);self.footer.configure(text=value)
                if kind in ("error","notice"):
                    self.action_result=value
        if not self.state.fresh() and time.monotonic()-self.last_gap>1:
            self.plot.add(None,None);self.last_gap=time.monotonic()
        self.render()
        self.root.after(100,self.pump)

    def close(self):
        self.closing=True
        if self.worker:
            self.worker.stop()
        self.root.destroy()
