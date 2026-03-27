import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import serial
import serial.tools.list_ports
import threading
import queue
import time
from datetime import datetime

BAUD = 115200  # Arduino USB serial baud

class VentGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Vent Control GUI (Arduino + Jrk)")

        self.ser = None
        self.rx_queue = queue.Queue()
        self.stop_thread = False

        # log buffering for CSV
        self.capture_enabled = False
        self.log_lines = []  # list[str], each is one line (CSV rows + comments)

        self.build_ui()
        self.update_ports()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.set_connected(False)

    # ---------- UI ----------
    def build_ui(self):
        frame_conn = ttk.LabelFrame(self.root, text="Connection")
        frame_conn.pack(fill="x", padx=10, pady=5)

        self.port_combo = ttk.Combobox(frame_conn, width=18)
        self.port_combo.pack(side="left", padx=5)

        ttk.Button(frame_conn, text="Refresh", command=self.update_ports).pack(side="left")
        ttk.Button(frame_conn, text="Connect", command=self.connect).pack(side="left", padx=5)
        ttk.Button(frame_conn, text="Disconnect", command=self.disconnect).pack(side="left")

        # ---- Controls (entries only) ----
        frame_ctrl = ttk.LabelFrame(self.root, text="Commands")
        frame_ctrl.pack(fill="x", padx=10, pady=5)

        # BPM
        ttk.Label(frame_ctrl, text="BPM (1–150):").grid(row=0, column=0, sticky="e", padx=5, pady=3)
        self.ent_bpm = ttk.Entry(frame_ctrl, width=10)
        self.ent_bpm.insert(0, "50")
        self.ent_bpm.grid(row=0, column=1, sticky="w", padx=5, pady=3)
        self.ent_bpm.bind("<Return>", lambda e: self.send_bpm())

        self.btn_bpm = ttk.Button(frame_ctrl, text="Send", command=self.send_bpm)
        self.btn_bpm.grid(row=0, column=2, padx=5, pady=3)

        # AMPL
        ttk.Label(frame_ctrl, text="AMPL (0–1500):").grid(row=1, column=0, sticky="e", padx=5, pady=3)
        self.ent_ampl = ttk.Entry(frame_ctrl, width=10)
        self.ent_ampl.insert(0, "250")
        self.ent_ampl.grid(row=1, column=1, sticky="w", padx=5, pady=3)
        self.ent_ampl.bind("<Return>", lambda e: self.send_ampl())

        self.btn_ampl = ttk.Button(frame_ctrl, text="Send", command=self.send_ampl)
        self.btn_ampl.grid(row=1, column=2, padx=5, pady=3)

        # MAXDUTY fraction
        ttk.Label(frame_ctrl, text="MaxDuty frac (0.05–1.0):").grid(row=2, column=0, sticky="e", padx=5, pady=3)
        self.ent_maxd = ttk.Entry(frame_ctrl, width=10)
        self.ent_maxd.insert(0, "0.35")
        self.ent_maxd.grid(row=2, column=1, sticky="w", padx=5, pady=3)
        self.ent_maxd.bind("<Return>", lambda e: self.send_maxduty())

        self.btn_maxd = ttk.Button(frame_ctrl, text="Send", command=self.send_maxduty)
        self.btn_maxd.grid(row=2, column=2, padx=5, pady=3)

        # Buttons row
        frame_btn = ttk.Frame(self.root)
        frame_btn.pack(fill="x", padx=10, pady=5)

        self.btn_get = ttk.Button(frame_btn, text="GET State", command=lambda: self.send_cmd("GET"))
        self.btn_get.pack(side="left")

        self.btn_start = ttk.Button(frame_btn, text="START", command=lambda: self.send_cmd("START"))
        self.btn_start.pack(side="left", padx=8)

        self.btn_stop = ttk.Button(frame_btn, text="STOP (E-Stop)", command=lambda: self.send_cmd("STOP"))
        self.btn_stop.pack(side="left", padx=8)

        self.btn_clear = ttk.Button(frame_btn, text="Clear Log", command=self.clear_log)
        self.btn_clear.pack(side="left", padx=8)
        
        # NEW: Capture toggle
        self.btn_capture = ttk.Button(frame_btn, text="Start Capture", command=self.toggle_capture)
        self.btn_capture.pack(side="left", padx=8)

        self.btn_save = ttk.Button(frame_btn, text="Save Log to CSV…", command=self.save_log_to_csv)
        self.btn_save.pack(side="left", padx=8)

        # ---- Log window ----
        self.log = tk.Text(self.root, height=16)
        self.log.pack(fill="both", expand=True, padx=10, pady=5)

        # capturing off by default
        self.capture_enabled = False

    def set_connected(self, connected: bool):
        state = "normal" if connected else "disabled"
        for w in [self.btn_bpm, self.btn_ampl, self.btn_maxd,
                  self.btn_get, self.btn_start, self.btn_stop,
                  self.btn_clear, self.btn_capture, self.btn_save,
                  self.ent_bpm, self.ent_ampl, self.ent_maxd]:
            w.config(state=state)

    # ---------- Serial ----------
    def update_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_combo.current(0)

    def connect(self):
        port = self.port_combo.get()
        if not port:
            self.write_log("# No port selected.")
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.2)
            self.write_log(f"# Connected to {port} @ {BAUD}")
            self.stop_thread = False
            threading.Thread(target=self.reader_thread, daemon=True).start()
            self.set_connected(True)
            time.sleep(0.15)
            self.send_cmd("GET")
        except Exception as e:
            self.write_log(f"# Connect failed: {e}")
            self.ser = None
            self.set_connected(False)

    def disconnect(self):
        # stop before disconnect
        try:
            if self.ser:
                self.ser.write(b"STOP\n")
                time.sleep(0.05)
        except Exception:
            pass

        self.stop_thread = True
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        self.set_connected(False)
        self.write_log("# Disconnected")

    def reader_thread(self):
        while not self.stop_thread and self.ser:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors="replace").strip()
                    if line:
                        self.rx_queue.put(line)
                        self.root.after(0, self.process_rx)
                else:
                    time.sleep(0.02)
            except Exception:
                break

    def process_rx(self):
        while not self.rx_queue.empty():
            line = self.rx_queue.get()
            self.write_log(line)

    # ---------- Commands ----------
    def send_cmd(self, cmd: str):
        if not self.ser:
            self.write_log("# Not connected.")
            return
        try:
            self.ser.write((cmd + "\n").encode())
            self.write_log(f"> {cmd}")
        except Exception as e:
            self.write_log(f"# Send error: {e}")

    def send_bpm(self):
        try:
            bpm = float(self.ent_bpm.get())
        except ValueError:
            self.write_log("# Invalid BPM")
            return
        bpm = max(1.0, min(150.0, bpm))
        self.ent_bpm.delete(0, tk.END)
        self.ent_bpm.insert(0, f"{bpm:.2f}" if bpm % 1 else f"{int(bpm)}")
        self.send_cmd(f"BPM {bpm}")

    def send_ampl(self):
        try:
            ampl = int(float(self.ent_ampl.get()))
        except ValueError:
            self.write_log("# Invalid AMPL")
            return
        ampl = max(0, min(1500, ampl))
        self.ent_ampl.delete(0, tk.END)
        self.ent_ampl.insert(0, str(ampl))
        self.send_cmd(f"AMPL {ampl}")

    def send_maxduty(self):
        try:
            md = float(self.ent_maxd.get())
        except ValueError:
            self.write_log("# Invalid MaxDuty")
            return
        md = max(0.05, min(1.0, md))
        self.ent_maxd.delete(0, tk.END)
        self.ent_maxd.insert(0, f"{md:.3f}")
        self.send_cmd(f"MAXDUTY {md:.3f}")

    # ---------- Logging ----------
    def clear_log(self):
        self.log.delete("1.0", tk.END)
        self.log_lines.clear()
        self.write_log("# Log cleared")

    def toggle_capture(self):
        self.capture_enabled = not self.capture_enabled

        # Temporarily bypass capture while printing the marker
        was = self.capture_enabled
        self.capture_enabled = False
        self.log.insert("end", ("# CAPTURE START\n" if was else "# CAPTURE STOP\n"))
        self.log.see("end")
        self.capture_enabled = was

        self.btn_capture.config(text="Stop Capture" if was else "Start Capture")

    def write_log(self, msg: str):
        # display
        self.log.insert("end", msg + "\n")
        self.log.see("end")

        # capture
        if self.capture_enabled:
            self.log_lines.append(msg)

    def save_log_to_csv(self):
        if not self.log_lines:
            messagebox.showinfo("Save Log", "No log data to save yet.")
            return

        default_name = f"vent_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not path:
            return

        try:
            with open(path, "w", newline="") as f:
                for line in self.log_lines:
                    f.write(line + "\n")
            messagebox.showinfo("Save Log", f"Saved {len(self.log_lines)} lines to:\n{path}")
        except Exception as e:
            messagebox.showerror("Save Log", f"Failed to save:\n{e}")

    def on_close(self):
        # STOP on exit
        try:
            if self.ser:
                self.ser.write(b"STOP\n")
                time.sleep(0.05)
        except Exception:
            pass
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = VentGUI(root)
    root.mainloop()