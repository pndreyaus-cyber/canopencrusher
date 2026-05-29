import csv
import math
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import numpy as np

try:
    import serial
    import serial.tools.list_ports
except Exception:
    serial = None

POSE_RE =re.compile(
  r'RPC OK '
  r'PX(?P<PX>-?(?:\d*\.)?\d+)\s*'
  r'PY(?P<PY>-?(?:\d*\.)?\d+)\s*'
  r'PZ(?P<PZ>-?(?:\d*\.)?\d+)\s*'
  r'OR(?P<OR>-?(?:\d*\.)?\d+)\s*'
  r'OP(?P<OP>-?(?:\d*\.)?\d+)\s*'
  r'OW(?P<OW>-?(?:\d*\.)?\d+)')


class RobotSerial:
    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()

    def available_ports(self):
        if serial is None:
            return []
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port, baudrate=115200, timeout=10.0):
        if serial is None:
            raise RuntimeError('pyserial is not installed. Install with: pip install pyserial')
        self.disconnect()
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, write_timeout=timeout)
        time.sleep(0.2)
        self.reset_buffers()

    def disconnect(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def reset_buffers(self):
        if self.ser is not None:
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception:
                pass

    def send_line(self, line):
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError('Serial port is not connected')
        with self.lock:
            self.ser.write((line.strip() + '\n').encode('ascii', errors='ignore'))
            self.ser.flush()

    def read_line(self):
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError('Serial port is not connected')
        line = self.ser.readline().decode('ascii', errors='ignore').strip()
        if not line:
            raise TimeoutError('Timeout waiting for robot reply')
        return line

    def transact(self, line, expect_prefix=None):
        with self.lock:
            self.ser.write((line.strip() + '\n').encode('ascii', errors='ignore'))
            self.ser.flush()
            reply = self.ser.readline().decode('ascii', errors='ignore').strip()
        if not reply:
            raise TimeoutError(f'No reply for command: {line}')
        if expect_prefix and not reply.startswith(expect_prefix):
            raise RuntimeError(f'Unexpected reply: {reply}')
        return reply

    def read_pose(self):
        reply = self.transact('RPC', expect_prefix='RPC OK')
        return parse_pose_reply(reply), reply

    def move_absolute(self, px, py, pz, speed, accel, roll=180.0, pitch=0.0, yaw=0.0):
        cmd = (
            f'MAC PX{px:.3f} PY{py:.3f} PZ{pz:.3f} '
            f'OR{roll:.3f} OP{pitch:.3f} OW{yaw:.3f} '
            f'SP{speed:.3f} AC{accel:.3f}'
        )
        reply = self.transact(cmd, expect_prefix='MAC OK')
        return reply, cmd
    
    def move_home(self):
        cmd = "MHJ SP10.0 AC1.0"
        reply = self.transact(cmd, expect_prefix="MHJ OK")
        return reply, cmd


def parse_pose_reply(reply):
    matches = POSE_RE.match(reply)
    if matches is None:
        raise RuntimeError("Could not find some variables in the position reuqest answer")
    
    return {
        'x': float(matches.group('PX')), 
        'y': float(matches.group('PY')),
        'z': float(matches.group('PZ')),
        'roll': float(matches.group('OR')), 
        'pitch': float(matches.group('OP')),
        'yaw': float(matches.group('OW'))
    }


def rigid_transform(theoretical_pts, robot_pts):
    A = np.asarray(theoretical_pts, dtype=float)
    B = np.asarray(robot_pts, dtype=float)
    print("A")
    print(A)
    print("B")
    print(B)
    polyfit_transform(theoretical_pts, robot_pts)
    print("Polyfit ended")
    ca = A.mean(axis=0)
    cb = B.mean(axis=0)
    AA = A - ca
    BB = B - cb
    H = AA.T @ BB
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = cb - R @ ca
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    errs = []
    for a, b in zip(A, B):
        pred = R @ a + t
        errs.append(float(np.linalg.norm(pred - b)))
    rms = math.sqrt(sum(e * e for e in errs) / len(errs)) if errs else 0.0
    print("T")
    print(T)
    return T, errs, rms, max(errs) if errs else 0.0

def polyfit_transform(theoretical_pts, robot_pts):
    theoretical_np = np.asarray(theoretical_pts, dtype=float)
    theoretical_4D = np.hstack((theoretical_np , np.ones((3, 1))))
    
    robot_np = np.asarray(robot_pts, dtype=float)
    print("robot_np", robot_np)
    row_one = np.linalg.lstsq(theoretical_4D, robot_np[:, 0], rcond=None)[0]
    row_two = np.linalg.lstsq(theoretical_4D, robot_np[:, 1], rcond=None)[0]
    row_three = np.linalg.lstsq(theoretical_4D, robot_np[:, 2], rcond=None)[0]

    transform_matrix = np.vstack([row_one, row_two, row_three])
    
    x_pred = theoretical_4D @ transform_matrix[0].T
    y_pred = theoretical_4D @ transform_matrix[1].T
    z_pred = theoretical_4D @ transform_matrix[2].T

    x_err = x_pred - robot_np[:,0].T
    y_err = y_pred - robot_np[:,1].T
    z_err = z_pred - robot_np[:,2].T
    
    return transform_matrix, [], 0, np.max([np.max(x_err), np.max(y_err), np.max(z_err)])

class App:
    def __init__(self, root):
        self.root = root
        self.root.title('Калибровка робота')
        self.robot = RobotSerial()
        self.ui_queue = queue.Queue()
        self.measured_points = []
        self.theoretical_points = {}
        self.transform = None
        self.current_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': -90.0, 'yaw': 0.0}
        self.build_ui()
        self.refresh_ports()
        self.root.after(100, self.process_ui_queue)

    def build_ui(self):
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill='both', expand=True)

        top = ttk.Frame(main)
        top.pack(fill='x')
        ttk.Label(top, text='Serial').grid(row=0, column=0, sticky='w')
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=18, state='readonly')
        self.port_combo.grid(row=0, column=1, padx=4)
        ttk.Button(top, text='Обновить порты', command=self.refresh_ports).grid(row=0, column=2, padx=4)
        ttk.Label(top, text='Baud').grid(row=0, column=3, sticky='w')
        self.baud_var = tk.StringVar(value='115200')
        ttk.Entry(top, textvariable=self.baud_var, width=10).grid(row=0, column=4, padx=4)
        ttk.Button(top, text='Подключиться', command=self.connect).grid(row=0, column=5, padx=4)
        ttk.Button(top, text='Отключиться', command=self.disconnect).grid(row=0, column=6, padx=4)
        ttk.Button(top, text='Домой', command=self.move_home).grid(row=0, column=7, padx=4)

        cfg = ttk.LabelFrame(main, text='Настройки движения', padding=8)
        cfg.pack(fill='x', pady=8)
        self.step_var = tk.StringVar(value='1.0')
        self.speed_var = tk.StringVar(value='15.0')
        self.accel_var = tk.StringVar(value='2.0')
        self.roll_var = tk.StringVar(value='0.0')
        self.pitch_var = tk.StringVar(value='-90.0')
        self.yaw_var = tk.StringVar(value='0.0')
        labels = [('Шаг мм', self.step_var), ('Скорость', self.speed_var), ('Ускорение', self.accel_var),]
        for i, (txt, var) in enumerate(labels):
            ttk.Label(cfg, text=txt).grid(row=0, column=i*2, sticky='w')
            ttk.Entry(cfg, textvariable=var, width=10).grid(row=0, column=i*2+1, padx=4)

        body = ttk.Frame(main)
        body.pack(fill='both', expand=True)
        left = ttk.Frame(body)
        left.pack(side='left', fill='y')
        right = ttk.Frame(body)
        right.pack(side='left', fill='both', expand=True, padx=(10, 0))

        jogf = ttk.LabelFrame(left, text='Управление роботом в декартовой С.К.', padding=8)
        jogf.pack(fill='x')
        self.pose_vars = {k: tk.StringVar(value=v) for k, v in [('x', '0.0'), ('y', '0.0'), ('z', '0.0'), ('roll', '0.0'), ('pitch', '-90.0'), ('yaw', '0.0')]}
        ttk.Button(jogf, text='Считать текущую позу (RPC)', command=self.read_pose).grid(row=0, column=0, columnspan=2, sticky='ew', pady=2)
        coords = [('X', 'x'), ('Y', 'y'), ('Z', 'z')]
        for i, (txt, key) in enumerate(coords, start=1):
            ttk.Label(jogf, text=txt).grid(row=i, column=0, sticky='w')
            ttk.Entry(jogf, textvariable=self.pose_vars[key], width=14).grid(row=i, column=1, sticky='ew', pady=2)
        ttk.Button(jogf, text='+X', command=lambda: self.jog_axis('x', +1)).grid(row=1, column=2, padx=2)
        ttk.Button(jogf, text='-X', command=lambda: self.jog_axis('x', -1)).grid(row=1, column=3, padx=2)
        ttk.Button(jogf, text='+Y', command=lambda: self.jog_axis('y', +1)).grid(row=2, column=2, padx=2)
        ttk.Button(jogf, text='-Y', command=lambda: self.jog_axis('y', -1)).grid(row=2, column=3, padx=2)
        ttk.Button(jogf, text='+Z', command=lambda: self.jog_axis('z', +1)).grid(row=3, column=2, padx=2)
        ttk.Button(jogf, text='-Z', command=lambda: self.jog_axis('z', -1)).grid(row=3, column=3, padx=2)

        savef = ttk.LabelFrame(left, text='Точки для калибровки в С.К. робота', padding=8)
        savef.pack(fill='both', expand=True, pady=8)
        self.point_name_var = tk.StringVar(value='P1')
        ttk.Label(savef, text='Имя точки').pack(anchor='w')
        ttk.Entry(savef, textvariable=self.point_name_var).pack(fill='x', pady=2)
        ttk.Button(savef, text='Сохранить текущую позицию', command=self.save_measured_point).pack(fill='x', pady=4)
        ttk.Button(savef, text='Удалить выбранную точку', command=self.delete_selected_measured).pack(fill='x', pady=2)
        ttk.Button(savef, text='Сохранить измеренные точки в CSV', command=self.save_measured_csv).pack(fill='x', pady=2)
        self.measured_list = tk.Listbox(savef, height=10)
        self.measured_list.pack(fill='both', expand=True, pady=4)

        theo = ttk.LabelFrame(right, text='Теоретические точки', padding=8)
        theo.pack(fill='x')
        ttk.Label(theo, text='Вставьте строки в формате: имя,x,y,z').pack(anchor='w')
        self.theoretical_text = tk.Text(theo, height=8, width=70)
        self.theoretical_text.pack(fill='x', pady=4)
        btns = ttk.Frame(theo)
        btns.pack(fill='x')
        ttk.Button(btns, text='Распарсить точки', command=self.load_theoretical_from_text).pack(side='left', padx=2)
        ttk.Button(btns, text='Загрузить точки в С.К. камеры из CSV', command=self.load_theoretical_csv).pack(side='left', padx=2)
        ttk.Button(btns, text='Сохранить точки в С.К. камеры в CSV', command=self.save_theoretical_csv).pack(side='left', padx=2)

        pairs = ttk.LabelFrame(right, text='Сопоставленные пары и расчет матрицы перехода', padding=8)
        pairs.pack(fill='both', expand=True, pady=8)
        cols = ('name', 'theoretical', 'measured')
        self.tree = ttk.Treeview(pairs, columns=cols, show='headings', height=10)
        for col, width in [('name', 80), ('theoretical', 220), ('measured', 220)]:
            self.tree.heading(col, text=col.capitalize())
            self.tree.column(col, width=width, anchor='w')
        self.tree.pack(fill='both', expand=True)
        row = ttk.Frame(pairs)
        row.pack(fill='x', pady=4)
        ttk.Button(row, text='Обновить пары', command=self.refresh_pairs).pack(side='left', padx=2)
        ttk.Button(row, text='Вычислить матрицу перехода', command=self.compute_transform).pack(side='left', padx=2)
        ttk.Button(row, text='Сохранить матрицу перехода в CSV', command=self.save_transform_csv).pack(side='left', padx=2)

        self.metrics_var = tk.StringVar(value='-')
        ttk.Label(pairs, textvariable=self.metrics_var).pack(anchor='w', pady=4)

        logf = ttk.LabelFrame(main, text='Log', padding=8)
        logf.pack(fill='both', expand=True, pady=(8, 0))
        self.log = tk.Text(logf, height=10)
        self.log.pack(fill='both', expand=True)

    def log_msg(self, msg):
        self.log.insert('end', msg + '\n')
        self.log.see('end')

    def refresh_ports(self):
        ports = self.robot.available_ports()
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        self.log_msg(f'Ports: {ports}')

    def connect(self):
        try:
            self.robot.connect(self.port_var.get(), int(self.baud_var.get()))
            print(f'Connected to {self.port_var.get()} at {self.baud_var.get()}')
        except Exception as e:
            messagebox.showerror('Connect error', str(e))

    def disconnect(self):
        self.robot.disconnect()
        self.log_msg('Disconnected')

    def move_home(self):
        self.robot.move_home()
        self.log_msg("At home position")
        

    def process_ui_queue(self):
        try:
            while True:
                fn, args = self.ui_queue.get_nowait()
                fn(*args)
        except queue.Empty:
            pass
        self.root.after(100, self.process_ui_queue)

    def run_worker(self, target):
        threading.Thread(target=target, daemon=True).start()

    def _update_pose_ui(self, pose, raw_reply=None):
        self.current_pose = pose
        for key in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
            self.pose_vars[key].set(f"{pose[key]}")
        if raw_reply:
            self.log_msg(raw_reply)

    def read_pose(self):
        def worker():
            try:
                pose, raw = self.robot.read_pose()
                self.ui_queue.put((self._update_pose_ui, (pose, raw)))
            except Exception as e:
                self.ui_queue.put((self.log_msg, (f'RPC error: {e}',)))
        self.run_worker(worker)

    def jog_axis(self, axis, sign):
        def worker():
            try:
                current, raw = self.robot.read_pose()
                step = float(self.step_var.get()) * sign
                target = dict(current)
                target[axis] += step
                reply, cmd = self.robot.move_absolute(
                    px=target['x'], py=target['y'], pz=target['z'],
                    speed=float(self.speed_var.get()), accel=float(self.accel_var.get()),
                    roll=float(self.roll_var.get()), pitch=float(self.pitch_var.get()), yaw=float(self.yaw_var.get())
                )
                pose2, raw2 = self.robot.read_pose()
                self.ui_queue.put((self.log_msg, (f'SENT: {cmd}',)))
                self.ui_queue.put((self.log_msg, (reply,)))
                self.ui_queue.put((self._update_pose_ui, (pose2, raw2)))
                print("Move finished")
            except Exception as e:
                self.ui_queue.put((self.log_msg, (f'Jog error: {e}',)))
        self.run_worker(worker)

    def save_measured_point(self):
        try:
            name = self.point_name_var.get().strip()
            if not name:
                raise ValueError('Point name is empty')
            point = {
                'name': name,
                'x': float(self.pose_vars['x'].get()),
                'y': float(self.pose_vars['y'].get()),
                'z': float(self.pose_vars['z'].get())
            }
            idx = next((i for i, p in enumerate(self.measured_points) if p['name'] == name), None)
            if idx is None:
                self.measured_points.append(point)
            else:
                self.measured_points[idx] = point
            self.refresh_measured_list()
            self.refresh_pairs()
            self.point_name_var.set(f'P{len(self.measured_points)+1}')
            self.log_msg(f'Saved measured point {name}: {point}')
        except Exception as e:
            messagebox.showerror('Save point error', str(e))

    def refresh_measured_list(self):
        self.measured_list.delete(0, 'end')
        for p in self.measured_points:
            self.measured_list.insert('end', f"{p['name']}: {p['x']:.3f}, {p['y']:.3f}, {p['z']:.3f}")

    def delete_selected_measured(self):
        sel = self.measured_list.curselection()
        if not sel:
            return
        del self.measured_points[sel[0]]
        self.refresh_measured_list()
        self.refresh_pairs()

    def parse_points_text(self, text):
        pts = {}
        for line in text.splitlines():
            line = line.strip()
            if not line:
                continue
            parts = [p.strip() for p in re.split(r'[;,\s]+', line) if p.strip()]
            if len(parts) < 4:
                continue
            if parts[0].lower() == 'name':
                continue
            pts[parts[0]] = {'name': parts[0], 'x': float(parts[1]), 'y': float(parts[2]), 'z': float(parts[3])}
        return pts

    def load_theoretical_from_text(self):
        try:
            self.theoretical_points = self.parse_points_text(self.theoretical_text.get('1.0', 'end'))
            self.refresh_pairs()
            self.log_msg(f'Loaded {len(self.theoretical_points)} theoretical points from text')
        except Exception as e:
            messagebox.showerror('Load theoretical error', str(e))

    def load_theoretical_csv(self):
        path = filedialog.askopenfilename(filetypes=[('CSV files', '*.csv'), ('Text files', '*.txt'), ('All files', '*.*')])
        if not path:
            return
        try:
            with open(path, 'r', encoding='utf-8') as f:
                text = f.read()
            self.theoretical_text.delete('1.0', 'end')
            self.theoretical_text.insert('1.0', text)
            self.load_theoretical_from_text()
        except Exception as e:
            messagebox.showerror('Load CSV error', str(e))

    def save_measured_csv(self):
        if not self.measured_points:
            messagebox.showinfo('Measured points', 'No measured points to save')
            return
        path = filedialog.asksaveasfilename(defaultextension='.csv', filetypes=[('CSV files', '*.csv')], initialfile='measured_points.csv')
        if not path:
            return
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['name', 'x', 'y', 'z'])
            for p in self.measured_points:
                w.writerow([p['name'], p['x'], p['y'], p['z']])
        self.log_msg(f'Saved measured points: {path}')

    def save_theoretical_csv(self):
        if not self.theoretical_points:
            messagebox.showinfo('Theoretical points', 'No theoretical points to save')
            return
        path = filedialog.asksaveasfilename(defaultextension='.csv', filetypes=[('CSV files', '*.csv')], initialfile='theoretical_points.csv')
        if not path:
            return
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['name', 'x', 'y', 'z'])
            for name in sorted(self.theoretical_points):
                p = self.theoretical_points[name]
                w.writerow([p['name'], p['x'], p['y'], p['z']])
        self.log_msg(f'Saved theoretical points: {path}')

    def get_pairs(self):
        meas = {p['name']: p for p in self.measured_points}
        pairs = []
        for name, t in sorted(self.theoretical_points.items()):
            if name in meas:
                pairs.append((name, t, meas[name]))
        return pairs

    def refresh_pairs(self):
        for item in self.tree.get_children():
            self.tree.delete(item)
        for name, t, m in self.get_pairs():
            self.tree.insert('', 'end', values=(
                name,
                f"{t['x']:.3f}, {t['y']:.3f}, {t['z']:.3f}",
                f"{m['x']:.3f}, {m['y']:.3f}, {m['z']:.3f}"
            ))
        self.metrics_var.set(f'Matched pairs: {len(self.get_pairs())}')

    def compute_transform(self):
        try:
            pairs = self.get_pairs()
            if len(pairs) < 3:
                raise ValueError('Need at least 3 matched points')
            theoretical = [[t['x'], t['y'], t['z']] for _, t, _ in pairs]
            measured = [[m['x'], m['y'], m['z']] for _, _, m in pairs]
            T, errs, rms, max_err = polyfit_transform(theoretical, measured)
            self.transform = T
            self.metrics_var.set(f'Matched pairs: {len(pairs)} | RMS error: {rms:.6f} mm | Max error: {max_err:.6f} mm')
            self.log_msg('Computed transform matrix:')
            for row in T:
                self.log_msg('  ' + ', '.join(f'{v:.9f}' for v in row))
            for (name, _, _), err in zip(pairs, errs):
                self.log_msg(f'Point {name} residual = {err:.6f} mm')
        except Exception as e:
            messagebox.showerror('Transform error', str(e))

    def save_transform_csv(self):
        if self.transform is None:
            messagebox.showinfo('Transform', 'Compute transform first')
            return
        path = filedialog.asksaveasfilename(defaultextension='.csv', filetypes=[('CSV files', '*.csv')], initialfile='transform_matrix.csv')
        if not path:
            return
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            for row in self.transform:
                w.writerow([float(v) for v in row])
        self.log_msg(f'Saved transform matrix: {path}')


def main():
    root = tk.Tk()
    try:
        root.geometry('1180x760')
    except Exception:
        pass
    ttk.Style().theme_use('clam')
    App(root)
    root.mainloop()


if __name__ == '__main__':
    main()
