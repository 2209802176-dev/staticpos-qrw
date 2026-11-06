"""V3 read-only viewer. SPDX-License-Identifier: GPL-3.0-or-later."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import re
import sys
import threading
import time
import uuid

sys.dont_write_bytecode = True
from aggregation import FLOAT_FIELDS, INT_FIELDS, DIAGNOSTIC_COLUMNS, TelemetryAggregator

BASE = Path(__file__).resolve().parents[1]
LOG_DIRECTORY = BASE / 'monitor_logs'
CSV_FIELDS = [
    'utc_time', 'monotonic_s', 'connection', 'system_id', 'component_id', 'connected', 'status',
    'raw_valid', 'raw_lat', 'raw_lon', 'raw_fix', 'raw_satellites',
    'raw_time_usec', 'raw_age_s', 'optimized_valid', 'optimized_lat',
    'optimized_lon', 'state', 'count', 'batch_time_boot_ms',
    'batch_receive_age_s', 'batch_interval_s', 'result_receive_age_s',
    'east_m', 'north_m', 'gps_age_s', 'raw_h_acc_mm', 'raw_hdop_centi',
    'official_valid', 'official_lat', 'official_lon',
] + list(DIAGNOSTIC_COLUMNS)
RECORDED_TYPES = frozenset(('GPS_RAW_INT', 'ATTITUDE', 'RAW_IMU', 'SCALED_IMU2',
                            'HEARTBEAT', 'COMMAND_ACK', 'STATUSTEXT', 'SYS_STATUS',
                            'EKF_STATUS_REPORT', 'LOCAL_POSITION_NED',
                            'GLOBAL_POSITION_INT', 'SYSTEM_TIME', 'TIMESYNC'))
# Message IDs and intervals are part of the collection protocol, not parameters.
REQUESTED_STREAMS = (('GPS_RAW_INT', 24, 200000),
                     ('ATTITUDE', 30, 100000), ('RAW_IMU', 27, 100000))


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def recordable_message(message) -> bool:
    kind = message.get('mavpackettype')
    if kind in RECORDED_TYPES:
        return True
    if kind not in ('NAMED_VALUE_INT', 'NAMED_VALUE_FLOAT'):
        return False
    name = message.get('name', '')
    if isinstance(name, bytes):
        name = name.decode('ascii', errors='replace')
    return str(name).split('\x00', 1)[0] in INT_FIELDS | FLOAT_FIELDS


def validate_endpoint(endpoint: str) -> str:
    endpoint = endpoint.strip()
    if re.fullmatch(r'COM[1-9][0-9]*', endpoint, flags=re.IGNORECASE):
        return endpoint.upper()
    match = re.fullmatch(r'(udpin|udpout):([A-Za-z0-9_.-]+):([0-9]+)', endpoint)
    if match and 1 <= int(match.group(3)) <= 65535:
        return endpoint
    raise ValueError('请输入 COM 端口（例如 COM5）或 udpin:127.0.0.1:14551 / udpout:主机:端口。')


def serial_ports() -> list[str]:
    from serial.tools import list_ports
    return [p.device for p in list_ports.comports()]


class Receiver(threading.Thread):
    """Send a GCS heartbeat and bounded telemetry interval requests only."""

    def __init__(self, endpoint, baud, system_id, events, token):
        super().__init__(name='static-position-receiver', daemon=True)
        self.endpoint, self.baud, self.system_id = endpoint, baud, system_id
        self.events, self.token = events, token
        self.stop_event = threading.Event()

    def emit(self, kind, payload=None):
        self.events.put((self.token, kind, payload))

    def run(self):
        connection = None
        try:
            from pymavlink import mavutil
            connection = mavutil.mavlink_connection(
                self.endpoint, baud=self.baud, source_system=255,
                source_component=190, autoreconnect=False, dialect='ardupilotmega',
            )
            self.emit('opened')
            last_heartbeat, requested = -float('inf'), False
            while not self.stop_event.is_set():
                now = time.monotonic()
                if now-last_heartbeat >= 1:
                    connection.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
                    )
                    last_heartbeat = now
                message = connection.recv_match(blocking=True, timeout=0.2)
                if message is None or message.get_type() == 'BAD_DATA':
                    continue
                source = (message.get_srcSystem(), message.get_srcComponent())
                if source != (self.system_id, 1):
                    continue
                arrival, received_utc = time.monotonic(), utc_now()
                data = message.to_dict()
                if recordable_message(data):
                    self.emit('packet', (data, arrival, received_utc, *source))
                if message.get_type() == 'HEARTBEAT' and not requested:
                    for name, message_id, interval_us in REQUESTED_STREAMS:
                        connection.mav.command_long_send(
                            self.system_id, 1,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            message_id, interval_us, 0, 0, 0, 0, 0,
                        )
                        self.emit('request', {
                            'utc_time': utc_now(), 'monotonic_s': time.monotonic(),
                            'command': mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                            'message_name': name, 'message_id': message_id,
                            'interval_us': interval_us,
                        })
                    requested = True
                    self.emit('heartbeat', '已收到飞控心跳；请求 GPS 5 Hz、姿态/原始 IMU 各 10 Hz；实际接收与 ACK 逐条记录')
        except Exception as exc:
            if not self.stop_event.is_set():
                self.emit('error', f'{type(exc).__name__}: {exc}')
        finally:
            if connection is not None:
                connection.close()
            self.emit('closed')


class CsvRecorder:
    def __init__(self, endpoint: str, system_id: int, directory=None):
        directory = LOG_DIRECTORY if directory is None else Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = directory / f'static_monitor_v3_{timestamp}_{uuid.uuid4().hex[:8]}.csv'
        self.messages_path = self.path.with_suffix('.messages.jsonl')
        self.events_path = self.path.with_suffix('.events.jsonl')
        self.endpoint, self.system_id = endpoint, system_id
        self.closed = False
        self.streams = []
        try:
            self.stream = self.path.open('x', newline='', encoding='utf-8-sig')
            self.streams.append(self.stream)
            self.messages = self.messages_path.open('x', encoding='utf-8')
            self.streams.append(self.messages)
            self.events = self.events_path.open('x', encoding='utf-8')
            self.streams.append(self.events)
        except Exception:
            for stream in self.streams:
                stream.close()
            raise
        self.writer = csv.DictWriter(self.stream, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self.write_event('recording_started', schema_version=3,
                         requested_streams=[{'name': n, 'message_id': m, 'interval_us': i}
                                            for n, m, i in REQUESTED_STREAMS])

    def write(self, snapshot):
        row = {k: snapshot.get(k) for k in CSV_FIELDS}
        row.update(utc_time=utc_now(),
                   connection=self.endpoint, system_id=self.system_id, component_id=1)
        self.writer.writerow(row)
        self.stream.flush()

    @staticmethod
    def _json_default(value):
        if isinstance(value, bytes):
            return value.decode('ascii', errors='backslashreplace')
        raise TypeError(f'Unsupported JSON value: {type(value).__name__}')

    def _json_row(self, stream, row):
        stream.write(json.dumps(row, ensure_ascii=False, default=self._json_default)+'\n')
        stream.flush()

    def write_message(self, message, received_at, received_utc, system_id, component_id):
        if not recordable_message(message):
            return
        self._json_row(self.messages, {
            'schema_version': 3, 'utc_time': received_utc, 'monotonic_s': received_at,
            'connection': self.endpoint, 'system_id': system_id,
            'component_id': component_id, 'message': message,
        })

    def write_event(self, event, **details):
        row = {'schema_version': 3, 'utc_time': utc_now(),
               'monotonic_s': time.monotonic(), 'connection': self.endpoint,
               'system_id': self.system_id, 'component_id': 1, 'event': event}
        row.update(details)
        self._json_row(self.events, row)

    def mark(self, text, snapshot):
        text = text.strip()
        if not text:
            return False
        self.write_event('operator_marker', text=text,
                         batch_time_boot_ms=snapshot.get('batch_time_boot_ms'),
                         state=snapshot.get('state'), count=snapshot.get('count'),
                         sequence=snapshot.get('sequence'),
                         gps_source_ms=snapshot.get('gps_source_ms'))
        return True

    def close(self):
        if self.closed:
            return
        try:
            self.write_event('recording_stopped')
        finally:
            for stream in self.streams:
                stream.close()
            self.closed = True


class MonitorApp:
    def __init__(self, root):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.ttk, self.root = tk, ttk, root
        self.events = queue.Queue()
        self.token = 0
        self.receiver = None
        self.recorder = None
        self.closing = False
        self.last_log = -float('inf')
        self.aggregator = TelemetryAggregator()
        root.title('自研定位 V3 · 90秒稳健窗口 · 坐标由飞控计算')
        root.geometry('1200x930')
        root.minsize(1120, 650)
        self.endpoint = tk.StringVar(value='')
        self.baud = tk.StringVar(value='115200')
        self.system_id = tk.StringVar(value='1')
        self.record = tk.BooleanVar(value=True)
        self.marker_text = tk.StringVar(value='')
        self.link_status = tk.StringVar(value='未连接。先关闭 Mission Planner 对同一串口的连接。')
        self.algorithm_status = tk.StringVar(value='未连接')
        self.log_status = tk.StringVar(value='CSV 只在手工连接后创建；保存在 版本3/monitor_logs。')
        outer = ttk.Frame(root)
        outer.pack(fill='both', expand=True)
        canvas = tk.Canvas(outer, highlightthickness=0)
        scrollbar = ttk.Scrollbar(outer, orient='vertical', command=canvas.yview)
        scrollbar.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)
        canvas.configure(yscrollcommand=scrollbar.set)
        wrap = ttk.Frame(canvas, padding=16)
        window = canvas.create_window((0, 0), window=wrap, anchor='nw')
        wrap.bind('<Configure>', lambda event: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>', lambda event: canvas.itemconfigure(window, width=event.width))
        ttk.Label(wrap, text='仅显示飞控计算结果；电脑端不计算或修正定位坐标。', font=('Microsoft YaHei UI', 13, 'bold')).pack(anchor='w')
        ttk.Label(wrap, text='适用于单 M8N、全程固定支架；坐标清空时限为 5 秒，持续更新不代表精度保证。').pack(anchor='w', pady=(4, 12))
        connection = ttk.LabelFrame(wrap, text='连接（不会自动连接串口）', padding=10)
        connection.pack(fill='x')
        ttk.Label(connection, text='COM / UDP').grid(row=0, column=0, sticky='w')
        self.endpoint_box = ttk.Combobox(connection, textvariable=self.endpoint, width=36)
        self.endpoint_box.grid(row=0, column=1, padx=6)
        self.refresh_button = ttk.Button(connection, text='列出串口', command=self.refresh_ports)
        self.refresh_button.grid(row=0, column=2, padx=3)
        ttk.Label(connection, text='波特率').grid(row=0, column=3, padx=(12, 0))
        self.baud_box = ttk.Combobox(connection, textvariable=self.baud, width=9,
                                     values=('57600', '115200', '230400', '460800', '921600'))
        self.baud_box.grid(row=0, column=4, padx=6)
        ttk.Label(connection, text='系统 ID').grid(row=0, column=5)
        self.id_box = ttk.Entry(connection, textvariable=self.system_id, width=4)
        self.id_box.grid(row=0, column=6, padx=6)
        self.connect_button = ttk.Button(connection, text='连接', command=self.connect)
        self.connect_button.grid(row=1, column=0, pady=(10, 0))
        self.disconnect_button = ttk.Button(connection, text='断开', command=self.disconnect, state='disabled')
        self.disconnect_button.grid(row=1, column=1, sticky='w', pady=(10, 0))
        self.record_box = ttk.Checkbutton(connection, text='记录 CSV 和逐消息日志', variable=self.record)
        self.record_box.grid(row=1, column=2, columnspan=3, sticky='w', pady=(10, 0))
        ttk.Label(wrap, textvariable=self.link_status, wraplength=1120).pack(anchor='w', pady=5)
        panels = ttk.Frame(wrap)
        panels.pack(fill='both', expand=True)
        panels.columnconfigure(0, weight=1)
        panels.columnconfigure(1, weight=1)
        panels.columnconfigure(2, weight=1)
        self.raw_labels = self.make_panel(panels, 0, 'M8N 原始 GPS', (
            ('lat', '纬度 / °'), ('lon', '经度 / °'), ('fix', '定位类型 / 卫星数'), ('age', '本机收到新数据后 / s'),
        ))
        self.opt_labels = self.make_panel(panels, 2, '自研定位 V3（飞控计算）', (
            ('lat', '纬度 / °'), ('lon', '经度 / °'), ('count', '本段有效样本数'),
            ('batch', '批次 time_boot_ms'), ('east', '相对本段首帧东向 / m'),
            ('north', '相对本段首帧北向 / m'), ('age', 'GPS 数据年龄 / s'),
        ))
        self.official_labels = self.make_panel(panels, 1, '原导航接口输出', (
            ('lat', '纬度 / °'), ('lon', '经度 / °'),
        ))
        diagnostics = ttk.LabelFrame(wrap, text='当前完整批次诊断', padding=10)
        diagnostics.pack(fill='x', pady=(8, 0))
        self.diagnostic_labels = {}
        diagnostic_fields = (
            ('quality_sigma_m', '质量尺度 / m'), ('motion_multiplier', '转向倍率 λ'),
            ('huber_factor', '最新点 Huber 因子'), ('newest_weight_fraction', '最新点权重占比'),
            ('imu_state', 'IMU 转向状态'), ('window_count', '窗口点数'),
            ('window_span_s', '窗口跨度 / s'), ('effective_weight_count', '权重支持数'),
            ('sequence', '遥测快照序号'), ('gps_source_ms', 'GPS 源时间 / ms'),
            ('compute_us', '最近计算时耗 / μs'), ('batch_interval_s', '实收批次间隔 / s'),
            ('max_compute_us', '最长计算时耗 / μs'), ('budget_overruns', '超过500μs预约次数'),
        )
        for index, (key, label) in enumerate(diagnostic_fields):
            row, col = divmod(index, 4)
            diagnostics.columnconfigure(col, weight=1)
            block = ttk.Frame(diagnostics, padding=5)
            block.grid(row=row, column=col, sticky='ew')
            ttk.Label(block, text=label).pack(anchor='w')
            value = tk.Label(block, text='—', font=('Consolas', 12), fg='#777777')
            value.pack(anchor='w')
            self.diagnostic_labels[key] = value
        ttk.Label(wrap, textvariable=self.algorithm_status, font=('Microsoft YaHei UI', 11, 'bold'), wraplength=1120).pack(anchor='w', pady=10)
        ttk.Label(wrap, text='灰色“—”表示未收到完整有效批次或数据已过期。E/N 是相对首帧的位置偏移，不是 RTK 真值误差。', wraplength=1120).pack(anchor='w')
        ttk.Label(wrap, textvariable=self.log_status, wraplength=1120).pack(anchor='w', pady=(8, 0))
        markers = ttk.LabelFrame(wrap, text='操作标记（按下时刻记入独立事件日志）', padding=8)
        markers.pack(fill='x', pady=(8, 0))
        self.marker_entry = ttk.Entry(markers, textvariable=self.marker_text, state='disabled')
        self.marker_entry.pack(side='left', fill='x', expand=True)
        self.marker_entry.bind('<Return>', lambda _event: self.add_marker())
        self.marker_button = ttk.Button(markers, text='记录标记', command=self.add_marker, state='disabled')
        self.marker_button.pack(side='left', padx=(8, 0))
        ttk.Label(wrap, text='示例：第一次转向开始 / 转向结束。IMU转向状态由飞控给出；原导航接口有效不等于健康EKF认证。').pack(anchor='w')
        root.protocol('WM_DELETE_WINDOW', self.close)
        self._after = root.after(100, self.poll)

    def make_panel(self, parent, column, title, fields):
        frame = self.ttk.LabelFrame(parent, text=title, padding=12)
        frame.grid(row=0, column=column, padx=4, sticky='nsew')
        result = {}
        for index, (key, label) in enumerate(fields):
            self.ttk.Label(frame, text=label).grid(row=index, column=0, sticky='w', pady=5)
            value = self.tk.Label(frame, text='—', font=('Consolas', 11), fg='#777777')
            value.grid(row=index, column=1, padx=8, sticky='e')
            result[key] = value
        return result

    def refresh_ports(self):
        try:
            ports = serial_ports()
            self.endpoint_box['values'] = ports
            self.link_status.set('可选串口：'+('、'.join(ports) if ports else '未发现串口')+'；请手工选择，再点击连接。')
        except Exception as exc:
            self.link_status.set(f'无法列出串口：{exc}')

    def set_controls(self, active):
        for widget in (self.endpoint_box, self.baud_box, self.id_box, self.refresh_button, self.record_box):
            widget.configure(state='disabled' if active else 'normal')
        self.connect_button.configure(state='disabled' if active else 'normal')
        self.disconnect_button.configure(state='normal' if active else 'disabled')
        marker_state = 'normal' if active and self.recorder is not None else 'disabled'
        self.marker_entry.configure(state=marker_state)
        self.marker_button.configure(state=marker_state)

    def add_marker(self):
        if self._record('mark', self.marker_text.get(), self.aggregator.snapshot(time.monotonic())):
            self.log_status.set('操作标记已记录：'+self.marker_text.get().strip())
            self.marker_text.set('')

    def _record(self, method, *args, **kwargs):
        if self.recorder is None:
            return None
        try:
            return getattr(self.recorder, method)(*args, **kwargs)
        except Exception as exc:
            self.log_status.set(f'日志写入失败，显示继续：{exc}')
            try:
                self.recorder.close()
            except Exception:
                pass  # close() still closes all handles if its final write fails.
            self.recorder = None
            self.marker_entry.configure(state='disabled')
            self.marker_button.configure(state='disabled')
            return None

    def connect(self):
        from tkinter import messagebox
        if self.receiver is not None and self.receiver.is_alive():
            return
        try:
            endpoint = validate_endpoint(self.endpoint.get())
            baud, system_id = int(self.baud.get()), int(self.system_id.get())
            if not 1 <= system_id <= 254 or not 1200 <= baud <= 4_000_000:
                raise ValueError('系统 ID 必须为 1–254；波特率必须为 1200–4000000。')
            recorder = CsvRecorder(endpoint, system_id) if self.record.get() else None
        except Exception as exc:
            messagebox.showerror('无法连接', str(exc))
            return
        self.token += 1
        self.aggregator = TelemetryAggregator(system_id=system_id)
        self.recorder = recorder
        self.last_log = -float('inf')
        self.receiver = Receiver(endpoint, baud, system_id, self.events, self.token)
        self.set_controls(True)
        self.link_status.set('正在打开连接，等待指定飞控心跳…')
        self.log_status.set(f'日志：{recorder.path.name} + 同名 messages/events JSONL' if recorder else '本次不记录日志和操作标记。')
        self.receiver.start()

    def disconnect(self):
        self._record('write_event', 'disconnect_requested')
        if self.receiver is not None:
            self.receiver.stop_event.set()
        self.aggregator.disconnect()
        self.render(self.aggregator.snapshot(time.monotonic()))
        self.link_status.set('正在关闭连接…')
        self.disconnect_button.configure(state='disabled')
        self.marker_entry.configure(state='disabled')
        self.marker_button.configure(state='disabled')
        self.root.after(100, self._finish_disconnect)

    def _finish_disconnect(self):
        if self.receiver is not None and self.receiver.is_alive():
            self.root.after(100, self._finish_disconnect)
            return
        self._drain_events(final=True)
        self._close_log()
        self.token += 1  # Reject queued packets from the prior connection epoch.
        self.receiver = None
        self.aggregator.disconnect()
        self.set_controls(False)
        self.link_status.set('已断开；原始与优化坐标均已清空。')

    def _close_log(self):
        if self.recorder is not None:
            try:
                self._record('write', self.aggregator.snapshot(time.monotonic()))
            finally:
                self._record('close')
                self.recorder = None

    def _drain_events(self, final=False):
        # The receiver stops before final draining; this also preserves packets
        # already queued when the user clicks disconnect or closes the window.
        for _ in range(self.events.qsize() if final else 2000):
            try:
                token, kind, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if token != self.token:
                continue
            if kind == 'opened' and self.receiver and not self.receiver.stop_event.is_set():
                self.aggregator.connect()
                self.link_status.set('端口已打开；等待系统 ID 对应的飞控数据…')
            elif kind == 'packet':
                message, arrival, received_utc, system_id, component_id = payload
                self._record('write_message', message, arrival, received_utc, system_id, component_id)
                self.aggregator.ingest(message, arrival, system_id, component_id)
            elif kind == 'request':
                self._record('write_event', 'telemetry_interval_requested', **payload)
            elif kind == 'heartbeat':
                self.link_status.set(payload)
            elif kind == 'error':
                self._record('write_event', 'receiver_error', detail=payload)
                self.link_status.set('连接失败或中断：'+payload+'。请确认 Mission Planner 已释放该串口。')
            elif kind == 'closed':
                self._record('write_event', 'connection_closed')
                self.aggregator.disconnect()
                self._close_log()
                self.set_controls(False)

    def poll(self):
        self._drain_events()
        now = time.monotonic()
        snapshot = self.aggregator.snapshot(now)
        self.render(snapshot)
        if self.recorder is not None and now-self.last_log >= 1:
            self._record('write', snapshot)
            self.last_log = now
        self._after = self.root.after(100, self.poll)

    @staticmethod
    def number(value, decimals=3):
        return '—' if value is None else f'{value:.{decimals}f}'

    def render(self, value):
        raw = {
            'lat': self.number(value['raw_lat'], 7), 'lon': self.number(value['raw_lon'], 7),
            'fix': f'{value["raw_fix"]} / {value["raw_satellites"]}' if value['raw_valid'] else '—',
            'age': self.number(value['raw_age_s'], 1),
        }
        opt = {
            'lat': self.number(value['optimized_lat'], 7), 'lon': self.number(value['optimized_lon'], 7),
            'count': str(value['count']) if value['count'] is not None else '—',
            'batch': str(value['batch_time_boot_ms']) if value['optimized_valid'] else '—',
            'east': self.number(value['east_m']), 'north': self.number(value['north_m']),
            'age': self.number(value['gps_age_s'], 1),
        }
        for labels, data, valid in ((self.raw_labels, raw, value['raw_valid']), (self.opt_labels, opt, value['optimized_valid'])):
            for key, widget in labels.items():
                widget.configure(text=data[key], fg='#17653A' if valid else '#777777')
        official = {'lat': self.number(value['official_lat'], 7),
                    'lon': self.number(value['official_lon'], 7)}
        for key, widget in self.official_labels.items():
            widget.configure(text=official[key], fg='#527BA6' if value['official_valid'] else '#777777')
        integer_fields = {'window_count', 'sequence', 'gps_source_ms', 'budget_overruns'}
        imu_names = {0: '无效/回退', 1: '平稳', 2: '转向门控'}
        for key, widget in self.diagnostic_labels.items():
            item = value.get(key)
            text = ('—' if item is None else
                    imu_names.get(item, str(item)) if key == 'imu_state' else
                    str(item) if key in integer_fields else self.number(item))
            widget.configure(text=text, fg='#17653A' if value['optimized_valid'] else '#777777')
        self.algorithm_status.set(value['status'])

    def close(self):
        if self.closing:
            return
        self.closing = True
        self._record('write_event', 'window_close_requested')
        if self.receiver is not None:
            self.receiver.stop_event.set()
        self.aggregator.disconnect()
        self.root.after_cancel(self._after)
        self._finish_close()

    def _finish_close(self):
        if self.receiver is not None and self.receiver.is_alive():
            self.root.after(100, self._finish_close)
            return
        self._drain_events(final=True)
        self._close_log()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description='仅显示飞控原生静止定位输出；不自动连接。')
    parser.add_argument('--self-test', action='store_true', help='离线运行聚合/状态测试，不打开串口')
    parser.add_argument('--list-ports', action='store_true', help='列出串口后退出，不打开串口')
    parser.add_argument('--smoke-gui', '--smoke-test', dest='smoke_gui', action='store_true', help='建立隐藏 GUI，处理定时刷新后退出，不连接设备')
    args = parser.parse_args()
    if args.self_test:
        import unittest
        suite = unittest.defaultTestLoader.discover(str(BASE/'tests'), pattern='test_monitor.py')
        return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1
    if args.list_ports:
        print('\n'.join(serial_ports()) or '未发现串口')
        return 0
    import tkinter as tk
    root = tk.Tk()
    if args.smoke_gui:
        root.withdraw()
    app = MonitorApp(root)
    if args.smoke_gui:
        root.after(350, app.close)
        root.mainloop()
        print('GUI construction PASS; no connection opened.')
    else:
        root.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
