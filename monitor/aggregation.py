"""V3 telemetry assembly. Coordinates and estimator diagnostics come from the FC.

SPDX-License-Identifier: GPL-3.0-or-later
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

STALE_SECONDS = 5.0
MAX_FC_GPS_AGE_SECONDS = 5.0
VALID_STATES = frozenset((1, 2))
STATE_NAMES = {
    0: '等待有效 GPS', 1: '有效：持续更新', 2: '有效：持续更新',
    3: '失效：飞控 GPS 数据过期', 4: '暂停：飞控已解锁',
    5: '暂停：检测到多个 GPS', 6: '失效：窗口容量不足',
    7: '失效：数值计算异常',
}
INT_FIELDS = frozenset(('ST_VER', 'ST_LAT', 'ST_LON', 'ST_STATE', 'ST_COUNT',
                        'ST_SEQ', 'ST_GPSMS', 'ST_NWIN', 'ST_SEG', 'ST_IMU',
                        'ST_FCOK', 'ST_FCLAT', 'ST_FCLON', 'ST_MSGMS', 'ST_OVR'))
FLOAT_FIELDS = frozenset(('ST_E', 'ST_N', 'ST_AGE', 'ST_SIG', 'ST_LAM',
                          'ST_HUB', 'ST_WGT', 'ST_NEFF', 'ST_SPAN', 'ST_US',
                          'ST_MAXUS', 'ST_DT'))
REQUIRED_FIELDS = (INT_FIELDS | FLOAT_FIELDS) - {
    'ST_FCLAT', 'ST_FCLON', 'ST_MSGMS', 'ST_MAXUS', 'ST_DT', 'ST_OVR'}
DIAGNOSTIC_COLUMNS = {
    'algorithm_version': 'ST_VER', 'sequence': 'ST_SEQ', 'gps_source_ms': 'ST_GPSMS',
    'window_count': 'ST_NWIN', 'segment': 'ST_SEG', 'imu_state': 'ST_IMU',
    'quality_sigma_m': 'ST_SIG', 'motion_multiplier': 'ST_LAM',
    'huber_factor': 'ST_HUB', 'newest_weight_fraction': 'ST_WGT',
    'effective_weight_count': 'ST_NEFF', 'window_span_s': 'ST_SPAN',
    'compute_us': 'ST_US',
    'gps_message_ms': 'ST_MSGMS', 'max_compute_us': 'ST_MAXUS',
    'capture_interval_s': 'ST_DT',
    'budget_overruns': 'ST_OVR',
}


def newer_u32(value: int, previous: int) -> bool:
    delta = (value - previous) & 0xFFFFFFFF
    return 0 < delta < 0x80000000


def valid_coordinates(lat: Any, lon: Any) -> bool:
    return (isinstance(lat, int) and not isinstance(lat, bool)
            and isinstance(lon, int) and not isinstance(lon, bool)
            and (lat, lon) != (0, 0)
            and -900_000_000 <= lat <= 900_000_000
            and -1_800_000_000 <= lon <= 1_800_000_000)


@dataclass
class Batch:
    timestamp: int
    first_received: float
    fields: dict[str, int | float] = field(default_factory=dict)
    corrupt: bool = False
    completed: bool = False
    result_received: float | None = None
    interval_s: float | None = None


class TelemetryAggregator:
    """Assemble exact-time V3 batches and retain the previous complete batch.

    Incomplete newer batches cannot refresh the previous result. Explicit
    invalid states, conflicting packets and a wrong version clear immediately.
    A flight-controller reboot requires a manual disconnect/reconnect.
    """

    def __init__(self, system_id=1, component_id=1,
                 stale_seconds=STALE_SECONDS,
                 max_fc_gps_age_seconds=MAX_FC_GPS_AGE_SECONDS):
        self.system_id, self.component_id = system_id, component_id
        self.stale_seconds = stale_seconds
        self.max_fc_gps_age_seconds = max_fc_gps_age_seconds
        self.disconnect()

    def disconnect(self):
        self.connected = False
        self.latest_timestamp = None
        self.pending: Batch | None = None
        self.current: Batch | None = None
        self.raw = None
        self.raw_time = self.raw_received = None
        self.last_state = None
        self.last_complete_received = None
        self.last_result_key = None
        self.last_transport_key = None
        self.last_result_received = None
        self.clock_regression = False
        self.reason = '未连接'

    def connect(self):
        self.disconnect()
        self.connected = True
        self.reason = '等待飞控完整 V3 数据批次'

    def _clear(self, reason):
        self.current = None
        self.reason = reason

    def ingest(self, message, received_at, system_id=1, component_id=1):
        if (not self.connected or (system_id, component_id) !=
                (self.system_id, self.component_id) or not math.isfinite(received_at)):
            return False
        kind = message.get('mavpackettype')
        if kind == 'GPS_RAW_INT':
            return self._raw(message, received_at)
        if kind not in ('NAMED_VALUE_INT', 'NAMED_VALUE_FLOAT'):
            return False
        name = message.get('name', '')
        if isinstance(name, bytes):
            name = name.decode('ascii', errors='ignore')
        name = str(name).split('\x00', 1)[0]
        if name not in INT_FIELDS | FLOAT_FIELDS:
            return False
        if (name in INT_FIELDS) != (kind == 'NAMED_VALUE_INT'):
            return False
        timestamp, value = message.get('time_boot_ms'), message.get('value')
        if (not isinstance(timestamp, int) or isinstance(timestamp, bool)
                or not 0 <= timestamp <= 0xFFFFFFFF):
            return False
        if name in INT_FIELDS:
            if (not isinstance(value, int) or isinstance(value, bool)
                    or not -0x80000000 <= value <= 0x7FFFFFFF):
                return False
        elif (not isinstance(value, (float, int)) or isinstance(value, bool)
              or not math.isfinite(value)):
            return False
        if self.latest_timestamp is not None and timestamp != self.latest_timestamp:
            if not newer_u32(timestamp, self.latest_timestamp):
                if self.latest_timestamp - timestamp > 10_000:
                    self.clock_regression = True
                return False
        if timestamp != self.latest_timestamp:
            self.latest_timestamp = timestamp
            self.pending = Batch(timestamp, received_at)
            self.reason = '等待新批次收齐；保留上一完整结果及其原年龄'
        batch = self.pending
        if name in batch.fields and batch.fields[name] != value:
            batch.corrupt = True
            self._clear('同一批次字段冲突；等待新批次')
            return False
        batch.fields[name] = value
        if name == 'ST_STATE':
            self.last_state = value
            if value not in VALID_STATES:
                self._clear(STATE_NAMES.get(value, f'未知状态 {value}'))
        if name == 'ST_VER' and value != 3:
            self._clear(f'固件版本 {value} 与 V3 显示器不匹配')
        if name == 'ST_AGE' and not 0 <= value <= self.max_fc_gps_age_seconds:
            self._clear('飞控报告 GPS 年龄无效或过期')
        if name == 'ST_COUNT' and value <= 0:
            self._clear('当前没有有效 GPS 样本')
        if REQUIRED_FIELDS <= batch.fields.keys() and not batch.corrupt:
            if not self._valid_batch(batch, received_at):
                self._clear('V3 批次中的版本、坐标、状态、数量、年龄或诊断无效')
                return True
            if not batch.completed:
                key = (batch.fields['ST_SEG'] & 0xFFFFFFFF,
                       batch.fields['ST_COUNT'], batch.fields['ST_GPSMS'] & 0xFFFFFFFF)
                transport_key = (key[0], batch.fields['ST_SEQ'] & 0xFFFFFFFF, key[2])
                old = self.last_transport_key
                if (old is not None and transport_key[0] != old[0]
                        and not newer_u32(transport_key[0], old[0])):
                    batch.corrupt = True
                    self._clear('估计段编号回退；请核对重启状态')
                    return False
                if old is not None and transport_key[0] == old[0]:
                    seq_forward = newer_u32(transport_key[1], old[1])
                    same_seq = transport_key[1] == old[1]
                    gps_forward = newer_u32(transport_key[2], old[2])
                    same_gps = transport_key[2] == old[2]
                    if (not (seq_forward or (same_seq and same_gps))
                            or not (gps_forward or same_gps)):
                        batch.corrupt = True
                        self._clear('结果序号或 GPS 源时间回退；请核对重启状态')
                        return False
                    if same_gps and self.last_result_key is not None and key[1] != self.last_result_key[1]:
                        batch.corrupt = True
                        self._clear('同一 GPS 源时间对应的样本数冲突')
                        return False
                self.last_transport_key = transport_key
                if self.last_result_key == key:
                    batch.result_received = self.last_result_received
                else:
                    batch.result_received = received_at
                    self.last_result_key, self.last_result_received = key, received_at
                batch.interval_s = (None if self.last_complete_received is None else
                                    batch.first_received-self.last_complete_received)
                self.last_complete_received = batch.first_received
                batch.completed = True
            self.current = batch
            self.reason = STATE_NAMES[batch.fields['ST_STATE']]
        return True

    def _valid_batch(self, batch, now):
        f = batch.fields
        return (f['ST_VER'] == 3 and f['ST_STATE'] in VALID_STATES
                and f['ST_COUNT'] > 0 and f['ST_NWIN'] > 0
                and valid_coordinates(f['ST_LAT'], f['ST_LON'])
                and 0 <= f['ST_AGE'] <= self.max_fc_gps_age_seconds
                and 0.59 <= f['ST_SIG'] <= 8.01
                and f['ST_LAM'] in (1, 4) and 0 < f['ST_HUB'] <= 1.000001
                and 0 < f['ST_WGT'] <= 1.000001
                and 0.99 <= f['ST_NEFF'] <= f['ST_NWIN'] + 0.01
                and 0 <= f['ST_SPAN'] <= 90.001 and f['ST_US'] >= 0
                and f['ST_IMU'] in (0, 1, 2) and f['ST_FCOK'] in (0, 1)
                and now-batch.first_received <= self.stale_seconds)

    def _raw(self, message, received_at):
        timestamp, fix = message.get('time_usec'), message.get('fix_type', 0)
        if (not isinstance(fix, int) or isinstance(fix, bool) or fix < 3
                or not valid_coordinates(message.get('lat'), message.get('lon'))):
            self.raw = dict(message, fix_type=fix if isinstance(fix, int) else 0)
            # A loss-of-fix packet is immediate evidence of invalid raw GPS.
            # It does not refresh the last valid source epoch or its age.
            return True
        if (not isinstance(timestamp, int) or isinstance(timestamp, bool)
                or timestamp <= 0):
            self.raw = dict(message)
            self.raw_received = None
            return False
        if self.raw_time is not None and timestamp <= self.raw_time:
            if timestamp == self.raw_time and fix < 3:
                self.raw = dict(message)
                return True
            return False
        self.raw_time, self.raw_received = timestamp, received_at
        self.raw = dict(message)
        return True

    def snapshot(self, now):
        raw_age = None if self.raw_received is None else max(0, now-self.raw_received)
        raw_valid = bool(self.connected and self.raw and raw_age is not None
                         and raw_age <= self.stale_seconds and self.raw['fix_type'] >= 3
                         and valid_coordinates(self.raw.get('lat'), self.raw.get('lon')))
        batch = self.current
        fields = batch.fields if batch else {}
        batch_age = None if batch is None else max(0, now-batch.first_received)
        result_age = None if batch is None else max(0, now-batch.result_received)
        gps_age = None if batch is None else fields['ST_AGE'] + batch_age
        valid = bool(self.connected and batch and not batch.corrupt
                     and batch_age <= self.stale_seconds
                     and result_age <= self.stale_seconds
                     and gps_age <= self.max_fc_gps_age_seconds)
        status = self.reason
        if self.connected and batch is not None and not valid:
            status = '结果或 GPS 数据年龄超过 5 秒，坐标已清空'
        if self.connected and self.clock_regression and not valid:
            status += '；时间回退，如飞控已重启请断开后重新连接'
        fc_valid = bool(valid and fields.get('ST_FCOK') == 1 and
                        valid_coordinates(fields.get('ST_FCLAT'), fields.get('ST_FCLON')))
        ans = {
            'connected': self.connected, 'status': status, 'monotonic_s': now,
            'raw_valid': raw_valid, 'raw_age_s': raw_age,
            'raw_lat': self.raw['lat']/1e7 if raw_valid else None,
            'raw_lon': self.raw['lon']/1e7 if raw_valid else None,
            'raw_fix': self.raw.get('fix_type') if self.raw else None,
            'raw_satellites': self.raw.get('satellites_visible') if self.raw else None,
            'raw_h_acc_mm': self.raw.get('h_acc') if self.raw else None,
            'raw_hdop_centi': self.raw.get('eph') if self.raw else None,
            'raw_time_usec': self.raw_time,
            'optimized_valid': valid,
            'optimized_lat': fields['ST_LAT']/1e7 if valid else None,
            'optimized_lon': fields['ST_LON']/1e7 if valid else None,
            'batch_time_boot_ms': batch.timestamp if batch else None,
            'batch_receive_age_s': batch_age,
            'batch_interval_s': batch.interval_s if batch else None,
            'result_receive_age_s': result_age,
            'state': fields.get('ST_STATE', self.last_state),
            'count': fields.get('ST_COUNT') if valid else None,
            'east_m': fields.get('ST_E') if valid else None,
            'north_m': fields.get('ST_N') if valid else None,
            'gps_age_s': gps_age if valid else None,
            'official_valid': fc_valid,
            'official_lat': fields['ST_FCLAT']/1e7 if fc_valid else None,
            'official_lon': fields['ST_FCLON']/1e7 if fc_valid else None,
        }
        for column, field_name in DIAGNOSTIC_COLUMNS.items():
            value = fields.get(field_name) if valid else None
            if field_name in ('ST_GPSMS', 'ST_SEQ', 'ST_SEG', 'ST_MSGMS', 'ST_OVR') and value is not None:
                value &= 0xFFFFFFFF
            ans[column] = value
        return ans
