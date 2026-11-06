"""Offline protocol/recording tests. No serial port or UDP socket is opened.

All coordinates and timestamps are generated synthetic examples.

SPDX-License-Identifier: GPL-3.0-or-later
"""
from pathlib import Path
import csv
import json
import queue
import random
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'monitor'))
from aggregation import (INT_FIELDS, FLOAT_FIELDS, REQUIRED_FIELDS,
                         TelemetryAggregator, newer_u32)
from static_monitor import (CsvRecorder, MonitorApp, Receiver, REQUESTED_STREAMS,
                            recordable_message, validate_endpoint)


def named(name, value, timestamp=1000):
    return {'mavpackettype': 'NAMED_VALUE_INT' if name in INT_FIELDS else 'NAMED_VALUE_FLOAT',
            'name': name, 'value': value, 'time_boot_ms': timestamp}


def gps(timestamp=1_000_000, fix=3, lat=100000000, lon=200000000):
    return {'mavpackettype': 'GPS_RAW_INT', 'time_usec': timestamp,
            'fix_type': fix, 'lat': lat, 'lon': lon,
            'satellites_visible': 12, 'h_acc': 1200, 'eph': 115}


def fields(**overrides):
    data = dict(ST_VER=3, ST_LAT=100000001, ST_LON=200000002,
                ST_STATE=1, ST_COUNT=20, ST_SEQ=20, ST_GPSMS=980,
                ST_NWIN=20, ST_SEG=1, ST_IMU=1, ST_FCOK=1,
                ST_FCLAT=100000010, ST_FCLON=200000010,
                ST_E=1.25, ST_N=-2.5, ST_AGE=.02, ST_SIG=1.2,
                ST_LAM=1.0, ST_HUB=.75, ST_WGT=.045, ST_NEFF=19.1,
                ST_SPAN=3.8, ST_US=45.)
    data.update(overrides)
    return data


class AggregationTests(unittest.TestCase):
    def setUp(self):
        self.a = TelemetryAggregator()
        self.a.connect()

    def complete(self, timestamp=1000, received=10., values=None):
        for name, value in (fields() if values is None else values).items():
            self.a.ingest(named(name, value, timestamp), received)

    def test_all_required_fields_and_v3_needed(self):
        data = fields()
        final_name = 'ST_US'
        for name, value in data.items():
            if name != final_name:
                self.a.ingest(named(name, value), 10)
        self.assertFalse(self.a.snapshot(10)['optimized_valid'])
        self.a.ingest(named(final_name, data[final_name]), 10)
        snap = self.a.snapshot(10)
        self.assertTrue(snap['optimized_valid'])
        self.assertAlmostEqual(snap['optimized_lat'], 10.0000001)
        self.assertEqual(snap['quality_sigma_m'], 1.2)

    def test_random_order_fields_are_same_batch(self):
        data = list(fields().items())
        random.Random(9).shuffle(data)
        for name, value in data:
            self.a.ingest(named(name, value), 10)
        self.assertTrue(self.a.snapshot(10)['optimized_valid'])

    def test_incomplete_new_batch_retains_old_without_mixing(self):
        self.complete()
        self.a.ingest(named('ST_VER', 3, 2000), 11)
        self.a.ingest(named('ST_SIG', 7.0, 2000), 11)
        snap = self.a.snapshot(11)
        self.assertTrue(snap['optimized_valid'])
        self.assertEqual(snap['batch_time_boot_ms'], 1000)
        self.assertEqual(snap['quality_sigma_m'], 1.2)
        self.assertAlmostEqual(snap['batch_receive_age_s'], 1.0)

    def test_late_old_fields_do_not_complete_new_batch(self):
        data = fields()
        for name, value in data.items():
            if name != 'ST_US':
                self.a.ingest(named(name, value, 2000), 11)
        self.a.ingest(named('ST_US', 42., 1000), 12)
        self.assertFalse(self.a.snapshot(12)['optimized_valid'])

    def test_invalid_state_immediately_clears_cached_result(self):
        for state in (0, 3, 4, 5, 6, 7, 99):
            self.a.connect()
            self.complete()
            self.a.ingest(named('ST_STATE', state, 2000), 10.1)
            self.assertFalse(self.a.snapshot(10.1)['optimized_valid'])

    def test_wrong_version_immediately_clears(self):
        self.complete()
        self.a.ingest(named('ST_VER', 2, 2000), 11)
        self.assertFalse(self.a.snapshot(11)['optimized_valid'])
        self.complete(timestamp=2000, received=11, values=fields(ST_VER=2))
        self.assertFalse(self.a.snapshot(11)['optimized_valid'])

    def test_both_valid_states_same_label(self):
        statuses = []
        for state in (1, 2):
            self.a.connect()
            self.complete(values=fields(ST_STATE=state))
            statuses.append(self.a.snapshot(10)['status'])
        self.assertEqual(statuses, ['有效：持续更新', '有效：持续更新'])

    def test_stale_result_clears_diagnostics(self):
        self.complete()
        self.assertTrue(self.a.snapshot(14.98)['optimized_valid'])
        snap = self.a.snapshot(15.01)
        self.assertFalse(snap['optimized_valid'])
        for name in ('optimized_lat', 'quality_sigma_m', 'huber_factor', 'official_lat'):
            self.assertIsNone(snap[name])

    def test_duplicate_packets_do_not_refresh(self):
        self.complete()
        self.a.ingest(named('ST_US', 45.), 14.8)
        self.assertFalse(self.a.snapshot(15.1)['optimized_valid'])

    def test_same_sequence_new_batch_does_not_refresh(self):
        self.complete(values=fields(ST_AGE=0.))
        self.complete(timestamp=5000, received=14.8, values=fields(ST_AGE=0.))
        self.assertFalse(self.a.snapshot(15.1)['optimized_valid'])

    def test_new_snapshot_sequence_same_gps_does_not_refresh_result(self):
        self.complete(values=fields(ST_AGE=0.))
        self.complete(timestamp=5000, received=14.8,
                      values=fields(ST_SEQ=21, ST_AGE=0.))
        self.assertTrue(self.a.snapshot(14.9)['optimized_valid'])
        self.assertFalse(self.a.snapshot(15.1)['optimized_valid'])

    def test_source_or_sequence_regression_rejected(self):
        for update in (dict(ST_SEQ=19, ST_GPSMS=1980), dict(ST_SEQ=21, ST_GPSMS=979),
                       dict(ST_SEQ=20, ST_GPSMS=1980)):
            self.a.connect()
            self.complete()
            self.complete(timestamp=2000, received=11, values=fields(**update))
            self.assertFalse(self.a.snapshot(11)['optimized_valid'])

    def test_segment_regression_and_same_source_count_conflict_rejected(self):
        for old, new in ((fields(ST_SEG=2), fields(ST_SEG=1, ST_SEQ=21, ST_GPSMS=1980)),
                         (fields(), fields(ST_SEQ=21, ST_COUNT=21))):
            self.a.connect()
            self.complete(values=old)
            self.complete(timestamp=2000, received=11, values=new)
            self.assertFalse(self.a.snapshot(11)['optimized_valid'])

    def test_optional_diagnostics_never_inherit_older_batch(self):
        self.complete(values=fields(ST_MSGMS=-10, ST_MAXUS=90., ST_DT=.2, ST_OVR=2))
        snap = self.a.snapshot(10)
        self.assertEqual(snap['gps_message_ms'], 0xFFFFFFF6)
        self.assertEqual(snap['max_compute_us'], 90.)
        self.assertEqual(snap['budget_overruns'], 2)
        self.complete(timestamp=2000, received=11, values=fields(ST_SEQ=21, ST_GPSMS=1980))
        snap = self.a.snapshot(11)
        self.assertIsNone(snap['gps_message_ms'])
        self.assertIsNone(snap['max_compute_us'])
        self.assertIsNone(snap['budget_overruns'])

    def test_batch_interval_measured_from_first_receipt(self):
        self.complete()
        self.complete(timestamp=5015, received=14.015,
                      values=fields(ST_SEQ=40, ST_GPSMS=4995))
        snap = self.a.snapshot(14.015)
        self.assertAlmostEqual(snap['batch_interval_s'], 4.015)
        self.assertTrue(self.a.snapshot(18.99)['optimized_valid'])

    def test_slow_assembly_is_not_fresh(self):
        self.a.ingest(named('ST_VER', 3), 10)
        self.complete(received=16)
        self.assertFalse(self.a.snapshot(16)['optimized_valid'])

    def test_conflicting_same_batch_rejected(self):
        self.complete()
        self.a.ingest(named('ST_LAT', 100000002), 10.1)
        self.assertFalse(self.a.snapshot(10.1)['optimized_valid'])
        self.a.ingest(named('ST_US', 45.), 10.2)
        self.assertFalse(self.a.snapshot(10.2)['optimized_valid'])

    def test_conflicting_pending_batch_also_clears_old(self):
        self.complete()
        self.a.ingest(named('ST_SIG', 1., 2000), 11)
        self.a.ingest(named('ST_SIG', 2., 2000), 11.1)
        self.assertFalse(self.a.snapshot(11.1)['optimized_valid'])

    def test_wrong_source_and_type_rejected(self):
        for name, value in fields().items():
            self.a.ingest(named(name, value), 10, system_id=2)
            self.a.ingest(named(name, value), 10, component_id=2)
        self.assertFalse(self.a.snapshot(10)['optimized_valid'])
        packet = named('ST_LAT', 100000000)
        packet['mavpackettype'] = 'NAMED_VALUE_FLOAT'
        self.assertFalse(self.a.ingest(packet, 10))

    def test_invalid_quality_diagnostics_rejected(self):
        for override in (dict(ST_LAT=910000000), dict(ST_LAT=0, ST_LON=0),
                         dict(ST_COUNT=0), dict(ST_NWIN=0), dict(ST_AGE=-1.),
                         dict(ST_AGE=5.01), dict(ST_SIG=0.), dict(ST_LAM=2.),
                         dict(ST_HUB=0.), dict(ST_WGT=1.1), dict(ST_SPAN=91.),
                         dict(ST_NEFF=30.), dict(ST_IMU=4)):
            self.a.connect()
            self.complete(values=fields(**override))
            self.assertFalse(self.a.snapshot(10)['optimized_valid'], str(override))

    def test_nan_inf_bool_and_invalid_integer_rejected(self):
        for name, value in (('ST_US', float('nan')), ('ST_SIG', float('inf')),
                            ('ST_US', True), ('ST_VER', True), ('ST_SEQ', 2**32)):
            self.assertFalse(self.a.ingest(named(name, value), 10))

    def test_official_position_requires_same_batch_coordinates(self):
        data = fields()
        del data['ST_FCLAT'], data['ST_FCLON']
        self.complete(values=data)
        snap = self.a.snapshot(10)
        self.assertTrue(snap['optimized_valid'])
        self.assertFalse(snap['official_valid'])
        self.a.ingest(named('ST_FCLAT', 100000020), 10.1)
        self.a.ingest(named('ST_FCLON', 200000020), 10.1)
        self.assertTrue(self.a.snapshot(10.1)['official_valid'])
        self.assertAlmostEqual(self.a.snapshot(10.1)['official_lat'], 10.000002)

    def test_raw_freshness_duplicate_and_fix_loss(self):
        self.a.ingest(gps(), 10)
        self.a.ingest(gps(), 14.9)
        self.assertFalse(self.a.snapshot(15.1)['raw_valid'])
        self.a.ingest(gps(timestamp=2_000_000), 16)
        self.assertTrue(self.a.snapshot(16)['raw_valid'])
        self.a.ingest(gps(timestamp=2_000_000, fix=1), 16.1)
        self.assertFalse(self.a.snapshot(16.1)['raw_valid'])

    def test_raw_zero_time_fix_loss_and_invalid_coordinates_clear_immediately(self):
        for packet in (gps(timestamp=0, fix=0), gps(timestamp=0),
                       gps(timestamp=1_000_001, lat=0, lon=0)):
            self.a.connect()
            self.a.ingest(gps(), 10)
            self.a.ingest(packet, 10.1)
            self.assertFalse(self.a.snapshot(10.1)['raw_valid'])

    def test_disconnect_reconnect_and_reboot_regression(self):
        self.complete(timestamp=60000)
        self.complete(timestamp=1000, received=16)
        self.assertFalse(self.a.snapshot(16)['optimized_valid'])
        self.assertIn('时间回退', self.a.snapshot(16)['status'])
        self.a.disconnect()
        self.complete()
        self.assertFalse(self.a.snapshot(16)['connected'])
        self.a.connect()
        self.complete()
        self.assertTrue(self.a.snapshot(10)['optimized_valid'])

    def test_u32_rollover_and_signed_payload(self):
        self.assertTrue(newer_u32(500, 0xFFFFFF00))
        self.complete(timestamp=0xFFFFFF00, values=fields(ST_SEQ=-3, ST_GPSMS=-1000))
        self.complete(timestamp=500, received=11, values=fields(ST_SEQ=2, ST_GPSMS=480))
        snap = self.a.snapshot(11)
        self.assertTrue(snap['optimized_valid'])
        self.assertEqual(snap['sequence'], 2)
        self.a.connect()
        self.complete(values=fields(ST_SEQ=-1, ST_GPSMS=-10))
        self.assertEqual(self.a.snapshot(10)['gps_source_ms'], 0xFFFFFFF6)


class ConnectionTests(unittest.TestCase):
    def test_manual_endpoints_only(self):
        self.assertEqual(validate_endpoint(' com5 '), 'COM5')
        self.assertEqual(validate_endpoint('udpin:127.0.0.1:14551'), 'udpin:127.0.0.1:14551')
        for endpoint in ('', 'sample.tlog', 'tcp:127.0.0.1:5760', 'udpin:localhost:99999'):
            with self.assertRaises(ValueError):
                validate_endpoint(endpoint)

    def test_transport_only_sends_heartbeat_and_collection_requests(self):
        events, sent = queue.Queue(), []
        worker = Receiver('COM5', 115200, 1, events, 1)

        class FakeMessage:
            def get_type(self): return 'HEARTBEAT'
            def get_srcSystem(self): return 1
            def get_srcComponent(self): return 1
            def to_dict(self): return {'mavpackettype': 'HEARTBEAT', 'base_mode': 0}

        class FakeMav:
            def heartbeat_send(self, *args): sent.append(('heartbeat', args))
            def command_long_send(self, *args): sent.append(('command', args))

        class FakeConnection:
            def __init__(self): self.mav, self.reads, self.closed = FakeMav(), 0, False
            def recv_match(self, **kwargs):
                self.reads += 1
                if self.reads == 1: return FakeMessage()
                worker.stop_event.set()
            def close(self): self.closed = True

        fake = FakeConnection()
        mavutil = SimpleNamespace(mavlink_connection=Mock(return_value=fake),
            mavlink=SimpleNamespace(MAV_TYPE_GCS=6, MAV_AUTOPILOT_INVALID=8,
                                    MAV_STATE_ACTIVE=4, MAV_CMD_SET_MESSAGE_INTERVAL=511))
        with patch.dict(sys.modules, {'pymavlink': SimpleNamespace(mavutil=mavutil)}):
            worker.run()
        self.assertEqual([kind for kind, _ in sent], ['heartbeat', 'command', 'command', 'command'])
        self.assertEqual([(args[4], args[5]) for _, args in sent[1:]],
                         [(24, 200000), (30, 100000), (27, 100000)])
        for _, args in sent[1:]:
            self.assertEqual(args[:4], (1, 1, 511, 0))
            self.assertEqual(args[6:], (0, 0, 0, 0, 0))
        self.assertTrue(fake.closed)
        self.assertEqual(sum(kind == 'request' for _, kind, _ in events.queue), len(REQUESTED_STREAMS))


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.recorder = CsvRecorder('COM5', 1, self.temp.name)

    def tearDown(self):
        self.recorder.close()
        self.temp.cleanup()

    def records(self, path):
        return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]

    def test_raw_epochs_keep_message_and_receive_clocks(self):
        for i in range(25):
            self.recorder.write_message(gps(timestamp=1_000_000+i*40_000),
                10+i*.04, '2000-01-01T00:00:00+00:00', 1, 1)
        rows = self.records(self.recorder.messages_path)
        self.assertEqual(len(rows), 25)
        self.assertEqual(rows[-1]['message']['time_usec'], 1_960_000)
        self.assertEqual(rows[-1]['monotonic_s'], 10.96)
        self.assertEqual(rows[-1]['schema_version'], 3)

    def test_snapshot_csv_contains_clocks_and_same_batch_diagnostics(self):
        a = TelemetryAggregator()
        a.connect()
        for name, value in fields().items(): a.ingest(named(name, value), 10)
        self.recorder.write(a.snapshot(10.1))
        with self.recorder.path.open(encoding='utf-8-sig') as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['monotonic_s'], '10.1')
        self.assertEqual(rows[0]['sequence'], '20')
        self.assertEqual(rows[0]['quality_sigma_m'], '1.2')
        self.assertTrue(rows[0]['utc_time'].endswith('+00:00'))

    def test_diagnostic_whitelist_and_unrelated_messages_excluded(self):
        for name in INT_FIELDS | FLOAT_FIELDS:
            self.assertTrue(recordable_message(named(name, 1)))
        for kind in ('STATUSTEXT', 'SYS_STATUS', 'EKF_STATUS_REPORT',
                     'LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT', 'RAW_IMU'):
            self.recorder.write_message({'mavpackettype': kind}, 1., 'utc', 1, 1)
        self.recorder.write_message({'mavpackettype': 'PARAM_VALUE'}, 1., 'utc', 1, 1)
        self.assertEqual(len(self.records(self.recorder.messages_path)), 6)

    def test_operator_marker_has_own_time_and_source_context(self):
        self.assertFalse(self.recorder.mark(' ', {}))
        self.assertTrue(self.recorder.mark('  转向开始\n约90°  ',
            dict(batch_time_boot_ms=2000, state=1, count=10, sequence=10, gps_source_ms=1990)))
        marker = self.records(self.recorder.events_path)[-1]
        self.assertEqual(marker['text'], '转向开始\n约90°')
        self.assertEqual(marker['sequence'], 10)
        self.assertEqual(marker['gps_source_ms'], 1990)
        self.assertIsInstance(marker['monotonic_s'], float)

    def test_close_is_idempotent(self):
        self.recorder.close()
        self.recorder.close()
        self.assertTrue(all(stream.closed for stream in self.recorder.streams))

    def test_disconnect_drains_queued_packets(self):
        app = MonitorApp.__new__(MonitorApp)
        app.events, app.token = queue.Queue(), 1
        app.recorder = self.recorder
        app.aggregator = TelemetryAggregator()
        app.receiver = SimpleNamespace(is_alive=lambda: False)
        app.set_controls, app.link_status = Mock(), Mock()
        for i in range(2010):
            app.events.put((1, 'packet', (gps(timestamp=i+1), i*.01, 'utc', 1, 1)))
        app.events.put((1, 'closed', None))
        app._finish_disconnect()
        self.assertEqual(len(self.records(self.recorder.messages_path)), 2010)
        self.assertTrue(all(stream.closed for stream in self.recorder.streams))
        self.assertTrue(app.events.empty())


if __name__ == '__main__':
    unittest.main(verbosity=2)
