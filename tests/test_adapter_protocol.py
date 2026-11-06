"""Actual C++ adapter JSONL -> Python V3 receiver protocol integration tests.

Compile runtime/adapter_test first (tools/run_checks.py does this). The fixture
uses simulated boundary devices, not a serial connection or physical hardware.
SPDX-License-Identifier: GPL-3.0-or-later
"""
from pathlib import Path
import json
import os
import subprocess
import sys
import unittest

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'monitor'))
from aggregation import (DIAGNOSTIC_COLUMNS, FLOAT_FIELDS, INT_FIELDS,
                         REQUIRED_FIELDS, TelemetryAggregator, newer_u32)


class ActualAdapterProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        executable = Path(os.environ.get('ADAPTER_TEST_BINARY', str(
            ROOT/'runtime'/('adapter_test.exe' if os.name == 'nt' else 'adapter_test'))))
        if not executable.is_file():
            raise RuntimeError('Compile the actual adapter fixture with tools/run_checks.py first.')
        run = subprocess.run([str(executable), '--telemetry-fixture'],
                             capture_output=True, text=True, encoding='utf-8', timeout=30)
        if run.returncode:
            raise RuntimeError(f'Adapter fixture failed: {run.stdout}\n{run.stderr}')
        cls.rows = [json.loads(line) for line in run.stdout.splitlines() if line.strip()]
        if not cls.rows or any('message' not in row or 'sent_us' not in row for row in cls.rows):
            raise RuntimeError('Expected adapter JSONL rows with sent_us and message.')
        cls.batches = {}
        for row in cls.rows:
            packet = row['message']
            cls.batches.setdefault(packet['time_boot_ms'], {})[packet['name']] = packet['value']

    def test_wire_schema_and_snapshot_source_consistency(self):
        self.assertGreater(len(self.rows), 60)
        previous_sent = -1
        valid_batches = []
        for row in self.rows:
            packet = row['message']
            self.assertGreater(row['sent_us'], previous_sent)
            previous_sent = row['sent_us']
            self.assertLessEqual(len(packet['name']), 10)
            self.assertIn(packet['name'], INT_FIELDS | FLOAT_FIELDS)
            self.assertEqual(packet['mavpackettype'],
                'NAMED_VALUE_INT' if packet['name'] in INT_FIELDS else 'NAMED_VALUE_FLOAT')
        for timestamp, data in self.batches.items():
            self.assertEqual(data['ST_VER'], 3)
            if data.get('ST_STATE') not in (1, 2):
                continue
            self.assertTrue(REQUIRED_FIELDS <= data.keys())
            source_ms = data['ST_GPSMS'] & 0xFFFFFFFF
            source_age_ms = (timestamp-source_ms) & 0xFFFFFFFF
            self.assertAlmostEqual(data['ST_AGE']*1000, source_age_ms, delta=1.1)
            self.assertLessEqual(source_age_ms, 2000)
            self.assertGreater(data['ST_NWIN'], 0)
            self.assertLessEqual(data['ST_NWIN'], data['ST_COUNT'])
            valid_batches.append((timestamp, data))
        self.assertGreaterEqual(len(valid_batches), 3)
        advancing = [(left, right) for left, right in zip(valid_batches, valid_batches[1:])
                     if left[1]['ST_SEG'] == right[1]['ST_SEG']
                     and 0 < right[1]['ST_COUNT']-left[1]['ST_COUNT'] <= 6]
        self.assertTrue(any(4 <= right[1]['ST_COUNT']-left[1]['ST_COUNT'] <= 6
                            for left, right in advancing), 'Fixture must exercise 5 Hz updates / 1 Hz snapshots')

    def test_replay_retains_complete_results_between_snapshot_fields(self):
        aggregator = TelemetryAggregator()
        aggregator.connect()
        became_valid = False
        reached_invalid = False
        held_old_during_new_batch = False
        for row in self.rows:
            packet, now = row['message'], 10+row['sent_us']/1e6
            before = aggregator.snapshot(now)
            aggregator.ingest(packet, now)
            after = aggregator.snapshot(now)
            if packet['name'] == 'ST_STATE' and packet['value'] not in (1, 2):
                reached_invalid = True
                self.assertFalse(after['optimized_valid'])
            if after['optimized_valid']:
                became_valid = True
                if after['batch_time_boot_ms'] != packet['time_boot_ms']:
                    held_old_during_new_batch = True
            if became_valid and not reached_invalid:
                self.assertTrue(after['optimized_valid'], f'Unexpected gap at {packet}')
            if before['optimized_valid'] and packet['name'] == 'ST_VER':
                if packet['time_boot_ms'] != before['batch_time_boot_ms']:
                    self.assertTrue(after['optimized_valid'])
        self.assertTrue(became_valid)
        self.assertTrue(held_old_during_new_batch)
        self.assertTrue(reached_invalid, 'Fixture must include an explicit invalid transition')

    def test_committed_diagnostics_belong_to_exact_source_batch(self):
        aggregator = TelemetryAggregator()
        aggregator.connect()
        seen_batches = set()
        recovered = False
        saw_invalid = False
        valid_segments = []
        for row in self.rows:
            packet, now = row['message'], 10+row['sent_us']/1e6
            aggregator.ingest(packet, now)
            snap = aggregator.snapshot(now)
            if packet['name'] == 'ST_STATE' and packet['value'] not in (1, 2):
                saw_invalid = True
                for name in ('optimized_lat', 'quality_sigma_m', 'huber_factor', 'budget_overruns'):
                    self.assertIsNone(snap[name])
            if not snap['optimized_valid']:
                continue
            timestamp = snap['batch_time_boot_ms']
            data = self.batches[timestamp]
            self.assertEqual(snap['gps_source_ms'], data['ST_GPSMS'] & 0xFFFFFFFF)
            self.assertEqual(snap['sequence'], data['ST_SEQ'] & 0xFFFFFFFF)
            self.assertEqual(snap['count'], data['ST_COUNT'])
            self.assertAlmostEqual(snap['optimized_lat'], data['ST_LAT']/1e7, places=10)
            for column, wire in DIAGNOSTIC_COLUMNS.items():
                if wire in data and snap[column] is not None:
                    expected = data[wire]
                    if wire in ('ST_SEQ', 'ST_GPSMS', 'ST_MSGMS', 'ST_SEG', 'ST_OVR'):
                        expected &= 0xFFFFFFFF
                    self.assertEqual(snap[column], expected, column)
            if timestamp not in seen_batches:
                seen_batches.add(timestamp)
                valid_segments.append(snap['segment'])
                recovered = recovered or saw_invalid
        self.assertGreaterEqual(len(seen_batches), 3)
        self.assertTrue(recovered, 'Fixture must include valid recovery after the invalid transition')
        self.assertTrue(all(b == a or newer_u32(b, a)
                            for a, b in zip(valid_segments, valid_segments[1:])))


if __name__ == '__main__':
    unittest.main(verbosity=2)
