"""Offline summaries of empirical servo JSONL evidence; never opens hardware."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median


def read_records(path):
    records = []
    previous = float('-inf')
    for number, line in enumerate(Path(path).read_text().splitlines(), 1):
        if not line.strip():
            continue
        record = json.loads(line)
        stamp = record.get('time')
        if not isinstance(record.get('event'), str) or type(stamp) not in (int, float):
            raise ValueError(f'invalid event at line {number}')
        if not math.isfinite(stamp) or stamp < previous:
            raise ValueError(f'invalid/nonmonotonic time at line {number}')
        previous = stamp
        records.append(record)
    if not records:
        raise ValueError('empty servo log')
    return records


def distribution(values):
    if not values:
        return {'count': 0}
    return {'count': len(values), 'min': min(values), 'median': median(values),
            'max': max(values)}


def summarize(records):
    cycles, motions, pending = [], [], []
    joint_samples = defaultdict(list)
    first_error = None
    last_counter = 0
    rejected_counters = []
    invalid_motions = []
    for record in records:
        event = record['event']
        if event == 'motion':
            requested_names = [name for name, goal in record['goal'].items()
                               if goal != record['before'][name]]
            verified = bool(requested_names) and all(
                abs(record['actual'][name]-record['before'][name]) >= 2
                and (record['actual'][name]-record['before'][name]) *
                    (record['goal'][name]-record['before'][name]) > 0
                for name in requested_names
            )
            if verified:
                pending.append(record)
            else:
                invalid_motions.append(record['time'])
            for name, goal in record['goal'].items():
                before = record['before'][name]
                actual = record['actual'][name]
                requested = goal - before
                if requested:
                    row = {'time': record['time'], 'joint': name,
                           'requested_counts': requested,
                           'actual_counts': actual-before,
                           'residual_counts': goal-actual}
                    motions.append(row)
                    joint_samples[name].append(row)
        elif event == 'response' and first_error is None:
            first_error = record['error_before']
        elif event == 'locked':
            counter = record.get('completed_cycles')
            if counter is None:
                counter = last_counter + bool(pending)  # Earlier log schema.
            if type(counter) is not int or counter < last_counter:
                raise ValueError('nonmonotonic correction counter')
            if counter > last_counter:
                # A displayed count alone is not physical acceptance evidence.
                if not pending or counter != last_counter + 1:
                    rejected_counters.append(counter)
                else:
                    error = record['error']
                    cycles.append({
                        'cycle': counter, 'motion_steps': len(pending),
                        'initial_error_x_px': None if first_error is None else first_error[0],
                        'initial_error_y_px': None if first_error is None else first_error[1],
                        'final_error_x_px': error[0], 'final_error_y_px': error[1],
                        'final_center_distance_px': math.hypot(*error),
                        'first_confirmed_motion_to_aligned_s': record['time']-pending[0]['time'],
                    })
                last_counter = counter
            pending = []
            first_error = None
    events = Counter(record['event'] for record in records)
    joint_stats = {}
    for name, samples in joint_samples.items():
        joint_stats[name] = {
            'requested_amplitude_counts': distribution([abs(s['requested_counts']) for s in samples]),
            'actual_amplitude_counts': distribution([abs(s['actual_counts']) for s in samples]),
            'absolute_residual_counts': distribution([abs(s['residual_counts']) for s in samples]),
            'directions': dict(Counter('positive' if s['requested_counts'] > 0 else 'negative' for s in samples)),
        }
    summary = {
        'schema_version': 1, 'scope': 'empirical center-region MVP; not precision/four-corner acceptance',
        'settings': next((r for r in records if r['event'] == 'settings'), None),
        'calibration': next((r for r in records if r['event'] == 'calibration'), None),
        'events': dict(events), 'motion_steps': events['motion'],
        'displayed_completed_cycles': last_counter, 'evidence_backed_cycles': len(cycles),
        'unsupported_counter_increments': rejected_counters,
        'invalid_motion_events': invalid_motions,
        'final_center_distance_px': distribution([c['final_center_distance_px'] for c in cycles]),
        'first_confirmed_motion_to_aligned_s': distribution([c['first_confirmed_motion_to_aligned_s'] for c in cycles]),
        'timing_definition': 'starts at first confirmed motion event; excludes that first action execution and is not full settling time',
        'joint_response': joint_stats,
        'stop_reasons': [r.get('reason') for r in records if r['event'] == 'hold'],
        'unclosed_motion_steps': len(pending),
    }
    return summary, cycles, motions


def write_report(source, output):
    source, output = Path(source), Path(output)
    summary, cycles, motions = summarize(read_records(source))
    summary['source'] = str(source)
    summary['source_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
    output.mkdir(parents=True, exist_ok=True)
    (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    for filename, rows in (('cycles.csv', cycles), ('joint_motion.csv', motions)):
        if rows:
            with (output/filename).open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    summary = write_report(args.log, args.output_dir)
    print(json.dumps({key: summary[key] for key in (
        'motion_steps', 'evidence_backed_cycles', 'final_center_distance_px', 'stop_reasons')}, ensure_ascii=False, indent=2))
    return 0
