#!/usr/bin/env python3
"""Passive Gazebo/ROS performance capture; optional measured Qt image display."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import gz.transport13
from gz.msgs10.image_pb2 import Image as GzImage


def percentiles(values):
    values = [float(v) for v in values if v is not None and np.isfinite(v)]
    return dict(n=len(values), p50=float(np.median(values)),
                p95=float(np.percentile(values, 95)), max=float(max(values))) if values else dict(n=0)


def cdr_stamp(data):
    if len(data) < 12 or data[1] not in (0, 1):
        raise ValueError('Expected standard CDR sensor_msgs/Image encapsulation')
    sec, nanosec = struct.unpack_from('<iI' if data[1] == 1 else '>iI', data, 4)
    return sec * 1_000_000_000 + nanosec


def frame_metrics(row):
    """Wall durations and source-clock ages are separate; missing RMW stays null."""
    def wall(end, start):
        return (row[end] - row[start]) / 1e6 if row.get(end) and row.get(start) else None
    result = {k: row.get(k) for k in ('image_decode_ms', 'yolo_ms', 'tracker_ms', 'reid_ms',
              'gmc_ms', 'reid_crop_ms', 'reid_preprocess_ms', 'reid_backend_ms', 'guard_ms', 'processing_ms')}
    result.update(dds_transport_ms=wall('rmw_received_wall_ns', 'rmw_published_wall_ns'),
                  queue_wait_wall_ms=wall('callback_wall_ns', 'rmw_received_wall_ns'),
                  callback_to_processing_ms=wall('processing_start_mono_ns', 'callback_mono_ns'),
                  decision_publish_ms=wall('output_publish_mono_ns', 'guard_decision_mono_ns'),
                  control_path_ms=wall('output_publish_mono_ns', 'processing_start_mono_ns'),
                  display_diagnostics_ms=wall('complete_mono_ns', 'output_publish_mono_ns'))
    for name, key in [('receive_age_source_ms', 'receive_source_clock_ns'),
                      ('decision_age_source_ms', 'guard_decision_source_clock_ns'),
                      ('output_age_source_ms', 'output_source_clock_ns')]:
        result[name] = (row[key] - row['source_stamp_ns']) / 1e6 if key in row else None
    for k, v in row.get('yolo_stages_ms', {}).items():
        result['yolo_' + k + '_ms'] = v
    if row.get('tracker_ms') is not None:
        result['association_adapter_ms'] = row['tracker_ms'] - row.get('reid_ms', 0) - row.get('gmc_ms', 0)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--warmup-frames', type=int, default=10)
    parser.add_argument('--stop-source-time', type=float)
    parser.add_argument('--perception-pid', type=int)
    parser.add_argument('--display', action='store_true')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    events = {k: [] for k in ('source', 'ros_camera_observer', 'display_delivery', 'paint', 'performance', 'resource')}
    latest = [None]
    stop = threading.Event()
    def finish(*_args): stop.set()
    signal.signal(signal.SIGINT, finish); signal.signal(signal.SIGTERM, finish)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = Node('perception_pipeline_profiler')
    executor = SingleThreadedExecutor(); executor.add_node(node)
    started = time.monotonic(); window = [None]
    source_node = gz.transport13.Node()
    def source(msg):
        events['source'].append((time.monotonic(), int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nsec)))
    assert source_node.subscribe(GzImage, '/custom_x500/camera/image_raw', source)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Image, '/custom_x500/camera/image_raw',
                             lambda data: events['ros_camera_observer'].append((time.monotonic(), cdr_stamp(data))), qos, raw=True)
    log = (args.output_dir/'performance.jsonl').open('w', buffering=1)
    completed = [0]
    def performance(message):
        row = json.loads(message.data); now = time.monotonic()
        if row['status'] in ('processed', 'expired_output'):
            completed[0] += 1
        if window[0] is None and completed[0] > args.warmup_frames:
            window[0] = now
        events['performance'].append((now, row)); log.write(json.dumps(dict(observed_mono=now, **row))+'\n')
        if args.stop_source_time is not None and row['source_stamp_ns'] >= args.stop_source_time*1e9:
            stop.set()
    node.create_subscription(String, '/yolo/performance', performance, 100)
    if args.display:
        from PyQt5.QtWidgets import QApplication, QWidget
        from PyQt5.QtGui import QImage, QPainter, QColor
        from PyQt5.QtCore import QTimer, Qt
        app = QApplication([])
        def image(msg):
            stamp = int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)
            events['display_delivery'].append((time.monotonic(), stamp))
            fmt = {'rgb8': QImage.Format_RGB888, 'bgr8': QImage.Format_BGR888,
                   'mono8': QImage.Format_Grayscale8, '8uc1': QImage.Format_Grayscale8}[msg.encoding.lower()]
            latest[0] = (stamp, QImage(bytes(msg.data), msg.width, msg.height, msg.step, fmt).copy())
        node.create_subscription(Image, '/yolo/annotated_image', image, qos)
        class Viewer(QWidget):
            painted_stamp = None
            def paintEvent(self, event):
                painter = QPainter(self); painter.fillRect(self.rect(), QColor('#18212f'))
                current = latest[0]
                if current is not None:
                    painter.drawImage(self.rect().adjusted(0, 0, 0, -40), current[1])
                    if current[0] != self.painted_stamp:
                        events['paint'].append((time.monotonic(), current[0])); self.painted_stamp = current[0]
                now = time.monotonic(); fps = sum(t > now-5 for t, _ in events['paint'])/5
                painter.setPen(Qt.white); painter.drawText(15, self.height()-15, f'Actual unique frames painted: {fps:.1f} FPS (5 s)')
        viewer = Viewer(); viewer.setWindowTitle('PERCEPTION AUDIT — actual painted frames'); viewer.resize(960, 580); viewer.show()
        def refresh():
            viewer.update()
            if stop.is_set(): app.quit()
        timer = QTimer(); timer.timeout.connect(refresh); timer.start(16)
    else:
        node.create_subscription(Image, '/yolo/annotated_image',
                                 lambda data: events['display_delivery'].append((time.monotonic(), cdr_stamp(data))), qos, raw=True)
    def resource():
        previous = None
        while not stop.wait(1.):
            row = dict(mono=time.monotonic(), wall=time.time())
            if args.perception_pid:
                try:
                    path = Path('/proc')/str(args.perception_pid); raw = (path/'stat').read_text(); fields=raw[raw.rfind(')')+2:].split()
                    ticks = int(fields[11])+int(fields[12]); rss = int(fields[21])*os.sysconf('SC_PAGE_SIZE')
                    if previous: row['perception_cpu_pct']=100*(ticks-previous[1])/(os.sysconf('SC_CLK_TCK')*(row['mono']-previous[0]))
                    previous=(row['mono'],ticks); row['perception_rss_bytes']=rss
                except (FileNotFoundError, ProcessLookupError): pass
            try:
                gpu = subprocess.run(['nvidia-smi','--query-gpu=utilization.gpu,memory.used,power.draw','--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=3)
                if gpu.returncode==0:
                    values=gpu.stdout.strip().splitlines()[0].split(',');row.update(gpu_util_pct=float(values[0]),gpu_memory_mib=float(values[1]),gpu_power_w=float(values[2]))
            except (FileNotFoundError,subprocess.TimeoutExpired,ValueError): pass
            events['resource'].append(row)
            if window[0] is not None and row['mono']-window[0]>=args.duration: stop.set()
            if row['mono']-started>args.duration+120: stop.set()
    worker=threading.Thread(target=executor.spin,daemon=True); worker.start()
    sampler=threading.Thread(target=resource,daemon=True); sampler.start()
    try:
        if args.display: app.exec_()
        else:
            deadline=started+args.duration+120
            while not stop.wait(.1):
                if time.monotonic()>deadline: stop.set()
    finally:
        stop.set();executor.shutdown();worker.join(5);node.destroy_node();rclpy.try_shutdown();log.close()
    end=time.monotonic(); begin=window[0] if window[0] is not None else end; elapsed=max(end-begin,.001)
    streams={k:[r for r in v if begin<=r[0]<=end] for k,v in events.items() if k!='resource'}
    rows=[r for _,r in streams['performance'] if r['status'] in ('processed','expired_output')]
    metrics=[frame_metrics(r) for r in rows];keys=set().union(*(r.keys() for r in metrics)) if metrics else set()
    resources=[r for r in events['resource'] if begin<=r['mono']<=end]
    stamps=[r['source_stamp_ns'] for _,r in streams['performance']]
    bounds=(min(stamps),max(stamps)) if stamps else (1,0)
    source_interior={s for _,s in events['source'] if bounds[0]<=s<=bounds[1]}
    offsets=[r['callback_wall_ns']/1e9-r['callback_mono_ns']/1e9 for r in rows if 'callback_wall_ns' in r]
    summary=dict(window_wall_s=elapsed,warmup_completed_frames=args.warmup_frames,completed_frames=len(rows),
        fps={k:len(v)/elapsed for k,v in streams.items() if k!='performance'},processing_fps=len(rows)/elapsed,
        receive_callback_fps=((max(r['received'] for _, r in streams['performance']) -
                               min(r['received'] for _, r in streams['performance']) + 1)/elapsed
                              if streams['performance'] else 0.),
        stages_ms={k:percentiles([r.get(k) for r in metrics]) for k in sorted(keys)},
        resources={k:percentiles([r.get(k) for r in resources]) for k in ('perception_cpu_pct','perception_rss_bytes','gpu_util_pct','gpu_memory_mib','gpu_power_w')},
        statuses=dict(Counter(r['status'] for _,r in streams['performance'])),
        skipped_source_stamps_interior=len(source_interior-set(stamps)),source_stamps_interior=len(source_interior),
        clock_wall_minus_monotonic_range_ms=(max(offsets)-min(offsets))*1000 if offsets else None,
        source_window_ns=[min(stamps),max(stamps)] if stamps else None,
        graph_active_frames=sum(bool(r.get('cuda_graph_active')) for r in rows),
        reid_crops=dict(Counter(r.get('reid_crops',0) for r in rows)))
    (args.output_dir/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    (args.output_dir/'streams.json').write_text(json.dumps({k:v for k,v in streams.items() if k!='performance'})+'\n')
    (args.output_dir/'resources.json').write_text(json.dumps(events['resource'])+'\n')
    print(json.dumps(summary,indent=2))


if __name__ == '__main__':main()
