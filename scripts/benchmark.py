#!/usr/bin/env python3
"""Sample status counters; CPU percent is relative to one logical CPU."""
import argparse
import csv
import os
import time
import rospy
from odin2_ros_driver.msg import DeviceStatus


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', default='odin2')
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--output', default='odin2-benchmark.csv')
    parser.add_argument('--pid', type=int)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('odin2_benchmark', anonymous=True)
    last_cpu = None
    last_channels = {}
    with open(args.output, 'w', newline='') as output:
        writer = csv.writer(output)
        writer.writerow(['wall_time', 'channel', 'received', 'published', 'decoded_frames',
                         'sequence_gaps', 'queue_drops', 'decode_errors', 'queue_depth',
                         'queue_bytes', 'last_latency_ms', 'rss_kb', 'cpu_percent',
                         'receive_hz', 'publish_hz', 'decode_fps'])

        def callback(status):
            nonlocal last_cpu
            now = time.monotonic()
            rss, cpu = '', ''
            if args.pid:
                try:
                    with open('/proc/%d/stat' % args.pid) as f:
                        stat = f.read().split(') ', 1)[1].split()
                        ticks = int(stat[11]) + int(stat[12])
                        process_start = stat[19]
                    if last_cpu and last_cpu[2] == process_start:
                        cpu = 100 * (ticks - last_cpu[1]) / os.sysconf('SC_CLK_TCK') / (now - last_cpu[0])
                    last_cpu = (now, ticks, process_start)
                    with open('/proc/%d/status' % args.pid) as f:
                        rss = next(line.split()[1] for line in f if line.startswith('VmRSS:'))
                except (OSError, StopIteration, ValueError):
                    last_cpu = None
            for c in status.channels:
                counts = (c.received, c.published, c.decoded_frames)
                previous = last_channels.get(c.channel)
                rates = ['', '', '']
                if previous:
                    rates = [(count - old) / (now - previous[0])
                             for count, old in zip(counts, previous[1])]
                last_channels[c.channel] = (now, counts)
                writer.writerow([time.time(), c.channel, *counts, c.sequence_gaps,
                                 c.queue_drops, c.decode_errors, c.queue_depth, c.queue_bytes,
                                 c.last_publish_latency_ms, rss, cpu, *rates])
            output.flush()

        subscriber = rospy.Subscriber('/' + args.device + '/status', DeviceStatus, callback, queue_size=1)
        deadline = time.monotonic() + args.seconds
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            time.sleep(.1)
        subscriber.unregister()


if __name__ == '__main__':
    main()
