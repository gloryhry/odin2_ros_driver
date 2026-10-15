#!/usr/bin/env python3
"""Opt-in GPU ROS check inside the test image; uses the mock SDK, not Odin2.

The mock advertises 16x16 modes. Replace its compressed payloads with 256x256
fixtures to exercise real hardware decoding and diagnostics through the worker.
"""
import collections
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Image
from odin2_ros_driver.msg import EncodedFrame, StreamConfig
from odin2_ros_driver.srv import GetChannelConfig, SetStream
from roscpp.srv import GetLoggers, SetLoggerLevel


def wait_for(condition, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(.05)
    raise AssertionError('GPU ROS condition timed out')


def main():
    root = Path(os.environ['ODIN_TEST_FIXTURES'])
    for name in ('mjpeg.jpg', 'h264.bin', 'h265.bin'):
        shutil.copyfile(root / ('hw-' + name), root / name)
    core = subprocess.Popen(['roscore'], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    node = None
    log = open('/tmp/gpu-ros-driver.log', 'w')
    try:
        time.sleep(2)
        rospy.init_node('gpu_ros_validation', anonymous=True)
        stream = {'enabled': True, 'format': 'mjpeg', 'width': 16, 'height': 16, 'fps': 10}
        rospy.set_param('/odin2_gpu_mock/devices', [
            {'name': 'gpu_ros', 'sn': 'MOCK1',
             'streams': {'camera0': stream, 'camera1': stream}}])
        node = subprocess.Popen([
            'rosrun', 'odin2_ros_driver', 'odin2_mock_node', '__name:=odin2_gpu_mock'],
            stdout=log, stderr=subprocess.STDOUT)
        images = [collections.deque(maxlen=256) for _ in range(2)]
        encoded = [collections.deque(maxlen=256) for _ in range(2)]
        diagnostics = []
        subscribers = []
        for camera in range(2):
            prefix = '/gpu_ros/camera%d/' % camera
            subscribers += [rospy.Subscriber(prefix + 'image_raw', Image, images[camera].append),
                            rospy.Subscriber(prefix + 'encoded', EncodedFrame, encoded[camera].append)]
        def on_diagnostics(message):
            diagnostics[:] = message.status
        subscribers.append(rospy.Subscriber('/diagnostics', DiagnosticArray, on_diagnostics))
        rospy.wait_for_service('/gpu_ros/get_channel_config', timeout=20)
        get_config = rospy.ServiceProxy('/gpu_ros/get_channel_config', GetChannelConfig)
        set_stream = rospy.ServiceProxy('/gpu_ros/set_stream', SetStream)
        for value, codec in ((StreamConfig.MJPEG, 'mjpeg'),
                             (StreamConfig.H264, 'h264'), (StreamConfig.H265, 'hevc')):
            for camera in range(2):
                config = get_config(16 + camera).effective
                config.format = value
                assert set_stream(config).result.success
                images[camera].clear()
                encoded[camera].clear()
            expected = 'cuda/' + codec
            def ready():
                for status in diagnostics:
                    values = {item.key: item.value for item in status.values}
                    if all(values.get('camera%d/decoder_backend' % c) == expected and
                           values.get('camera%d/hardware_active' % c, '').lower() in
                           ('true', '1') for c in range(2)):
                        return all(images) and all(encoded)
                return False
            wait_for(ready)
            for camera in range(2):
                def matching():
                    wire = {(p.header.seq, p.header.stamp.to_nsec()) for p in list(encoded[camera])
                            if p.format == value}
                    return next((p for p in list(images[camera])
                                 if (p.header.seq, p.header.stamp.to_nsec()) in wire), None)
                wait_for(matching)
                image = matching()
                assert (image.width, image.height, image.encoding) == (256, 256, 'bgr8')
                assert image.header.frame_id == 'gpu_ros/camera%d' % camera
                assert image.data[2] > 200 and image.data[0] < 30
            print(codec + ': both cameras publish valid bgr8; diagnostics=' + expected +
                  ', hardware_active=true; encoded/image headers match', flush=True)
        rospy.wait_for_service('/odin2_gpu_mock/get_loggers', timeout=5)
        loggers = rospy.ServiceProxy('/odin2_gpu_mock/get_loggers', GetLoggers)().loggers
        assert loggers, 'ROS logger enumeration is unavailable'
        set_level = rospy.ServiceProxy('/odin2_gpu_mock/set_logger_level', SetLoggerLevel)
        logger = loggers[0].name
        set_level(logger, 'debug')
        levels = rospy.ServiceProxy('/odin2_gpu_mock/get_loggers', GetLoggers)().loggers
        assert any(item.name == logger and item.level.lower() == 'debug' for item in levels)
        set_level(logger, loggers[0].level)
        print('ROS logger enumeration and level adjustment: PASS', flush=True)
        for subscriber in subscribers:
            subscriber.unregister()
    finally:
        for process in (node, core):
            if process is not None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=15)
        assert node is None or node.returncode == 0, 'driver did not shut down normally'
        log.close()
    print('GPU ROS worker, diagnostics and shutdown: PASS', flush=True)


if __name__ == '__main__':
    main()
