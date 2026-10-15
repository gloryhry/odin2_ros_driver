#!/usr/bin/env python3
"""Check relocated install, ROS Python messages and live SDK startup without hardware."""
import os
import subprocess
import time
import xmlrpc.client

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11317'
os.environ['ROS_IP'] = '127.0.0.1'
import rospy
from odin2_ros_driver.msg import DeviceStatus

master = subprocess.Popen(['roscore', '-p', '11317'], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
node = None
try:
    deadline = time.monotonic() + 15
    while True:
        try:
            if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/smoke')[0] == 1:
                break
        except OSError:
            pass
        if time.monotonic() > deadline:
            raise RuntimeError('ROS master did not start')
        time.sleep(.1)
    os.environ.pop('ROS_HOSTNAME', None)
    rospy.init_node('install_smoke', anonymous=True)
    rospy.set_param('/odin2_driver/devices', [{
        'name': 'smoke', 'device_ip': '127.0.0.2', 'host_ip': '127.0.0.1',
        'streams': {'raw_point': {'enabled': False}, 'imu': {'enabled': False},
                    'telemetry': {'enabled': False}}}])
    node = subprocess.Popen(['/opt/odin2/lib/odin2_ros_driver/odin2_node'])
    latest = []
    sub = rospy.Subscriber('/smoke/status', DeviceStatus, lambda s: latest.append(s), queue_size=1)
    deadline = time.monotonic() + 20
    while not latest or latest[-1].state != 'retrying':
        if node.poll() is not None:
            raise RuntimeError('installed node exited with code %s' % node.returncode)
        if time.monotonic() > deadline:
            raise RuntimeError('installed node did not publish connection failure status')
        time.sleep(.1)
    status = latest[-1]
    sub.unregister()
    assert status.name == 'smoke'
    assert node.poll() is None, 'installed node exited'
    assert all(not c.enabled for c in status.channels)
    print('Installed live SDK node, Python messages and status publication: PASS')
finally:
    for process in (node, master):
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise RuntimeError('ROS process did not shut down cleanly')
