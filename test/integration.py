#!/usr/bin/env python3
"""ROS acceptance with independently addressed devices and recoverable SDK faults."""
import struct
import time
import unittest
import rospy
import rostest
from dynamic_reconfigure.client import Client
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import PointCloud2, Imu, Image, CompressedImage
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from odin2_ros_driver.msg import DeviceStatus, Telemetry, OdomPacket, EncodedFrame
from odin2_ros_driver.srv import (GetDeviceInfo, GetChannelConfig, SetStream, SetFilters,
                                SetFiltersRequest, GetFilters, StartFileTransfer,
                                GetTransferStatus, SendAlgoData, DeviceCommand, GetMode, SetMode, SetNetwork)


class Integration(unittest.TestCase):
    def message(self, topic, kind):
        return rospy.wait_for_message(topic, kind, timeout=20)

    def until(self, fn, seconds=25):
        deadline = time.monotonic() + seconds
        while not rospy.is_shutdown():
            if fn():
                return
            self.assertLess(time.monotonic(), deadline, 'condition timed out')
            time.sleep(.1)

    def fault(self, key, value=True):
        rospy.set_param('/odin2_mock/MOCK1/' + key, value)

    def state(self):
        return self.message('/first/status', DeviceStatus)

    def test_driver(self):
        info = '/first/get_device_info'
        rospy.wait_for_service(info, timeout=30)
        self.until(lambda: rospy.ServiceProxy(info, GetDeviceInfo)().result.success)
        a = self.message('/first/points_raw', PointCloud2)
        b = self.message('/second/points_raw', PointCloud2)
        self.assertEqual(a.header.frame_id, 'first/lidar')
        self.assertEqual(b.header.frame_id, 'second/lidar')
        world = self.message('/second/points_slam', PointCloud2)
        self.assertEqual(world.header.frame_id, 'second/odom')
        self.assertAlmostEqual(struct.unpack_from('<f', world.data)[0], 2)
        dependency = rospy.ServiceProxy('/second/get_channel_config', GetChannelConfig)(48)
        self.assertTrue(dependency.subscribed)
        self.assertFalse(dependency.effective.enabled)
        self.assertAlmostEqual(struct.unpack_from('<f', a.data)[0], 1)
        imu = self.message('/first/imu/data_raw', Imu)
        self.assertEqual(imu.orientation_covariance[0], -1)
        self.assertAlmostEqual(imu.linear_acceleration.z, 9.81, places=4)
        for camera in (0, 1):
            image = self.message('/first/camera%d/image_raw' % camera, Image)
            self.assertEqual((image.width, image.height, image.encoding), (16, 16, 'bgr8'))
            self.assertEqual(image.header.frame_id, 'first/camera%d' % camera)
        jpeg = self.message('/first/camera1/image_raw/compressed', CompressedImage)
        encoded = self.message('/first/camera1/encoded', EncodedFrame)
        self.assertEqual(bytes(jpeg.data), bytes(encoded.data))
        self.fault('corrupt_camera1')
        self.until(lambda: len(self.message('/first/camera1/encoded', EncodedFrame).data) == 4, seconds=5)
        self.until(lambda: next(c for c in self.state().channels if c.channel == 17).decode_errors > 0, seconds=5)
        self.fault('corrupt_camera1', False)
        self.message('/first/camera1/image_raw', Image)
        raw = self.message('/first/odom_lf/sdk', OdomPacket)
        self.assertEqual(raw.samples[0].sdk_covariance[17], 17)
        odom = self.message('/first/odom_hf', Odometry)
        self.assertEqual(odom.child_frame_id, 'first/imu')
        tf = self.message('/first/transforms', TFMessage)
        self.assertEqual(tf.transforms[0].header.frame_id, 'first/odom')
        self.assertEqual(self.message('/first/telemetry', Telemetry).cpu_pct, 25)
        state = self.message('/second/status', DeviceStatus)
        self.assertFalse(next(c for c in state.channels if c.channel == 32).enabled)
        proxy = rospy.ServiceProxy('/first/get_channel_config', GetChannelConfig)
        set_stream = rospy.ServiceProxy('/first/set_stream', SetStream)
        stream = proxy(0).effective
        stream.enabled = False
        self.assertTrue(set_stream(stream).result.success)
        self.assertFalse(proxy(0).subscribed)
        received = []
        sub = rospy.Subscriber('/first/points_raw', PointCloud2, received.append)
        time.sleep(.6)
        self.assertEqual(len(received), 0, 'disabled channel published')
        sub.unregister()
        stream.enabled = True
        self.assertTrue(set_stream(stream).result.success)
        self.message('/first/points_raw', PointCloud2)
        bad = proxy(16).effective
        bad.width = 32
        self.assertFalse(set_stream(bad).result.success)
        self.message('/first/camera0/image_raw', Image)

        self.fault('fail_next_start')
        stream.publish_rate = 3
        reply = set_stream(stream)
        self.assertFalse(reply.result.success)
        self.assertIn('restored', reply.result.message)
        self.assertEqual(reply.effective.publish_rate, 0)
        self.message('/first/points_raw', PointCloud2)
        self.fault('fail_all_starts')
        failed_restore = set_stream(stream)
        self.assertFalse(failed_restore.result.success)
        self.assertIn('rollback failed', failed_restore.result.message)
        self.until(lambda: self.state().state == 'retrying', seconds=5)
        self.assertFalse(rospy.ServiceProxy(info, GetDeviceInfo)().result.success)
        self.message('/second/points_raw', PointCloud2)
        self.fault('fail_all_starts', False)
        self.until(lambda: self.state().state == 'running', seconds=10)
        f = SetFiltersRequest()
        f.filters.has_adaptive = True
        f.filters.cdf_percentile = 8000
        f.filters.outdoor_threshold = 4000
        set_filters = rospy.ServiceProxy('/first/set_filters', SetFilters)
        self.assertTrue(set_filters(f).result.success)
        self.fault('fail_next_adaptive')
        f.filters.cdf_percentile = 9000
        failed = set_filters(f)
        self.assertFalse(failed.result.success)
        self.assertTrue(failed.effective.has_adaptive)
        self.assertEqual(failed.effective.cdf_percentile, 8000)
        self.assertEqual(rospy.ServiceProxy('/first/get_filters', GetFilters)().filters.cdf_percentile, 8000)
        f.filters.has_confidence = True
        unsupported = set_filters(f)
        self.assertFalse(unsupported.result.success)
        self.assertTrue(unsupported.result.message.startswith('UNSUPPORTED_FILTER:'))
        self.assertFalse(unsupported.effective.has_adaptive)
        client = Client('/first', timeout=10)
        self.assertEqual(client.update_configuration({'raw_point_publish_rate': 5})['raw_point_publish_rate'], 5)
        client.close()

        job = rospy.ServiceProxy('/first/start_file_transfer', StartFileTransfer)(0, 1, '/tmp/odin2-mock-calibration.bin', 5000)
        self.assertTrue(job.result.success)
        status = rospy.ServiceProxy('/first/get_transfer_status', GetTransferStatus)
        self.until(lambda: status(job.job_id).status.state >= 2, seconds=5)
        self.assertEqual(status(job.job_id).status.state, 2)
        self.assertEqual(status(job.job_id).status.progress, 100)
        bad_job = rospy.ServiceProxy('/first/start_file_transfer', StartFileTransfer)(
            0, 1, '/tmp/odin2-nonexistent-parent/calibration.bin', 5000)
        self.assertTrue(bad_job.result.success)
        self.until(lambda: status(bad_job.job_id).status.state >= 2, seconds=5)
        self.assertEqual(status(bad_job.job_id).status.state, 3)
        self.assertFalse(status(bad_job.job_id).status.result.success)
        self.assertFalse(rospy.ServiceProxy('/first/start_file_transfer', StartFileTransfer)(
            0, 3, '/tmp/firmware.bin', 5000).result.success)
        send = rospy.ServiceProxy('/first/send_algo_data', SendAlgoData)
        self.assertFalse(send([]).result.success)
        self.assertTrue(send([1]).result.success)
        self.assertTrue(send([1] * 262144).result.success)
        self.assertFalse(send([1] * 262145).result.success)
        self.assertFalse(status(999999).result.success)

        self.fault('fail_telemetry')
        stream = proxy(66).effective
        self.assertTrue(set_stream(stream).result.success)
        self.assertFalse(proxy(66).subscribed)
        self.until(lambda: not next(c for c in self.state().channels if c.channel == 66).receiving, seconds=10)
        def telemetry_warning():
            d = self.message('/diagnostics', DiagnosticArray)
            return any(x.name.endswith('first/driver') and x.level >= 1 and 'telemetry' in x.message for x in d.status)
        self.until(telemetry_warning, seconds=10)
        self.fault('fail_telemetry', False)
        self.assertTrue(set_stream(stream).result.success)
        self.message('/first/telemetry', Telemetry)

        self.fault('duplicate_packets')
        self.until(lambda: next(c for c in self.state().channels if c.channel == 0).out_of_order > 0, seconds=5)
        self.fault('duplicate_packets', False)
        self.fault('force_offline')
        self.until(lambda: self.state().state == 'retrying', seconds=10)
        self.message('/second/points_raw', PointCloud2)
        self.fault('force_offline', False)
        self.until(lambda: self.state().state == 'running', seconds=10)
        self.message('/first/points_raw', PointCloud2)
        # Healthy command plane with a stalled data plane must reconnect as well.
        before = self.state().reconnects
        self.fault('stop_data')
        self.until(lambda: self.state().reconnects > before, seconds=30)
        self.fault('stop_data', False)
        resumed = self.message('/first/points_raw', PointCloud2)
        self.assertLess(abs((rospy.Time.now() - resumed.header.stamp).to_sec()), 3)
        self.assertTrue(rospy.ServiceProxy('/first/set_sensor_mode', SetMode)(3).result.success)
        self.assertTrue(rospy.ServiceProxy('/first/set_sync_mode', SetMode)(1).result.success)
        self.assertFalse(rospy.ServiceProxy('/first/set_sensor_mode', SetMode)(4).result.success)
        self.assertTrue(rospy.ServiceProxy('/first/set_device_mode', SetMode)(0).result.success)
        self.assertEqual(rospy.ServiceProxy('/first/get_device_mode', GetMode)().mode, 0)
        self.assertTrue(rospy.ServiceProxy('/first/set_device_mode', SetMode)(1).result.success)
        device = rospy.ServiceProxy(info, GetDeviceInfo)()
        device.network.ip_address = '127.0.0.11'
        device.network.netmask = '255.0.0.0'
        device.network.gateway = '127.0.0.1'
        changed = rospy.ServiceProxy('/first/set_network', SetNetwork)(device.network)
        self.assertTrue(changed.result.success)
        self.assertTrue(changed.reboot_required)
        self.assertTrue(rospy.ServiceProxy('/first/reboot', DeviceCommand)().result.success)
        self.message('/first/points_raw', PointCloud2)
        self.until(lambda: self.state().device_ip == '127.0.0.11', seconds=10)
        self.assertEqual(rospy.ServiceProxy('/first/get_sensor_mode', GetMode)().mode, 3)


if __name__ == '__main__':
    rospy.init_node('odin2_integration_test')
    rostest.rosrun('odin2_ros_driver', 'integration', Integration)
