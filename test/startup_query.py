#!/usr/bin/env python3
"""Fixed IP still performs the device query before stream subscription."""
import time
import unittest

import rospy
import rostest
from sensor_msgs.msg import PointCloud2
from odin2_ros_driver.msg import DeviceStatus
from odin2_ros_driver.srv import GetChannelConfig, SetStream


class StartupQuery(unittest.TestCase):
    def test_fixed_targets(self):
        for name, sn in [('fixed_first', 'MOCK1'), ('fixed_second', 'MOCK2')]:
            deadline = time.monotonic() + 7
            while True:
                status = rospy.wait_for_message('/' + name + '/status', DeviceStatus, timeout=3)
                if status.state == 'running':
                    break
                self.assertLess(time.monotonic(), deadline, status.last_error)
            self.assertEqual(status.serial_number, sn)
            cloud = rospy.wait_for_message('/' + name + '/points_raw', PointCloud2, timeout=3)
            self.assertEqual(cloud.header.frame_id, name + '/lidar')

        query = rospy.ServiceProxy('/fixed_first/get_channel_config', GetChannelConfig)
        stream = query(0).effective
        stream.publish_rate = 5
        result = rospy.ServiceProxy('/fixed_first/set_stream', SetStream)(stream)
        self.assertTrue(result.result.success, result.result.message)
        for name in ['fixed_first', 'fixed_second']:
            rospy.wait_for_message('/' + name + '/points_raw', PointCloud2, timeout=3)

        for name in ['wrong_sn', 'wrong_ip', 'wrong_host']:
            status = rospy.wait_for_message('/' + name + '/status', DeviceStatus, timeout=3)
            self.assertEqual(status.state, 'retrying')
            self.assertIn('not discovered', status.last_error)
            self.assertEqual(status.reconnects, 0)
            self.assertFalse(any(channel.subscribed for channel in status.channels))


if __name__ == '__main__':
    rospy.init_node('startup_query_test')
    rostest.rosrun('odin2_ros_driver', 'startup_query', StartupQuery)
