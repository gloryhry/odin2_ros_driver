#!/usr/bin/env python3
"""Exercise the actual worker/publication path with delayed software-backed frames."""
import time
import unittest
import rospy
import rostest
from sensor_msgs.msg import Image
from odin2_ros_driver.msg import EncodedFrame, DeviceStatus
from odin2_ros_driver.srv import GetChannelConfig, SetStream


class AsyncDecoder(unittest.TestCase):
    def test_final_frame_and_generation(self):
        images, encoded = [], []
        image_sub = rospy.Subscriber('/async_camera/camera1/image_raw', Image, images.append)
        encoded_sub = rospy.Subscriber('/async_camera/camera1/encoded', EncodedFrame, encoded.append)
        deadline = time.monotonic() + 15
        while not images or not encoded:
            self.assertLess(time.monotonic(), deadline)
            time.sleep(.02)
        rospy.set_param('/odin2_mock/MOCK1/pause_camera1', True)
        time.sleep(.15)  # Allow the mock callback currently in progress to finish.
        final_header = encoded[-1].header
        deadline = time.monotonic() + 3
        while not any(image.header.stamp == final_header.stamp for image in images):
            self.assertLess(time.monotonic(), deadline, 'last delayed frame needs another packet')
            time.sleep(.01)
        status = rospy.wait_for_message('/async_camera/status', DeviceStatus, timeout=3)
        camera = next(c for c in status.channels if c.channel == 17)
        self.assertLessEqual(camera.published, camera.received, 'encoded and decoded counted twice')
        rospy.set_param('/odin2_mock/MOCK1/pause_camera1', False)
        count = len(encoded)
        deadline = time.monotonic() + 3
        while len(encoded) < count + 2:
            self.assertLess(time.monotonic(), deadline)
            time.sleep(.01)
        rospy.set_param('/odin2_mock/MOCK1/pause_camera1', True)
        get_config = rospy.ServiceProxy('/async_camera/get_channel_config', GetChannelConfig)
        set_stream = rospy.ServiceProxy('/async_camera/set_stream', SetStream)
        config = get_config(17).effective
        config.enabled = False
        self.assertTrue(set_stream(config).result.success)
        config.enabled = True
        self.assertTrue(set_stream(config).result.success)
        time.sleep(.05)
        images.clear()
        time.sleep(.7)
        self.assertEqual(images, [], 'old generation decoded frames escaped after re-enable')
        image_sub.unregister()
        encoded_sub.unregister()


if __name__ == '__main__':
    rospy.init_node('async_decoder_acceptance')
    rostest.rosrun('odin2_ros_driver', 'async_decoder', AsyncDecoder)
