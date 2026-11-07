"""Isolated ROS2 integration test; synthetic Go2 data, no simulator or SRU."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class RuntimeTest(unittest.TestCase):
    def setUp(self):
        self.log = tempfile.TemporaryFile(mode='w+')
        self.proc = subprocess.Popen([BINARY, '--ros-args', '-p', 'observe:=true'],
                                     stdout=self.log, stderr=self.log)
        self.node = Node('thermalnav_synthetic_go2')
        self.dynamic = TransformBroadcaster(self.node)
        self.static = StaticTransformBroadcaster(self.node)
        self.pubs = {key: self.node.create_publisher(kind, topic, qos_profile_sensor_data)
                     for key, topic, kind in [
                         ('depth', '/robot0/front_cam/depth', Image),
                         ('rgb', '/robot0/front_cam/rgb', Image),
                         ('info', '/robot0/front_cam/camera_info', CameraInfo),
                         ('odom', '/robot0/odom', Odometry),
                         ('cmd', '/robot0/cmd_vel', Twist)]}
        self.status = None
        self.grid = None
        self.goals = []
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs = [
            self.node.create_subscription(String, '/thermalnav/status', self.on_status, q),
            self.node.create_subscription(OccupancyGrid, '/thermalnav/bev', self.on_grid, q),
            self.node.create_subscription(PoseStamped, '/goal_pose', self.goals.append, 10)]
        self.base = time.monotonic()
        self.stamp = 10.
        self.pose = (0., 0., 0.)
        self.frozen = False
        self.bad_info = False
        self.millimeters = False
        self.send_depth = True
        self.send_rgb = True
        self.send_tf = True
        self.send_odom = True
        self.invalid_depth = False
        self.timer = self.node.create_timer(.1, self.publish)
        self.mount = TransformStamped()
        self.mount.header.frame_id = 'robot0/base_link'
        self.mount.child_frame_id = 'robot0/front_cam'
        self.mount.transform.translation.x = .32487
        self.mount.transform.translation.y = -.00095
        self.mount.transform.translation.z = .05362
        q = self.mount.transform.rotation
        q.x, q.y, q.z, q.w = -.5, .5, -.5, .5
        self.static.sendTransform(self.mount)

    def tearDown(self):
        self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
        self.node.destroy_node()
        if self.proc.returncode:
            self.log.seek(0)
            print(self.log.read())
        self.log.close()

    def on_status(self, message):
        self.status = json.loads(message.data)

    def on_grid(self, message):
        self.grid = message

    def publish(self):
        if not self.frozen:
            self.stamp += .1
        s = rclpy.time.Time(seconds=self.stamp).to_msg()
        x, y, yaw = self.pose
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.header.stamp = s
        odom.child_frame_id = 'robot0/base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = .35
        odom.pose.pose.orientation.z = float(np.sin(yaw/2))
        odom.pose.pose.orientation.w = float(np.cos(yaw/2))
        if self.send_odom:
            self.pubs['odom'].publish(odom)
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = odom.child_frame_id
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = .35
        tf.transform.rotation = odom.pose.pose.orientation
        if self.send_tf:
            self.dynamic.sendTransform(tf)
        self.pubs['cmd'].publish(Twist())
        info = CameraInfo()
        info.header.frame_id = 'robot0/front_cam'
        info.header.stamp = s
        info.width, info.height = (320 if self.bad_info else 640), 360
        info.k = [733., 0., 320., 0., 733., 180., 0., 0., 1.]
        self.pubs['info'].publish(info)
        depth = np.full((360, 640), 6., np.float32)
        for v in range(181, 360):
            depth[v] = min(6., .40362*733/(v-180))
        if self.invalid_depth:
            depth.fill(0.)
        message = Image()
        message.header = info.header
        message.width, message.height = 640, 360
        if self.millimeters:
            depth = (depth*1000).astype('<u2')
            message.encoding, message.step = '16UC1', 640*2
        else:
            message.encoding, message.step = '32FC1', 640*4
        message.data = depth.tobytes()
        if self.send_depth:
            self.pubs['depth'].publish(message)
        rgb = Image()
        rgb.header = info.header
        rgb.width, rgb.height, rgb.encoding, rgb.step = 640, 360, 'rgb8', 640*3
        rgb.data = bytes(640*360*3)
        if self.send_rgb:
            self.pubs['rgb'].publish(rgb)

    def wait_for(self, condition, timeout=12):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.assertIsNone(self.proc.poll(), 'C++ runtime exited')
            rclpy.spin_once(self.node, timeout_sec=.05)
            if condition():
                return
        self.fail('Timed out; last status: '+str(self.status))

    def call(self, topic):
        client = self.node.create_client(Trigger, topic)
        self.assertTrue(client.wait_for_service(timeout_sec=5))
        future = client.call_async(Trigger.Request())
        self.wait_for(future.done)
        self.assertTrue(future.result().success)
        self.node.destroy_client(client)

    def test_world_mapping_timing_encodings_and_stop(self):
        self.wait_for(lambda: self.status and self.status['map_frames'] >= 3 and self.grid)
        self.assertEqual(self.status['runtime'], 'cpp')
        self.assertEqual(self.status['map_frame'], 'odom')
        self.assertEqual(self.status['selected'], 0)
        self.assertEqual(self.goals, [])
        old = self.grid
        self.assertTrue(set(old.data).issubset({-1, 0, 100}))
        self.assertGreater(old.data.count(100), 0)
        self.assertGreater(old.data.count(0), 0)
        # Camera mount + horizontal optical axis leaves the real near field unknown.
        def at(grid, x, y):
            col = int(np.floor((x-grid.info.origin.position.x)/grid.info.resolution))
            row = int(np.floor((y-grid.info.origin.position.y)/grid.info.resolution))
            return grid.data[row*grid.info.width+col]
        self.assertEqual(at(old, .5, 0), -1)
        checksum = self.status['occupancy_grid_checksum']
        self.pose = (1., 0., np.pi/2)
        self.wait_for(lambda: self.status['occupancy_grid_checksum'] != checksum and
                      self.status['last_depth']['camera_pose_odom_m'][1] > .3)
        self.wait_for(lambda: self.grid.header.stamp.sec > old.header.stamp.sec)
        new = self.grid
        # Previously observed world cells survive the robot translation/rotation.
        for row, col in np.argwhere(np.array(old.data).reshape(old.info.height, old.info.width) == 100)[::50]:
            x = old.info.origin.position.x+(col+.5)*old.info.resolution
            y = old.info.origin.position.y+(row+.5)*old.info.resolution
            self.assertEqual(at(new, x, y), 100)
        self.millimeters = True
        self.wait_for(lambda: self.status['last_depth'].get('encoding') == '16UC1')
        self.frozen = True
        self.wait_for(lambda: abs(self.status['last_depth']['stamp']-self.stamp) < 1e-6)
        frames = self.status['map_frames']
        end = time.monotonic()+2.5
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.05)
        self.assertEqual(self.status['map_frames'], frames, 'duplicate depth integrated twice')
        self.frozen = False
        self.call('/thermalnav/start')
        self.wait_for(lambda: self.status['enabled'])
        self.assertEqual(self.status['selected'], 0, 'blind zone must not be bypassed')
        self.call('/thermalnav/stop')
        self.wait_for(lambda: not self.status['enabled'] and self.goals)
        self.assertAlmostEqual(self.goals[-1].pose.position.x, self.pose[0])
        self.assertAlmostEqual(self.goals[-1].pose.position.y, self.pose[1])
        self.call('/thermalnav/start')
        self.invalid_depth = True
        self.wait_for(lambda: self.status['bev'].get('valid_depth') == 0 and 'stale sensors' in self.status['blocked'])
        self.invalid_depth = False
        self.wait_for(lambda: self.status['bev'].get('valid_depth', 0) > 0)
        self.send_depth = False
        self.wait_for(lambda: 'stale sensors' in self.status['blocked'], timeout=10)
        self.call('/thermalnav/stop')
        self.send_depth = True
        self.bad_info = True
        self.wait_for(lambda: 'depth/info mismatch' in self.status['blocked'])
        frames = self.status['map_frames']
        self.bad_info = False
        self.wait_for(lambda: self.status['map_frames'] > frames)
        self.assertNotIn('depth/info mismatch', self.status['blocked'])
        self.pose = (0., 0., 0.)
        self.stamp = .5
        self.wait_for(lambda: self.status['epoch'] == 1 and self.status['map_frames'] >= 2)
        self.assertLess(self.status['last_depth']['stamp'], 10.)
        self.assertEqual(self.status['completed'], 0)
        # A late-joining retained subscriber sees the world-frame map.
        received = []
        sub = self.node.create_subscription(OccupancyGrid, '/thermalnav/bev', received.append,
                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.wait_for(lambda: bool(received))
        self.assertEqual(received[-1].header.frame_id, 'odom')
        self.node.destroy_subscription(sub)
        print('PASS: best-effort RGB-D, stamped TF, translation/rotation history, '
              'blind-zone safety, 16UC1, duplicates, stale sensors, stop, reset, retained map')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', required=True)
    args = parser.parse_args()
    BINARY = str(Path(args.binary).resolve())
    os.environ['ROS_DOMAIN_ID'] = str(130+os.getpid()%60)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = '/tmp/thermalnav_test_ros'
    rclpy.init(args=[])
    try:
        result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(RuntimeTest))
    finally:
        rclpy.shutdown()
    raise SystemExit(0 if result.wasSuccessful() else 1)
