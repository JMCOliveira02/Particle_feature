#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import zmq
import json


class SendScanNode(Node):
    def __init__(self):
        super().__init__('send_scan_node')

        # ZMQ setup: PUSH → container’s PULL @127.0.0.1:5555
        ctx = zmq.Context()
        self._sender = ctx.socket(zmq.PUSH)
        self._sender.connect("tcp://127.0.0.1:5555")

        # Subscribe to the RGB-D cloud from Webots
        self.create_subscription(
            PointCloud2,
            '/rbgd_cloud',
            self.callback,
            10)

        self.get_logger().info("SendScanNode: listening on /rbgd_cloud")

    def callback(self, msg: PointCloud2):
        # 1) read_points gives (x,y,z,rgb_packed)
        raw = list(pc2.read_points(
            msg,
            field_names=('x', 'y', 'z', 'rgb'),
            skip_nans=True
        ))
        if not raw:
            return

        tuples = [(r[0], r[1], r[2], r[3]) for r in raw]
        arr = np.array(tuples, dtype=np.float32)

        xyz = arr[:, :3]                             # (N,3)
        rgb_packed = arr[:, 3].view(np.uint32)       # (N,)

        # unpack to separate channels
        r = ((rgb_packed >> 16) & 0xFF).astype(np.float32)
        g = ((rgb_packed >> 8) & 0xFF).astype(np.float32)
        b = (rgb_packed & 0xFF).astype(np.float32)

        # stack into (N,6): x,y,z,r,g,b
        cloud = np.vstack((xyz.T, r, g, b)).T        # (N,6)

        # 2) send header + binary payload
        header = {
            'dtype': str(cloud.dtype),
            'shape': cloud.shape
        }
        # multipart: header JSON, then raw bytes
        self._sender.send_json(header, zmq.SNDMORE)
        self._sender.send(cloud.tobytes(), copy=False)

        self.get_logger().debug(f"Sent cloud shape={cloud.shape}")


def main(args=None):
    rclpy.init(args=args)
    node = SendScanNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
