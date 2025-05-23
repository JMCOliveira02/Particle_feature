#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from robot_msgs.msg import Results
import zmq
import json
import numpy as np


class RecvResultsNode(Node):
    def __init__(self):
        super().__init__('recv_results_node')

        # 1) ZMQ: bind a PULL socket so the container's PUSH can connect to it
        ctx = zmq.Context()
        self._socket = ctx.socket(zmq.PULL)
        self._socket.connect("tcp://127.0.0.1:5556")

        # 2) ROS 2 publisher for aggregated segmentation Results
        self._pub_results = self.create_publisher(
            Results, '/seg/results', 10)

        # 3) Poll ZMQ at 50 Hz
        self.create_timer(0.02, self._on_timer)

        self.get_logger().info('RecvResultsNode: bound ZMQ PULL @127.0.0.1:5556')

    def _on_timer(self):
        # Attempt to read header non-blocking
        try:
            hdr = self._socket.recv_json(flags=zmq.NOBLOCK)
        except zmq.Again:
            return

        # Read binary payload (mask)
        buf = self._socket.recv()
        mask = np.frombuffer(buf,
                             dtype=hdr['mask_dtype']).reshape(hdr['mask_shape'])
        labels = np.array(hdr['labels'], dtype=np.int32)
        scores = np.array(hdr['scores'], dtype=np.float32)

        # Build and publish Results message
        msg = Results()
        msg.mask = mask.astype(np.int32).tolist()
        msg.labels = labels.tolist()
        msg.scores = scores.tolist()

        self._pub_results.publish(msg)
        self.get_logger().info(
            f'Published Results: mask({mask.shape}), '
            f'labels({labels.shape}), scores({scores.shape})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RecvResultsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
