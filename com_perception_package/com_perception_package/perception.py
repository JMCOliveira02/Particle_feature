#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from robot_msgs.msg import Results, Feature, FeatureArray
import numpy as np

s3dis_class_names = [
    'ceiling', 'floor', 'wall', 'beam', 'column', 'window', 'door',
    'table', 'chair', 'sofa', 'bookcase', 'board', 'clutter', 'unlabeled'
]


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self._cloud = None   # will hold an (N×6) numpy array

        # 1) subscribe to raw RGBD → unpack into Nx6 and cache
        self.create_subscription(
            PointCloud2, '/rbgd_cloud', self._cloud_cb, 10)

        # 2) subscribe to ZMQ‐segmentation results
        self.create_subscription(
            Results, '/seg/results', self._results_cb, 10)

        # 3) publisher for your final FeatureArray
        self._pub_feat = self.create_publisher(
            FeatureArray, '/segmented_features', 10)

    def _cloud_cb(self, msg: PointCloud2):
        raw = list(pc2.read_points(
            msg,
            field_names=('x', 'y', 'z', 'rgb'),
            skip_nans=True
        ))
        if not raw:
            return

        # rip out plain Python tuples of floats
        tuples = [(p[0], p[1], p[2], p[3]) for p in raw]
        arr = np.array(tuples, dtype=np.float32)  # now shape (N,4)

        xyz = arr[:, :3]                             # (N,3)
        rgb_packed = arr[:, 3].view(np.uint32)       # (N,)

        # unpack channels
        r = ((rgb_packed >> 16) & 0xFF).astype(np.float32)
        g = ((rgb_packed >> 8) & 0xFF).astype(np.float32)
        b = (rgb_packed & 0xFF).astype(np.float32)

        # build final (N,6)
        self._cloud = np.column_stack((xyz, r, g, b))

    def _results_cb(self, msg: Results):
        if self._cloud is None:
            self.get_logger().warn('No cloud yet, skipping results')
            return

        mask = np.array(msg.mask, dtype=np.int32)   # length N
        labels = np.array(msg.labels, dtype=np.int32)
        scores = np.array(msg.scores, dtype=np.float32)

        feats = []
        for inst_id in range(1, len(labels)+1):
            idxs = np.nonzero(mask == inst_id)[0]
            if idxs.size == 0:
                continue
            subcloud = self._cloud[idxs]              # shape (ni,6)
            centroid = subcloud[:, :3].mean(axis=0)
            # compute orientation…
            # build your Feature msg, etc.
            # feats.append(feature)

        # publish your FeatureArray…
        self._cloud = None   # clear for next cycle

        # print the Results
        self.get_logger().info(
            f'Published Results: mask({mask.shape}), '
            f'labels({labels.shape}), scores({scores.shape})'
        )
        self.get_logger().info(
            f'Published Features: {len(feats)} instances'
        )


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
