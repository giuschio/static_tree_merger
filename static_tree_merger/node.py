# !/usr/bin/env python3

import numpy as np

from scipy.spatial.transform import Rotation as Rot

import rclpy

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener


class TreeMerger(Node):

    def __init__(self):
        super().__init__("static_tree_merger")
        # parent link in the main TF tree
        self.declare_parameter('parent_link', "")
        # child link in the "subordinate" TF tree (i.e. you have the child_to_parent transform)
        self.declare_parameter('child_link', "")
        # root link of the child link (i.e. root link of the "subordinate" tree)
        self.declare_parameter('child_link_root', "")

        # position(xyz) quaternion(xyzw)
        self.declare_parameter('child_to_parent_tf', "0 0 0 0 0 0 1")

        self.parent_link = str(self.get_parameter('parent_link').value)
        self.child_link = str(self.get_parameter('child_link').value)
        self.child_link_root = str(self.get_parameter('child_link_root').value)
        self.child_to_parent_tf = str(self.get_parameter('child_to_parent_tf').value)
        separator = "," if "," in self.child_to_parent_tf else " "
        self.child_to_parent_tf = [float(item) for item in self.child_to_parent_tf.split(sep=separator)]
        # Initialize the transform broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Transform listener.
        self.tf_buffer = Buffer()
        self._listener = TransformListener(self.tf_buffer, self)

        self.T_child_link_to_child_root = None

        self.wait_for_tf(target_frame=self.child_link_root,
                         source_frame=self.child_link,
                         time=rclpy.time.Time())
        self.T_child_link_to_child_root = self.get_tf(target_frame=self.child_link_root,
                                                      source_frame=self.child_link,
                                                      time=rclpy.time.Time())
        # Publish static transforms once at startup
        self.publish_transform()

    def wait_for_tf(self, target_frame, source_frame, time):
        self.get_logger().info("Waiting for transform "
                f"{self.child_link} -> {self.child_link_root}")
        self.future = self.tf_buffer.wait_for_transform_async(target_frame, source_frame, time)
        rclpy.spin_until_future_complete(self, self.future)
        return True

    def get_tf(self, target_frame, source_frame, time):
        M = np.eye(4)
        try:
            t = self.tf_buffer.lookup_transform(target_frame, source_frame, time)
            tr, qt = t.transform.translation, t.transform.rotation
            self.get_logger().info("Got " f"{source_frame} -> {target_frame}" " transform")
            self.get_logger().info(str(t))
            rt = Rot.from_quat([qt.x, qt.y, qt.z, qt.w]).as_matrix()
            M[:3, :3], M[0, 3], M[1, 3], M[2, 3] = rt[:, :], tr.x, tr.y, tr.z
        except TransformException as ex:
            self.get_logger().error("Could not get transform "
                f"{self.child_link} -> {self.child_link_root}")
        return M

    def publish_transform(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_link
        t.child_frame_id = self.child_link_root

        # get child-to-parent transform
        c2p = self.child_to_parent_tf
        rt = Rot.from_quat([c2p[3], c2p[4], c2p[5], c2p[6]]).as_matrix()        

        self.T_child_link_to_parent = np.eye(4)
        self.T_child_link_to_parent[:3, :3] = rt[:, :]
        self.T_child_link_to_parent[0, 3] = c2p[0]
        self.T_child_link_to_parent[1, 3] = c2p[1]
        self.T_child_link_to_parent[2, 3] = c2p[2]

        self.T_child_root_to_parent = self.T_child_link_to_parent @ np.linalg.inv(self.T_child_link_to_child_root)

        self.get_logger().info(f"{self.child_link} -> {self.parent_link}" " transform")
        self.get_logger().info(str(self.child_to_parent_tf))

        quat = Rot.from_matrix(self.T_child_root_to_parent[:3, :3]).as_quat()
        tr = self.T_child_root_to_parent[:3, 3].flatten()

        t.transform.translation.x = tr[0]
        t.transform.translation.y = tr[1]
        t.transform.translation.z = tr[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.get_logger().info(f"{self.child_link_root} -> {self.parent_link}" " transform")
        self.get_logger().info(str(t))
        self.get_logger().info("broadcasting to tf_static...")

        self.tf_static_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = TreeMerger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()