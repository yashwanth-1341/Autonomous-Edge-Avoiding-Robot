#!/usr/bin/env python3
import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, TransformException

from gz.transport13 import Node as GzNode
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.marker_pb2 import Marker as GzMarker
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_v_pb2 import StringMsg_V


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_rotate(q, v):
    x, y, z, _ = quat_multiply(quat_multiply(q, (*v, 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def transform_point(pose, p):
    """Apply pose = ((x, y, z), (qx, qy, qz, qw)) to point p."""
    r = quat_rotate(pose[1], p)
    return (pose[0][0] + r[0], pose[0][1] + r[1], pose[0][2] + r[2])


def compose(a, b):
    return (transform_point(a, b[0]), quat_multiply(a[1], b[1]))


class ScanRays(Node):
    """Draws each lidar beam as a line from the sensor to its hit point, in RViz and in Gazebo."""

    def __init__(self):
        super().__init__('scan_rays')
        self.declare_parameter('model_name', 'bot')
        self.model_name = self.get_parameter('model_name').value

        self.publisher = self.create_publisher(Marker, '/scan_rays', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Gazebo side: robot world pose comes from the simulator, rays go to its marker service
        self.gz_node = GzNode()
        self.world_name = None
        self.robot_pose = None

    def scan_callback(self, msg):
        rays = Marker()
        rays.header = msg.header
        rays.ns = 'scan_rays'
        rays.type = Marker.LINE_LIST
        rays.action = Marker.ADD
        rays.pose.orientation.w = 1.0
        rays.scale.x = 0.01  # line width (m)
        rays.color.g = 1.0
        rays.color.a = 0.9

        endpoints = []
        for i, r in enumerate(msg.ranges):
            # Beams that hit nothing are drawn out to max range
            r = min(max(r, msg.range_min), msg.range_max) if math.isfinite(r) else msg.range_max
            angle = msg.angle_min + i * msg.angle_increment
            endpoints.append((r * math.cos(angle), r * math.sin(angle), 0.0))
            rays.points.append(Point())
            rays.points.append(Point(x=endpoints[-1][0], y=endpoints[-1][1], z=0.0))

        self.publisher.publish(rays)
        self.draw_in_gazebo(msg.header.frame_id, endpoints)

    def draw_in_gazebo(self, sensor_frame, endpoints):
        if self.world_name is None and not self.connect_to_world():
            return
        if self.robot_pose is None:
            return
        try:
            # Sensor pose on the robot (static, from robot_state_publisher)
            t = self.tf_buffer.lookup_transform('base_footprint', sensor_frame, Time())
        except TransformException:
            return
        tr, rot = t.transform.translation, t.transform.rotation
        sensor_pose = compose(self.robot_pose, ((tr.x, tr.y, tr.z), (rot.x, rot.y, rot.z, rot.w)))

        marker = GzMarker()
        marker.ns = 'scan_rays'
        marker.id = 1
        marker.action = GzMarker.ADD_MODIFY
        marker.type = GzMarker.LINE_LIST
        for c in (marker.material.ambient, marker.material.diffuse, marker.material.emissive):
            c.r, c.g, c.b, c.a = 0.0, 1.0, 0.0, 1.0
        origin = sensor_pose[0]
        for end in endpoints:
            for p in (origin, transform_point(sensor_pose, end)):
                pt = marker.point.add()
                pt.x, pt.y, pt.z = p
        # The GUI's marker service doesn't send a reply, so don't wait long for one
        self.gz_node.request('/marker', marker, GzMarker, Empty, 50)

    def connect_to_world(self):
        ok, reply = self.gz_node.request('/gazebo/worlds', Empty(), Empty, StringMsg_V, 500)
        if not ok or not reply.data:
            return False
        self.world_name = reply.data[0]
        self.pose_topic = f'/world/{self.world_name}/pose/info'
        self.gz_node.subscribe(Pose_V, self.pose_topic, self.pose_callback)
        return True

    def pose_callback(self, msg):
        for p in msg.pose:
            if p.name == self.model_name:
                o = p.orientation
                self.robot_pose = ((p.position.x, p.position.y, p.position.z), (o.x, o.y, o.z, o.w))
                return

    def destroy_node(self):
        # gz-transport's Python bindings segfault at exit if a subscription is still active
        if self.world_name is not None:
            self.gz_node.unsubscribe(self.pose_topic)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScanRays()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
