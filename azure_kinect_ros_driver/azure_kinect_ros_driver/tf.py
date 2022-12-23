from typing import Optional, List, Dict, Any

import PyKDL
import tf2_ros
import geometry_msgs.msg
import rclpy.node
import rclpy.duration
import yaml


def transform_from_msg(msg: geometry_msgs.msg.Transform) -> PyKDL.Frame:
    position_msg = msg.translation
    pos = PyKDL.Vector(position_msg.x, position_msg.y, position_msg.z)
    quaterion_msg = msg.rotation
    rot = PyKDL.Rotation.Quaternion(quaterion_msg.x, quaterion_msg.y, quaterion_msg.z, quaterion_msg.w)
    return PyKDL.Frame(V=pos, R=rot)


def point(msg: geometry_msgs.msg.PointStamped, frame: PyKDL.Frame) -> PyKDL.Vector:
    return frame * PyKDL.Vector(msg.point.x, msg.point.y, msg.point.z)


def frame_from_msg(msg: geometry_msgs.msg.PoseStamped) -> PyKDL.Frame:
    q = msg.pose.orientation
    p = msg.pose.position
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w), PyKDL.Vector(p.x, p.y, p.z))


def pose(msg: geometry_msgs.msg.PoseStamped, frame: PyKDL.Frame) -> PyKDL.Frame:
    q = msg.pose.orientation
    p = msg.pose.position
    return frame * PyKDL.Frame(
        PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w), PyKDL.Vector(p.x, p.y, p.z))


def pose_msg(frame: PyKDL.Frame) -> geometry_msgs.msg.PoseStamped:
    msg = geometry_msgs.msg.PoseStamped()
    q = msg.pose.orientation
    p = msg.pose.position
    p.x, p.y, p.z = tuple(frame.p)
    q.x, q.y, q.z, q.w = frame.M.GetQuaternion()
    return msg



def _parent(frame: str, frames: Dict[str, Any]) -> Optional[str]:
    if frame not in frames:
        return None
    return frames[frame]['parent']


def _ancestors(frame: str, frames: Dict[str, Any]) -> List[str]:
    parent_frame = _parent(frame, frames)
    if parent_frame:
        return [parent_frame] + _ancestors(parent_frame, frames)
    return []

def _ancestor(frame: str, frames: Dict[str, Any], before: Optional[str] = None) -> str:
    ancestors = _ancestors(frame, frames)
    if before:
        if before in ancestors:
            i = ancestors.index(before)
            if i:
                return ancestors[i-1]
            return frame
    if ancestors:
        return ancestors[-1]
    return frame


# TODO(Jerome): use buffer.transform instead of PyKDL
class TF:

    def __init__(self, node: rclpy.node.Node) -> None:
        super().__init__()
        self.node = node
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(buffer=self.buffer, node=node)
        self.broadcaster = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster(node)

    def get_transform(self, from_frame: str, to_frame: str, at: Optional[rclpy.time.Time] = None,
                      timeout: float = 0.0) -> Optional[PyKDL.Frame]:
        if from_frame == to_frame:
            return PyKDL.Frame()
        if at is None:
            at = rclpy.time.Time()
        try:
            transform_msg = self.buffer.lookup_transform(
                to_frame, from_frame, at,
                timeout=rclpy.duration.Duration(nanoseconds=timeout * 1e9))
            return transform_from_msg(transform_msg.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            # self.node.get_logger().warn(f"{e}")
            return None

    def point(self, msg: geometry_msgs.msg.PointStamped, frame: str) -> Optional[PyKDL.Vector]:
        t = self.get_transform(msg.header.frame_id, frame)
        if t is None:
            return None
        return point(msg, t)

    def pose(self, msg: geometry_msgs.msg.PoseStamped, frame: str) -> Optional[PyKDL.Frame]:
        t = self.get_transform(msg.header.frame_id, frame)
        if t is None:
            return None
        pos = PyKDL.Vector(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        rot = PyKDL.Rotation.Quaternion(
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w)
        return t * PyKDL.Frame(rot, pos)

    def root(self, frame_id: str) -> str:
        frames = yaml.load(self.buffer.all_frames_as_yaml(), yaml.SafeLoader)
        return _ancestor(frame_id, frames, None)

    def parent(self, frame_id: str) -> Optional[str]:
        frames = yaml.load(self.buffer.all_frames_as_yaml(), yaml.SafeLoader)
        return _parent(frame_id, frames)

    def ancestors(self, frame_id: str) -> List[str]:
        frames = yaml.load(self.buffer.all_frames_as_yaml(), yaml.SafeLoader)
        return _ancestors(frame_id, frames)

    def is_ancestor(self, frame_id: str, of: str) -> bool:
        if frame_id == of:
            return True
        return frame_id in self.ancestors(of)

    def ancestor(self, frame_id: str, up_to: str) -> str:
        frames = yaml.load(self.buffer.all_frames_as_yaml(), yaml.SafeLoader)
        return _ancestor(frame_id, frames, up_to)

    def fix(self, parent_link: str, source_link: str, target_link: str) -> bool:
        try:
            msg = self.buffer.lookup_transform(parent_link, source_link, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.node.get_logger().warn(f"{e}")
            return False
        msg.child_frame_id = target_link
        self.node.get_logger().info(f"{msg}")
        self.broadcaster.sendTransform(msg)
        return True

    def set_transform(self, parent_link: str, target_link: str, transform: PyKDL.Frame) -> None:
        msg = geometry_msgs.msg.TransformStamped()
        msg.header.stamp = msg.header.stamp
        msg.header.frame_id = parent_link
        msg.child_frame_id = target_link
        q = transform.M.GetQuaternion()
        msg.transform.translation.x = transform.p.x()
        msg.transform.translation.y = transform.p.y()
        msg.transform.translation.z = transform.p.z()
        msg.transform.rotation.w = q[3]
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        self.node.get_logger().info(f"Sending transform {msg}")
        self.broadcaster.sendTransform(msg)
