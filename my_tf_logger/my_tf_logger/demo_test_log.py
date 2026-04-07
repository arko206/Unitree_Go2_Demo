#!/usr/bin/env python3
import os
from datetime import datetime
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped

from scipy.spatial.transform import Rotation as R
import json


# ============================================================
# FIXED CALIBRATION MATRIX P*(Rc->T1)
# ============================================================
# P_star_Rc_to_T1 = np.array([
#     [0.998434, -0.051057, -0.022868, 0.002303],
#     [-0.023986, -0.021393, -0.999483, -0.047973],
#     [0.050541, 0.998467, -0.022584, -0.089412],
#     [0.000000, 0.000000, 0.000000, 1.000000]
# ])

# P_star_Rc_to_T1 = np.array([
#     [0.993767, -0.110441, -0.015138, -0.003802],
# [-0.016527, -0.011670 ,-0.999795, -0.048409],
# [0.110242, 0.993814, -0.013423, -0.090582],
# [0.000000, 0.000000 ,0.000000, 1.000000]
# ])

# P_star_Rc_to_T1 = np.array([
#     [0.999212, -0.038571, 0.009362, 0.026219],
#     [0.008859, -0.013171, -0.999874, -0.046079],
#     [0.038689, 0.999169, -0.012819, -0.095978],
#     [0.000000, 0.000000, 0.000000, 1.000000]
# ])

P_star_Rc_to_T1 = np.array([
    [0.998833, 0.039814, -0.027331, 0.008661],
    [-0.025861, -0.036982, -0.998981, -0.049915],
    [-0.040784, 0.998523, -0.035909, -0.107167],
    [0.000000, 0.000000, 0.000000, 1.000000]
])






# ============================================================
# HELPERS
# ============================================================
def quat_to_rot(qx, qy, qz, qw):
    """Quaternion (x,y,z,w) -> rotation matrix and RPY."""
    quat_array = np.array([qx, qy, qz, qw], dtype=float)
    rotation = R.from_quat(quat_array)
    rpy = rotation.as_euler('xyz', degrees=False)
    return rotation.as_matrix(), rpy


def translation_error(prev_T, curr_T):
    prev_p = prev_T[:3, 3]
    curr_p = curr_T[:3, 3]
    return np.linalg.norm(curr_p - prev_p)


def rotation_error(prev_T, curr_T):
    prev_R = prev_T[:3, :3]
    curr_R = curr_T[:3, :3]

    R_err = prev_R.T @ curr_R
    trace_val = np.trace(R_err)

    cos_theta = (trace_val - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)

    return np.arccos(cos_theta)


def rpy_error(prev_rpy, curr_rpy):
    """Absolute component-wise RPY difference."""
    prev_rpy = np.array(prev_rpy, dtype=float)
    curr_rpy = np.array(curr_rpy, dtype=float)
    return np.abs(curr_rpy - prev_rpy)


def euclidean_distance_from_camera(T_matrix):
    p = T_matrix[:3, 3]
    return np.linalg.norm(p)


# ============================================================
# NODE
# ============================================================
class TFLogger(Node):
    def __init__(self):
        super().__init__('tf_logger')

        # --------------------------------------------------
        # PARAMETERS
        # --------------------------------------------------
        self.declare_parameter('parent_frame', 'camera')
        self.declare_parameter('child_frame', 'object_1')
        self.declare_parameter('intermediate_frame', 'object_2')
        self.declare_parameter('goal_frame', 'object_3')
        self.declare_parameter('floor_frame', 'floor')
        self.declare_parameter('second_obstacle', 'new_obstacle')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('deque_maxlen', 1000)

        self.declare_parameter(
            'bridge_pose_file',
            '/home/arka/unitree_sdk2/live_bridge/Fifth_DraftDemo_floor_robot_pose.txt'
        )

        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        self.intermediate_frame = self.get_parameter('intermediate_frame').get_parameter_value().string_value
        self.goal_frame = self.get_parameter('goal_frame').get_parameter_value().string_value
        self.floor_frame = self.get_parameter('floor_frame').get_parameter_value().string_value
        self.second_obstacle_frame = self.get_parameter('second_obstacle').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.deque_maxlen = self.get_parameter('deque_maxlen').get_parameter_value().integer_value
        self.bridge_pose_file = self.get_parameter('bridge_pose_file').get_parameter_value().string_value

        # --------------------------------------------------
        # TF BUFFER / LISTENER / BROADCASTER
        # --------------------------------------------------
        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.listener = tf2_ros.TransformListener(self.buffer, self, spin_thread=False)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # --------------------------------------------------
        # FLOOR LOCK STATE (Transformation from camera to floor)
        # --------------------------------------------------
        self.last_valid_C_to_floor = None
        self.have_floor_lock = False

        ###-- Object-1 locking state -----####
        self.last_valid_C_to_object1 = None

        # --------------------------------------------------
        # DEQUES
        # --------------------------------------------------
        maxlen = None if self.deque_maxlen <= 0 else self.deque_maxlen

        self.cam_to_obj1_history = deque(maxlen=maxlen)
        self.cam_to_obj2_history = deque(maxlen=maxlen)
        self.cam_to_obj3_history = deque(maxlen=maxlen)
        self.cam_to_base_history = deque(maxlen=maxlen)
        self.cam_to_floor_history = deque(maxlen=maxlen)
        self.cam_to_secondobstacle_history = deque(maxlen=maxlen)

        self.floor_to_baselink_history = deque(maxlen=maxlen)
        self.floor_to_object2_history = deque(maxlen=maxlen)
        self.floor_to_object3_history = deque(maxlen=maxlen)
        self.floor_to_secondobstacle_history = deque(maxlen=maxlen)

        self.relative_distance_history = deque(maxlen=maxlen)

        # --------------------------------------------------
        # TIMER
        # --------------------------------------------------
        period = 1.0 / max(self.rate_hz, 0.1)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(
            f"Logging TF '{self.parent_frame}' -> '{self.child_frame}' at {self.rate_hz:.1f} Hz"
        )
        self.get_logger().info("Using deques for live transform history.")
        self.get_logger().info(
            "Deques initialized for: camera->object_1, camera->object_2, camera->object_3, camera->base_link"
        )
        self.get_logger().info("Floor latch enabled: last valid camera->floor will be reused if floor tag disappears.")

        self.json_save_counter = 0
        self.json_save_every_n_ticks = 10

    # =======================================================
    # DEQUE STORAGE
    # =======================================================
    def append_relative_distance_history(self, timestamp_str, p_base, p_obj2, p_obj3):
        dist_base_to_obj2 = float(np.linalg.norm(p_obj2 - p_base))
        dist_base_to_obj3 = float(np.linalg.norm(p_obj3 - p_base))
        dist_obj2_to_obj3 = float(np.linalg.norm(p_obj3 - p_obj2))

        entry = {
            "timestamp": timestamp_str,
            "base_link_to_object_2": dist_base_to_obj2,
            "base_link_to_object_3": dist_base_to_obj3,
            "object_2_to_object_3": dist_obj2_to_obj3,
        }

        self.relative_distance_history.append(entry)

        # self.get_logger().info(
        #     f"[relative_distances] "
        #     f"base->obj2={dist_base_to_obj2:.6f} m | "
        #     f"base->obj3={dist_base_to_obj3:.6f} m | "
        #     f"obj2->obj3={dist_obj2_to_obj3:.6f} m"
        # )

    def append_transform_history(self, history_deque, name, T_matrix, timestamp_str):
        tx, ty, tz = T_matrix[:3, 3]

        quat = R.from_matrix(T_matrix[:3, :3]).as_quat()
        _, rpy = quat_to_rot(quat[0], quat[1], quat[2], quat[3])

        euclidean_distance = euclidean_distance_from_camera(T_matrix)

        if len(history_deque) == 0:
            trans_err = 0.0
            rot_err = 0.0
            rpy_err = np.array([0.0, 0.0, 0.0])
        else:
            prev_entry = history_deque[-1]
            prev_T = prev_entry["T"]
            prev_rpy = prev_entry["rotation_rpy"]

            trans_err = translation_error(prev_T, T_matrix)
            rot_err = rotation_error(prev_T, T_matrix)
            rpy_err = rpy_error(prev_rpy, rpy)

        entry = {
            "timestamp": timestamp_str,
            "name": name,
            "translation": [tx, ty, tz],
            "rotation_rpy": [rpy[0], rpy[1], rpy[2]],
            "euclidean_distance": float(euclidean_distance),
            "T": T_matrix.copy(),
            "translation_error": float(trans_err),
            "rotation_error": float(rot_err),
            "rotation_error_deg": float(np.degrees(rot_err)),
            "rpy_error": [float(rpy_err[0]), float(rpy_err[1]), float(rpy_err[2])],
        }

        history_deque.append(entry)

        # self.get_logger().info(
        #     f"[{name}] "
        #     f"trans=({tx:.6f}, {ty:.6f}, {tz:.6f}) | "
        #     f"dist_from_camera={euclidean_distance:.6f} m | "
        #     f"rpy=({rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f}) | "
        #     f"Δtrans={trans_err:.6f} m | "
        #     f"Δrot={rot_err:.6f} rad ({np.degrees(rot_err):.3f} deg)"
        # )

    def get_deque_data(self, deque_name):
        deques = {
            'camera_to_object1': self.cam_to_obj1_history,
            'camera_to_object2': self.cam_to_obj2_history,
            'camera_to_object3': self.cam_to_obj3_history,
            'camera_to_baselink': self.cam_to_base_history,
            'camera_to_secondobstacle': self.cam_to_secondobstacle_history,
            'relative_distances': self.relative_distance_history,
            'floor_to_baselink': self.floor_to_baselink_history,
            'floor_to_object2': self.floor_to_object2_history,
            'floor_to_object3': self.floor_to_object3_history,
            'floor_to_secondobstacle': self.floor_to_secondobstacle_history
        }

        if deque_name in deques:
            return list(deques[deque_name])
        return None

    def print_deque_summary(self, deque_name):
        data = self.get_deque_data(deque_name)
        if data is None:
            self.get_logger().info(f"Invalid deque name: {deque_name}")
            return

        self.get_logger().info(f"\n========== {deque_name.upper()} DEQUE SUMMARY ==========")
        self.get_logger().info(f"Total entries: {len(data)}")

        if len(data) > 0:
            latest = data[-1]
            self.get_logger().info(f"Latest timestamp: {latest['timestamp']}")

            if deque_name == "relative_distances":
                self.get_logger().info(f"base_link_to_object_2: {latest['base_link_to_object_2']}")
                self.get_logger().info(f"base_link_to_object_3: {latest['base_link_to_object_3']}")
                self.get_logger().info(f"object_2_to_object_3: {latest['object_2_to_object_3']}")
            else:
                self.get_logger().info(f"Translation (x,y,z): {latest['translation']}")
                self.get_logger().info(f"Euclidean distance from camera: {latest['euclidean_distance']}")
                self.get_logger().info(f"Rotation (r,p,y) rad: {latest['rotation_rpy']}")
                self.get_logger().info(f"Translation Error: {latest['translation_error']}")
                self.get_logger().info(f"Rotation Error (rad): {latest['rotation_error']}")
                self.get_logger().info(f"Rotation Error (deg): {latest['rotation_error_deg']}")
                self.get_logger().info(f"RPY Error: {latest['rpy_error']}")

    # =======================================================
    # SAVE JSON
    # =======================================================
    def save_deques_to_json(self):
        save_dir = "/home/arka/Desktop/Deque_Debug_Demo_Outputs"
        os.makedirs(save_dir, exist_ok=True)

        data_map = {
            "camera_to_object_1.json": list(self.cam_to_obj1_history),
            "camera_to_object_2.json": list(self.cam_to_obj2_history),
            "camera_to_object_3.json": list(self.cam_to_obj3_history),
            "camera_to_base_link.json": list(self.cam_to_base_history),
            "camera_to_floor.json": list(self.cam_to_floor_history),
            "camera_to_secondobstacle.json":list(self.cam_to_secondobstacle_history),
            "floor_to_base_link.json": list(self.floor_to_baselink_history),
            "floor_to_object_2.json": list(self.floor_to_object2_history),
            "floor_to_object_3.json": list(self.floor_to_object3_history),
            "floor_to_secondobstacle.json": list(self.floor_to_secondobstacle_history),
            "relative_distances.json": list(self.relative_distance_history),
        }

        for filename, data in data_map.items():
            serializable_data = []
            for entry in data:
                serializable_entry = dict(entry)
                if "T" in serializable_entry and isinstance(serializable_entry["T"], np.ndarray):
                    serializable_entry["T"] = serializable_entry["T"].tolist()
                serializable_data.append(serializable_entry)

            with open(os.path.join(save_dir, filename), "w") as f:
                json.dump(serializable_data, f, indent=2)

    # =======================================================
    # BROADCAST HELPER
    # =======================================================
    def broadcast_transform(self, parent, child, T_matrix, stamp_msg):
        msg = TransformStamped()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(T_matrix[0, 3])
        msg.transform.translation.y = float(T_matrix[1, 3])
        msg.transform.translation.z = float(T_matrix[2, 3])

        quat = R.from_matrix(T_matrix[:3, :3]).as_quat()
        msg.transform.rotation.x = float(quat[0])
        msg.transform.rotation.y = float(quat[1])
        msg.transform.rotation.z = float(quat[2])
        msg.transform.rotation.w = float(quat[3])

        self.broadcaster.sendTransform(msg)

    # =======================================================
    # MAIN TICK
    # =======================================================
    def tick(self):
        # --------------------------------------------------
        # CAMERA -> OBJECT_1
        # --------------------------------------------------
        try:
            t = self.buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                Time(),
                timeout=Duration(seconds=0.2)
            )
        except Exception as e:
            self.get_logger().warn(f"TF for object_1 not available yet: {e}")
            return

        common_time = Time.from_msg(t.header.stamp)
        stamp_msg = common_time.to_msg()
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        tr = t.transform.translation
        q = t.transform.rotation
        rot_matrix, roll_pitch_yaw_radians = quat_to_rot(q.x, q.y, q.z, q.w)

        T = np.eye(4, dtype=float)
        T[:3, :3] = rot_matrix
        T[:3, 3] = np.array([tr.x, tr.y, tr.z])

        # --------------------------------------------------
        # CAMERA -> OBJECT_2
        # --------------------------------------------------
        try:
            t2 = self.buffer.lookup_transform(
                self.parent_frame,
                self.intermediate_frame,
                common_time,
                timeout=Duration(seconds=0.2)
            )
        except Exception as e:
            self.get_logger().warn(f"TF for object_2 not available at common time: {e}")
            return

        tr2 = t2.transform.translation
        q2 = t2.transform.rotation
        rot_matrix2, _ = quat_to_rot(q2.x, q2.y, q2.z, q2.w)

        T2 = np.eye(4, dtype=float)
        T2[:3, :3] = rot_matrix2
        T2[:3, 3] = np.array([tr2.x, tr2.y, tr2.z])

        # --------------------------------------------------
        # CAMERA -> OBJECT_3
        # --------------------------------------------------
        try:
            t3 = self.buffer.lookup_transform(
                self.parent_frame,
                self.goal_frame,
                common_time,
                timeout=Duration(seconds=0.2)
            )
        except Exception as e:
            self.get_logger().warn(f"TF for object_3 not available at common time: {e}")
            return

        tr3 = t3.transform.translation
        q3 = t3.transform.rotation
        rot_matrix3, _ = quat_to_rot(q3.x, q3.y, q3.z, q3.w)

        T3 = np.eye(4, dtype=float)
        T3[:3, :3] = rot_matrix3
        T3[:3, 3] = np.array([tr3.x, tr3.y, tr3.z])

        # --------------------------------------------------
        # CAMERA -> FLOOR (MEASUREMENT, MAY DISAPPEAR)
        # --------------------------------------------------
        T_floor = None
        floor_visible_now = False

        try:
            t_floor = self.buffer.lookup_transform(
                self.parent_frame,
                self.floor_frame,
                common_time,
                timeout=Duration(seconds=0.2)
            )

            tr_floor = t_floor.transform.translation
            q_floor = t_floor.transform.rotation
            rot_matrix_floor, _ = quat_to_rot(q_floor.x, q_floor.y, q_floor.z, q_floor.w)

            T_floor = np.eye(4, dtype=float)
            T_floor[:3, :3] = rot_matrix_floor
            T_floor[:3, 3] = np.array([tr_floor.x, tr_floor.y, tr_floor.z])

           
            ##--copying the transformation from camera to floor---##
            self.last_valid_C_to_floor = T_floor.copy()

            ##-- when getting the transformations from floor to camera---##
            ##-- floor lock is set to True---###
            self.have_floor_lock = True
            floor_visible_now = True

            self.get_logger().info("Floor tag visible: updating latched camera->floor transform.")

        except Exception as e:
            if self.have_floor_lock:
                ##--getting the transformations from camera to floor
                T_floor = self.last_valid_C_to_floor.copy()
                self.get_logger().warn(
                    f"TF for camera to floor not available right now: {e}. "
                    f"Using latched camera->floor transform."
                )
            else:
                self.get_logger().warn(
                    f"TF for floor not available and no floor lock yet: {e}"
                )
                return
            


        ####----Camera --> NewObstacle ------####
        # --------------------------------------------------
        try:
            t4 = self.buffer.lookup_transform(
                self.parent_frame,
                self.second_obstacle_frame,
                common_time,
                timeout=Duration(seconds=0.2)
            )
        except Exception as e:
            self.get_logger().warn(f"TF for second obstacle not available at common time: {e}")
            return

        tr4 = t4.transform.translation
        q4 = t4.transform.rotation
        rot_matrix4, _ = quat_to_rot(q4.x, q4.y, q4.z, q4.w)

        T4 = np.eye(4, dtype=float)
        T4[:3, :3] = rot_matrix4
        T4[:3, 3] = np.array([tr4.x, tr4.y, tr4.z])


        # self.get_logger().info(
        #     f"Common time = {t.header.stamp.sec}.{t.header.stamp.nanosec:09d}, "
        #     f"object_2 time = {t2.header.stamp.sec}.{t2.header.stamp.nanosec:09d}, "
        #     f"object_3 time = {t3.header.stamp.sec}.{t3.header.stamp.nanosec:09d}"
        # )

        # --------------------------------------------------
        # CAMERA -> TAG-T1
        # --------------------------------------------------
        P_C_to_T1 = T

        # --------------------------------------------------
        # BASE -> HEAD_UPPER
        # --------------------------------------------------
        P_base_to_Head_Upper = np.eye(4)
        rot_matrix_base_to_Head_upper, _ = quat_to_rot(0, 0, 0, 1)
        P_base_to_Head_Upper[:3, :3] = rot_matrix_base_to_Head_upper
        P_base_to_Head_Upper[:3, 3] = np.array([0.285, 0, 0.01])

        # --------------------------------------------------
        # HEAD_UPPER -> Rc
        # --------------------------------------------------
        P_Head_Upper_to_Rc = np.eye(4)
        rot_matrix_Head_Upper_to_Rc, _ = quat_to_rot(-0.5, 0.500002, -0.5, 0.499998)
        P_Head_Upper_to_Rc[:3, :3] = rot_matrix_Head_Upper_to_Rc
        P_Head_Upper_to_Rc[:3, 3] = np.array([0.045, 0, 0.03])

        # --------------------------------------------------
        # BASE -> Rc / BASE -> T1
        # --------------------------------------------------
        P_base_to_Rc = P_base_to_Head_Upper @ P_Head_Upper_to_Rc
        P_base_to_T1 = P_base_to_Head_Upper @ P_Head_Upper_to_Rc @ P_star_Rc_to_T1

        # --------------------------------------------------
        # T1 -> Rc / T1 -> BASE
        # --------------------------------------------------
        P_T1_to_Rc = np.linalg.inv(P_star_Rc_to_T1)
        P_T1_to_base = P_T1_to_Rc @ np.linalg.inv(P_base_to_Rc)

        # --------------------------------------------------
        # CAMERA -> BASE
        # --------------------------------------------------
        P_C_to_base = P_C_to_T1 @ P_T1_to_base

        # --------------------------------------------------
        # FLOOR -> BASE / OBJECTS
        # --------------------------------------------------
         # floor -> camera
        P_floor_to_C = np.linalg.inv(T_floor)
        P_floor_to_base = P_floor_to_C @ P_C_to_base
        quat_floor_to_base = R.from_matrix(P_floor_to_base[:3, :3]).as_quat()
        _, rpy_floor_to_base = quat_to_rot(
            quat_floor_to_base[0], quat_floor_to_base[1], quat_floor_to_base[2], quat_floor_to_base[3]
        )


        P_floor_to_obj2 = P_floor_to_C @ T2
        P_floor_to_obj3 = P_floor_to_C @ T3
        P_floor_to_secondobstacle = P_floor_to_C @T4

        # --------------------------------------------------
        # BRIDGE FILE FOR DOCKER
        # --------------------------------------------------
        # --------------------------------------------------
        # BRIDGE FILE FOR DOCKER
        # --------------------------------------------------
        bridge_dir = os.path.dirname(self.bridge_pose_file)
        if bridge_dir:
            os.makedirs(bridge_dir, exist_ok=True)

        x_bridge = float(P_floor_to_base[0, 3])
        y_bridge = float(P_floor_to_base[1, 3])
        theta_bridge = float(np.arctan2(P_floor_to_base[1, 0], P_floor_to_base[0, 0]))

        with open(self.bridge_pose_file, 'w') as f:
            f.write(f"{x_bridge:.6f} {y_bridge:.6f} {theta_bridge:.6f}\n")

        # self.get_logger().info(
        #     f"Bridge pose written: x={x_bridge:.6f}, y={y_bridge:.6f}, theta={theta_bridge:.6f}"
        # )
        # --------------------------------------------------
        # BROADCASTS
        # --------------------------------------------------
        # floor -> camera
        self.broadcast_transform(self.floor_frame, self.parent_frame, P_floor_to_C, stamp_msg)

        # camera -> base_link
        self.broadcast_transform(self.parent_frame, "base_link", P_C_to_base, stamp_msg)

        # camera -> object_1
        self.broadcast_transform(self.parent_frame, self.child_frame, P_C_to_T1, stamp_msg)

        # camera -> object_2
        self.broadcast_transform(self.parent_frame, self.intermediate_frame, T2, stamp_msg)

        # camera -> object_3
        self.broadcast_transform(self.parent_frame, self.goal_frame, T3, stamp_msg)

        self.broadcast_transform(self.parent_frame, self.second_obstacle_frame, T4, stamp_msg)

        # optional persistent floor-based frames
        self.broadcast_transform(self.floor_frame, "base_link_floor", P_floor_to_base, stamp_msg)
        self.broadcast_transform(self.floor_frame, "object_2_floor", P_floor_to_obj2, stamp_msg)
        self.broadcast_transform(self.floor_frame, "object_3_floor", P_floor_to_obj3, stamp_msg)
        self.broadcast_transform(self.floor_frame, "new_obstacle_floor", P_floor_to_secondobstacle, stamp_msg)

        # --------------------------------------------------
        # APPEND TO DEQUES
        # --------------------------------------------------
        self.append_transform_history(
            self.cam_to_obj1_history,
            "camera_to_object_1",
            T,
            ts
        )

        self.append_transform_history(
            self.cam_to_obj2_history,
            "camera_to_object_2",
            T2,
            ts
        )

        self.append_transform_history(
            self.cam_to_obj3_history,
            "camera_to_object_3",
            T3,
            ts
        )

        self.append_transform_history(
            self.cam_to_base_history,
            "camera_to_base_link",
            P_C_to_base,
            ts
        )

        self.append_transform_history(
            self.cam_to_secondobstacle_history,
            "camera_to_second_obstacle",
            T4,
            ts
        )



        self.append_transform_history(
            self.floor_to_secondobstacle_history,
            "floor_to_secondobstacle",
            P_floor_to_secondobstacle,
            ts
        )

        if floor_visible_now and T_floor is not None:
            self.append_transform_history(
                self.cam_to_floor_history,
                "camera_to_floor",
                T_floor,
                ts
            )
        else:
            # store the implied camera->floor from the latched transform
            T_C_to_floor_latched = self.last_valid_C_to_floor
            self.append_transform_history(
                self.cam_to_floor_history,
                "camera_to_floor_latched",
                T_C_to_floor_latched,
                ts
            )

        self.append_transform_history(
            self.floor_to_baselink_history,
            "floor_to_baselink",
            P_floor_to_base,
            ts
        )

        self.append_transform_history(
            self.floor_to_object2_history,
            "floor_to_object2",
            P_floor_to_obj2,
            ts
        )

        self.append_transform_history(
            self.floor_to_object3_history,
            "floor_to_object3",
            P_floor_to_obj3,
            ts
        )

        # relative distances in floor frame
        p_base_floor = P_floor_to_base[:3, 3]
        p_obj2_floor = P_floor_to_obj2[:3, 3]
        p_obj3_floor = P_floor_to_obj3[:3, 3]

        self.append_relative_distance_history(
            ts,
            p_base_floor,
            p_obj2_floor,
            p_obj3_floor
        )

        self.json_save_counter += 1
        if self.json_save_counter >= self.json_save_every_n_ticks:
            self.save_deques_to_json()
            self.json_save_counter = 0


# ============================================================
# MAIN
# ============================================================
def main():
    rclpy.init()
    node = TFLogger()
    try:
        rclpy.spin(node)
    finally:
        node.save_deques_to_json()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()