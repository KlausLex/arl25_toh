#!/usr/bin/env python3
import sys
import os

# --- Path Setup ---
# Get the directory containing the current script (assignment_2)
current_script_dir = os.path.dirname(os.path.abspath(__file__))
# Get the parent directory (my_scripts)
parent_dir = os.path.dirname(current_script_dir)
# Add the parent directory to sys.path so Python can find assignment_1
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

# --- Standard Imports ---
import rospy
import tf
from tf.transformations import quaternion_from_matrix
import numpy as np
import geometry_msgs.msg
import time
import pickle
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # ADDED
from scipy.interpolate import interp1d
import threading

# --- Imports from Sibling Directory (assignment_1) ---
try:
    from assignment_1.dmp_motions import (
        DMPMotionGenerator,
        ROSTrajectoryPublisher as BaseROSTrajectoryPublisher,
        interpolate_joint_trajectory,
        animation_callback
    )
except ImportError as e:
    print(f"ERROR: Failed to import from assignment_1.dmp_motions: {e}")
    print("Ensure assignment_1 directory and dmp_motions.py exist and parent directory is in sys.path.")
    print(f"Current sys.path includes: {parent_dir}")
    exit()

# --- Imports for DMP/Kinematics/Visualization ---
try:
    import pytransform3d.visualizer as pv
    import pytransform3d.trajectories as ptr
    from movement_primitives.kinematics import Kinematics
    from movement_primitives.dmp import CartesianDMP
except ImportError as e:
    log_func = rospy.logerr if rospy.core.is_initialized() else print
    log_func(f"Failed to import dependencies: {e}. Make sure pytransform3d and movement_primitives are installed.")
    exit()

# --- Helper function for manual PQS conversion ---
def _manual_pqs_from_transform(transform_matrix):
    position = transform_matrix[0:3, 3]
    q_tf = quaternion_from_matrix(transform_matrix)
    quaternion_pqs = np.array([q_tf[3], q_tf[0], q_tf[1], q_tf[2]])
    return np.concatenate((position, quaternion_pqs))

def get_cube_number_from_name(cube_name):
    if not isinstance(cube_name, str) or not cube_name.startswith("cube_"):
        return -1
    try:
        return int(cube_name.split('_')[-1])
    except (ValueError, IndexError):
        return -1

# --- Renamed and Enhanced Class: ROSTrajectoryHandler ---
class ROSTrajectoryHandler(BaseROSTrajectoryPublisher):
    def __init__(self,
                 arm_joint_names_for_publishing,
                 gripper_joint_names_for_publishing,
                 all_joint_names_for_subscription,
                 arm_publish_topic_name='/open_manipulator_6dof/arm_controller/command',
                 gripper_publish_topic_name='/open_manipulator_6dof/gripper_controller/command',
                 publish_rate_hz=20,
                 subscribe_joint_state_topic="/joint_states"):

        self.arm_joint_names = list(arm_joint_names_for_publishing)
        self.gripper_joint_names = list(gripper_joint_names_for_publishing)
        self.rate_hz = publish_rate_hz

        self.arm_publisher = rospy.Publisher(arm_publish_topic_name, JointTrajectory, queue_size=10)
        self.gripper_publisher = rospy.Publisher(gripper_publish_topic_name, JointTrajectory, queue_size=10)
        
        rospy.loginfo(f"[ROS Handler] Arm Publisher ready on topic '{arm_publish_topic_name}'")
        rospy.loginfo(f"[ROS Handler] Gripper Publisher ready on topic '{gripper_publish_topic_name}'")

        self.joint_state_topic = subscribe_joint_state_topic
        self._latest_joint_data = {
            "name": [], "position": [], "velocity": [], "effort": []
        }
        self._joint_state_lock = threading.Lock()
        self._joint_name_to_index = {}
        self.ordered_joint_names = list(all_joint_names_for_subscription) 

        try:
            self.joint_state_subscriber = rospy.Subscriber(
                self.joint_state_topic,
                JointState,
                self._joint_state_callback,
                queue_size=1
            )
            rospy.loginfo(f"[ROS Handler] Subscribed to joint states on '{self.joint_state_topic}'")
        except Exception as e:
            rospy.logerr(f"[ROS Handler] Failed to subscribe to {self.joint_state_topic}: {e}")
            self.joint_state_subscriber = None

    def _joint_state_callback(self, msg):
        with self._joint_state_lock:
            self._latest_joint_data["name"] = list(msg.name)
            self._latest_joint_data["position"] = list(msg.position)
            if len(msg.velocity) == len(msg.name):
                self._latest_joint_data["velocity"] = list(msg.velocity)
            else:
                self._latest_joint_data["velocity"] = [0.0] * len(msg.name)
            if len(msg.effort) == len(msg.name):
                self._latest_joint_data["effort"] = list(msg.effort)
            else:
                self._latest_joint_data["effort"] = [0.0] * len(msg.name)
            self._joint_name_to_index = {name: i for i, name in enumerate(msg.name)}

    def get_joint_states(self, desired_joint_order=None):
        if self.joint_state_subscriber is None and not self._latest_joint_data["name"]:
            rospy.logwarn_throttle(5.0, "[ROS Handler] Cannot get joint states: subscriber not available or no data received.")
            return None
        with self._joint_state_lock:
            if not self._latest_joint_data["name"]:
                rospy.logwarn_throttle(5.0, "[ROS Handler] No joint state data received yet.")
                return None
            target_order = desired_joint_order if desired_joint_order is not None else self.ordered_joint_names
            ordered_positions = []
            ordered_velocities = []
            ordered_efforts = []
            for name in target_order:
                if name in self._joint_name_to_index:
                    idx = self._joint_name_to_index[name]
                    ordered_positions.append(self._latest_joint_data["position"][idx])
                    ordered_velocities.append(self._latest_joint_data["velocity"][idx])
                    ordered_efforts.append(self._latest_joint_data["effort"][idx])
                else:
                    rospy.logwarn_throttle(10.0, f"[ROS Handler] Joint '{name}' not found in latest joint state message. Appending None.")
                    ordered_positions.append(None)
                    ordered_velocities.append(None)
                    ordered_efforts.append(None)
            return {
                "name": list(target_order),
                "position": ordered_positions,
                "velocity": ordered_velocities,
                "effort": ordered_efforts
            }

    def publish_trajectory(self, arm_trajectory_points, gripper_trajectory_points_1d, timestamps, execute_time_factor=1.0):
        if len(arm_trajectory_points) == 0:
            rospy.logwarn("[ROS Handler] Empty arm trajectory provided. Cannot publish.")
            return
        if len(gripper_trajectory_points_1d) == 0:
            rospy.logwarn("[ROS Handler] Empty gripper trajectory provided. Cannot publish.")
        if len(arm_trajectory_points) != len(gripper_trajectory_points_1d) or len(arm_trajectory_points) != len(timestamps):
            rospy.logerr("[ROS Handler] Trajectory components have mismatched lengths. Cannot publish.")
            rospy.logerr(f"  Arm: {len(arm_trajectory_points)}, Gripper: {len(gripper_trajectory_points_1d)}, Timestamps: {len(timestamps)}")
            return

        rospy.loginfo(f"[ROS Handler] Publishing trajectories with {len(timestamps)} points.")

        arm_msg = JointTrajectory()
        arm_msg.header.stamp = rospy.Time.now()
        arm_msg.joint_names = self.arm_joint_names
        
        gripper_msg = JointTrajectory()
        gripper_msg.header.stamp = arm_msg.header.stamp
        gripper_msg.joint_names = self.gripper_joint_names
        
        start_time_sec = timestamps[0]

        for i in range(len(timestamps)):
            time_from_start_duration = rospy.Duration.from_sec(
                (timestamps[i] - start_time_sec) * execute_time_factor
            )

            arm_point = JointTrajectoryPoint()
            arm_point.positions = arm_trajectory_points[i].tolist()
            arm_point.velocities = [0.0] * len(self.arm_joint_names)
            arm_point.accelerations = [0.0] * len(self.arm_joint_names)
            arm_point.time_from_start = time_from_start_duration
            arm_msg.points.append(arm_point)
            
            gripper_point = JointTrajectoryPoint()
            original_gripper_value = gripper_trajectory_points_1d[i]
            transformed_gripper_value = -2.0 * original_gripper_value
            gripper_point.positions = [transformed_gripper_value] * len(self.gripper_joint_names)
            gripper_point.velocities = [0.0] * len(self.gripper_joint_names)
            gripper_point.accelerations = [0.0] * len(self.gripper_joint_names)
            gripper_point.time_from_start = time_from_start_duration
            gripper_msg.points.append(gripper_point)
        
        if arm_msg.points:
            rospy.loginfo(f"[ROS Handler] Publishing arm trajectory with {len(arm_msg.points)} points.")
            self.arm_publisher.publish(arm_msg)
        
        if gripper_msg.points:
            rospy.loginfo(f"[ROS Handler] Publishing gripper trajectory with {len(gripper_msg.points)} points.")
            self.gripper_publisher.publish(gripper_msg)
        
        rospy.loginfo(f"[ROS Handler] Trajectories published successfully.")


# --- Main Node Class ---
class DMPCubeManipulator:
    def __init__(self, dmp_paths, urdf_path, mesh_path=None,
                 base_frame="world",
                 robot_joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                 gripper_joint_names=["gripper", "gripper_sub"],
                 arm_controller_topic='/open_manipulator_6dof/arm_controller/command',
                 gripper_controller_topic='/open_manipulator_6dof/gripper_controller/command',
                 publish_rate=20.0,
                 tf_update_rate=5.0,
                 joint_states_topic="/joint_states",
                 cube_height_approx=0.04,
                 cube_xy_proximity_threshold=0.03,
                 use_sim = False):
        if not rospy.core.is_initialized():
             rospy.init_node('dmp_cube_manipulator_node', anonymous=True)
        else:
             rospy.logwarn("ROS node 'dmp_cube_manipulator_node' already initialized.")

        rospy.loginfo("Initializing DMPCubeManipulator Node...")

        self.base_frame = base_frame
        self.robot_joint_names = list(robot_joint_names)
        self.gripper_joint_names = list(gripper_joint_names)
        self.all_joint_names_ordered = self.robot_joint_names + self.gripper_joint_names

        self.tf_listener = tf.TransformListener()
        self.tracked_cube_name = None
        self.current_cube_pose = None
        self.tf_update_rate = tf_update_rate
        self.tf_timer = None
        self.last_ee_pqs = None

        self.dmp_generators = {}
        required_motions = ['pick', 'lift', 'place', 'home']
        loaded_motions = []
        for motion_type, dmp_file in dmp_paths.items():
            rospy.loginfo(f"Loading DMP for motion '{motion_type}' from {dmp_file}...")
            generator = DMPMotionGenerator(
                urdf_path=urdf_path,
                mesh_path=mesh_path,
                joint_names=self.robot_joint_names,
                base_link=self.base_frame
            )
            try:
                generator.load_dmp(dmp_file)
                if generator.dmp is None:
                    raise RuntimeError(f"DMP object is None after loading from {dmp_file}")
                if motion_type in ['pick', 'lift', 'place'] and generator.gripper_trajectory is None:
                    rospy.logwarn(f"Gripper trajectory not found in DMP file for '{motion_type}'.")
                elif generator.gripper_trajectory is None:
                     rospy.loginfo(f"Gripper trajectory not found for '{motion_type}' (may be expected).")
                self.dmp_generators[motion_type] = generator
                loaded_motions.append(motion_type)
                rospy.loginfo(f"Successfully loaded DMP for '{motion_type}'.")
            except Exception as e:
                rospy.logerr(f"Failed to load DMP for motion '{motion_type}' from {dmp_file}: {e}")
                if motion_type in required_motions:
                    raise RuntimeError(f"Failed to load required DMP for '{motion_type}'.")

        missing_required = [m for m in required_motions if m not in loaded_motions]
        if missing_required:
             raise RuntimeError(f"Missing required DMPs for motions: {missing_required}")
        if not self.dmp_generators:
            rospy.logfatal("No DMPs were loaded successfully. Exiting.")
            raise RuntimeError("Failed to load any DMPs.")

        self.trajectory_handler = ROSTrajectoryHandler(
            arm_joint_names_for_publishing=self.robot_joint_names,
            gripper_joint_names_for_publishing=self.gripper_joint_names,
            all_joint_names_for_subscription=self.all_joint_names_ordered,
            arm_publish_topic_name=arm_controller_topic,
            gripper_publish_topic_name=gripper_controller_topic,
            publish_rate_hz=publish_rate,
            subscribe_joint_state_topic=joint_states_topic
        )
        self.publish_rate = publish_rate
        self.cube_height_approx = cube_height_approx
        self.cube_xy_proximity_threshold = cube_xy_proximity_threshold
        rospy.loginfo(f"DMPCubeManipulator initialized. Loaded DMPs for: {list(self.dmp_generators.keys())}")

    def start_tf_listener(self, cube_name):
        if self.tf_timer is not None:
            if self.tracked_cube_name == cube_name:
                rospy.loginfo(f"TF listener already running for '{cube_name}'.")
                return
            else:
                rospy.loginfo(f"Switching TF listener from '{self.tracked_cube_name}' to '{cube_name}'.")
                self.stop_tf_listener()
        rospy.loginfo(f"Starting TF listener for '{cube_name}' at {self.tf_update_rate} Hz.")
        self.tracked_cube_name = cube_name
        self.current_cube_pose = None
        self.tf_timer = rospy.Timer(rospy.Duration(1.0 / self.tf_update_rate),
                                    self.update_single_cube_pose_callback,
                                    oneshot=False)

    def stop_tf_listener(self):
        if self.tf_timer is not None:
            rospy.loginfo(f"Stopping TF listener for '{self.tracked_cube_name}'.")
            self.tf_timer.shutdown()
            self.tf_timer = None
            self.tracked_cube_name = None
        else:
            rospy.loginfo("TF listener is not currently running.")

    def update_single_cube_pose_callback(self, event=None):
        if self.tracked_cube_name is None:
            rospy.logwarn_throttle(10.0, "TF update callback called but no cube is being tracked.")
            return
        try:
            self.tf_listener.waitForTransform(self.base_frame, self.tracked_cube_name, rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(self.base_frame, self.tracked_cube_name, rospy.Time(0))
            self.current_cube_pose = (trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            if self.current_cube_pose is not None:
                 rospy.logwarn_throttle(5.0, f"Could not get transform for '{self.tracked_cube_name}': {e}. Using last known pose.")
            else:
                 rospy.logwarn_throttle(10.0, f"Could not get transform for '{self.tracked_cube_name}': {e}. Pose not yet available.")
        except Exception as e:
             rospy.logerr(f"Unexpected error getting transform for {self.tracked_cube_name}: {e}")

    def get_latest_cube_pose(self, cube_name):
        if cube_name != self.tracked_cube_name:
            rospy.logerr(f"Requested pose for '{cube_name}', but TF listener is tracking '{self.tracked_cube_name}'. Call start_tf_listener('{cube_name}') first.")
            return None
        if self.current_cube_pose is None:
             rospy.logwarn(f"Pose for tracked cube '{cube_name}' is not yet available.")
             return None
        return self.current_cube_pose
    
    def _generate_dmp_trajectory(self, motion_type, start_pose_pqs=None, goal_pose_pqs=None, target_tf_frame=None, offset = np.array([0., 0., 0.01])):
        if motion_type not in self.dmp_generators:
            rospy.logerr(f"No DMP loaded for motion type '{motion_type}'. Cannot generate trajectory.")
            return None, None

        generator = self.dmp_generators[motion_type]
        dmp = generator.dmp
        # final_start_pqs = dmp.start_y.copy()
        # if start_pose_pqs is not None and start_pose_pqs.any():
        #     final_start_pqs[:3] = start_pose_pqs[:3]
        final_start_pqs = start_pose_pqs if start_pose_pqs is not None else dmp.start_y.copy()
        final_goal_pqs = dmp.goal_y.copy()

        if target_tf_frame and motion_type in ['pick', 'place']:
            latest_pose = self.get_latest_cube_pose(target_tf_frame)
            if latest_pose is None:
                rospy.logerr(f"No pose available for TF frame '{target_tf_frame}'. Cannot generate trajectory for '{motion_type}'.")
                return None, None
            trans, rot = latest_pose
            rospy.loginfo(f"Using TF pose for '{target_tf_frame}' as goal position for '{motion_type}': Translation={trans}")
            final_goal_pqs[:3] = np.array(trans)
            final_goal_pqs[:3] += offset
        elif goal_pose_pqs is not None and goal_pose_pqs.any():
            final_goal_pqs = goal_pose_pqs.copy()

        rospy.loginfo(f"Generating '{motion_type}' trajectory:")
        if final_start_pqs is not None:
            rospy.loginfo(f"  Start PQS (effective): {np.round(final_start_pqs, 3)}")
        else:
            rospy.loginfo(f"  Start PQS (effective): Using DMP default start_y: {np.round(dmp.start_y,3)}")
        rospy.loginfo(f"  Goal PQS (effective):  {np.round(final_goal_pqs, 3)}")

        T_cartesian, cartesian_trajectory = generator.generate_trajectory(
            start_y=final_start_pqs,
            goal_y=final_goal_pqs
        )
        if T_cartesian is None or cartesian_trajectory is None:
            rospy.logerr(f"DMP trajectory generation failed for '{motion_type}'.")
            return None, None
        return T_cartesian, cartesian_trajectory

    def _compute_joint_trajectory(self, motion_type, T_cartesian, cartesian_trajectory, subsample_factor=1):
        if motion_type not in self.dmp_generators:
            rospy.logerr(f"No DMP loaded for motion type '{motion_type}'. Cannot compute IK.")
            return None, None, None
        if cartesian_trajectory is None or T_cartesian is None:
            rospy.logerr(f"Cannot compute IK for None Cartesian trajectory for '{motion_type}'.")
            return None, None, None

        generator = self.dmp_generators[motion_type]
        _, joint_trajectory_arm, gripper_traj, T_ik = generator.compute_IK_trajectory(
            trajectory=cartesian_trajectory,
            time_stamp=T_cartesian,
            subsample_factor=subsample_factor
        )
        if joint_trajectory_arm is None:
             rospy.logerr(f"IK computation failed for '{motion_type}'.")
             return None, None, None
        if gripper_traj is None:
             rospy.logwarn(f"IK computation for '{motion_type}' did not return a gripper trajectory. Using default (0.0).")
             gripper_traj = np.zeros(len(joint_trajectory_arm))
        return joint_trajectory_arm, gripper_traj, T_ik

    def execute_motion(self, motion_type, start_pose_pqs=None, goal_pose_pqs=None,
                       target_tf_frame=None, wait_for_tf_sec=1.0, subsample_factor_ik=1,
                       smoothing_window_size=15): # Added smoothing_window_size
        rospy.loginfo(f"--- Initialising motion sequence: '{motion_type}' ---")
        effective_start_pqs = start_pose_pqs

        if effective_start_pqs is None:
            if self.last_ee_pqs is not None:
                generator = self.dmp_generators[motion_type]
                dmp_default_start_orientation = generator.dmp.start_y[3:]
                last_ee_position = self.last_ee_pqs[:3]
                if motion_type in ['home']:
                    effective_start_pqs = np.concatenate((last_ee_position, dmp_default_start_orientation))
                else:
                    effective_start_pqs = self.last_ee_pqs.copy()
                rospy.loginfo(f"For '{motion_type}', using last commanded EE PQS: {np.round(effective_start_pqs,3)}")
            else:
                rospy.loginfo(f"For '{motion_type}', no explicit start_pose_pqs and no last_ee_pqs. Attempting to use current robot pose via FK.")
                current_joint_states_data = self.trajectory_handler.get_joint_states(desired_joint_order=self.robot_joint_names)
                if current_joint_states_data and all(p is not None for p in current_joint_states_data["position"]):
                    current_arm_positions = np.array(current_joint_states_data["position"])
                    generator = self.dmp_generators[motion_type]
                    try:
                        current_ee_transform = generator.chain.forward(current_arm_positions)
                        current_ee_pqs = _manual_pqs_from_transform(current_ee_transform)
                        effective_start_pqs = current_ee_pqs
                        rospy.loginfo(f"For '{motion_type}', using current FK EE PQS: {np.round(effective_start_pqs,3)} ")
                    except Exception as e:
                        rospy.logwarn(f"Failed to get current EE pose via FK for '{motion_type}': {e}. DMP will use its default start_y.")
                        effective_start_pqs = None
                else:
                    rospy.logwarn(f"Could not retrieve valid current joint states for FK for '{motion_type}'. DMP will use its default start_y.")
                    effective_start_pqs = None
        else:
            rospy.loginfo(f"For '{motion_type}', using explicitly provided start_pose_pqs: {np.round(effective_start_pqs,3)}")

        goal_offset = np.array([0., 0., 0.01])
        needs_tf = target_tf_frame and motion_type in ['pick', 'place']
        if needs_tf:
            self.start_tf_listener(target_tf_frame)
            rospy.loginfo(f"Waiting {wait_for_tf_sec} seconds for TF data for '{target_tf_frame}'...")
            rospy.sleep(wait_for_tf_sec)

        if motion_type == 'place':
            goal_offset = np.array([-0.01, 0., 0.02])
        
        T_cartesian, cartesian_trajectory = self._generate_dmp_trajectory(
            motion_type, start_pose_pqs=effective_start_pqs, goal_pose_pqs=goal_pose_pqs, target_tf_frame=target_tf_frame, offset=goal_offset
        )
        if cartesian_trajectory is None:
            rospy.logerr(f"Failed to generate Cartesian trajectory for '{motion_type}'.")
            if needs_tf: self.stop_tf_listener()
            return None, None, None, None

        joint_trajectory_ik, gripper_traj, T_ik = self._compute_joint_trajectory(
            motion_type, T_cartesian, cartesian_trajectory, subsample_factor=subsample_factor_ik
        )

        if needs_tf:
            self.stop_tf_listener()

        if joint_trajectory_ik is None:
            rospy.logerr(f"Failed to compute joint trajectory via IK for '{motion_type}'.")
            return None, None, None, None

        # Apply moving average filter to smooth IK trajectory
        if smoothing_window_size > 0 and len(joint_trajectory_ik) > smoothing_window_size:
            rospy.loginfo(f"Applying moving average filter (window: {smoothing_window_size}) to '{motion_type}' IK trajectory.")
            original_start_joints = joint_trajectory_ik[0,:].copy()
            original_end_joints = joint_trajectory_ik[-1,:].copy()

            smoothed_joint_trajectory_ik = np.zeros_like(joint_trajectory_ik)
            for i in range(joint_trajectory_ik.shape[1]): # For each joint
                smoothed_joint_trajectory_ik[:, i] = np.convolve(
                    joint_trajectory_ik[:, i], 
                    np.ones(smoothing_window_size)/smoothing_window_size, 
                    mode='same'
                )

            # Preserve original start and end points
            smoothed_joint_trajectory_ik[0,:] = original_start_joints
            smoothed_joint_trajectory_ik[-1,:] = original_end_joints

            # Blend the smoothed trajectory back to original start/end to avoid jumps
            half_window = smoothing_window_size // 2
            if half_window > 0: # Ensure half_window is at least 1 for blending
                for i in range(joint_trajectory_ik.shape[1]): # For each joint
                    # Blend start
                    for j in range(1, half_window): # Start from 1 to keep original_start_joints at index 0
                        if j < len(joint_trajectory_ik): # Boundary check
                            alpha = float(j) / half_window
                            smoothed_joint_trajectory_ik[j, i] = (1 - alpha) * original_start_joints[i] + alpha * smoothed_joint_trajectory_ik[j, i]
                    # Blend end
                    for j in range(1, half_window): # Start from 1 to keep original_end_joints at index -1
                        idx_from_end = len(joint_trajectory_ik) - 1 - j
                        if idx_from_end >= 0: # Boundary check
                            alpha = float(j) / half_window
                            smoothed_joint_trajectory_ik[idx_from_end, i] = (1 - alpha) * original_end_joints[i] + alpha * smoothed_joint_trajectory_ik[idx_from_end, i]
            
            joint_trajectory_ik = smoothed_joint_trajectory_ik
            rospy.loginfo(f"Finished applying moving average filter to '{motion_type}' IK trajectory.")
        elif smoothing_window_size > 0:
            rospy.logwarn(f"Trajectory for '{motion_type}' too short (length {len(joint_trajectory_ik)}) for moving average window {smoothing_window_size}. Skipping smoothing.")

        if cartesian_trajectory is not None and len(cartesian_trajectory) > 0:
            last_transform_matrix = cartesian_trajectory[-1]
            self.last_ee_pqs = _manual_pqs_from_transform(last_transform_matrix)
            rospy.loginfo(f"Updated last_ee_pqs for next motion: {np.round(self.last_ee_pqs,3)}")

        rospy.loginfo(f"Successfully generated joint trajectory for '{motion_type}' with {len(joint_trajectory_ik)} points.")
        return joint_trajectory_ik, gripper_traj, T_ik, cartesian_trajectory

    def pick_cube(self, cube_name, **kwargs):
        rospy.loginfo(f"Executing PICK motion for cube '{cube_name}'")
        if 'pick' not in self.dmp_generators:
            rospy.logerr("Cannot execute 'pick_cube': 'pick' DMP not loaded.")
            return None, None, None, None
        return self.execute_motion(motion_type='pick', target_tf_frame=cube_name, smoothing_window_size=SMOOTHING_WINDOW,**kwargs)

    def lift_cube(self, **kwargs):
        rospy.loginfo(f"Executing LIFT motion")
        if 'lift' not in self.dmp_generators:
            rospy.logerr("Cannot execute 'lift_cube': 'lift' DMP not loaded.")
            return None, None, None, None
        return self.execute_motion(motion_type='lift', smoothing_window_size=SMOOTHING_WINDOW,**kwargs)

    def place_cube(self, target_pose_pqs=None, target_tf=None, **kwargs):
        rospy.loginfo(f"Executing PLACE motion")
        if 'place' not in self.dmp_generators:
            rospy.logerr("Cannot execute 'place_cube': 'place' DMP not loaded.")
            return None, None, None, None
        if target_tf:
            rospy.loginfo(f"  Targeting TF frame: {target_tf}")
            return self.execute_motion(motion_type='place', target_tf_frame=target_tf,smoothing_window_size=SMOOTHING_WINDOW, **kwargs)
        elif target_pose_pqs is not None:
             rospy.loginfo(f"  Targeting PQS pose: {np.round(target_pose_pqs, 3)}")
             return self.execute_motion(motion_type='place', goal_pose_pqs=target_pose_pqs, smoothing_window_size=SMOOTHING_WINDOW,**kwargs)
        else:
             rospy.logwarn("Executing 'place' motion with default DMP goal (no target specified).")
             return self.execute_motion(motion_type='place', smoothing_window_size=SMOOTHING_WINDOW,**kwargs)

    def go_home(self, **kwargs):
        rospy.loginfo(f"Executing GO_HOME motion")
        if 'home' not in self.dmp_generators:
            rospy.logerr("Cannot execute 'go_home': 'home' DMP not loaded.")
            return None, None, None, None
        return self.execute_motion(motion_type='home', smoothing_window_size=SMOOTHING_WINDOW,**kwargs)

    def simulate_trajectory(self, motion_type, joint_trajectory, cartesian_trajectory):
        if motion_type not in self.dmp_generators:
            rospy.logwarn(f"Cannot simulate, no DMP generator for motion '{motion_type}'.")
            return
        if joint_trajectory is None or cartesian_trajectory is None:
            rospy.logwarn(f"Cannot simulate None trajectory for '{motion_type}'.")
            return
        generator = self.dmp_generators[motion_type]
        if len(joint_trajectory) != len(cartesian_trajectory):
             rospy.logwarn(f"Simulation Warning ({motion_type}): Joint trajectory length ({len(joint_trajectory)}) "
                           f"differs from Cartesian trajectory length ({len(cartesian_trajectory)}).")
        rospy.loginfo(f"Starting simulation for '{motion_type}' (requires graphical environment)...")
        try:
            generator.visualize_trajectory(cartesian_trajectory, joint_trajectory)
            rospy.loginfo("Simulation finished.")
        except Exception as e:
             rospy.logerr(f"Simulation failed for '{motion_type}': {e}.")

    def publish_trajectory(self, joint_trajectory_arm, timestamps, gripper_state, execute_time_factor=1.0):
        if joint_trajectory_arm is None or timestamps is None or gripper_state is None:
            rospy.logwarn("Cannot publish None trajectory components.")
            return
        
        if not (len(joint_trajectory_arm) == len(timestamps) == len(gripper_state)):
            rospy.logerr("Mismatched lengths in trajectory components for interpolation.")
            rospy.logerr(f"  Arm: {len(joint_trajectory_arm)}, Timestamps: {len(timestamps)}, Gripper: {len(gripper_state)}")
            return

        rospy.loginfo(f"Preparing to publish interpolated trajectory at ~{self.publish_rate} Hz...")

        try:
            interpolated_arm_traj, interpolated_common_timestamps = interpolate_joint_trajectory(
                joint_trajectory_arm, timestamps, target_freq=self.publish_rate
            )
        except Exception as e:
            rospy.logerr(f"Interpolation failed for arm trajectory: {e}. Cannot publish.")
            return
        
        if interpolated_arm_traj is None or len(interpolated_arm_traj) == 0:
            rospy.logerr("Arm trajectory interpolation resulted in empty trajectory. Cannot publish.")
            return

        gripper_state_2d = gripper_state.reshape(-1, 1)
        try:
            interpolated_gripper_traj_2d, _ = interpolate_joint_trajectory(
                gripper_state_2d, timestamps, target_freq=self.publish_rate
            )
        except Exception as e:
            rospy.logerr(f"Interpolation failed for gripper trajectory: {e}. Cannot publish.")
            return

        if interpolated_gripper_traj_2d is None or len(interpolated_gripper_traj_2d) == 0:
            rospy.logerr("Gripper trajectory interpolation resulted in empty trajectory. Cannot publish.")
            return
            
        interpolated_gripper_traj_1d = interpolated_gripper_traj_2d.flatten()

        if len(interpolated_arm_traj) != len(interpolated_gripper_traj_1d):
            rospy.logwarn(f"Interpolated arm ({len(interpolated_arm_traj)}) and gripper ({len(interpolated_gripper_traj_1d)}) trajectories have different lengths. Attempting to use shorter length.")
            min_len = min(len(interpolated_arm_traj), len(interpolated_gripper_traj_1d))
            interpolated_arm_traj = interpolated_arm_traj[:min_len]
            interpolated_gripper_traj_1d = interpolated_gripper_traj_1d[:min_len]
            interpolated_common_timestamps = interpolated_common_timestamps[:min_len]

        rospy.loginfo(f"Publishing interpolated arm and gripper trajectories ({len(interpolated_common_timestamps)} points)...")
        try:
            self.trajectory_handler.publish_trajectory(
                interpolated_arm_traj, 
                interpolated_gripper_traj_1d, 
                interpolated_common_timestamps,
                execute_time_factor=execute_time_factor
            )
            rospy.loginfo("Trajectory publishing initiated by handler.")
        except Exception as e:
             rospy.logerr(f"Error during trajectory publishing via handler: {e}")

    def check_hanoi_tower_condition(self, found_cubes):
        if not found_cubes or len(found_cubes) < 2:
            rospy.loginfo("Not enough cubes to check Tower of Hanoi condition.")
            return True
        
        cube_positions = {}
        for cube_name in found_cubes:
            try:
                self.tf_listener.waitForTransform(self.base_frame, cube_name, rospy.Time(0), rospy.Duration(0.1))
                (trans, rot) = self.tf_listener.lookupTransform(self.base_frame, cube_name, rospy.Time(0))
                cube_number = get_cube_number_from_name(cube_name)
                if cube_number != -1:
                    cube_positions[cube_name] = {
                        'pos': trans, 
                        'number': cube_number
                    }
            except Exception as e:
                rospy.logwarn(f"Could not get transform for '{cube_name}': {e}")
        
        if len(cube_positions) < 2:
            return True
        
        all_rules_satisfied = True
        for cube1_name, cube1_data in cube_positions.items():
            for cube2_name, cube2_data in cube_positions.items():
                if cube1_name == cube2_name:
                    continue
                    
                pos1, num1 = cube1_data['pos'], cube1_data['number']
                pos2, num2 = cube2_data['pos'], cube2_data['number']
                
                x_aligned = abs(pos1[0] - pos2[0]) < self.cube_xy_proximity_threshold
                y_aligned = abs(pos1[1] - pos2[1]) < self.cube_xy_proximity_threshold
                
                if x_aligned and y_aligned:
                    z_diff = abs(pos1[2] - pos2[2])
                    if abs(z_diff - self.cube_height_approx) < (self.cube_height_approx / 2.0):
                        if pos1[2] > pos2[2]:
                            if num1 > num2:
                                rospy.logwarn(f"Hanoi violation! Cube {cube1_name} (number {num1}) is on top of {cube2_name} (number {num2})")
                                all_rules_satisfied = False
                        elif pos2[2] > pos1[2]:
                            if num2 > num1:
                                rospy.logwarn(f"Hanoi violation! Cube {cube2_name} (number {num2}) is on top of {cube1_name} (number {num1})")
                                all_rules_satisfied = False
        
        if all_rules_satisfied:
            rospy.loginfo("Tower of Hanoi condition is satisfied.")
        else:
            rospy.logwarn("Tower of Hanoi condition is violated!")
        
        return all_rules_satisfied

    def run(self):
        rospy.loginfo("DMPCubeManipulator node running. Waiting for commands or shutdown.")
        rospy.spin()

# -------------------------------------- MAIN Example --------------------------------------#
if __name__ == "__main__":
    grasper_node = None
    try:
        DMP_FILES = {
            'pick': '/root/catkin_ws/recordings/learned_pick_motion_11.pkl',
            'lift': '/root/catkin_ws/recordings/learned_lift_motion_4.pkl',
            'place': '/root/catkin_ws/recordings/learned_place_motion_14.pkl',
            'home': '/root/catkin_ws/recordings/learned_release_motion_2.pkl'
        }
        URDF_FILE = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/urdf/open_manipulator_6dof.urdf'
        MESH_DIR = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/meshes'
        WORLD_FRAME = "world"
        CUBE_TO_GRASP = "blue_cube"
        CUBE_TO_PLACE = "green_cube" 
        SMOOTHING_WINDOW = 30 # Define smoothing window for the main sequence

        LIFT_TARGET_PQS = np.array([0.08039667, -0.00823571, 0.12112987, 0.40824577, 0.03776871, 0.91182508, -0.02199852]) 

        TF_WAIT_TIME = 1.5
        IK_SUBSAMPLE = 2
        PUB_RATE = 20.0
        TF_RATE = 10.0
        PAUSE_BETWEEN_MOTIONS = 1.0
        JOINT_STATES_TOPIC = "/joint_states"
        GRIPPER_JOINT_NAMES = ["gripper", "gripper_sub"]
        ARM_CONTROLLER_TOPIC = '/open_manipulator_6dof/arm_controller/command'
        GRIPPER_CONTROLLER_TOPIC = '/open_manipulator_6dof/gripper_controller/command'

        grasper_node = DMPCubeManipulator(
            dmp_paths=DMP_FILES,
            urdf_path=URDF_FILE,
            mesh_path=MESH_DIR,
            base_frame=WORLD_FRAME,
            robot_joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            gripper_joint_names=GRIPPER_JOINT_NAMES,
            arm_controller_topic=ARM_CONTROLLER_TOPIC,
            gripper_controller_topic=GRIPPER_CONTROLLER_TOPIC,
            tf_update_rate=TF_RATE,
            publish_rate=PUB_RATE,
            joint_states_topic=JOINT_STATES_TOPIC
        )

        rospy.loginfo("Waiting a moment for initial joint states...")
        rospy.sleep(1.0)

        motion_sequence = [
            {'name': 'pick', 'action': lambda: grasper_node.pick_cube(CUBE_TO_GRASP, subsample_factor_ik=IK_SUBSAMPLE, wait_for_tf_sec=TF_WAIT_TIME)},
            {'name': 'lift', 'action': lambda: grasper_node.lift_cube(goal_pose_pqs=LIFT_TARGET_PQS, subsample_factor_ik=IK_SUBSAMPLE)},
            {'name': 'place', 'action': lambda: grasper_node.place_cube(target_tf=CUBE_TO_PLACE ,subsample_factor_ik=IK_SUBSAMPLE)},
            {'name': 'home', 'action': lambda: grasper_node.go_home(goal_pose_pqs=LIFT_TARGET_PQS, subsample_factor_ik=IK_SUBSAMPLE)}
        ]

        for motion_info in motion_sequence:
            rospy.loginfo(f"Executing: {motion_info['name']}")
            results = motion_info['action']()
            if results is None or results[0] is None:
                raise RuntimeError(f"{motion_info['name'].capitalize()} motion failed.")
            joint_traj_arm, gripper_traj, time_stamps, cartesian_traj_viz = results
            grasper_node.simulate_trajectory(motion_info['name'], joint_traj_arm, cartesian_traj_viz) # Optional
            key = input("Press any button to continue to the next motion... Abort with S: ")  # Optional pause for visualization
            print(f"Key pressed: {key}")
            if key.lower() == 's':
                rospy.loginfo("Stopping sequence due to user request.")
                break
            grasper_node.publish_trajectory(joint_traj_arm, time_stamps, gripper_traj, execute_time_factor=1.0)
            rospy.loginfo(f"Waiting {PAUSE_BETWEEN_MOTIONS}s after {motion_info['name']}...")
            rospy.sleep(PAUSE_BETWEEN_MOTIONS)
            

        rospy.loginfo("Full motion sequence finished successfully.")
        if grasper_node: grasper_node.stop_tf_listener()

    except rospy.ROSInterruptException:
        rospy.loginfo("DMPCubeManipulator node interrupted.")
    except RuntimeError as e:
         rospy.logerr(f"A runtime error occurred during the motion sequence: {e}")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred in the main execution: {e}", exc_info=True)
    finally:
        if grasper_node is not None:
            rospy.loginfo("Ensuring TF listener is stopped in finally block.")
            grasper_node.stop_tf_listener()
        rospy.loginfo("DMPCubeManipulator node finished.")