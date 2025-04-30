# TODO: 
# set ROBOT_DISTANCE_THRESHOLD
# set arm
# - implement:
# - _reach_and_grasp (uses _execute_grasp with action[self.robot.controller_action_idx["gripper_right_hand"]] = -1.0  which might be too specific)
# - _get_obj_in_hand (uses self.arm - check that generalises)
# - _move_gripper_to_pose (uses self.arm - check that generalises)
# - _get_grasp_pose_for_object (double-check get_grasp_poses_for_object implementation)

from .igibson_semantic_actions_env import iGibsonSemanticActionEnv

import random
import numpy as np
import pybullet as p

from igibson.envs.igibson_env import iGibsonEnv
from igibson.utils.grasp_planning_utils import get_grasp_poses_for_object_gripper

from igibson.custom_utils import get_env_config
import igibson.render_utils as render_utils

class FetchRobotSemanticActionEnv(iGibsonSemanticActionEnv):
    ROBOT_DISTANCE_THRESHOLD = 1.2 # just hardcoded for now
    DEFAULT_BODY_OFFSET_FROM_FLOOR = 0.007 # not fond, but whatever
    arm = 'right_hand' # might be already correct - untested

    def __init__(self, task, scene_id, verbose=False):
        env_config = get_env_config()
        env_config["task"] = task
        env_config["scene_id"] = scene_id
        env_config["task_id"] = 0
        env_config["instance_id"] = 0
        env_config["robot"]["name"] = "Fetch" # this should init Fetch instead of Behavior robot
        robot_name = env_config["robot"]["name"] # Keep this in memory as it's removed from the config when we init iGibson

        env_config["robot"]["grasping_mode"] = "sticky" #"assisted"
        self.env = iGibsonEnv(
                config_file=env_config,
                mode="headless",
                action_timestep=1.0 / 30.0,
                physics_timestep=1.0 / 120.0,
            )

        env_config["robot"]["name"] = robot_name
        self.env_config = env_config
        
        self._settle_physics()
        
        self.task = task
        self.scene = self.env.scene
        self.robot = self.env.robots[0]
        self.robot_id = self.robot._body_ids[0]
        self.gripper_link_idx = self.robot._links['gripper_link'].link_id
        self.verbose = verbose

    def close(self, container_obj_name): 
        pass

    def place_inside(self, trg_obj_name, container_obj_name): 
        pass
        
    # debug this
    def _move_gripper_to_pose(self, pose):
        joint_pos = self._solve_ik(pose) 
        if joint_pos is None:
            raise ValueError("IK failed to find a solution for pose: {}".format(pose))
        
        # Print gripper pose before
        if self.verbose: 
            self._check_gripper_in_pose(pose, name='before')
        
        self.robot.set_joint_positions(joint_pos)

        # Print gripper pose after
        if self.verbose: 
            self._check_gripper_in_pose(pose, name='after set_join_positions')

    def _check_gripper_in_pose(self, trg_pose, name, pos_tol=1e-2, orn_tol=1e-2):

        target_pos, target_orn = trg_pose
        
        # Get actual gripper pose
        gripper_state = p.getLinkState(self.robot_id, self.gripper_link_idx)
        actual_pos = gripper_state[4]  # position of COM of link
        actual_orn = gripper_state[5]  # orientation as quaternion
        
        # Compare to desired
        pos_error = np.linalg.norm(np.array(actual_pos) - np.array(target_pos))
        orn_error = quaternion_distance(actual_orn, target_orn)
        
        # Set your own tolerances
        success = pos_error < pos_tol and orn_error < orn_tol
        
        # Print debug info
        print(f"\n==== Gripper Pose Check: {name} ====")
        print(f"Target Position     : {np.round(target_pos, 4)}")
        print(f"Actual Position     : {np.round(actual_pos, 4)}")
        print(f"Position Error      : {pos_error:.6f} (tol={pos_tol})")
    
        print(f"Target Orientation  : {np.round(target_orn, 4)}")
        print(f"Actual Orientation  : {np.round(actual_orn, 4)}")
        print(f"Orientation Error   : {orn_error:.6f} (tol={orn_tol})")

    
    def _solve_ik(self, pose):
        """
        Return joint positions (14,) that move the gripper to `pose`, using IK.
        """
        position, orientation = pose

        # Consider only joints between body and end effector 
        indexes = self.robot.arm_control_idx[self.robot.default_arm]

        # Get joint ranges between lower and upper values
        joint_ranges = np.array([
            upper - lower
            for lower, upper in zip(self.robot.joint_lower_limits, self.robot.joint_upper_limits)
        ])

        joint_angles = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id, #self.robot.robot_id does not exist
            endEffectorLinkIndex=self.gripper_link_idx,
            targetPosition=position,
            targetOrientation=orientation,
            lowerLimits=self.robot.joint_lower_limits[indexes],
            upperLimits=self.robot.joint_upper_limits[indexes],
            jointRanges=joint_ranges[indexes],
            restPoses=self.robot.untucked_default_joint_pos[indexes],
            maxNumIterations=100,
            residualThreshold=1e-4
        )

        return np.array(joint_angles)


    def _get_obj_in_hand(self): 
        obj_in_hand_id = self.robot._ag_obj_in_hand['0'] # this is the self.robot.default_arm for Fetch apparently
        obj_in_hand = self.scene.objects_by_id[obj_in_hand_id] if obj_in_hand_id is not None else None
        return obj_in_hand

    def _reach_and_grasp(self, trg_obj, object_direction, grasp_orn, reach=False, reach_offset=0.05):
        if reach:
            target_pos = trg_obj.get_position() - reach_offset * object_direction

            # TODO: add this method to the class
            #for _ in controller._move_hand((target_pos, grasp_orn), stop_on_contact=True):
            #    yield _  # Executes planned motion until contact
    
        # Contact happened — now grasp!
        yield from self._execute_grasp()
        
    def _execute_grasp(self):
        action = np.zeros(self.robot.action_dim)
        action[self.robot.controller_action_idx[f"gripper_{self.robot.default_arm}"]] = np.array([-1.0]) # seems to work
        for _ in range(self.MAX_STEPS_FOR_GRASP_OR_RELEASE):
            yield action

    def _get_distance_from_robot(self, pos):
        shoulder_pos_in_base_frame = np.array(
            self.robot.links["shoulder_pan_link"].get_local_position_orientation()[0]
        )
        point_in_base_frame = np.array(self._get_pose_in_robot_frame((pos, [0, 0, 0, 1]))[0])
        shoulder_to_hand = point_in_base_frame - shoulder_pos_in_base_frame
        return np.linalg.norm(shoulder_to_hand)

    def _detect_robot_collision(self):
        if self.verbose:
            print("Start collision test.")
    
        collision_links = [
            "torso_lift_link",      # body equivalent
            "elbow_flex_link",      # mid arm
            "wrist_flex_link",      # wrist
            "gripper_link",         # hand
        ]
    
        any_collision = False
    
        for link_name in collision_links:
            link = self.robot.links.get(link_name)
            if not link:
                continue # just in case
                
            collisions = self._detect_collision(link.body_id)
            if len(collisions)>0:
                any_collision = True
                if self.verbose:
                    print(f"{link_name} has collision with objects {collisions}")
    
        if self.verbose:
            print("End collision test.")
        return any_collision

    def _get_grasp_pose_for_object(self, trg_obj, robot_pos, pick_closest=False, render=False, **kwargs): 
        grasp_poses = get_grasp_poses_for_object_gripper(self.robot, trg_obj, force_allow_any_extent=False)

        if render:
            gps = [grasp_pose for (grasp_pose, obj_dir) in grasp_poses]
            print(f"len(grasp_poses):{len(grasp_poses)}, len(gps):{len(gps)}, len(grasp_poses[0]):{len(grasp_poses[0])}, len(gps[0]):{len(gps[0])}")
            render_utils.render_frame_with_grasp_poses(
                    self.env, gps, show=True, save=True,
                    name='grasp_poses'
                )
            
        if pick_closest:
            grasp_pose, object_direction = self.pick_closest_pose(robot_pos, grasp_poses)
        else:
            grasp_pose, object_direction = random.choice(grasp_poses)
        return grasp_pose, object_direction
        
def quaternion_distance(q1, q2):
    """Compute shortest angle between two quaternions, accounting for double covering."""
    q1 = np.array(q1)
    q2 = np.array(q2)
    dot_product = np.abs(np.dot(q1, q2))  # Take abs to fix sign flip ambiguity
    dot_product = np.clip(dot_product, -1.0, 1.0)
    return 2 * np.arccos(dot_product)

