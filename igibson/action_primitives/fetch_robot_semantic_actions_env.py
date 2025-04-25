# TODO: 
# set ROBOT_DISTANCE_THRESHOLD
# set arm
# - implement:
# - _reach_and_grasp (uses _execute_grasp with action[self.robot.controller_action_idx["gripper_right_hand"]] = -1.0  which might be too specific)
# - _get_obj_in_hand (uses self.arm - check that generalises)
# - _move_gripper_to_pose (uses self.arm - check that generalises)
# - _get_grasp_pose_for_object (double-check get_grasp_poses_for_object implementation)

from .igibson_semantic_actions_env import iGibsonSemanticActionEnv

import numpy as np
import pybullet as p

from igibson.envs.igibson_env import iGibsonEnv
from igibson.utils.grasp_planning_utils import get_grasp_poses_for_object

from igibson.custom_utils import get_env_config

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
        self.verbose = verbose

    def close(self, container_obj_name): 
        pass

    def place_inside(self, trg_obj_name, container_obj_name): 
        pass
        
    def place_on_top(self, trg_obj_name, container_obj_name): 
        pass

    # if this works fine, move to super class
    def _get_grasp_pose_for_object(self, trg_obj, robot_pos, **kwargs): 
        grasp_poses = get_grasp_poses_for_object(self.robot, trg_obj, force_allow_any_extent=False)
        grasp_pose, object_direction = self.pick_closest_pose(robot_pos, grasp_poses)
        return grasp_pose, object_direction

    # if this works fine, move to super class
    def _move_gripper_to_pose(self, pose): 
        self.robot._parts[self.arm].set_position_orientation(pose[0], pose[1]) # can we even do this?

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
        # The following command might require position controls ([0.001,0.001]) or velocity controls ([-1.0,-1.0]) - try both
        action[self.robot.controller_action_idx[self.robot.default_arm]] = np.array([0.001,0.001]) # symmetric distance from the center of the gripper - for reference the open gripper has coord [0.05,0.05]
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
