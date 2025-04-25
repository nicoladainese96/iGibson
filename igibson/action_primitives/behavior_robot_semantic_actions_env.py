# Port here all the useful stuff inside action_primitives/starter_semantic_action_primitives.py
from .igibson_semantic_actions_env import iGibsonSemanticActionEnv

import numpy as np
import pybullet as p

from igibson.robots.behavior_robot import DEFAULT_BODY_OFFSET_FROM_FLOOR, BehaviorRobot
from igibson.robots import BaseRobot, behavior_robot
from igibson.utils.grasp_planning_utils import get_grasp_poses_for_object

class BehaviorRobotSemanticActionEnv(iGibsonSemanticActionEnv):
    ROBOT_DISTANCE_THRESHOLD = behavior_robot.HAND_DISTANCE_THRESHOLD # actually it's just not needed anymore as we don't need the object to be really near to be grasped
    DEFAULT_BODY_OFFSET_FROM_FLOOR = DEFAULT_BODY_OFFSET_FROM_FLOOR # make it a class attribute
    arm = 'right_hand'
    
    def close(self, container_obj_name): 
        pass

    def place_inside(self, trg_obj_name, container_obj_name): 
        pass
        
    def place_on_top(self, trg_obj_name, container_obj_name): 
        pass

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
        action[self.robot.controller_action_idx["gripper_right_hand"]] = -1.0 # fix this
        for _ in range(self.MAX_STEPS_FOR_GRASP_OR_RELEASE):
            yield action

        # Do nothing for a bit so that AG can trigger. - this was in the original code, but I don't get what it's the purpose of it
        #for _ in range(MAX_WAIT_FOR_GRASP_OR_RELEASE):
        #    yield np.zeros(self.robot.action_dim)

    def _get_obj_in_hand(self):
        obj_in_hand_id = self.robot._ag_obj_in_hand[self.arm] #["right_hand"]  
        obj_in_hand = self.scene.objects_by_id[obj_in_hand_id] if obj_in_hand_id is not None else None
        return obj_in_hand
        
    def _move_gripper_to_pose(self, pose): 
        # Teleport right hand in right pose - TODO: add assisted/sticky grip to Behavior Robot at initialization
        self.robot._parts[self.arm].set_position_orientation(pose[0], pose[1]) # no idea how else to do that

    def _get_grasp_pose_for_object(self, trg_obj, robot_pos, **kwargs): 
        grasp_poses = get_grasp_poses_for_object(self.robot, trg_obj, force_allow_any_extent=False)
        grasp_pose, object_direction = self.pick_closest_pose(robot_pos, grasp_poses)
        return grasp_pose, object_direction
        
    def _get_distance_from_robot(self, pos):
        shoulder_pos_in_base_frame = np.array(
            self.robot.links["%s_shoulder" % self.arm].get_local_position_orientation()[0]
        )
        point_in_base_frame = np.array(self._get_pose_in_robot_frame((pos, [0, 0, 0, 1]))[0])
        shoulder_to_hand = point_in_base_frame - shoulder_pos_in_base_frame
        return np.linalg.norm(shoulder_to_hand)
        
    def _detect_robot_collision(self):
        if self.verbose: print("Start collision test.")
            
        body = self._detect_collision(self.robot.links["body"].body_id)
        if body:
            if self.verbose: print("Body has collision with objects %s", body)
                
        left = self._detect_collision(self.robot.eef_links["left_hand"].body_id)
        if left:
            if self.verbose: print("Left hand has collision with objects %s", left)
                
        right = self._detect_collision(self.robot.eef_links[self.arm].body_id, self._get_obj_in_hand())
        if right:
            if self.verbose: print("Right hand has collision with objects %s", right)
                
        if self.verbose: print("End collision test.")
        return body or left or right