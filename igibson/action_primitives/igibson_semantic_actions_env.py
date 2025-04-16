# This file contains all the low-level logic to implement the high-level actions common to all embodiments

import random
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation
from abc import ABC, abstractmethod

from igibson.envs.igibson_env import iGibsonEnv
from igibson.utils.utils import restoreState
from igibson import object_states
from igibson.object_states.utils import get_center_extent
from igibson.action_primitives.action_primitive_set_base import ActionPrimitiveError

from igibson.primitives_utils import open_and_make_all_obj_visible, settle_physics
import igibson.render_utils as render_utils
from igibson.custom_utils import get_env_config

MAX_ATTEMPTS_FOR_SAMPLING_POSE_NEAR_OBJECT = 120
PHYSICS_STEPS = 3

class UndoableContext(object):
    def __init__(self, robot):
        self.robot = robot

    def __enter__(self):
        self.robot_data = self.robot.dump_state()
        self.state = p.saveState()

    def __exit__(self, *args):
        self.robot.load_state(self.robot_data)
        restoreState(self.state)
        p.removeState(self.state)
        
class iGibsonSemanticActionEnv(ABC):
    ROBOT_DISTANCE_THRESHOLD = None

    def __init__(self, task, scene_id, verbose=False):
        env_config = get_env_config()
        env_config["task"] = task
        env_config["scene_id"] = scene_id
        env_config["task_id"] = 0
        env_config["instance_id"] = 0
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

    # Some methods are left as abstract just because are still to be implemented, others because they are embodiment-depenedent
    @abstractmethod
    def close(self, container_obj_name): pass

    @abstractmethod
    def place_inside(self, trg_obj_name, container_obj_name): pass
        
    @abstractmethod
    def place_on_top(self, trg_obj_name, container_obj_name): pass
        
    @abstractmethod
    def _get_grasp_pose_for_object(self, trg_obj, robot_pos, **kwargs): pass
        
    @abstractmethod
    def _move_gripper_to_pose(self, pose): pass

    @abstractmethod
    def _get_obj_in_hand(self): pass

    @abstractmethod
    def _reach_and_grasp(trg_obj, object_direction, hand_orn): pass

    @abstractmethod
    def _get_distance_from_robot(self, position): pass

    @abstractmethod
    def _get_robot_pose_from_2d_pose(self, pose_2d): pass

    @abstractmethod
    def _detect_robot_collision(self): pass

    def apply_action(self, action):
        if hasattr(self, action['action']):
            return getattr(self, action['action'])(**action['params'])
        else:
            raise ValueError(f"Unknown action '{action['action']}' in plan.")

    def get_state_and_image(self):
        return self._finish_action()
        
    def go_to(self, obj_name): 
        trg_obj = self.env.task.object_scope[obj_name] 
        
        pose_2d = self._sample_pose_near_object(trg_obj) 
        self.robot.set_position_orientation(*self._get_robot_pose_from_2d_pose(pose_2d))
        success = True
        image, symbolic_state = self._finish_action(do_physics_steps=True)
        return success, image, symbolic_state
        
    def open(self, obj_name, **kwargs): 
        container_obj = self.env.task.object_scope[obj_name] 
        
        success = open_and_make_all_obj_visible(
            self.env,
            container_obj,
            max_distance_from_shoulder=self.ROBOT_DISTANCE_THRESHOLD,
            debug=self.verbose,
            **kwargs
        )
        image, symbolic_state = self._finish_action()
        return success, image, symbolic_state

    def grasp(self, obj_name, forward_downward_dir=None, camera_offset=None, distance_from_camera=0.2): 
        trg_obj = self.env.task.object_scope[obj_name] 
        
        # Get robot position
        robot_pos, quaternion_pose = render_utils.get_robot_pos_and_q_rotation(self.robot)
        if self.verbose: print(f"robot_pos: {robot_pos}, quaternion_pose: {quaternion_pose}")
        
        # Set trg obj position to be in front of the robot - hopefully it doesn't collide with anything 
        # TODO: make it more robust via sampling (e.g. in a 3d sphere centered in the best position) + constraint checking
        if forward_downward_dir is None:
            forward_downward_dir = np.array([1, 0, -0.25]) # direction in which the camera is looking
        if camera_offset is None:
            camera_offset = np.array([0.1, 0.1, 0.7])
            
        trg_obj_pos = robot_pos + quaternion_pose.rotate(camera_offset) + quaternion_pose.rotate(forward_downward_dir)*distance_from_camera
        trg_obj.set_position(trg_obj_pos) # Move object to target position
        if self.verbose: print(f"Target trg_obj_pos: {trg_obj_pos}")
        
        # Render after setting position, but before doing any physics step
        if self.verbose: render_utils.render_frame_with_trg_obj(self.env, trg_obj, show=True, save=True, add_bbox=True, name='object_to_grasp_t0')
        
        # Get object grasp pose
        grasp_pose, object_direction = self._get_grasp_pose_for_object(trg_obj, robot_pos) 
        if self.verbose: print(f"grasp_pose: {grasp_pose} - object_direction: {object_direction}")
        
        self._move_gripper_to_pose(grasp_pose) # Move gripper to target pose
        
        action_generator = self._reach_and_grasp(trg_obj, object_direction, grasp_orn=grasp_pose[1])
        for i, action in enumerate(action_generator):
            if self.verbose: print(f"Step {i} - object in hand: {self._get_obj_in_hand()}")
            trg_obj.set_position(trg_obj_pos) # keep the object still, while trying to grasp it - not sure it's needed
            self.robot.apply_action(action)
            
            self._settle_physics(steps=1)
            
            if self.verbose: render_utils.render_frame_with_trg_obj(self.env, trg_obj, show=True, save=True, add_bbox=True, name=f'object_to_grasp_t{i+1}')

        # How do we check that the object in hand is the right one (condition to return success)
        if self._get_obj_in_hand() is trg_obj:
            success = True
        else:
            success = False

        image, symbolic_state = self._finish_action()
        return success, image, symbolic_state

    def _settle_physics(self, *args):
        settle_physics(self.env, *args)
        
    def _finish_action(self, do_physics_steps=False, steps=2):
        if do_physics_steps:
            self._settle_physics(steps)
        render_utils.set_camera_look_ahead(self.env)
        image = render_utils.get_image_from_camera(self.env.simulator)
        symbolic_state = self._get_symbolic_state()
        return image, symbolic_state

    def _get_symbolic_state(self):
        # TODO: implement it in such a way that it returns all the predicates needed for the ground-truth planner/state
        return None
    
    def _sample_pose_near_object(self, obj):
        object_center, orientation = obj.get_position_orientation()
    
        def point_in_front_with_local_yaw(position, orientation, distance=1.0, yaw_offset_deg=0.0, local_forward=[0, 1, 0]):
            def wxyz_to_xyzw(quat_wxyz):
                w, x, y, z = quat_wxyz
                return [x, y, z, w]
    
            yaw_offset_rad = np.radians(yaw_offset_deg)
            local_yaw_rot = Rotation.from_euler('z', yaw_offset_rad).apply(local_forward)
            orientation_xyzw = wxyz_to_xyzw(orientation)
            world_direction = p.rotateVector(orientation_xyzw, local_yaw_rot)
    
            position = np.array(position)
            world_direction = np.array(world_direction)
    
            return position + distance * world_direction
    
        for _ in range(MAX_ATTEMPTS_FOR_SAMPLING_POSE_NEAR_OBJECT):
            pos_on_obj = np.array(self._sample_position_on_aabb_face(obj))
            obj_rooms = obj.in_rooms if obj.in_rooms else [self.scene.get_room_instance_by_point(pos_on_obj[:2])]
    
            yaw = np.random.uniform(-45, 45)
            distance = np.random.uniform(0.4, 0.9)
    
            pose_2d = point_in_front_with_local_yaw(object_center, orientation, distance, yaw_offset_deg=yaw)
            new_yaw = np.arctan2(object_center[1] - pose_2d[1], object_center[0] - pose_2d[0])
            pose_2d[2] = new_yaw
    
            if not self._test_pose(pose_2d, obj, pos_on_obj=pos_on_obj):
                continue
    
            return pose_2d
    
        raise ActionPrimitiveError(
            ActionPrimitiveError.Reason.SAMPLING_ERROR, "Could not find valid position near object."
        )

    def _test_pose(self, pose_2d, obj, pos_on_obj=None):
        with UndoableContext(self.robot):
            self.robot.set_position_orientation(*self._get_robot_pose_from_2d_pose(pose_2d))

            # Run some steps to let physics settle
            s = self.env.simulator
            for _ in range(PHYSICS_STEPS):
                s.step() 
            
            #if not obj.states[object_states.InFOVOfRobot].get_value(): # TODO: should we use isVisible instead?
            if not obj.states[object_states.IsVisible].get_value(env=self.env):
                if self.verbose: print("Object not visible.")
                return False
                
            if pos_on_obj is not None:
                robot_distance = self._get_distance_from_robot(pos_on_obj) # this is robot-dependent
                if robot_distance > self.ROBOT_DISTANCE_THRESHOLD:
                    if self.verbose: print("Candidate position failed robot distance test.")
                    return False

            if self._detect_robot_collision():
                if self.verbose: print("Candidate position failed collision test.")
                return False

            return True

    def _get_pose_in_robot_frame(self, pose):
        body_pose = self.robot.get_position_orientation()
        world_to_body_frame = p.invertTransform(*body_pose)
        relative_target_pose = p.multiplyTransforms(*world_to_body_frame, *pose)
        return relative_target_pose
        
    @staticmethod
    def _sample_position_on_aabb_face(target_obj):
        aabb_center, aabb_extent = get_center_extent(target_obj.states)
        # We want to sample only from the side-facing faces.
        face_normal_axis = random.choice([0, 1])
        face_normal_direction = random.choice([-1, 1])
        face_center = aabb_center + np.eye(3)[face_normal_axis] * (aabb_extent*0.5) * face_normal_direction
        face_lateral_axis = 0 if face_normal_axis == 1 else 1
        face_lateral_half_extent = np.eye(3)[face_lateral_axis] * aabb_extent / 2
        face_vertical_half_extent = np.eye(3)[2] * aabb_extent / 2
        face_min = face_center - face_vertical_half_extent - face_lateral_half_extent
        face_max = face_center + face_vertical_half_extent + face_lateral_half_extent
        return np.random.uniform(face_min, face_max)

    @staticmethod
    def pick_closest_pose(robot_pos, poses):
        positions = np.stack([pose_and_dist[0][0] for pose_and_dist in poses]) # (n_poses, 3)
        distances = np.linalg.norm(positions-robot_pos)
        closest = np.argmin(distances)
        return poses[closest]

    @staticmethod
    def _detect_collision(body, obj_in_hand=None):
        collision = []
        obj_in_hand_id = None
        if obj_in_hand is not None:
            [obj_in_hand_id] = obj_in_hand.get_body_ids()
        for body_id in range(p.getNumBodies()):
            if body_id == body or body_id == obj_in_hand_id:
                continue
            closest_points = p.getClosestPoints(body, body_id, distance=0.01)
            if len(closest_points) > 0:
                collision.append(body_id)
                break
        return collision
    

    
        
    
    
        
    
