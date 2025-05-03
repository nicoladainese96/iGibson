# This file contains all the low-level logic to implement the high-level actions common to all embodiments

import random
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation
from abc import ABC, abstractmethod
from collections import defaultdict

from igibson.envs.igibson_env import iGibsonEnv
from igibson.utils.utils import restoreState
from igibson import object_states
from igibson.object_states.utils import get_center_extent
from igibson.action_primitives.action_primitive_set_base import ActionPrimitiveError
from igibson.utils.grasp_planning_utils import get_grasp_poses_for_object, GUARANTEED_GRASPABLE_WIDTH

from igibson.primitives_utils import open_and_make_all_obj_visible, settle_physics, get_task_objects, sample_point_on_top, sample_until_next_to, sample_point_in_container, place_until_visible, close_container
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
    DEFAULT_BODY_OFFSET_FROM_FLOOR = None
    MAX_STEPS_FOR_GRASP_OR_RELEASE = 10
    
    def __init__(self, task, scene_id, verbose=False):
        env_config = get_env_config()
        env_config["task"] = task
        env_config["scene_id"] = scene_id
        env_config["task_id"] = 0
        env_config["instance_id"] = 0
        robot_name = env_config["robot"]["name"] # Keep this in memory as it's removed from the config when we init iGibson

        # TODO: add grasping_mode="assisted" or "sticky" to be passed all the way to the robot 
        env_config["robot"]["grasping_mode"] = "assisted"
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
    def _move_gripper_to_pose(self, pose): pass

    @abstractmethod
    def _get_obj_in_hand(self): pass

    @abstractmethod
    def _reach_and_grasp(self, trg_obj, object_direction, grasp_orn): pass

    @abstractmethod
    def _execute_grasp(self): pass
        
    @abstractmethod
    def _get_distance_from_robot(self, position): pass

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
            self,
            container_obj,
            max_distance_from_shoulder=self.ROBOT_DISTANCE_THRESHOLD,
            debug=self.verbose,
            **kwargs
        )
        image, symbolic_state = self._finish_action()
        return success, image, symbolic_state

    def close(self, obj_name): 
        container_obj = self.env.task.object_scope[obj_name] 
        success = close_container(
            self,
            container_obj,
            debug=self.verbose,
        )
        image, symbolic_state = self._finish_action()
        return success, image, symbolic_state
        
    def place_inside(self, trg_obj_name, container_obj_name):
        for obj_name in [trg_obj_name, container_obj_name]:
            if obj_name not in self.env.task.object_scope:
                if self.verbose:
                    print(f"Object {obj_name} not in task object scope.")
                image, symbolic_state = self._finish_action()
                return False, image, symbolic_state

        trg_obj = self.env.task.object_scope[trg_obj_name]
        container_obj = self.env.task.object_scope[container_obj_name]

        if not container_obj.states[object_states.Open].get_value():
            if self.verbose:
                print(f"Container {container_obj_name} is not open.")
            image, symbolic_state = self._finish_action()
            return False, image, symbolic_state

        if self._get_obj_in_hand() != trg_obj:
            if self.verbose:
                print(f"Object {trg_obj_name} not in hand.")
            image, symbolic_state = self._finish_action()
            return False, image, symbolic_state

        # Release the object
        for i, action in enumerate(self._release_grasp()):
            if self.verbose:
                print(f"Step {i}: in_hand={self._get_obj_in_hand()}")
            self.robot.apply_action(action)
            self._settle_physics(steps=1)

        # Check if the object is in hand
        if self._get_obj_in_hand() is not None:
            if self.verbose:
                print(f"Unable to release {trg_obj_name} for place-inside action.")
            image, symbolic_state = self._finish_action()
            return False, image, symbolic_state
        
        success = place_until_visible(
            self, trg_obj, container_obj, max_distance_from_shoulder=self.ROBOT_DISTANCE_THRESHOLD,
            debug=self.verbose)

        # Reset the robot pose
        self.robot.reset()
        self.robot.keep_still()
        image, symbolic_state = self._finish_action(do_physics_steps=True)

        return success, image, symbolic_state

    def place_on_top(self, trg_obj_name, container_obj_name):
        for obj_name in [trg_obj_name, container_obj_name]:
            if obj_name not in self.env.task.object_scope:
                if self.verbose:
                    print(f"Object {obj_name} not in task object scope.")
                image, symbolic_state = self._finish_action()
                return False, image, symbolic_state

        trg_obj = self.env.task.object_scope[trg_obj_name]
        container_obj = self.env.task.object_scope[container_obj_name]

        if self._get_obj_in_hand() != trg_obj:
            if self.verbose:
                print(f"Object {trg_obj_name} not in hand.")
            image, symbolic_state = self._finish_action()
            return False, image, symbolic_state

        # Release the object
        for i, action in enumerate(self._release_grasp()):
            if self.verbose:
                print(f"Step {i}: in_hand={self._get_obj_in_hand()}")
            self.robot.apply_action(action)
            self._settle_physics(steps=1)

        # Check if the object is in hand
        if self._get_obj_in_hand() is not None:
            if self.verbose:
                print(f"Unable to release {trg_obj_name} for place-on-top action.")
            image, symbolic_state = self._finish_action()
            return False, image, symbolic_state

        # Does this always work at the first shot?
        candidate_pos = sample_point_on_top(container_obj)
        trg_obj.set_position(candidate_pos)
        # Reset the robot pose
        self.robot.reset()
        self.robot.keep_still()
        # Give object time to fall, if applicable -- some edge cases do need this long
        self._settle_physics(80)
        orientation = trg_obj.sample_orientation()
        trg_obj.set_orientation(orientation)
        trg_obj.force_wakeup()
        image, symbolic_state = self._finish_action(do_physics_steps=True)
        return True, image, symbolic_state

    def place_next_to(self, trg_obj_name, container_obj_name):
        for obj_name in [trg_obj_name, container_obj_name]:
            if obj_name not in self.env.task.object_scope:
                if self.verbose:
                    print(f"Object {obj_name} not in task object scope.")
                image, symbolic_state = self._finish_action()
                return False, image, symbolic_state

        trg_obj = self.env.task.object_scope[trg_obj_name]
        container_obj = self.env.task.object_scope[container_obj_name]

        if self._get_obj_in_hand() != trg_obj:
            if self.verbose:
                print(f"Object {trg_obj_name} not in hand.")
            image, symbolic_state = self._finish_action()
            return False, image, symbolic_state

        # Release the object
        for i, action in enumerate(self._release_grasp()):
            if self.verbose:
                print(f"Step {i}: in_hand={self._get_obj_in_hand()}")
            self.robot.apply_action(action)
            self._settle_physics(steps=1)

        # Check if the object is in hand
        if self._get_obj_in_hand() is not None:
            if self.verbose:
                print(f"Unable to release {trg_obj_name} for place-on-top action.")
            image, symbolic_state = self._finish_action()
            return False, image, symbolic_state

        success = sample_until_next_to(
            self, trg_obj, container_obj, max_distance=self.ROBOT_DISTANCE_THRESHOLD, 
            debug=self.verbose)

        # Reset the robot pose
        self.robot.reset()
        self.robot.keep_still()
        image, symbolic_state = self._finish_action(do_physics_steps=True)

        return success, image, symbolic_state
        
    # TODO: debug and double-check
    def grasp(self, obj_name, forward_downward_dir=None, camera_offset=None, distance_from_camera=0.2, sample_budget=20):
        """
        Attempts to grasp an object by sampling up to `sample_budget` grasp poses.
        Returns (success, image, symbolic_state).
        """
        trg_obj = self.env.task.object_scope[obj_name]
    
        # Get robot base position and orientation
        robot_pos, quat_rot = render_utils.get_robot_pos_and_q_rotation(self.robot)
        if self.verbose:
            print(f"robot_pos: {robot_pos}, quaternion_pose: {quat_rot}")
    
        # Default directions
        if forward_downward_dir is None:
            forward_downward_dir = np.array([1, 0, -0.25])
        if camera_offset is None:
            camera_offset = np.array([0.1, 0.1, 1.2]) # z dimension is hardcoded

        # Compute and set object position in front of robot
        target_obj_pos = (
            np.array([robot_pos[0], robot_pos[1], 0])
            + quat_rot.rotate(camera_offset)
            + quat_rot.rotate(forward_downward_dir) * distance_from_camera
        )
            
        # Try multiple grasp attempts
        last_image, last_state = None, None

        # Save state of the environment for resets between attempts
        robot_data = self.robot.dump_state()
        env_state = p.saveState()

        # For temporary debug - close the gripper and then revert the state
        #if self.verbose:
        #    max_steps = 10
        #    for i, action in enumerate(self._execute_grasp()):
        #        
        #        # Grasp in the air
        #        self.robot.apply_action(action)
        #        self._settle_physics(steps=1)
        #    
        #        # Render
        #        render_utils.render_frame_with_trg_obj(
        #            self.env, trg_obj,
        #            show=True, save=True,
        #            name=f'grasping_check_{i}'
        #        )
        #        
        #        # Exit at most after max_steps
        #        if i+1 == max_steps:
        #            break
        #            
        #    # Revert the state
        #    self.robot.load_state(robot_data)
        #    restoreState(env_state)
        #    self._settle_physics(steps=1)
            
        for attempt in range(1, sample_budget + 1):
            if self.verbose:
                print(f"\n--- Grasp attempt {attempt}/{sample_budget} ---")

            if attempt > 1:
                # Always reset after every unsuccessful attempt - doesn't really seem to work that well
                self.robot.load_state(robot_data)
                restoreState(env_state)
                self._settle_physics(steps=1)
                
            trg_obj.set_position(target_obj_pos) # is it needed here? I guess it's okay
            if self.verbose:
                print(f"Target object position: {target_obj_pos}")
                render_utils.render_frame_with_trg_obj(
                    self.env, trg_obj,
                    show=True, save=True,
                    add_bbox=True,
                    name=f'object_to_grasp_t0_attempt{attempt}'
                )

            # Get grasp pose and direction
            pick_closest = True if attempt == 1 else False
            grasp_pose, obj_dir = self._get_grasp_pose_for_object(trg_obj, robot_pos, pick_closest=pick_closest, render=self.verbose)
            if self.verbose:
                print(f"grasp_pose: {grasp_pose} - object_direction: {obj_dir}")
            
            # Run one grasp trial
            success, img, state = self._attempt_grasp_once(
                trg_obj, target_obj_pos, grasp_pose, obj_dir,
                forward_downward_dir, camera_offset,
                distance_from_camera
            )
    
            last_image, last_state = img, state
            if success:
                if self.verbose:
                    print(f"Grasp succeeded on attempt {attempt}.")
                return True, img, state
            else:
                if self.verbose:
                    print(f"Grasp failed on attempt {attempt}, retrying next sample.")
        # All attempts failed
        if self.verbose:
            print("All grasp attempts failed.")
        return False, last_image, last_state
    
    
    def _attempt_grasp_once(
        self, trg_obj, target_obj_pos, grasp_pose, obj_dir,
        forward_downward_dir, camera_offset, distance_from_camera, steps_obj_in_place=2
    ):
        """
        Single grasp attempt: move gripper, reach, and grasp.
        Returns (success, image, symbolic_state).
        """
    
        # Move gripper close to the grasp pose without rendering
        self._move_gripper_to_pose(grasp_pose)
    
        # Execute reach and grasp actions
        for i, action in enumerate(self._reach_and_grasp(trg_obj, obj_dir, grasp_orn=grasp_pose[1])):
            if self.verbose:
                print(
                    f"Step {i}: in_hand={self._get_obj_in_hand()} | "
                    f"is_grasping={self.robot.is_grasping(candidate_obj=trg_obj)}" # this is shit
                )
            
            if i < steps_obj_in_place:
                # Keep object in place
                trg_obj.set_position(target_obj_pos)
            self.robot.apply_action(action)
            self._settle_physics(steps=1)
            # Only render final step if verbose
            if self.verbose and i%2 == 0:
                render_utils.render_frame_with_trg_obj(
                    self.env, trg_obj, show=True, save=True,
                    add_bbox=True, name=f'object_to_grasp_step{i}'
                )
    
        # Check grasp success
        success = (self._get_obj_in_hand() is trg_obj) # how is this _get_obj_in_hand computed in Fetch?
        # Finish action and get outputs
        image, symbolic_state = self._finish_action()
        return success, image, symbolic_state

    def _settle_physics(self, *args, **kwargs):
        settle_physics(self.env, *args, **kwargs)
        
    def _finish_action(self, do_physics_steps=False, steps=2):
        if do_physics_steps:
            self._settle_physics(steps)
        render_utils.set_camera_look_ahead(self.env)
        image = render_utils.get_image_from_camera(self.env.simulator)
        symbolic_state = self._get_symbolic_state()
        return image, symbolic_state

    def _get_symbolic_state(self):
        """
        Returns a dict:
          predicate_name -> { "arg1[,arg2]" : bool, ... }
        Only non-None results are stored.
        """
        state = defaultdict(dict)
        objs = get_task_objects(self.env)

        # unary - all commented one are computable but not needed
        uni_specs = {
            #'is_near':      self._is_near,
            #'is_visible':   self._is_visible,
            'reachable': self._is_reachable,
            'holding':   self._is_holding,
            #'is_movable':   self._is_movable,
            #'is_openable':  self._is_openable,
            'open':      self._is_open,
        }
        for o in objs:
            for pred, fn in uni_specs.items():
                v = fn(o)
                if v is not None:
                    state[pred][o] = v

        # binary
        bin_specs = {
            'ontop':  self._is_ontop,
            'inside': self._is_inside,
            'nextto': self._is_nextto,
        }

        # These need to be checked as well for the ontop predicate
        floors = [name for name in list(self.env.task.object_scope.keys()) if 'floor' in name]
        for pred, fn in bin_specs.items():
            d = defaultdict(bool)
            for a in objs:
                if pred == 'ontop':
                    obj_list = objs+floors
                else:
                    obj_list = objs
                for b in obj_list:
                    if a == b:
                        continue
                    v = fn(a, b)
                    if v is not None:
                        d[f"{a},{b}"] = v
            if d:
                state[pred] = d

        return state

    def get_obj_from_name(self, obj_name):
        return self.env.task.object_scope[obj_name]

    def _is_near(self, obj_name):
        obj = self.get_obj_from_name(obj_name)
        d = self._get_distance_from_robot(obj.get_position())
        return d < self.ROBOT_DISTANCE_THRESHOLD

    def _is_visible(self, obj_name):
        obj = self.get_obj_from_name(obj_name)
        return obj.states[object_states.IsVisible].get_value(env=self.env)

    def _is_reachable(self, obj_name):
        return self._is_near(obj_name) and self._is_visible(obj_name)

    def _is_holding(self, obj_name):
        if not self._is_movable(obj_name):
            return None
        obj = self.get_obj_from_name(obj_name)
        return self._get_obj_in_hand() is obj

    def _is_movable(self, obj_name):
        obj = self.get_obj_from_name(obj_name)
        _, _, extents, _ = obj.get_base_aligned_bounding_box(visual=False)
        return np.any(extents < GUARANTEED_GRASPABLE_WIDTH)

    def _is_openable(self, obj_name):
        obj = self.get_obj_from_name(obj_name)
        return object_states.Open in obj.states

    def _is_open(self, obj_name):
        if not self._is_openable(obj_name):
            return None
        obj = self.get_obj_from_name(obj_name)
        return obj.states[object_states.Open].get_value()

    def _is_ontop(self, obj_name, below_name):
        if not self._is_movable(obj_name):
            return None
        obj   = self.get_obj_from_name(obj_name)
        below = self.get_obj_from_name(below_name)
        return obj.states[object_states.OnTop].get_value(below)

    def _is_inside(self, obj_name, container_name):
        if not (self._is_movable(obj_name) and self._is_openable(container_name)):
            return None
        obj       = self.get_obj_from_name(obj_name)
        container = self.get_obj_from_name(container_name)
        return obj.states[object_states.Inside].get_value(container)

    def _is_nextto(self, obj_name, nextto_name):
        if not (self._is_movable(obj_name)):
            return None
        obj       = self.get_obj_from_name(obj_name)
        obj_nextto = self.get_obj_from_name(nextto_name)
        return obj.states[object_states.next_to.NextTo].get_value(obj_nextto)

    def _release_grasp(self):
        action = np.zeros(self.robot.action_dim)
        # opposite of close gripper
        action[self.robot.controller_action_idx[f"gripper_{self.robot.default_arm}"]] = np.array([1.0])
        for _ in range(self.MAX_STEPS_FOR_GRASP_OR_RELEASE):
            yield action

    def _get_robot_pose_from_2d_pose(self, pose_2d):
        pos = np.array([pose_2d[0], pose_2d[1], self.DEFAULT_BODY_OFFSET_FROM_FLOOR])
        orn = p.getQuaternionFromEuler([0, 0, pose_2d[2]])
        return pos, orn

    def _get_grasp_pose_for_object(self, trg_obj, robot_pos, pick_closest=False, render=False, **kwargs): 
        grasp_poses = get_grasp_poses_for_object(self.robot, trg_obj, force_allow_any_extent=False)

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

        for yaw_range in [45, 60, 90]:
            for _ in range(MAX_ATTEMPTS_FOR_SAMPLING_POSE_NEAR_OBJECT//3):
                pos_on_obj = np.array(self._sample_position_on_aabb_face(obj))
                obj_rooms = obj.in_rooms if obj.in_rooms else [self.scene.get_room_instance_by_point(pos_on_obj[:2])]
        
                yaw = np.random.uniform(-yaw_range, yaw_range)
                distance = np.random.uniform(0.4, 1.1) # max was 0.9, bumped up to 1.1
        
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
    def _detect_collision(body_id, obj_in_hand=None, tol=1e-2): # 1 cm of tolerance is quite high
        collision = []
        obj_in_hand_id = None
        if obj_in_hand is not None:
            [obj_in_hand_id] = obj_in_hand.get_body_ids()
    
        for other_body_id in range(p.getNumBodies()):
            if other_body_id == body_id or other_body_id == obj_in_hand_id:
                continue
    
            contact_points = p.getContactPoints(body_id, other_body_id)
            for contact in contact_points:
                contact_distance = contact[8]  # 9th element is contactDistance
                if contact_distance < 0 and abs(contact_distance) > tol:
                    collision.append(other_body_id)
                    break  # Found a meaningful collision, move on
    
        return collision


    
        
    
    
        
    
