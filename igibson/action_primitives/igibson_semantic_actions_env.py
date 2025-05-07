import random
import itertools
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
from igibson.utils.grasp_planning_utils import get_grasp_poses_for_object

from igibson.primitives_utils import open_and_make_all_obj_visible, settle_physics, get_task_objects, sample_point_on_top, sample_until_next_to, sample_point_in_container, place_until_visible, close_container
import igibson.render_utils as render_utils
from igibson.custom_utils import get_env_config

# Only for debugging
from igibson.object_states.robot_related_states import compute_projected_area

MAX_ATTEMPTS_FOR_SAMPLING_POSE_NEAR_OBJECT = 120
PHYSICS_STEPS = 3
GUARANTEED_GRASPABLE_WIDTH = 0.18

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
    SIM_ENV_ACTION_NAMES = {
        'grasp':'grasp',
        'place-on':'place_on_top',
        'place-next-to':'place_next_to',
        'place-inside':'place_inside',
        'open-container':'open',
        'close-container':'close',
        'navigate-to':'go_to',
        'slice':'slice' # not implemented yet, but let's see
    }
    
    def __init__(self, task, scene_id, instance_id=0, verbose=False, debug=False):
        env_config = get_env_config()
        env_config["task"] = task
        env_config["scene_id"] = scene_id
        env_config["task_id"] = 0
        env_config["instance_id"] = instance_id
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
        self.debug = debug # activates assert statements at the end of the actions

    # Some methods are left as abstract just because are still to be implemented, others because they are embodiment-depenedent
    @abstractmethod
    def slice(self, trg_obj_name): pass
    
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
        """
        action: dict with following entries
          action['action']: str
          action['params']: list[str]
        """
        # Translate from pddl convention to internal methods and objects' names
        # e.g. 
        translated_action = self.translate_action(action)
        
        if hasattr(self, translated_action['action']):
            success, info, image, symbolic_state = getattr(self, translated_action['action'])(*translated_action['params']) 
            # Convert state back to pddl notation 
            translated_symbolic_state = self.translate_symbolic_state(symbolic_state)
            return success, info, image,translated_symbolic_state
        else:
            raise ValueError(f"Unknown action '{translated_action['action']}' in plan.")
        
    def translate_action(self, action):
        action_name = action['action'] # str
        action_args = action['params'] # list[str]
        
        # Translate action_name
        translated_action_name = self.SIM_ENV_ACTION_NAMES[action_name]
        
        # Process action arguments

        # Loop over action_args and translate them to sim_env names
        object_names = list(self.env.task.object_scope.keys()) # this should include floors as well
    
        translated_action_args = []
        for arg_name in action_args:
            translated_arg = self.match_name_to_sim_env(arg_name, object_names)
            translated_action_args.append(translated_arg)
    
        sim_env_action = {'action':translated_action_name, 'params':translated_action_args}
        return sim_env_action
    
    def get_state_and_image(self):
        image, symbolic_state = self._finish_action()
        # Translate back to pddl convention
        translated_symbolic_state = self.translate_symbolic_state(symbolic_state)
        return image, translated_symbolic_state
        
    def go_to(self, obj_name): 
        # Check that object exist
        if obj_name not in self.env.task.object_scope:
            print(f"Object {obj_name} not in task object scope.")
            image, symbolic_state = self._finish_action()
            legal = False
            return legal, "not legal", image, symbolic_state

        # Enforce preconditions: don't navigate to things hidden in a closed container
        # Skip, the action is going to fail gracefully in case anyways
        
        trg_obj = self.env.task.object_scope[obj_name] 

        # Execute action
        pose_2d = self._sample_pose_near_object(trg_obj) 
        if pose_2d is not None:
            self.robot.set_position_orientation(*self._get_robot_pose_from_2d_pose(pose_2d))
            success = True
            image, symbolic_state = self._finish_action(do_physics_steps=True)

            if self.debug:
                assert symbolic_state['reachable'][obj_name] == True, "Action succeeded but target object is not reachable!"
            
        else:
            success = False
            image, symbolic_state = self._finish_action(do_physics_steps=True)

        # Last sanity check
        if not symbolic_state['reachable'][obj_name]:
            success = False
            
        if success:
            info = 'success'
        else:
            info = 'executed but failed'
        legal = True
        return legal, info, image, symbolic_state
        
    def open(self, obj_name, **kwargs): 
        # Check that object exist
        if obj_name not in self.env.task.object_scope:
            print(f"Object {obj_name} not in task object scope.")
            image, symbolic_state = self._finish_action()
            legal = False
            return legal, "not legal", image, symbolic_state

        container_obj = self.env.task.object_scope[obj_name] 
        
        # Enforce preconditions: reachable, not open, empty_hand
        #if self.verbose:
        #    distance_from_container = self._get_obj_distance_from_robot(container_obj)
        #    print("distance_from_container: ", distance_from_container)
        #    is_near = distance_from_container < self.ROBOT_DISTANCE_THRESHOLD
        #    print("is_near: ", is_near)
        #    _is_near = self._is_near(obj_name)
        #    print("_is_near: ", _is_near)
        #
        #    # Visibility sanity check
        #    total_visible_pixels = container_obj.states[object_states.VisiblePixelCountFromRobot].get_value()
        #    print("total_visible_pixels: ", total_visible_pixels)
        #    bbox_vertices_uv = render_utils.get_bbox_vertices_uv(self.env, container_obj)
        #    print("bbox_vertices_uv: ", bbox_vertices_uv)
        #    # Clip vertices within image bounds
        #    H = self.env.config['image_height']
        #    W = self.env.config['image_width']
        #    
        #    projected_bbox_pixels = compute_projected_area(bbox_vertices_uv, H, W)
        #    print("projected_bbox_pixels: ", projected_bbox_pixels)
        #    ratio = total_visible_pixels / (projected_bbox_pixels + 1)
        #    print("ratio: ", ratio)
        #    print("threshold: ", 0.08)
        #    is_visible = self._is_visible(obj_name)
        #    print("is_visible: ", is_visible)
            
        reachable = self._is_reachable(obj_name)
        is_open = self._is_open(obj_name)
        empty_hand = self._get_obj_in_hand() is None
        if not reachable or is_open or not empty_hand:
            print(f"Preconditions not satisfied. reachable={reachable} ; is_open={is_open} ; empty_hand={empty_hand}")
            image, symbolic_state = self._finish_action()
            legal = False
            return legal, "not legal", image, symbolic_state

        # Execute action
        success = open_and_make_all_obj_visible(
            self,
            container_obj,
            max_distance_from_shoulder=self.ROBOT_DISTANCE_THRESHOLD,
            debug=self.verbose,
            **kwargs
        )
        image, symbolic_state = self._finish_action()
        
        if success and self.debug:
            assert symbolic_state['open'][obj_name] == True, "Action succeeded but target object is not open!"

        # Last sanity check
        if not symbolic_state['open'][obj_name]:
            success = False

        if success:
            info = 'success'
        else:
            info = 'executed but failed'
        legal = True
        return legal, info, image, symbolic_state

    def close(self, obj_name): 
        # Check that object exist
        if obj_name not in self.env.task.object_scope:
            print(f"Object {obj_name} not in task object scope.")
            image, symbolic_state = self._finish_action()
            legal = False
            return legal, "not legal", image, symbolic_state

        container_obj = self.env.task.object_scope[obj_name] 

        #if self.verbose:
        #    # Distance sanity check
        #    distance_from_container = self._get_obj_distance_from_robot(container_obj)
        #    print("distance_from_container: ", distance_from_container)
        #    is_near = distance_from_container < self.ROBOT_DISTANCE_THRESHOLD
        #    print("is_near: ", is_near)
        #    _is_near = self._is_near(obj_name)
        #    print("_is_near: ", _is_near)
        #
        #    # Visibility sanity check
        #    total_visible_pixels = container_obj.states[object_states.VisiblePixelCountFromRobot].get_value()
        #    print("total_visible_pixels: ", total_visible_pixels)
        #    bbox_vertices_uv = render_utils.get_bbox_vertices_uv(self.env, container_obj)
        #    print("bbox_vertices_uv: ", bbox_vertices_uv)
        #    # Clip vertices within image bounds
        #    H = self.env.config['image_height']
        #    W = self.env.config['image_width']
        #    
        #    projected_bbox_pixels = compute_projected_area(bbox_vertices_uv, H, W)
        #    print("projected_bbox_pixels: ", projected_bbox_pixels)
        #    ratio = total_visible_pixels / (projected_bbox_pixels + 1)
        #    print("ratio: ", ratio)
        #    print("threshold: ", 0.08)
        #    is_visible = self._is_visible(obj_name)
        #    print("is_visible: ", is_visible)
            
        # Enforce preconditions: reachable, open, empty_hand
        reachable = self._is_reachable(obj_name)
        is_open = self._is_open(obj_name)
        empty_hand = self._get_obj_in_hand() is None
        if not reachable or not is_open or not empty_hand:
            print(f"Preconditions not satisfied. reachable={reachable} ; is_open={is_open} ; empty_hand={empty_hand}")
            image, symbolic_state = self._finish_action()
            legal = False
            return legal, "not legal", image, symbolic_state
        
        # Execute action
        success = close_container(
            self,
            container_obj,
            debug=self.verbose,
        )
        image, symbolic_state = self._finish_action()
        
        if success and self.debug:
            assert symbolic_state['open'][obj_name] == False, "Action succeeded but target object is not closed!"

        # Last sanity check
        if symbolic_state['open'][obj_name]: # should be False, i.e. closed
            success = False
            
        if success:
            info = 'success'
        else:
            info = 'executed but failed'
        legal = True
        return legal, info, image, symbolic_state
        
    def place_inside(self, trg_obj_name, container_obj_name):
        # Check that objects exist
        for obj_name in [trg_obj_name, container_obj_name]:
            if obj_name not in self.env.task.object_scope:
                print(f"Object {obj_name} not in task object scope.")
                image, symbolic_state = self._finish_action()
                legal = False
                return legal, "not legal", image, symbolic_state

        trg_obj = self.env.task.object_scope[trg_obj_name]
        container_obj = self.env.task.object_scope[container_obj_name]

        # Check precondition: open(container_obj), reachable(container_obj), holding(trg_obj)
        reachable = self._is_reachable(container_obj_name)
        is_open = self._is_open(container_obj_name)
        holding_trg_obj = self._is_holding(trg_obj_name)
        if not reachable or not is_open or not holding_trg_obj:
            print(f"Preconditions not satisfied. reachable={reachable} ; is_open={is_open} ; holding_trg_obj={holding_trg_obj}")
            image, symbolic_state = self._finish_action()
            legal = False
            return legal, "not legal", image, symbolic_state

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
            legal = True
            info = 'executed but failed'
            return legal, info, image, symbolic_state
        
        success = place_until_visible(
            self, trg_obj, container_obj, max_distance_from_shoulder=self.ROBOT_DISTANCE_THRESHOLD,
            debug=self.verbose)

        # Reset the robot pose
        self.robot.reset()
        self.robot.keep_still()
        image, symbolic_state = self._finish_action(do_physics_steps=True)

        if success and self.debug:
            assert symbolic_state['inside'][f'{trg_obj_name},{container_obj_name}'] == True, "Action succeeded but target object is not inside container object!"

        # Last sanity check
        if not symbolic_state['inside'][f'{trg_obj_name},{container_obj_name}']:
            success = False
            
        if success:
            info = 'success'
        else:
            info = 'executed but failed'
        legal = True
        return legal, info, image, symbolic_state

    def place_on_top(self, trg_obj_name, container_obj_name):
        # Check that objects exist
        for obj_name in [trg_obj_name, container_obj_name]:
            if obj_name not in self.env.task.object_scope:
                print(f"Object {obj_name} not in task object scope.")
                image, symbolic_state = self._finish_action()
                legal = False
                return legal, "not legal", image, symbolic_state

        trg_obj = self.env.task.object_scope[trg_obj_name]
        container_obj = self.env.task.object_scope[container_obj_name]

        # Enforce preconditions
        holding_trg_obj = self._is_holding(trg_obj_name)
        bottom_obj_reachable = self._is_reachable(container_obj_name)
        if not holding_trg_obj or not bottom_obj_reachable:
            print(f"Preconditions not satisfied. holding_trg_obj={holding_trg_obj} ; bottom_obj_reachable={bottom_obj_reachable}")
            image, symbolic_state = self._finish_action()
            legal = False
            return legal, "not legal", image, symbolic_state

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
            legal = True
            info = 'executed but failed'
            return legal, info, image, symbolic_state

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
        
        if self.debug:
            assert symbolic_state['ontop'][f'{trg_obj_name},{container_obj_name}'] == True, "Action succeeded but target object is not on top of the container object!"

        # Last sanity check
        if not symbolic_state['ontop'][f'{trg_obj_name},{container_obj_name}']:
            success = False
        else:
            # If it made it so far, it's considered successful
            success = True
            
        if success:
            info = 'success'
        else:
            info = 'executed but failed'
        legal = True
        return legal, info, image, symbolic_state

    def place_next_to(self, trg_obj_name, container_obj_name):
        # Check that objects exist
        for obj_name in [trg_obj_name, container_obj_name]:
            if obj_name not in self.env.task.object_scope:
                print(f"Object {obj_name} not in task object scope.")
                image, symbolic_state = self._finish_action()
                legal = False
                return legal, "not legal", image, symbolic_state

        trg_obj = self.env.task.object_scope[trg_obj_name]
        container_obj = self.env.task.object_scope[container_obj_name]

        # Enforce preconditions: reachable(container_obj), holding(trg_obj)
        reachable = self._is_reachable(container_obj_name)
        holding_trg_obj = self._is_holding(trg_obj_name)
        if not reachable or not holding_trg_obj:
            print(f"Preconditions not satisfied. holding_trg_obj={holding_trg_obj} ; reachable={reachable}")
            image, symbolic_state = self._finish_action()
            legal = False
            return legal, "not legal", image, symbolic_state

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
            legal = True
            info = 'executed but failed'
            return legal, info, image, symbolic_state

        success = sample_until_next_to(
            self, trg_obj, container_obj, max_distance=self.ROBOT_DISTANCE_THRESHOLD, 
            debug=self.verbose)

        # Reset the robot pose
        self.robot.reset()
        self.robot.keep_still()
        image, symbolic_state = self._finish_action(do_physics_steps=True)

        if success and self.debug:
            assert symbolic_state['nextto'][f'{trg_obj_name},{container_obj_name}'] == True, "Action succeeded but target object is not on top of the other object!"

        # Last sanity check
        if not symbolic_state['nextto'][f'{trg_obj_name},{container_obj_name}']:
            success = False
            
        if success:
            info = 'success'
        else:
            info = 'executed but failed'
        legal = True
        return legal, info, image, symbolic_state
        
    def grasp(self, obj_name, forward_downward_dir=None, camera_offset=None, distance_from_camera=0.2, sample_budget=20):
        """
        Attempts to grasp an object by sampling up to `sample_budget` grasp poses.
        Returns (success, image, symbolic_state).
        """
        # Check that object exist
        if obj_name not in self.env.task.object_scope:
            print(f"Object {obj_name} not in task object scope.")
            image, symbolic_state = self._finish_action()
            legal = False
            return legal, "not legal", image, symbolic_state
        
        # Enforce preconditions: reachable and nothing in hand
        reachable = self._is_reachable(obj_name)
        empty_hand = self._get_obj_in_hand() is None
        if not reachable or not empty_hand:
            print(f"Preconditions not satisfied. reachable({obj_name})={reachable} ; empty_hand()={empty_hand}")
            image, symbolic_state = self._finish_action()
            legal = False
            return legal, "not legal", image, symbolic_state

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
                # Double check for debugging
                if self.verbose: 
                    print(f"Grasp succeeded on attempt {attempt}.")
                    print("Holding predicates: ", last_state['holding'])
                if self.debug:
                    assert last_state['holding'][obj_name] == True, "Action succeeded but target object is not in hand!"

                legal = True
                info = 'success'
                return legal, info, img, last_state
            else:
                if self.verbose:
                    print(f"Grasp failed on attempt {attempt}, retrying next sample.")
                    
        # All attempts failed
        if self.verbose:
            print("All grasp attempts failed.")
            
        info = 'executed but failed'
        legal = True
        return legal, info, image, symbolic_state
    
    
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
        image = render_utils.add_drawings(image, self)
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

    def translate_symbolic_state(self, symbolic_state):

        def bddl_to_pddl(obj_name_bddl):
            # obj_name.n.0x_y -> obj_name_y
            prefix = obj_name_bddl.split('.')[0] # obj_name
            suffix = obj_name_bddl.split('_')[-1] # y
            obj_name_pddl = f"{prefix}_{suffix}" # obj_name_y
            return obj_name_pddl

        translated_symbolic_state = {}
        # First level contains predicate names, e.g. 'reachable' and 'ontop'
        for pred in symbolic_state.keys():
            translated_symbolic_state[pred] = {}
            # Second level contains one or two (comma separated) object names 
            # for unary and binary predicates respectively
            for arg in symbolic_state[pred].keys():
                if ',' in arg:
                    # Binary predicates case
                    a, b = arg.split(',')
                    a_new = bddl_to_pddl(a)
                    b_new = bddl_to_pddl(b)

                    # Store value in new dict
                    translated_symbolic_state[pred][f"{a_new},{b_new}"] = symbolic_state[pred][arg]
                else:
                    # Unary predicate case
                    arg_new = bddl_to_pddl(arg)
                    translated_symbolic_state[pred][arg_new] = symbolic_state[pred][arg]
        return translated_symbolic_state
                    
    def _get_task_objects(self):
        return get_task_objects(self.env)
        
    def get_obj_from_name(self, obj_name):
        return self.env.task.object_scope[obj_name]

    def _is_near(self, obj_name):
        obj = self.get_obj_from_name(obj_name)
        d = self._get_obj_distance_from_robot(obj)
        return bool(d < self.ROBOT_DISTANCE_THRESHOLD)

    def _is_visible(self, obj_name):
        obj = self.get_obj_from_name(obj_name)
        return bool(obj.states[object_states.IsVisible].get_value(env=self.env))

    def _is_reachable(self, obj_name):
        return bool(self._is_near(obj_name)) and bool(self._is_visible(obj_name))

    def _is_holding(self, obj_name):
        if not self._is_movable(obj_name):
            return None
        obj = self.get_obj_from_name(obj_name)
        return bool(self._get_obj_in_hand() is obj)

    def _is_movable(self, obj_name):
        obj = self.get_obj_from_name(obj_name)
        _, _, extents, _ = obj.get_base_aligned_bounding_box(visual=False)
        return bool(np.any(extents < GUARANTEED_GRASPABLE_WIDTH))

    def _is_openable(self, obj_name):
        obj = self.get_obj_from_name(obj_name)
        return bool(object_states.Open in obj.states)

    def _is_open(self, obj_name):
        if not self._is_openable(obj_name):
            return None
        obj = self.get_obj_from_name(obj_name)
        return bool(obj.states[object_states.Open].get_value())

    def _is_ontop(self, obj_name, below_name):
        if not self._is_movable(obj_name):
            return None
        obj   = self.get_obj_from_name(obj_name)
        below = self.get_obj_from_name(below_name)
        return bool(obj.states[object_states.OnTop].get_value(below))

    def _is_inside(self, obj_name, container_name):
        if not (self._is_movable(obj_name) and self._is_openable(container_name)):
            return None
        obj       = self.get_obj_from_name(obj_name)
        container = self.get_obj_from_name(container_name)
        return bool(obj.states[object_states.Inside].get_value(container))

    def _is_nextto(self, obj_name, nextto_name):
        if not (self._is_movable(obj_name)):
            return None
        obj       = self.get_obj_from_name(obj_name)
        obj_nextto = self.get_obj_from_name(nextto_name)
        return bool(obj.states[object_states.next_to.NextTo].get_value(obj_nextto))

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

        # Gradually expand the search with the budget
        yaw_ranges = [45, 60, 90]
        min_distances = [0.6, 0.5, 0.4]
        max_distances = [0.9, 1.1, 1.3]
        
        for yaw_range, min_distance, max_distance in zip(yaw_ranges, min_distances, max_distances):
            sampling_budget = MAX_ATTEMPTS_FOR_SAMPLING_POSE_NEAR_OBJECT//3
            # Pseudo-random efficient sampler
            sampler = AnnularSampler(
                center=object_center,
                orientation=orientation,
                r_min=min_distance,
                r_max=max_distance,
                yaw_range_deg=yaw_range,
                sampling_budget=sampling_budget
            )

            for _ in range(sampling_budget):
                try:
                    pose_2d = sampler.sample(repeat=True)
                except StopIteration:
                    break
            
                if self._test_pose(pose_2d, obj):
                    return pose_2d
    
        return None

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
                #robot_distance = self._get_distance_from_robot(pos_on_obj) # this is robot-dependent
                robot_distance = self._get_obj_distance_from_robot(obj)
                
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

    def _get_obj_distance_from_robot(self, obj):
        try:
            # Get the centers (3d vectors) of all the faces of the aabb
            face_centers = self.get_face_centers_of_aabb(obj.states) # np.array, shape (6,)
            
            # Compute distance for all vectors
            distances = [self._get_distance_from_robot(v) for v in face_centers]
                
            # Take the min and use that as a distance
            distance = min(distances)

        except Exception as e:
            if self.verbose:
                print("Defaulting to distance from the center of the object")
                print("Exception encountered:", e)
            distance = self._get_distance_from_robot(obj.get_position())
        return distance

    @staticmethod
    def get_face_centers_of_aabb(states):
        aabb_center, aabb_extent = get_center_extent(states)
        
        face_centers = []
        axes = np.eye(3)  # unit vectors for x, y, z axes
    
        for axis in range(3):
            for direction in [-1, 1]:
                offset = axes[axis] * (aabb_extent[axis] / 2) * direction
                face_center = aabb_center + offset
                face_centers.append(face_center)
        
        return np.array(face_centers)  # shape (6, 3)

    
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

    @staticmethod
    def match_name_to_sim_env(name, object_names):
        # Match prefix
        prefix = name.split('_')[0]
        filtered_object_names = [obj_name for obj_name in object_names if prefix in obj_name]
        
        # Match suffix
        suffix = '_'+name.split('_')[1]
        matches = [obj_name for obj_name in filtered_object_names if suffix in obj_name]

        # Assert uniqueness
        assert len(matches) == 1, f"Too many or too little matches ({len(matches)}), expected only 1."
        # Return only match
        return matches[0]

# Helper class to sample efficiently positions to navigate to
class AnnularSampler:
    def __init__(
        self,
        center,
        orientation,
        r_min,
        r_max,
        yaw_range_deg,
        sampling_budget,
        local_forward=[0, 1, 0]
    ):
        self.center = np.array(center)
        self.orientation = orientation
        self.r_min = r_min
        self.r_max = r_max
        self.yaw_range_deg = yaw_range_deg
        self.local_forward = np.array(local_forward)

        # Determine yaw and radial bin counts based on the budget
        self.num_yaw_bins = int(np.ceil(np.sqrt(sampling_budget)))
        self.num_radial_bins = int(np.ceil(sampling_budget / self.num_yaw_bins))

        # Create yaw bins (in degrees)
        self.yaw_bins = np.linspace(
            -yaw_range_deg, yaw_range_deg, self.num_yaw_bins, endpoint=False
        )

        # Area-uniform radial samples (via stratified sqrt of uniform area sampling)
        r2_min = r_min ** 2
        r2_max = r_max ** 2
        r2_samples = np.linspace(r2_min, r2_max, self.num_radial_bins, endpoint=False)
        self.radial_bins = np.sqrt(r2_samples)

        # Generate grid and shuffle
        self.grid_points = []
        for yaw in self.yaw_bins:
            for radius in self.radial_bins:
                self.grid_points.append((yaw, radius))
        np.random.shuffle(self.grid_points)

        self.index = 0

    def _point_from_yaw_and_distance(self, yaw_offset_deg, distance):
        yaw_offset_rad = np.radians(yaw_offset_deg)
        local_yaw_rot = Rotation.from_euler('z', yaw_offset_rad).apply(self.local_forward)

        def wxyz_to_xyzw(q):
            return [q[1], q[2], q[3], q[0]]

        orientation_xyzw = wxyz_to_xyzw(self.orientation)
        world_direction = p.rotateVector(orientation_xyzw, local_yaw_rot)

        target = self.center + distance * np.array(world_direction)
        return target

    def sample(self, repeat=False):
        if self.index >= len(self.grid_points) and not repeat:
            raise StopIteration("All grid points have been sampled.")

        yaw, distance = self.grid_points[self.index%len(self.grid_points)]
        self.index += 1

        pos = self._point_from_yaw_and_distance(yaw, distance)
        yaw_rad = np.arctan2(self.center[1] - pos[1], self.center[0] - pos[0])
        return np.array([pos[0], pos[1], yaw_rad])