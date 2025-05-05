import os
import bddl
import json
import behavior
import pyquaternion  
import numpy as np
from PIL import Image

from igibson.utils.utils import parse_config
from igibson import object_states
from igibson.envs.igibson_env import iGibsonEnv

from igibson.render_utils import set_camera_look_obj

def get_env_config():
    env_config_file = os.path.join(
            os.path.dirname(behavior.__file__),
            "configs/behavior_full_observability.yaml",
        )
    env_config = parse_config(env_config_file)
    # Modify env_config - later we can make our own
    env_config['image_width'] = 480 #1080
    env_config['image_height'] = 480 #1080

    # Improve realism
    env_config["enable_pbr"] = True
    env_config["enable_shadow"] = True
    #env_config["texture_scale"] = 1.0 # no difference from 1.0 to 2.0
    env_config["optimized_renderer"] = False # True needed for highlighting, False better for image quality 
    env_config["load_texture"] = True # necessary
    
    # change igibson/envs/env_base.py in order to read this, ~L76
    #- msaa=False
    #+ msaa=self.config.get("msaa", False),
    
    # with True it complains about a bunch of stuff :
    # Rendering segmentation masks with MSAA on may generate interpolation artifacts. 
    # It is recommended to turn MSAA off when rendering segmentation.
    env_config["msaa"] = False 
    
    # No idea what all this stuff is, but we might want to remove everything that is not useful once we inspect it
    env_config['output'] = ['proprioception','rgb','highlight','depth','seg','ins_seg','task_obs']
    return env_config

def get_variations_list(task):
    # Common to all activites
    scene_json = os.path.join(
        os.path.dirname(bddl.__file__),
        "activity_to_preselected_scenes.json"
    )
    
    with open(scene_json) as f:
        activity_to_scenes = json.load(f)
    
    scene_instance_pairs = []
    
    scene_ids = sorted(set(activity_to_scenes[task])) 
    instance_ids = [0, 20, 21, 22, 23] # original config and 4 OOD (different furniture and so on)
    
    if len(scene_ids) == 3:
        for scene_id in scene_ids[:-1]: # for the first 2 scenes
            scene_instance_pairs+=[(scene_id, instance_id) \
                                           for instance_id in instance_ids[:2]] # consider 2 instances
        # And only 1 for the last scene
        scene_instance_pairs+=[(scene_ids[-1], instance_ids[0])]
    
    elif len(scene_ids) == 2:
        # 3 instances for the first scene
        scene_instance_pairs+=[(scene_ids[0], instance_id) \
                                           for instance_id in instance_ids[:3]] 
        # 2 instances for the second one
        scene_instance_pairs+=[(scene_ids[1], instance_id) \
                                           for instance_id in instance_ids[:2]] 
    elif len(scene_ids) == 1:
        # 5 instances for the only scene
        scene_instance_pairs+=[(scene_ids[0], instance_id) \
                                           for instance_id in instance_ids] 
    else:
        raise NotImplementedError

    return scene_instance_pairs
    
def generate_scenes(activity, scene_instance_ids, gen_only_first=False):
    for scene_id in scene_instance_ids:
        for instance_id in scene_instance_ids[scene_id]:
            print('\n',activity, scene_id, instance_id)
            generate_scene(activity, scene_id, instance_id)
            
            if gen_only_first:
                print("Generating only the first instance of the scene")
                break
        
        if gen_only_first:
            print("Generating only the first scene for the task")
            break
        

def generate_scene(task, scene_id, instance_id):
    # Notice the change from activity to task - following the original code here
    env_config = get_env_config()
    env_config["scene_id"] = scene_id
    env_config["task"] = task
    env_config["task_id"] = 0
    env_config["instance_id"] = 0
    
    # Keep this in memory as it's removed from the config when we init iGibson - so we need to reload it
    robot_name = env_config["robot"]["name"]

    env = iGibsonEnv(
            config_file=env_config,
            mode="headless",
            action_timestep=1.0 / 30.0,
            physics_timestep=1.0 / 120.0,
        )

    # How do we execute the rendering in the env starting from the commands of the underlying simulator?
    render_robot_pov(env, env_config, step='initial')

def teleport_near_obj(controller, obj, env):
    controller.teleport_near_obj(obj, env)
    #set_camera_look_obj(env, obj) # not sure it's needed, but maybe?

def print_properties(obj):
    """
    Requires from igibson import object_states
    """
    logic_state = {}
    state_keys = [ 'InSameRoomAsRobot', 'InReachOfRobot'] # 'InFOVOfRobot' -> requires extra argument env!

    logic_state = {key: obj.states[getattr(object_states, key)].get_value() for key in state_keys}

    print(logic_state)
### Old functions ###

def render_robot_pov(env, env_config, step, show=False, save=True, path='./images/initial_scenes'):
    s = env.simulator
    robot = env.robots[0]
    robot_pos, robot_orientation = robot.get_position_orientation()
    
    # Convert quaternion to rotation matrix - takes w,x,y,z in input, but robot orientation is given as x,y,z,w !!!
    q = pyquaternion.Quaternion(x=robot_orientation[0], 
                                y=robot_orientation[1], 
                                z=robot_orientation[2], 
                                w=robot_orientation[3])
    
    forward_downward_direction = q.rotate(np.array([1, 0, -0.25]))  # Default forward vector (x-axis)
    up_direction = q.rotate(np.array([0, 0, 1]))  # Default up vector (z-axis)
    
    # Set the camera at the robot's head level (optional: raise it slightly)
    camera_pose = robot_pos + q.rotate(np.array([0.1, 0.1, 1])) # Slightly above the robot's center
    
    # Set the camera in the renderer
    s.renderer.set_camera(camera_pose, camera_pose + forward_downward_direction, up_direction)
    s.renderer.set_fov(90)
    frame = s.renderer.render(modes=("rgb"))[0]
    rgb_image = (frame[..., :3] * 255).astype(np.uint8) 
    
    # Ensure the directory exists
    os.makedirs(path, exist_ok=True)
    
    # Save using PIL
    image = Image.fromarray(rgb_image)
    if show:
        image.show()
    if save:
        image.save(os.path.join(path, f"{env_config['task']}_{env_config['scene_id']}_{step}.jpg"), "JPEG")

def render_robot_orientation_view(env, original_orientation, env_config, step, show=False, save=True, path='./images/initial_scenes'):
    s = env.simulator
    robot = env.robots[0]
    robot_pos, _ = robot.get_position_orientation()
    
    # Convert quaternion to rotation matrix - takes w,x,y,z in input, but robot orientation is given as x,y,z,w !!!
    q = pyquaternion.Quaternion(x=original_orientation[0], 
                                y=original_orientation[1], 
                                z=original_orientation[2], 
                                w=original_orientation[3])
    
    forward_downward_direction = q.rotate(np.array([1, 0, -0.25]))  # Default forward vector (x-axis)
    up_direction = q.rotate(np.array([0, 0, 1]))  # Default up vector (z-axis)
    
    # Set the camera at the robot's head level (optional: raise it slightly)
    camera_pose = robot_pos + q.rotate(np.array([0.1, 0.1, 1])) # Slightly above the robot's center
    
    # Set the camera in the renderer
    s.renderer.set_camera(camera_pose, camera_pose + forward_downward_direction, up_direction)
    s.renderer.set_fov(120)
    frame = s.renderer.render(modes=("rgb"))[0]
    rgb_image = (frame[..., :3] * 255).astype(np.uint8) 
    
    # Ensure the directory exists
    os.makedirs(path, exist_ok=True)
    
    # Save using PIL
    image = Image.fromarray(rgb_image)
    if show:
        image.show()
    if save:
        image.save(os.path.join(path, f"{env_config['task']}_{env_config['scene_id']}_{step}.jpg"), "JPEG")
        
def get_view_dir(angle_deg):
    """
    Given angle in degrees, return the view direction in the xy plane corresponding to the angle. 
    Consider that: 
    angle_deg=0 -> np.array([1,0,0]) 
    angle_deg=90 -> np.array([0,1,0])
    """
    import math
    angle_rad = angle_deg * math.pi / 180 
    x = np.cos(angle_rad)
    y = np.sin(angle_rad)
    return np.array([x,y,0])

def render_360(env, show=False, save=True, path='./images/rendering_attempts'):
    s = env.simulator
    robot = env.robots[0]
    robot_pos, robot_orientation = robot.get_position_orientation()
    # Convert quaternion to rotation matrix - takes w,x,y,z in input, but robot orientation is given as x,y,z,w !!!
    q = pyquaternion.Quaternion(x=robot_orientation[0], 
                                y=robot_orientation[1], 
                                z=robot_orientation[2], 
                                w=robot_orientation[3])

    
    angles = [0, 45, 90, 135, 180, 225, 270, 315, 360]
    for angle in angles:
        # from examples/renderer/mesh_renderer_gpu_example
        camera_pose = robot_pos + q.rotate(np.array([0.1, 0.1, 1])) #+ np.array([0.5, 0.5, 0.5])
        view_direction = q.rotate(get_view_dir(angle))
        print(f"Angle: {angle} - View direction: {view_direction}")
        
        s.renderer.set_camera(camera_pose, camera_pose + view_direction, q.rotate([0, 0, 1]))
        s.renderer.set_fov(90)
        
        frame = s.renderer.render(modes=("rgb"))[0]
        rgb_image = (frame[..., :3] * 255).astype(np.uint8) 
        
        # Ensure the directory exists
        os.makedirs(path, exist_ok=True)
        
        # Save using PIL
        image = Image.fromarray(rgb_image)
        if show:
            image.show()
        if save:
            image.save(os.path.join(path, f"img_{angle}_deg.jpg"), "JPEG")
        
            print("Image saved at "+f"{path}/img_{angle}_deg.jpg")