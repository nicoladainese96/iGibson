import os
import behavior
import pyquaternion  
import numpy as np
from PIL import Image
from igibson.utils.utils import parse_config

def get_env_config():
    env_config_file = os.path.join(
            os.path.dirname(behavior.__file__),
            "configs/behavior_full_observability.yaml",
        )
    env_config = parse_config(env_config_file)
    # Modify env_config - later we can make our own
    env_config['image_width'] = 720
    env_config['image_height'] = 720
    # No idea what all this stuff is, but we might want to remove everything that is not useful once we inspect it
    env_config['output'] = ['proprioception','rgb','highlight','depth','seg','ins_seg','task_obs']
    return env_config

def render_robot_pov(env, env_config, step, show=False, save=True):
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
    frame = s.renderer.render(modes=("rgb"))[0]
    rgb_image = (frame[..., :3] * 255).astype(np.uint8) 
    
    # Ensure the directory exists
    os.makedirs("./images/initial_scenes", exist_ok=True)
    
    # Save using PIL
    image = Image.fromarray(rgb_image)
    if show:
        image.show()
    if save:
        image.save(f"./images/initial_scenes/{env_config['task']}_{env_config['scene_id']}_{step}.jpg", "JPEG")

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