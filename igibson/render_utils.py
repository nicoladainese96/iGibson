import os
import random
import trimesh
import itertools
import pyquaternion
import numpy as np
from PIL import Image, ImageDraw

from igibson.utils import utils as ig_utils
from igibson.object_states.utils import get_center_extent

def get_camera_settings(s):
    """
    Input: 
    s (from env.simulator, instance of igibson.renderer.mesh_renderer.mesh_renderer_cpu.MeshRenderer

    Returns: camera_position, target_direction, up_direction

    Note: target direction is target_position - camera_position
    """
    return s.renderer.camera, s.renderer.target, s.renderer.up 

def get_robot_pos_and_q_rotation(robot):
    robot_pos, robot_orientation = robot.get_position_orientation()
    
    # Convert quaternion to rotation matrix - takes w,x,y,z in input, but robot orientation is given as x,y,z,w !!!
    q = pyquaternion.Quaternion(x=robot_orientation[0], 
                                y=robot_orientation[1], 
                                z=robot_orientation[2], 
                                w=robot_orientation[3])
    return robot_pos, q
    
def set_camera_look_ahead(env,
                          field_of_view=120,
                          forward_downward_dir=None,
                          up_dir=None,
                          camera_pos_offset=None,
                          apply_q_rotation=True
                         ):
    """
    Input: 
    env (instance of igibson.envs.igibson_env.iGibsonEnv)

    No return, as the changes are applied directly to the simulator's renderer camera.
    """
    if forward_downward_dir is None:
        forward_downward_dir = np.array([1, 0, -0.25])
    if up_dir is None:
        up_dir = np.array([0, 0, 1])
    if camera_pos_offset is None:
        camera_pos_offset = np.array([0.1, 0.1, 0.7])
        
    s = env.simulator
    robot_pos, q = get_robot_pos_and_q_rotation(env.robots[0])

    if apply_q_rotation:
        # Apply rotations to see all directions in the frame of the robot
        camera_pos = robot_pos + q.rotate(camera_pos_offset) # Slightly above the robot's center
        forward_downward_direction = q.rotate(forward_downward_dir) # Forward wrt the robot's frame of reference 
        up_direction = up_dir # up is up no matter the frame of reference, unless the robot is doing something weird
    else:
        camera_pos = robot_pos + camera_pos_offset
        forward_downward_direction = forward_downward_dir
        up_direction = up_dir

    # Set the camera in the renderer
    s.renderer.set_camera(camera_pos, camera_pos + forward_downward_direction, up_direction)
    s.renderer.set_fov(field_of_view)

# Totally off.. too much of a top-down view
def set_camera_look_obj(env, 
                        obj,
                        field_of_view=120,
                        up_dir=None,
                        camera_pos_offset=None,
                        apply_q_rotation=True
                       ):
    """
    Input: 
    env (instance of igibson.envs.igibson_env.iGibsonEnv)
    obj (instance of igibson.objects.articulated_objects.URDFObject or descendant classes)

    No return, as the changes are applied directly to the simulator's renderer camera.
    """
    if up_dir is None:
        up_dir = np.array([0, 0, 1])
    if camera_pos_offset is None:
        camera_pos_offset = np.array([0.1, 0.1, 0.7])

    s = env.simulator
    robot_pos, q = get_robot_pos_and_q_rotation(env.robots[0])
    obj_pos = obj.get_position()
    
    if apply_q_rotation:
        camera_pos = robot_pos + q.rotate(camera_pos_offset) # Slightly above the robot's center
        target_direction = obj_pos - robot_pos # this is already in the world frame of reference

        up_direction = up_dir
    else:
        camera_pos = robot_pos + camera_pos_offset
        target_direction = obj_pos - robot_pos
        up_direction = up_dir

    s.renderer.set_camera(camera_pos, target_direction, up_direction)
    s.renderer.set_fov(field_of_view)
    
def get_image_from_camera(s):
    """
    Given simulator s, returns PIL image.
    """
    frame = s.renderer.render(modes=("rgb"))[0]
    rgb_image = (frame[..., :3] * 255).astype(np.uint8) 
    image = Image.fromarray(rgb_image)
    return image

def render_frame(env, 
                 show=True, 
                 save=False, 
                 path='images', 
                 name='tmp', 
                 **kwargs
                ):
    set_camera_look_ahead(env, **kwargs)
    image = get_image_from_camera(env.simulator)
    if show:
        image.show()
    if save:
        image.save(os.path.join(path, name+'.jpg'), "JPEG")
        
def render_frame_with_trg_obj(env, 
                              obj,
                              add_obj_center=False,
                              add_contour_points=False,
                              add_bbox=False,
                              return_uv_coord=False,
                              show=True, 
                              save=False, 
                              path='images', 
                              name='tmp', 
                              **kwargs
                             ):
    #set_camera_look_obj(env, obj, **kwargs) # doesn't work well for now
    set_camera_look_ahead(env, **kwargs)
    image = get_image_from_camera(env.simulator)

    if add_obj_center:
        obj_center_uv = get_obj_center_uv(env, obj)
        image = draw_obj_center(image, obj_center_uv, obj)
    
    if add_contour_points:
        contour_points_uv = get_contour_points_uv(env, obj)
        image = draw_contour_points(image, contour_points_uv)
    
    if add_bbox:
        bbox_vertices_uv = get_bbox_vertices_uv(env, obj)
        image = draw_3d_bbox(image, bbox_vertices_uv)
        
    if show:
        image.show()
    if save:
        image.save(os.path.join(path, name+'.jpg'), "JPEG")
    if return_uv_coord and add_bbox:
        # Change u in image.width - u for completeness
        bbox_vertices_uv = np.stack([image.width - bbox_vertices_uv[:,0], bbox_vertices_uv[:,1]], axis=1)
        return bbox_vertices_uv

def render_robot_eye_pov(env, show=True, save=False, path='images/', name='robot_eyes_view'):
    r = env.simulator.renderer
    robot = env.robots[0]
    
    frame = r.render_single_robot_camera(robot)[0] # not sure if there would ever be more than 1, but right now there's a single one
    #frame.shape # (1080, 1080, 4)
    
    rgb_image = (frame[..., :3] * 255).astype(np.uint8) 
    image = Image.fromarray(rgb_image)
    
    if show:
        image.show()
    if save:
        image.save(os.path.join(path, name+'.jpg'), "JPEG")
        
def get_obj_center_uv(env, obj):
    s = env.simulator
    obj_pos = obj.get_position()
    
    # Express in camera frame of reference
    obj_center_cf = s.renderer.transform_point(obj_pos)

    # Project to uv plane
    K = s.renderer.get_intrinsics()
    uvw_center = np.dot(K, obj_center_cf)

    # Normalise
    obj_center_uv = uvw_center[:2]/uvw_center[2:]
    
    return obj_center_uv

def get_contour_points_uv(env, obj, n_points = 1000):
    s = env.simulator
    
    # Sample contour points
    positions_on_obj_surface = []
    for i in range(n_points):
        # Original 4-lateral faces sampling
        # pos_on_obj_surface = sample_position_on_aabb_face(obj)

        # Only frontal face
        pos_on_obj_surface = sample_position_on_front_face(obj)
        positions_on_obj_surface.append(pos_on_obj_surface)

    # Express in camera frame of reference
    objects_pos_camera_frame = [s.renderer.transform_point(obj_pos) for obj_pos in positions_on_obj_surface]
    objects_pos_camera_frame = np.stack(objects_pos_camera_frame)

    # Project to uv plane
    K = s.renderer.get_intrinsics()
    uvw_column_vectors = np.dot(K, objects_pos_camera_frame.T)
    
    # Normalise
    uv_prime_column_vectors = uvw_column_vectors[:2]/uvw_column_vectors[2:]
    contour_points_uv = uv_prime_column_vectors.T
    
    return contour_points_uv

def get_bbox_vertices_uv(env, obj):
    # wf = world frame, cf = camera frame
    s = env.simulator

    # Get bbox info
    bbox_center_in_wf, bbox_orn_in_wf, bbox_extent_obj_frame, bbox_center_obj_frame  = obj.get_base_aligned_bounding_box(visual=True) 

    # Form 8 vertices in the frame of reference of the object itself - we should use a more interpretable code maybe
    vertex_positions_obj_frame = np.array(list(itertools.product((1, -1), repeat=3))) * (
        bbox_extent_obj_frame / 2
    ) + bbox_center_obj_frame

    # 1. Convert position and quaternion to transformation matrix
    bbox_transform = ig_utils.quat_pos_to_mat(bbox_center_in_wf, bbox_orn_in_wf) 

    # 2. Convert coordinates to world frame - no idea where this function comes from
    # see https://github.com/StanfordVL/iGibson/blob/master/igibson/examples/objects/draw_bounding_box.py
    vertex_positions_wf = trimesh.transformations.transform_points(
        vertex_positions_obj_frame, bbox_transform
    )

    # 3. Convert coordinates to camera_frame
    vertex_positions_cf = np.array([s.renderer.transform_point(v) for v in vertex_positions_wf])

    # 4. Project to uv plane
    K = s.renderer.get_intrinsics()
    uvw_column_vectors = np.dot(K, vertex_positions_cf.T)
    
    # 5. Normalise
    uv_prime_column_vectors = uvw_column_vectors[:2]/uvw_column_vectors[2:]
    bbox_vertices_uv = uv_prime_column_vectors.T
    
    return bbox_vertices_uv
    
def draw_obj_center(image, obj_center_uv, obj, r=3, fill='red', outline='red'):
    draw = ImageDraw.Draw(image)
    W = image.width
    u, v = obj_center_uv
    # For some weird reason, we get this convention on the x axis of image.width - u
    draw.ellipse([(W -u - r, v - r), (W - u + r, v + r)], fill=fill, outline=outline)
    
    # Annotate the point with its index
    draw.text((W - u + r + 2, v - r - 2), f"{obj.bddl_object_scope}", font_size=100)

    return image

def draw_contour_points(image, contour_points_uv, r=3, fill='blue', outline='blue'):
    draw = ImageDraw.Draw(image)
    W = image.width
    for (u,v) in contour_points_uv:
        draw.ellipse([(W - u - r, v - r), (W - u + r, v + r)], fill="blue", outline="blue")
    return image
    
def draw_3d_bbox(image, bbox_vertices_uv):
    draw = ImageDraw.Draw(image)
    W = image.width
    
    # Define edges by vertex index pairs - assuming the ordering is canonical
    edges = [
        (0, 1), (0, 2), (1, 3), (2, 3),  # bottom face
        (4, 5), (4, 6), (6, 7), (5, 7),  # top face
        (0, 4), (1, 5), (2, 6), (3, 7)   # vertical edges
    ]
    
    # Draw all edges in red
    for edge in edges:
        u1, v1 = bbox_vertices_uv[edge[0]]
        u2, v2 = bbox_vertices_uv[edge[1]]
        # Apply the same coordinate transform as the points (flipping u)
        draw.line([(W - u1, v1), (W - u2, v2)], fill="red", width=2)
        
    return image

def sample_position_on_aabb_face(target_obj):
    
    aabb_center, aabb_extent = get_center_extent(target_obj.states)
    # We want to sample only from the side-facing faces.
    face_normal_axis = random.choice([0, 1])
    face_normal_direction = random.choice([-1, 1])
    
    # Use half of the extent for the offset
    face_center = aabb_center + np.eye(3)[face_normal_axis] * (aabb_extent * 0.5) * face_normal_direction
    
    face_lateral_axis = 0 if face_normal_axis == 1 else 1
    face_lateral_half_extent = np.eye(3)[face_lateral_axis] * aabb_extent / 2
    face_vertical_half_extent = np.eye(3)[2] * aabb_extent / 2
    face_min = face_center - face_vertical_half_extent - face_lateral_half_extent
    face_max = face_center + face_vertical_half_extent + face_lateral_half_extent
    return np.random.uniform(face_min, face_max)

def sample_position_on_front_face(target_obj):
    
    aabb_center, aabb_extent = get_center_extent(target_obj.states)
    # We want to sample only from the side-facing faces.
    face_normal_axis = 1 #random.choice([0, 1])
    face_normal_direction = 1 #random.choice([-1, 1])
    
    # Use half of the extent for the offset
    face_center = aabb_center + np.eye(3)[face_normal_axis] * (aabb_extent * 0.5) * face_normal_direction
    
    face_lateral_axis = 0 if face_normal_axis == 1 else 1
    face_lateral_half_extent = np.eye(3)[face_lateral_axis] * aabb_extent / 2
    face_vertical_half_extent = np.eye(3)[2] * aabb_extent / 2
    face_min = face_center - face_vertical_half_extent - face_lateral_half_extent
    face_max = face_center + face_vertical_half_extent + face_lateral_half_extent
    return np.random.uniform(face_min, face_max)