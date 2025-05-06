import numpy as np
import pyquaternion # import for custom code 
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon, box

from igibson.object_states.object_state_base import BooleanState, CachingEnabledObjectState
from igibson.object_states.pose import Pose
from igibson.object_states.room_states import InsideRoomTypes
from igibson.utils.constants import MAX_INSTANCE_COUNT

import igibson.render_utils as render_utils

_IN_REACH_DISTANCE_THRESHOLD = 2.0

_IN_FOV_PIXEL_FRACTION_THRESHOLD = 0.05

MIN_PIXEL_VISIBLE = 200

def _get_robot(simulator):
    valid_robots = [robot for robot in simulator.scene.robots]
    if not valid_robots:
        return None

    if len(valid_robots) > 1:
        raise ValueError("Multiple robots found.")

    return valid_robots[0]

def set_camera(s,
               robot,
               field_of_view=120,
               forward_downward_dir=None,
               up_dir=None,
               camera_pos_offset=None,
               camera_height=1.2,
               apply_q_rotation=True
              ):
    # Set default values
    if forward_downward_dir is None:
        forward_downward_dir = np.array([1, 0, -0.25])
    if up_dir is None:
        up_dir = np.array([0, 0, 1])
    if camera_pos_offset is None:
        camera_pos_offset = np.array([0.1, 0.1, 0.7])

    robot_pos, q = render_utils.get_robot_pos_and_q_rotation(robot)

    if apply_q_rotation:
        # Apply rotations to see all directions in the frame of the robot
        camera_pos = robot_pos + q.rotate(camera_pos_offset) # Slightly above the robot's center
        camera_pos[2] = camera_height
        forward_downward_direction = q.rotate(forward_downward_dir) # Forward wrt the robot's frame of reference 
        up_direction = up_dir # up is up no matter the frame of reference, unless the robot is doing something weird
    else:
        camera_pos = robot_pos + camera_pos_offset
        camera_pos[2] = camera_height
        forward_downward_direction = forward_downward_dir
        up_direction = up_dir

    # Set the camera in the renderer
    s.renderer.set_camera(camera_pos, camera_pos + forward_downward_direction, up_direction)
    s.renderer.set_fov(field_of_view)

def compute_projected_area(uv_coords, H, W):
    """
    uv_coords: (8, 2) array of projected 2D points (u,v) in image coordinates
    returns: area in pixels² of the convex hull of the projection
    """
    uv_coords = np.asarray(uv_coords) # are these points correctly written? u was flipped as W-u

    if uv_coords.shape[0] < 3:
        return 0  # Not enough points to define an area

    try:

        # Compute convex hull on all projected points
        hull = ConvexHull(uv_coords)
        hull_pts = uv_coords[hull.vertices]   # shape (m,2), m ≤ n
        
        # Build Shapely geometries
        poly = Polygon(hull_pts)
        img_rect = box(0, 0, W - 1, H - 1)    # minx, miny, maxx, maxy
        
        # Intersect
        clipped_poly = poly.intersection(img_rect)

        # Return area
        return clipped_poly.area
    except Exception as e:
        # Never encontered an error so far
        print(f"Warning: convex hull failed with error: {e}")
        return 1 # return minimum number of pixels possible that is not illegal as denominator

class InReachOfRobot(CachingEnabledObjectState, BooleanState):
    @staticmethod
    def get_dependencies():
        return CachingEnabledObjectState.get_dependencies() + [Pose]

    def _compute_value(self):
        robot = _get_robot(self.simulator)
        if not robot:
            return False

        robot_pos = robot.get_position()
        object_pos, _ = self.obj.states[Pose].get_value()
        return np.linalg.norm(object_pos - np.array(robot_pos)) < _IN_REACH_DISTANCE_THRESHOLD

    def _set_value(self, new_value):
        raise NotImplementedError("InReachOfRobot state currently does not support setting.")

    # Nothing to do here.
    def _dump(self):
        pass

    def load(self, data):
        pass


class InSameRoomAsRobot(CachingEnabledObjectState, BooleanState):
    @staticmethod
    def get_dependencies():
        return CachingEnabledObjectState.get_dependencies() + [Pose, InsideRoomTypes]

    def _compute_value(self):
        robot = _get_robot(self.simulator)
        if not robot:
            return False

        scene = self.simulator.scene
        if not scene or not hasattr(scene, "get_room_instance_by_point"):
            return False

        robot_pos = robot.get_position()
        robot_room = scene.get_room_instance_by_point(np.array(robot_pos[:2]))
        object_rooms = self.obj.states[InsideRoomTypes].get_value()

        return robot_room is not None and robot_room in object_rooms

    def _set_value(self, new_value):
        raise NotImplementedError("InSameRoomAsRobot state currently does not support setting.")

    # Nothing to do here.
    def _dump(self):
        pass

    def load(self, data):
        pass


class InHandOfRobot(CachingEnabledObjectState, BooleanState):
    def _compute_value(self):
        robot = _get_robot(self.simulator)
        if not robot:
            return False

        # We import this here to avoid cyclical dependency.
        from igibson.robots.manipulation_robot import IsGraspingState

        return any(
            robot.is_grasping(arm=arm, candidate_obj=bid) == IsGraspingState.TRUE
            for bid in self.obj.get_body_ids()
            for arm in robot.arm_names
        )

    def _set_value(self, new_value):
        raise NotImplementedError("InHandOfRobot state currently does not support setting.")

    # Nothing to do here.
    def _dump(self):
        pass

    def load(self, data):
        pass


class InFOVOfRobot(CachingEnabledObjectState, BooleanState):
    @staticmethod
    def get_optional_dependencies():
        return CachingEnabledObjectState.get_optional_dependencies() + [ObjectsInFOVOfRobot]

    def _compute_value(self):
        robot = _get_robot(self.simulator)
        if not robot:
            return False

        body_ids = set(self.obj.get_body_ids())
        return not body_ids.isdisjoint(robot.states[ObjectsInFOVOfRobot].get_value()) 

    def _set_value(self, new_value):
        raise NotImplementedError("InFOVOfRobot state currently does not support setting.")

    # Nothing to do here.
    def _dump(self):
        pass

    def load(self, data):
        pass



class IsVisible(CachingEnabledObjectState, BooleanState):
    @staticmethod
    def get_optional_dependencies():
        return CachingEnabledObjectState.get_optional_dependencies() + [VisiblePixelCountFromRobot] # no idea why this is needed!

    def _compute_value(self, env, relative_threshold=0.08, absolute_threshold=None, use_absolute_threshold=False):
        """
        Note that this function has a mandatory argument env!
        """
        robot = _get_robot(self.simulator)
        if not robot:
            return False

        body_ids = set(self.obj.get_body_ids())

        # Hopefully self.obj.states is the right thing to access - double check!
        total_visible_pixels = self.obj.states[VisiblePixelCountFromRobot].get_value()
        
        if use_absolute_threshold:
            if absolute_threshold is None:
                absolute_threshold = MIN_PIXEL_VISIBLE
            # Return True if the amount of visible pixels from all body parts of the object are more than absolute_threshold
            # Else return False
            return total_visible_pixels > absolute_threshold
        else:
            # In the relative case, take the ratio between the total visible pixels and the area of the convex hull
            # formed by the vertices of the object bounding box. As this is not a tight box, ratios of ~8% are usually 
            # enough to consider an object visible
            
            bbox_vertices_uv = render_utils.get_bbox_vertices_uv(env, self.obj)

            # Clip vertices within image bounds
            H = env.config['image_height']
            W = env.config['image_width']
        
            projected_bbox_pixels = compute_projected_area(bbox_vertices_uv, H, W)
            return (total_visible_pixels / (projected_bbox_pixels + 1)) >= relative_threshold 

    def _set_value(self, new_value):
        raise NotImplementedError("IsVisible state currently does not support setting.")

    def _get_value(self, *args, **kwargs):
        # If we don't have a value cached, compute it now.
        if self.value is None:
            self.value = self._compute_value(*args, **kwargs)

        return self.value
        
    # Nothing to do here.
    def _dump(self):
        pass

    def load(self, data):
        pass

class ObjectsInFOVOfRobot(CachingEnabledObjectState):
    def __init__(self, *args, **kwargs):
        super(CachingEnabledObjectState, self).__init__(*args, **kwargs)

    def _compute_value(self):
        # Pass the FOV through the instance-to-body ID mapping.
        seg = self.render_single_robot_camera(modes="ins_seg")[0][:, :, 0] 
        #seg = self.simulator.renderer.render_single_robot_camera(self.obj, modes="ins_seg")[0][:, :, 0] # why self.obj was passed?? - should be the robot?
        seg = np.round(seg * MAX_INSTANCE_COUNT).astype(int)
        body_ids = self.simulator.renderer.get_pb_ids_for_instance_ids(seg)

        # Flatten to 1D for easy counting of pixels per body ID
        flat_ids = body_ids.flatten()
    
        # Count pixels per body ID using bincount
        unique_ids, counts = np.unique(flat_ids, return_counts=True)
        
        # Debug print
        # print({obj_id: count for obj_id, count in zip(unique_ids, counts) if obj_id != -1})

        visible_ids = {
            obj_id for obj_id, count in zip(unique_ids, counts)
            if obj_id != -1 and count >= MIN_PIXEL_VISIBLE
        }
    
        return visible_ids

        # Pixels that don't contain an object are marked -1 but we don't want to include that
        # as a body ID.
        #return set(np.unique(body_ids)) - {-1}

    def _set_value(self, new_value):
        raise NotImplementedError("ObjectsInFOVOfRobot state currently does not support setting.")

    # Nothing to do here.
    def _dump(self):
        pass

    def load(self, data):
        pass

    def render_single_robot_camera(self, modes=("rgb")):
        # Apparently every object has access to the simulator as an attribute
        robot = self.simulator.scene.robots[0]
        r = self.simulator.renderer
        
        hide_instances = robot.renderer_instances if r.rendering_settings.hide_robot else []
        
        # Let's use our own camera instead
        set_camera(self.simulator, robot)
        
        # No idea why this loop is needed, anyway it's a single element list
        frames = []
        for item in r.render(modes=modes, hidden=hide_instances):
            frames.append(item)
        return frames

class VisiblePixelCountFromRobot(ObjectsInFOVOfRobot):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def _compute_value(self):
        # Render instance segmentation
        seg = self.render_single_robot_camera(modes="ins_seg")[0][:, :, 0]
        seg = np.round(seg * MAX_INSTANCE_COUNT).astype(int)
        body_ids = self.simulator.renderer.get_pb_ids_for_instance_ids(seg)

        flat_ids = body_ids.flatten()

        # Count pixels per body ID
        unique_ids, counts = np.unique(flat_ids, return_counts=True)
        pixel_counts = dict(zip(unique_ids, counts))

        total_visible_pixels = 0
        obj_body_ids = set(self.obj.get_body_ids())

        for body_id in obj_body_ids:
            total_visible_pixels += pixel_counts.get(body_id, 0)

        return total_visible_pixels



