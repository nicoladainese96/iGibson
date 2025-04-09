import numpy as np
import pyquaternion # import for custom code 

from igibson.object_states.object_state_base import BooleanState, CachingEnabledObjectState
from igibson.object_states.pose import Pose
from igibson.object_states.room_states import InsideRoomTypes
from igibson.utils.constants import MAX_INSTANCE_COUNT

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
        #print(robot.states)
        #print(ObjectsInFOVOfRobot)
        #print(robot.states[ObjectsInFOVOfRobot])
        return not body_ids.isdisjoint(robot.states[ObjectsInFOVOfRobot].get_value()) # this one is crushing badly

    def _set_value(self, new_value):
        raise NotImplementedError("InFOVOfRobot state currently does not support setting.")

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


def set_camera(s,
               robot,
               field_of_view=120,
               forward_downward_dir=None,
               up_dir=None,
               camera_pos_offset=None,
               apply_q_rotation=True
              ):
    # Set default values
    if forward_downward_dir is None:
        forward_downward_dir = np.array([1, 0, -0.25])
    if up_dir is None:
        up_dir = np.array([0, 0, 1])
    if camera_pos_offset is None:
        camera_pos_offset = np.array([0.1, 0.1, 0.7])

    robot_pos, q = get_robot_pos_and_q_rotation(robot)

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

def get_robot_pos_and_q_rotation(robot):
    robot_pos, robot_orientation = robot.get_position_orientation()
    
    # Convert quaternion to rotation matrix - takes w,x,y,z in input, but robot orientation is given as x,y,z,w !!!
    q = pyquaternion.Quaternion(x=robot_orientation[0], 
                                y=robot_orientation[1], 
                                z=robot_orientation[2], 
                                w=robot_orientation[3])
    return robot_pos, q