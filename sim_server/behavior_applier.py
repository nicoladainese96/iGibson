# Port here all the useful stuff inside action_primitives/starter_semantic_action_primitives.py
from .sim_action_applier import SimActionApplier

class BehaviorRobotSimApplier(SimActionApplier):
    def go_to(self, target):
        pos = self.env.get_object_position(target)
        self.env.set_robot_pose(pos + [-0.2, 0.1, 0])

    def open(self, object_name): 
        pass

    def close(self, object_name): 
        pass

    def grasp(self, object_name): 
        pass

    def place_inside(self, object_a, object_b): 
        pass
        
    def place_on_top(self, object_a, object_b): 
        pass
