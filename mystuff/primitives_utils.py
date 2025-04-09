import copy
import numpy as np
import pybullet as p

from igibson.utils.utils import restoreState
from igibson import object_states
from igibson.action_primitives.starter_semantic_action_primitives import StarterSemanticActionPrimitives

PHYSICS_STEPS = 50

def settle_physics(env, steps = None):
    if steps is None:
        steps = PHYSICS_STEPS
        
    s = env.simulator
    for _ in range(steps):
        s.step()
        
def check_collision(obj1, obj2, tol=1e-4):
    obj_ids1 = obj1.get_body_ids()
    obj_ids2 = obj2.get_body_ids()

    for obj_id1 in obj_ids1:
        for obj_id2 in obj_ids2:
            #closest_points = p.getClosestPoints(obj_id1, obj_id2, distance)
            contact_points = p.getContactPoints(obj_id1, obj_id2)
            for contact_point in contact_points:
                contactFlag, bodyUniqueIdA, bodyUniqueIdB, linkIndexA, linkIndexB, positionOnA, positionOnB, contactNormalOnB, contactDistance, normalForce, lateralFriction1, lateralFrictionDir1, lateralFriction2, lateralFrictionDir2 = contact_point
                # Debug print
                #print('contactDistance', contactDistance)
                if contactDistance < 0 and np.abs(contactDistance)>tol:
                    # Then we have significant penetration between the two bodies
                    print("\nContact distance of penetration point: ", contactDistance)
                    return True
    
    return False

def get_task_objects(env):
    # Let's also check for collisions here, assuming that there should be none, as it's the default state (sort of)
    filtered_object_names_list = [name for name in list(env.task.object_scope.keys()) \
                                  if name.split('.')[0] not in ['agent', 'floor']]
    return filtered_object_names_list

def get_names_of_visible_obj_inside(env, container_obj):
    filtered_object_names_list = get_task_objects(env)
    
    name_visible_objects_inside = []
    for name in filtered_object_names_list:
        obj2 = env.task.object_scope[name]
        is_inside = obj2.states[object_states.inside.Inside].get_value(container_obj)
    
        if is_inside:
            has_collisions = check_collision(container_obj, obj2)
            if has_collisions:
                print(f"{name} is colliding with the container")
            else:
                print(f"{name} is NOT colliding with the container")
                
            print(f'{name} is inside {container_obj.bddl_object_scope}')
            is_visible = obj2.states[object_states.InFOVOfRobot].get_value()
            print(f'{name} is visible: {is_visible}')
            if is_visible:
                name_visible_objects_inside.append(name)
    
    if len(name_visible_objects_inside) > 0:
        name = name_visible_objects_inside[0]
        obj2 = env.task.object_scope[name]
        
        # Check if it's visible
        is_visible = obj2.states[object_states.InFOVOfRobot].get_value()
        print(f'\n{name} is visible: {is_visible}')
        return name, obj2
    else:
        print("No visible object inside!")
        return None, None

def open_or_close(env, container_obj):
    print(f"{container_obj.bddl_object_scope} is open: {container_obj.states[object_states.Open].get_value()}")
    
    if not container_obj.states[object_states.Open].get_value():
        container_obj.states[object_states.Open].set_value(True) # check out if this is enough
    else:
        # For debugging purposes, if I execute the cell multiple times, swap the object state between open and close
        container_obj.states[object_states.Open].set_value(False)
    
    # Not clear how it works ... doesn't seem to change after the first time
    state = container_obj.dump_state()
    container_obj.load_state(state)
        
    settle_physics(env)

    print(f"{container_obj.bddl_object_scope} is open: {container_obj.states[object_states.Open].get_value()}")

def sample_point_in_container(container_obj):
    from igibson.object_states.utils import get_center_extent

    # Center and extension in 3 cardinal axes of the axis-aligned bounding box of the container object
    aabb_center, aabb_extent = get_center_extent(container_obj.states)

    # We sample one coordinate at the time for simplicity
    coordinates = []
    min3d = aabb_center - aabb_extent*0.5
    max3d = aabb_center + aabb_extent*0.5
    for axis in range(len(aabb_center)):
        coord = np.random.uniform(min3d[axis], max3d[axis])
        coordinates.append(coord)
    point = np.stack(coordinates)
    return point



def make_object_visible_inside_container(env, trg_obj, container_obj, sampling_budget = 100, physics_steps=10, max_distance_from_shoulder=0.8):
    # Define scene and robot here
    scene = env.scene
    robot = env.robots[0]
    controller = StarterSemanticActionPrimitives(None, scene, robot) 
    
    # 1. Check that container_obj is open, if not, raise an error (return success = False plus some info)
    container_is_open = container_obj.states[object_states.Open].get_value()
    if not container_is_open:
        success = False
        info = 'Container is not open - objects inside cannot be seen!'
        return success, info
        
    # Get initial predicates for the target object
    is_inside = trg_obj.states[object_states.inside.Inside].get_value(container_obj)
    is_visible = trg_obj.states[object_states.InFOVOfRobot].get_value()
    is_near = controller._get_dist_from_point_to_shoulder(trg_obj.get_position()) < max_distance_from_shoulder
    
    if is_inside and is_visible and is_near:
        # 2. If trg_obj is inside the container and is visible, return success = True plus some info
        success = True
        info = 'Target object is already inside the container and visible'
        return success, info
    elif not is_inside:
        # 3. If the trg_obj is not inside the container to start with, print a warning, add that to the info - we might turn it into a full error later
        info = 'Warning: Target object is not inside the container to start with.\n'
    elif not is_near:
        info = 'Initial object was out of reach'
    # If the object is inside but not visible, no additional action is needed
    
    # Save initial position of the trg_obj to reset it in case of failure 
    original_position = copy.deepcopy(trg_obj.get_position()) # not sure this is needed
    # Save initial state
    pb_initial_state = p.saveState()
    
    # 4. Sampling loop: 
    for i in range(sampling_budget):
        # Start every simulation by resetting the state of the simulator
        restoreState(pb_initial_state)
        
        # Sample 3d point inside the container volume
        candidate_pos = sample_point_in_container(container_obj)

        is_near = controller._get_dist_from_point_to_shoulder(candidate_pos) < max_distance_from_shoulder
        if is_near:
                
            # Set the object position to the candidate_pos
            trg_obj.set_position(candidate_pos)
            
            # Do 10-20 steps of the simulator and check for both isInside and isVisible
            settle_physics(env, physics_steps)
            
            is_inside = trg_obj.states[object_states.inside.Inside].get_value(container_obj)
            is_visible = trg_obj.states[object_states.InFOVOfRobot].get_value()
            is_near = controller._get_dist_from_point_to_shoulder(candidate_pos) < max_distance_from_shoulder
            has_collisions = check_collision(container_obj, trg_obj)
            
            if is_inside and is_visible and is_near and not has_collisions:
                # For promising candidates, check for longer times
                settle_physics(env, physics_steps)
                
                is_inside = trg_obj.states[object_states.inside.Inside].get_value(container_obj)
                is_visible = trg_obj.states[object_states.InFOVOfRobot].get_value()
                is_near = controller._get_dist_from_point_to_shoulder(candidate_pos) < max_distance_from_shoulder
                has_collisions = check_collision(container_obj, trg_obj)
                
                if is_inside and is_visible and is_near and not has_collisions:
                    break
        
        # If the budget ends without success, restore initial object position, return success = False and some info
        if i == (sampling_budget - 1):
            success = False
            trg_obj.set_position(original_position)
            restoreState(pb_initial_state)
            # Maybe it's not needed here
            #settle_physics(env, physics_steps)  # Pass physics_steps consistently
            info = 'Failed due to exhausting the budget'
            return success, info
            
    # 5. Finally, return success = True and some info
    success = True
    info = 'Object successfully moved to a visible position inside the container'
    return success, info

def get_objects_inside(env, container_obj):
    filtered_object_names_list = [name for name in list(env.task.object_scope.keys()) \
                              if name.split('.')[0] not in ['agent', 'floor']]
    names, objects = [], []
    for name in filtered_object_names_list:
        obj = env.task.object_scope[name]
        is_inside = obj.states[object_states.inside.Inside].get_value(container_obj)
        
        if is_inside:
            names.append(name)
            objects.append(obj)
            
    return names, objects