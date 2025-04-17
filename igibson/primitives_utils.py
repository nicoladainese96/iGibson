import copy
import numpy as np
import pybullet as p

from igibson.utils.utils import restoreState
from igibson import object_states
from igibson.action_primitives.starter_semantic_action_primitives import StarterSemanticActionPrimitives
import igibson.render_utils as render_utils

PHYSICS_STEPS = 30

def get_controller(env):
    controller = StarterSemanticActionPrimitives(None, env.scene, env.robots[0]) 
    return controller
    
def settle_physics(env, steps = None):
    if steps is None:
        steps = PHYSICS_STEPS
        
    s = env.simulator
    for _ in range(steps):
        s.step()
        
def check_collision_two_bodies(obj1, obj2, tol=1e-3):
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
                    #print("\nContact distance of penetration point: ", contactDistance)
                    return True
    
    return False

def check_collision_one_body(obj1, tol=1e-3):
    obj_ids1 = obj1.get_body_ids()

    for obj_id1 in obj_ids1:
            contact_points = p.getContactPoints(obj_id1)
            for contact_point in contact_points:
                contactFlag, bodyUniqueIdA, bodyUniqueIdB, linkIndexA, linkIndexB, positionOnA, positionOnB, contactNormalOnB, contactDistance, normalForce, lateralFriction1, lateralFrictionDir1, lateralFriction2, lateralFrictionDir2 = contact_point
                # Debug print
                #print('contactDistance', contactDistance)
                if contactDistance < 0 and np.abs(contactDistance)>tol:
                    # Then we have significant penetration between the two bodies
                    #print("\nContact distance of penetration point: ", contactDistance)
                    return True
    
    return False
    
def get_task_objects(env):
    # Let's also check for collisions here, assuming that there should be none, as it's the default state (sort of)
    filtered_object_names_list = [name for name in list(env.task.object_scope.keys()) \
                                  if name.split('.')[0] not in ['agent', 'floor']]
    return filtered_object_names_list

def print_status_within_container(env, container_obj, max_distance_from_shoulder=0.9):
    filtered_object_names_list = get_task_objects(env)
    controller = get_controller(env)
    
    for name in filtered_object_names_list:
        obj2 = env.task.object_scope[name]
        is_inside = obj2.states[object_states.inside.Inside].get_value(container_obj)
    
        if is_inside:
            print(f'{name} is inside {container_obj.bddl_object_scope}')
            
            has_collisions = check_collision_one_body(obj2)
            if has_collisions:
                print(f"{name} is colliding with some object")
            else:
                print(f"{name} is NOT colliding with any object")
                
            is_visible = obj2.states[object_states.IsVisible].get_value(env=env)
            pixel_count = obj2.states[object_states.VisiblePixelCountFromRobot].get_value()
            print(f'{name} is visible: {is_visible} - Pixels visible: {pixel_count}')

            is_near = controller._get_dist_from_point_to_shoulder(obj2.get_position()) < max_distance_from_shoulder
            print(f'{name} is near: {is_near}')
            print(f'Distance from shoulder: {controller._get_dist_from_point_to_shoulder(obj2.get_position()):.3f}')
        
def get_names_of_visible_obj_inside(env, container_obj):
    filtered_object_names_list = get_task_objects(env)
    
    name_visible_objects_inside = []
    for name in filtered_object_names_list:
        obj2 = env.task.object_scope[name]
        is_inside = obj2.states[object_states.inside.Inside].get_value(container_obj)
    
        if is_inside:
            has_collisions = check_collision_one_body(obj2)
            if has_collisions:
                print(f"{name} is colliding with some object")
            else:
                print(f"{name} is NOT colliding with any object")
                
            print(f'{name} is inside {container_obj.bddl_object_scope}')
            is_visible = obj2.states[object_states.IsVisible].get_value(env=env)
            pixel_count = obj2.states[object_states.VisiblePixelCountFromRobot].get_value()
            print(f'{name} is visible: {is_visible} - Pixels visible: {pixel_count}')
            if is_visible:
                name_visible_objects_inside.append(name)
    
    if len(name_visible_objects_inside) > 0:
        name = name_visible_objects_inside[0]
        obj2 = env.task.object_scope[name]
        
        # Check if it's visible
        is_visible = obj2.states[object_states.IsVisible].get_value(env=env)
        print(f'\n{name} is visible: {is_visible}')
        return name, obj2
    else:
        print("No visible object inside!")
        return None, None

def open_or_close(env, container_obj):
    state_before_action = p.saveState()
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
    return state_before_action, [container_obj]
    
def open_container(env, container_obj):
    state_before_action = p.saveState()
    print(f"{container_obj.bddl_object_scope} is open: {container_obj.states[object_states.Open].get_value()}")
    
    if not container_obj.states[object_states.Open].get_value():
        container_obj.states[object_states.Open].set_value(True) # check out if this is enough
    
        # Not clear how it works ... doesn't seem to change after the first time
        state = container_obj.dump_state()
        container_obj.load_state(state)
            
        settle_physics(env)

    print(f"{container_obj.bddl_object_scope} is open: {container_obj.states[object_states.Open].get_value()}")
    return state_before_action, [container_obj]

def reset_state(env, state_before, list_of_obj_to_awake):
    restoreState(state_before)
    for obj in list_of_obj_to_awake:
        obj.force_wakeup()
    settle_physics(env)

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

def open_and_make_all_obj_visible(
    env,
    container_obj,
    outer_attempts=5, 
    inner_attempts=300, 
    physics_steps=5, 
    physics_steps_extra=15,
    max_distance_from_shoulder=1.0,
    debug=False
):
    assert container_obj.states[object_states.Open].get_value() == False, "Container should be closed to use this action!"

    # Define controller - used to compute distances from robot shoulder - could be streamlined further as the full controller is not needed
    controller = get_controller(env)
    
    # Save original state - before being open
    original_container_state = p.saveState()
    objects_to_wake = []
    
    # Get list of objects inside the container_obj
    names, objects = get_objects_inside(env, container_obj)

    for i in range(outer_attempts):
        if i > 0:
            # If needed, restore initial state
            restoreState(original_container_state)
            for obj in objects_to_wake:
                obj.force_wakeup()
            settle_physics(env, steps=2) # no need to have much steps here

        if debug:
            render_utils.render_frame(env, show=True, save=True, name=f'debug-before-opening-{i}')
            
        # (Re-)Open container to obtain a new configuration 
        state_before_action, objects_to_wake = open_container(env, container_obj)

        if debug:
            render_utils.render_frame(env, show=True, save=True, name=f'debug-after-opening-{i}')

        objects_positioned = []
        
        # For every object inside the container, apply the function and render the state after the function call
        for name, trg_obj in zip(names, objects):
            if debug: print(f"Making visible {name}")
            
            success, info = make_visible(
                env, controller, trg_obj, container_obj, objects_positioned, 
                sampling_budget=inner_attempts, 
                physics_steps=physics_steps, 
                physics_steps_extra=physics_steps_extra, 
                max_distance=max_distance_from_shoulder
            )
            if debug:
                print(f"Success: {success}")
                print(info)

                # Print an update on all states of the objects involved
                print_status_within_container(env, container_obj)
            
            if success:
                objects_positioned.append(trg_obj) # Used to enforce more stingent constraints on the next samples
                if debug:
                    render_utils.render_frame_with_trg_obj(env, trg_obj, show=True, save=True, add_bbox=True, name=f'after-rearranging-{name}-attempt{i}')
            else:
                # If not successful, assume that the configuration of free visible space in the container is not enough to relocate the trg_obj
                # -> better sample a new one
                break

        # If all objects are visible, near (reachable) and inside the container: no need to do further attempts 
        all_visible = np.all([trg_obj.states[object_states.IsVisible].get_value(env=env) for trg_obj in objects])
        all_inside = np.all([trg_obj.states[object_states.inside.Inside].get_value(container_obj) for trg_obj in objects])
        all_near = np.all([controller._get_dist_from_point_to_shoulder(trg_obj.get_position()) < max_distance_from_shoulder for trg_obj in objects])

        if debug:
            for name, trg_obj in zip(names, objects):
                print(f"{name} IsVisible: ", trg_obj.states[object_states.IsVisible].get_value(env=env))
                print(f"{name} IsInside: ", trg_obj.states[object_states.inside.Inside].get_value(container_obj))
                print(f"{name} IsNear: ", controller._get_dist_from_point_to_shoulder(trg_obj.get_position()) < max_distance_from_shoulder)
                print(f'Distance from shoulder: {controller._get_dist_from_point_to_shoulder(trg_obj.get_position()):.3f}')
                
            print(f"all_visible: {all_visible} - all_inside: {all_inside} - all_near: {all_near}")
            
        if all_visible and all_inside and all_near:
            return True

    # If function fails, restore to initial state the environment
    restoreState(original_container_state)
    for obj in objects_to_wake:
        obj.force_wakeup()
    settle_physics(env, steps=2) # no need to have much steps here

    return False

def all_conditions_satisfied(env, controller, trg_obj, container_obj, objects_positioned, max_distance):
    """
    Efficient check for all 4 conditions needed for the sample to be accepted. Check them one by one from the fastest and return False at the first False instance.
    """
    is_visible = trg_obj.states[object_states.IsVisible].get_value(env=env)
    if not is_visible:
        return False
    else:
        is_inside = trg_obj.states[object_states.inside.Inside].get_value(container_obj)
        if not is_inside:
            return False
        else:
            has_collisions = check_collision_one_body(trg_obj)
            if has_collisions:
                return False
            else:
                is_near = controller._get_dist_from_point_to_shoulder(trg_obj.get_position()) < max_distance
                if not is_near:
                    return False
                else:
                    all_visible = np.all([obj_already_visible.states[object_states.IsVisible].get_value(env=env) for obj_already_visible in objects_positioned])
                    if all_visible:
                        return True
                    else:
                        return False
        
def make_visible(env, controller, trg_obj, container_obj, objects_positioned=[], sampling_budget = 100, physics_steps=5, physics_steps_extra=15, max_distance=0.8):
    # 1. Check that container_obj is open, if not, raise an error (return success = False plus some info)
    container_is_open = container_obj.states[object_states.Open].get_value()
    
    if not container_is_open:
        success = False
        info = 'Container is not open - objects inside cannot be seen!'
        return success, info
        
    # Get initial predicates for the target object - give for granted that has_collisions = False
    is_inside = trg_obj.states[object_states.inside.Inside].get_value(container_obj)
    is_visible = trg_obj.states[object_states.IsVisible].get_value(env=env)
    is_near = controller._get_dist_from_point_to_shoulder(trg_obj.get_position()) < max_distance
    
    if is_inside and is_visible and is_near:
        # 2. If trg_obj is inside the container and is visible, return success = True plus some info
        success = True
        pixel_count = trg_obj.states[object_states.VisiblePixelCountFromRobot].get_value()
        info = f'Target object is already inside the container and visible ({pixel_count} pixels visible)'
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

        is_near = controller._get_dist_from_point_to_shoulder(candidate_pos) < max_distance 
        if is_near:
                
            # Set the object position to the candidate_pos
            trg_obj.set_position(candidate_pos)
            
            # Do 'physics_steps' steps of the simulator and then check for all conditions - this is the heaviest part
            settle_physics(env, physics_steps)
            
            if all_conditions_satisfied(env, controller, trg_obj, container_obj, objects_positioned, max_distance):
                # For promising candidates, check for longer times
                settle_physics(env, physics_steps_extra)

                if all_conditions_satisfied(env, controller, trg_obj, container_obj, objects_positioned, max_distance):
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
    pixel_count = trg_obj.states[object_states.VisiblePixelCountFromRobot].get_value()
    info = f'Object successfully moved to a visible position inside the container ({pixel_count} pixels visible)'
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