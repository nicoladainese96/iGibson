import os
import json
from PIL import ImageDraw

def execute_plan(sim_env, plan, task, task_instance, split='default'):
    # Define base directories
    base_image_dir = os.path.join('images', split, f'{task}_{task_instance}')
    base_results_dir = os.path.join('results', split, f'{task}_{task_instance}')
    
    os.makedirs(base_image_dir, exist_ok=True)
    os.makedirs(base_results_dir, exist_ok=True)

    image_idx = 0
    image, symbolic_state = sim_env.get_state_and_image()

    # Save initial image
    image_path = os.path.join(base_image_dir, f'step_{image_idx}.png')
    image.save(image_path)
    image_idx += 1

    image.show()
    print_symbolic_state(symbolic_state)

    plan_status = {'plan': plan, 'successes': [], 'legals': []}
    for action in plan:
        try:
            legal, info, image, symbolic_state = sim_env.apply_action(action)
            print(f'Action {action} executed. Legal: {legal} - Info: {info}')

            # Save image after action
            image_path = os.path.join(base_image_dir, f'step_{image_idx}.png')
            image.save(image_path)
            image_idx += 1

            image.show()
            print_symbolic_state(symbolic_state)

            success = 'success' in info
            plan_status['successes'].append(success)
            plan_status['legals'].append(legal)

        except ValueError as e:
            print(f"Action {action} illegal: {e}")
            plan_status['successes'].append(False)
            plan_status['legals'].append(False)
            break

        except AssertionError as e:
            print(f"Action {action} failed: {e}")

            image, symbolic_state = sim_env.get_state_and_image()
            image_path = os.path.join(base_image_dir, f'step_{image_idx}.png')
            image.save(image_path)
            image_idx += 1

            image.show()
            print_symbolic_state(symbolic_state)

            plan_status['successes'].append(False)
            plan_status['legals'].append(True)

    plan_succeeded = all(plan_status['successes'])

    # Save results
    result_data = {
        'plan_succeeded': plan_succeeded,
        'plan_status': plan_status
    }
    result_path = os.path.join(base_results_dir, 'result.json')
    with open(result_path, 'w') as f:
        json.dump(result_data, f, indent=2)

    return plan_succeeded, plan_status



def translate_str_to_dict(action):
    action_name, action_args = action.split('(')

    # Remove trailing ')'
    action_args = action_args.split(')')[0] 
    
    if ',' in action_args:
        # Multiple arguments: split by comma, remove extra white spaces
        action_args = [s.strip() for s in action_args.split(',')]
    else:
        # Single arg
        action_args = [action_args]

    return {'action':action_name, 'params':action_args}
    
def print_symbolic_state(state, verbose=False):
    print("-"*76)
    print("Symbolic state: \n")
    for k in state:
        if verbose:
            print(k, state[k])
        else:
            true_states = {}
            for key in state[k].keys():
                val = state[k][key] # bool
                if val: 
                    true_states[key] = val
                    
            print(k, true_states)
    print("-"*76)