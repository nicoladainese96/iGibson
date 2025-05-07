import os
import json
import argparse
import numpy as np
from PIL import Image

from unified_planning.shortcuts import *
from unified_planning.io import PDDLReader
up.shortcuts.get_env().credits_stream = None  # Disable print of credits

import bddl
from igibson.action_primitives.fetch_robot_semantic_actions_env import FetchRobotSemanticActionEnv
from igibson.custom_utils import get_env_config, print_properties
import igibson.render_utils as render_utils

from planning_utils import print_symbolic_state, execute_plan, translate_str_to_dict

def main(split: str, entry_index: Optional[int] = None):
    assert split in ['simple', 'medium', 'hard'], f"Invalid split '{split}'. Must be one of: simple, medium, hard."

    # Setup file paths
    pddl_dir = 'pddl'
    domain_file = os.path.join(pddl_dir, 'domain.pddl')
    metadata_file = f'{split}_split_metadata.json'
    pddl_split_folder = os.path.join(pddl_dir, split)

    with open(metadata_file) as f:
        split_metadata = json.load(f)

    entry_keys = list(split_metadata.keys())

    if entry_index is not None:
        assert 0 <= entry_index < len(entry_keys), f"Invalid index {entry_index}. Must be in range 0 to {len(entry_keys)-1}"
        entries_to_process = [entry_keys[entry_index]]
    else:
        entries_to_process = entry_keys  # fallback to full loop for local testing

    # Load domain
    reader = PDDLReader()
    benchmark_results = {}

    for entry in entries_to_process:
        task = split_metadata[entry]["activity_name"]
        print("\n\n" + "-" * 80)
        print(f"Executing task {task}")

        pddl_problem_file = os.path.join(pddl_split_folder, entry)
        problem = reader.parse_problem(domain_file, pddl_problem_file)

        with OneshotPlanner(problem_kind=problem.kind) as planner:
            result = planner.solve(problem)

        if result.status == up.engines.PlanGenerationResultStatus.SOLVED_SATISFICING:
            print("Plan found.")
            plan = [translate_str_to_dict(str(action)) for action in result.plan.actions]
            for action in plan:
                print(action)
        else:
            print("No plan found.")
            plan = []

        benchmark_results[task] = {}

        for task_instance, (scene_id, instance_id) in enumerate(split_metadata[entry]["scene_instance_pairs"]):
            print(f"\nTask_instance: {task_instance} - scene_id: {scene_id} - instance_id: {instance_id}")

            sim_env = FetchRobotSemanticActionEnv(task, scene_id, instance_id, verbose=False)
            plan_succeeded, plan_status = execute_plan(sim_env, plan, task, task_instance, split=split)

            benchmark_results[task][task_instance] = {
                'scene_id': scene_id,
                'instance_id': instance_id,
                'plan_succeeded': plan_succeeded,
                'plan_status': plan_status
            }
            print(f"Plan succeeded: {plan_succeeded}.")
            print("Detailed plan breakdown: ", plan_status)

            sim_env.env.clean()

        # Save individual result file
        result_out_path = os.path.join('results', split, f'{task}_entry_result.json')
        with open(result_out_path, 'w') as f:
            json.dump(benchmark_results, f, indent=2)

    print(f"\nEntry processing complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--split', type=str, default='hard', help="Which difficulty split to run: simple, medium, hard")
    parser.add_argument('--index', type=int, default=None, help="Optional: index of metadata entry to run")
    args = parser.parse_args()
    main(args.split, args.index)

