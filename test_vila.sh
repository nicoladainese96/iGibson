#!/bin/bash -l

#SBATCH --job-name=igibson_vila
#SBATCH --output=./slurm/%j.out
#SBATCH --cpus-per-task=4
#SBATCH --gres=gpu:1
#SBATCH --partition=gpu-h100-80g,gpu-a100-80g,gpu-h200-141g-short
#SBATCH --mem=40G
#SBATCH --time=00:30:00

NODE=$(hostname -s)
echo "Running server+client on node: $NODE"

MODEL="Qwen/Qwen2.5-VL-7B-Instruct"

# Run server
# Start server in background, on this same node
srun --ntasks=1 --gres=gpu:1 --mem=40G -w $NODE \
    make run &

echo "Waiting for server to start..."
sleep 60
echo "Server should be running now."

BASE_URL="http://${NODE}:8000"
echo "Testing with BASE_URL=${BASE_URL}"

source activate test_llm_env
cd open-world-symbolic-planner

# python3 open-world-symbolic-planner/symbolic_planner/experiments/benchmark_igibson_vila.py \
python3 -m symbolic_planner.experiments.benchmark_igibson_vila \
    --base_url "${BASE_URL}" \
    --model_name "${MODEL}" \
    --domain_file "data/planning/igibson/domain.pddl" \
    --problems_dir "data/planning/igibson/easy" \
    --prompt_path "data/prompts/planning/vila_igibson_json.md" \
    --output_dir "results/planning/vila/igibson/easy"