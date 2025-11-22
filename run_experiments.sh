#!/bin/bash

SCENARIOS=("scenario_1" "scenario_2" "scenario_3")
METHODS=("static" "with_realloc")
RUNS=3

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $SCRIPT_DIR

echo $SCRIPT_DIR

for scenario in "${SCENARIOS[@]}"; do
    for method in "${METHODS[@]}"; do
        for run in $(seq 1 $RUNS); do
            echo ""
            echo "=========================================="
            echo "Running Experiment: Scenario=$scenario, Method=$method, Run=$run"
            echo "=========================================="

            # Create output directory
            # OUTPUT_DIR="./results/experiments/${scenario}/${method}/run_${run}/"
        done
    done
done

mkdir -p ./results/experiments/

echo ""
echo "=========================================="
echo "ALL EXPERIMENTS COMPLETE!"
echo "=========================================="
echo "Results saved in: ../results/experiments/"