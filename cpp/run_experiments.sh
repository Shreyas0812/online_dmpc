#!/bin/bash

SCENARIOS=("scenario_1" "scenario_2" "scenario_3")
METHODS=("static" "with_realloc")
RUNS=3

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $SCRIPT_DIR

# echo $SCRIPT_DIR

mkdir -p ./results/experiments/

echo ""
echo "=========================================="
echo "ALL EXPERIMENTS COMPLETE!"
echo "=========================================="
echo "Results saved in: ../results/experiments/"