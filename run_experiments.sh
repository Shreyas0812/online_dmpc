#!/bin/bash

SCENARIOS=("scenario_1" "scenario_2" "scenario_3")
METHODS=("static" "with_realloc")
RUNS=3

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo $SCRIPT_DIR
cd $SCRIPT_DIR
mkdir -p "$SCRIPT_DIR/cpp/results/experiments/"

for scenario in "${SCENARIOS[@]}"; do
    for method in "${METHODS[@]}"; do
        for run in $(seq 1 $RUNS); do
            echo ""
            echo "=========================================="
            echo "Running Experiment: Scenario=$scenario, Method=$method, Run=$run"
            echo "=========================================="

            # Create output directory
            OUTPUT_DIR="$SCRIPT_DIR/cpp/results/experiments/${scenario}/${method}/run_${run}/"
            echo "Creating output directory: $OUTPUT_DIR"
            mkdir -p "$OUTPUT_DIR"

            # Copy config file and modify for this run
            CONFIG_FILE="$SCRIPT_DIR/cpp/config/${scenario}.json"
            OUTPUT_CONFIG_FILE="$OUTPUT_DIR/config.json"
            echo "Using config file: $CONFIG_FILE"

            # Check if config file exists
            if [ ! -f "$CONFIG_FILE" ]; then
                echo "ERROR: Config file not found: $CONFIG_FILE"
                continue
            fi

            # Modify reallocation_enabled based on method
            if [ "$method" == "static" ]; then
                # Disable reallocation
                echo "Disabling reallocation in config."
                cat "$CONFIG_FILE" | sed 's/"reallocation_enabled": true/"reallocation_enabled": false/' > "$OUTPUT_CONFIG_FILE"
            else
                # Keep reallocation enabled
                echo "Enabling reallocation in config."
                cp "$CONFIG_FILE" "$OUTPUT_CONFIG_FILE"
            fi

            # Update output paths in config using Python
            python3 -c "
import json

with open('${OUTPUT_CONFIG_FILE}', 'r') as f:
    config = json.load(f)

config['output_trajectories_paths'] = ['${OUTPUT_DIR}trajectories.txt']
config['output_goals_paths'] = ['${OUTPUT_DIR}goals.txt']

with open('${OUTPUT_CONFIG_FILE}', 'w') as f:
    json.dump(config, f, indent=2)
"

        # Run Simulation
        cd "$SCRIPT_DIR/cpp/bin/"
        ./run "${OUTPUT_CONFIG_FILE}" > "${OUTPUT_DIR}console.log" 2>&1

        # Copy reallocation log if it exists
        if [ -f "$SCRIPT_DIR/cpp/bin/reallocation_log.csv" ]; then
            cp "$SCRIPT_DIR/cpp/bin/reallocation_log.csv" "${OUTPUT_DIR}reallocation_log.csv"
        fi

        cd $SCRIPT_DIR

        echo "Experiment complete. Results saved in: $OUTPUT_DIR"
        done
    done
done

mkdir -p ./results/experiments/

echo ""
echo "=========================================="
echo "ALL EXPERIMENTS COMPLETE!"
echo "=========================================="
echo "Results saved in: $SCRIPT_DIR/cpp/results/experiments/"