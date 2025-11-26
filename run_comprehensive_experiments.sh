#!/bin/bash

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Script directory: $SCRIPT_DIR"

cd $SCRIPT_DIR

RUNS=5
SCENARIOS=(
    "scenario_1"  # Cross pattern - clear benefit case
    "scenario_2"  # Dense cross - moderate conflict
    "scenario_3"  # Circle formation - high conflict
    "scenario_4"  # Cross with BVC - conservative collision
    "scenario_5"  # Cross with on-demand - aggressive collision
    "scenario_6"  # Dense collision method comparison
    "scenario_7"  # Translating goals - dynamic targets
    "scenario_8"  # Circular goals - complex motion
    "scenario_9"  # Combined motion - ultimate challenge
)

// Creating results directory
mkdir -p "$SCRIPT_DIR/cpp/results/experiments/"


# Function to modify JSON config parameters
modify_json_config() {
    local config_file="$1"
    local param_name="$2"
    local param_value="$3"

    python3 << END_PYTHON
import json
with open('$config_file', 'r') as f:
    config = json.load(f)

if '${param_value}' == 'true':
    config['$param_name'] = True
elif '${param_value}' == 'false':
    config['$param_name'] = False
else:
    try:
        config['$param_name'] = int('${param_value}')
    except ValueError:
        try:
            config['$param_name'] = float('${param_value}')
        except ValueError:
            config['$param_name'] = '${param_value}'

with open('$config_file', 'w') as f:
    json.dump(config, f, indent=2)
END_PYTHON
}

# Function to run 1 experiment
run_single_experiment() {
    local scenario="$1"
    local method="$2"
    local run="$3"
    local config_modifications="$4"

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
        return 1
    fi

    cp "$CONFIG_FILE" "$OUTPUT_CONFIG_FILE"

    # Apply additional config modifications
    IFS=',' read -ra MODS <<< "$config_modifications"
    for mod in "${MODS[@]}"; do
        IFS=':' read -ra PARAM <<< "$mod"
        if [ "${#PARAM[@]}" -eq 2 ]; then
            echo "Modifying config: ${PARAM[0]} = ${PARAM[1]}"
            modify_json_config "$OUTPUT_CONFIG_FILE" "${PARAM[0]}" "${PARAM[1]}"
        fi
    done

    # Update output paths in config using Python
    python3 << END_PYTHON
import json
with open('${OUTPUT_CONFIG_FILE}', 'r') as f:
    config = json.load(f)
config['output_trajectories_paths'] = ['${OUTPUT_DIR}trajectories.txt']
config['output_goals_paths'] = ['${OUTPUT_DIR}goals.txt']
with open('${OUTPUT_CONFIG_FILE}', 'w') as f:
    json.dump(config, f, indent=2)
END_PYTHON

    # Run Simulation
    cd "$SCRIPT_DIR/cpp/bin/"
    ./run "${OUTPUT_CONFIG_FILE}" > "${OUTPUT_DIR}console.log" 2>&1

    # Copy reallocation log if it exists
    if [ -f "$SCRIPT_DIR/cpp/bin/reallocation_log.csv" ]; then
        cp "$SCRIPT_DIR/cpp/bin/reallocation_log.csv" "${OUTPUT_DIR}reallocation_log.csv"
    fi

    cd $SCRIPT_DIR

    echo "Experiment complete. Results saved in: $OUTPUT_DIR"
}

