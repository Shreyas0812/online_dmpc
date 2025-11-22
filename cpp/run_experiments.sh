#!/bin/bash

SCENARIOS=("scenario_1" "scenario_2" "scenario_3")
METHODS=("static" "with_realloc")
RUNS=3

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Create results directories
mkdir -p ../results/experiments

for scenario in "${SCENARIOS[@]}"; do
    for method in "${METHODS[@]}"; do
        for run in $(seq 1 $RUNS); do
            echo ""
            echo "=========================================="
            echo "Running: $scenario - $method - run $run"
            echo "=========================================="
            
            # Create output directory (use absolute path)
            OUT_DIR="$SCRIPT_DIR/../results/experiments/${scenario}/${method}/run_${run}"
            mkdir -p "$OUT_DIR"
            
            # Copy config and modify for this run
            CONFIG_FILE="$SCRIPT_DIR/config/${scenario}.json"
            TEMP_CONFIG="${OUT_DIR}/config_temp.json"
            
            # Check if config file exists
            if [ ! -f "$CONFIG_FILE" ]; then
                echo "ERROR: Config file not found: $CONFIG_FILE"
                continue
            fi
            
            # Modify reallocation_enabled based on method
            if [ "$method" == "static" ]; then
                # Disable reallocation
                cat "$CONFIG_FILE" | sed 's/"reallocation_enabled": true/"reallocation_enabled": false/' > "$TEMP_CONFIG"
            else
                # Keep reallocation enabled
                cp "$CONFIG_FILE" "$TEMP_CONFIG"
            fi
            
            # Update output paths in config (modify the arrays)
            python3 -c "
import json

with open('${TEMP_CONFIG}', 'r') as f:
    config = json.load(f)

config['output_trajectories_paths'] = ['${OUT_DIR}/trajectories.txt']
config['output_goals_paths'] = ['${OUT_DIR}/goals.txt']

with open('${OUT_DIR}/config.json', 'w') as f:
    json.dump(config, f, indent=2)
"
            
            # Run simulation
            cd "$SCRIPT_DIR/bin"
            ./run "${OUT_DIR}/config.json" > "${OUT_DIR}/console.log" 2>&1
            
            # Copy reallocation log if it exists
            if [ -f "$SCRIPT_DIR/build/reallocation_log.csv" ]; then
                mv "$SCRIPT_DIR/build/reallocation_log.csv" "${OUT_DIR}/"
            fi
            
            cd "$SCRIPT_DIR"
            
            # Clean up temp file
            rm -f "$TEMP_CONFIG"
            
            echo "✓ Complete - Output saved to $OUT_DIR"
        done
    done
done

echo ""
echo "=========================================="
echo "ALL EXPERIMENTS COMPLETE!"
echo "=========================================="
echo "Results saved in: ../results/experiments/"