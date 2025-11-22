#!/bin/bash

SCENARIOS=("scenario_1" "scenario_2" "scenario_3")
METHODS=("static" "with_realloc")
RUNS=3

# Create results directories
mkdir -p ../results/experiments

for scenario in "${SCENARIOS[@]}"; do
    for method in "${METHODS[@]}"; do
        for run in $(seq 1 $RUNS); do
            echo ""
            echo "=========================================="
            echo "Running: $scenario - $method - run $run"
            echo "=========================================="
            
            # Create output directory
            OUT_DIR="../results/experiments/${scenario}/${method}/run_${run}"
            mkdir -p $OUT_DIR
            
            # Copy config and modify for this run
            CONFIG_FILE="../config/${scenario}.json"
            TEMP_CONFIG="${OUT_DIR}/config_temp.json"
            
            # Modify reallocation_enabled based on method
            if [ "$method" == "static" ]; then
                # Disable reallocation
                cat $CONFIG_FILE | sed 's/"reallocation_enabled": true/"reallocation_enabled": false/' > $TEMP_CONFIG
            else
                # Keep reallocation enabled
                cp $CONFIG_FILE $TEMP_CONFIG
            fi
            
            # Update output paths in config
            cat $TEMP_CONFIG | \
                sed "s|\"../results/.*_trajectories.txt\"|\"${OUT_DIR}/trajectories.txt\"|" | \
                sed "s|\"../results/.*_goals.txt\"|\"${OUT_DIR}/goals.txt\"|" \
                > "${OUT_DIR}/config.json"
            
            # Run simulation
            cd build
            ./online_dmpc "${OUT_DIR}/config.json" > "${OUT_DIR}/console.log" 2>&1
            
            # Copy reallocation log if it exists
            if [ -f "reallocation_log.csv" ]; then
                mv reallocation_log.csv "${OUT_DIR}/"
            fi
            
            cd ..
            
            # Clean up temp file
            rm $TEMP_CONFIG
            
            echo "✓ Complete - Output saved to $OUT_DIR"
        done
    done
done

echo ""
echo "=========================================="
echo "ALL EXPERIMENTS COMPLETE!"
echo "=========================================="
echo "Results saved in: ../results/experiments/"