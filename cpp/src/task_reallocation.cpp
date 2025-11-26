//
// Created by raorane on 24/11/25.
//

#include "task_reallocation.h"

TaskReallocationManager::TaskReallocationManager(
    double reallocation_period,
    bool use_predictive,
    double prediction_horizon)
    : reallocation_period_(reallocation_period), last_reallocation_time_(-reallocation_period), reallocation_count_(0), _use_predictive(use_predictive), _prediction_horizon(prediction_horizon)
{
    log_file_.open("reallocation_log.csv");
    log_file_ << "timestamp,reallocation_id,agent_id,old_goal,new_goal,distance,method\n";
}

std::vector<int> TaskReallocationManager::computeOptimalAssignment(
    const std::vector<Eigen::Vector3d>& agent_positions,
    const std::vector<Eigen::Vector3d>& goal_positions
) {
    int n = agent_positions.size();

    // Create cost matrix based on Euclidean distances
    std::vector<std::vector<double>> cost_matrix(n, std::vector<double>(n));

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            cost_matrix[i][j] = (agent_positions[i] - goal_positions[j]).norm();
        }
    }

    // Solve assignment problem using Hungarian algorithm
    HungarianAlgorithm hungarian;
    std::vector<int> assignment;
    double cost = hungarian.Solve(cost_matrix, assignment);

    std::cout << "Total assignment cost: " << cost << std::endl;

    return assignment;
}

std::vector<int> TaskReallocationManager::computePredictiveAssignment(
    const std::vector<Eigen::Vector3d>& current_positions,
    const std::vector<Eigen::MatrixXd>& predicted_horizons,
    const std::vector<Eigen::Vector3d>& goal_positions,
    double Ts
) {
    // For simplicity, using current positions for cost calculation
    // return computeOptimalAssignment(current_positions, goal_positions);

    int n = current_positions.size();
    std::vector<std::vector<double>> cost_matrix(n, std::vector<double>(n));

    // timestamp at which to predict
    int prediction_step = static_cast<int>(_prediction_horizon / Ts);

    for (int i = 0; i < n; i++) {
        Eigen::Vector3d predicted_pos;

        if (prediction_step < predicted_horizons[i].cols()) {
            predicted_pos = predicted_horizons[i].col(prediction_step);
        } else {
            predicted_pos = predicted_horizons[i].col(predicted_horizons[i].cols() - 1);  // Fallback to last predicted position
        }

        for (int j = 0; j < n; j++) {
            cost_matrix[i][j] = (predicted_pos - goal_positions[j]).norm();
        }
    }


    // Solve assignment problem using Hungarian algorithm
    HungarianAlgorithm hungarian;
    std::vector<int> assignment;
    double cost = hungarian.Solve(cost_matrix, assignment);

    std::cout << "[PREDICTIVE] Total assignment cost: " << cost << std::endl;

    return assignment;
}

bool TaskReallocationManager::updateAssignment(
    double current_time,
    const std::vector<Eigen::Vector3d>& agent_positions,
    const std::vector<Eigen::MatrixXd>& predicted_horizons,
    const std::vector<Eigen::Vector3d>& goal_positions,
    std::vector<int>& assignment,
    double Ts
) {
    if (!shouldReallocate(current_time)) {
        return false;
    }
    
    std::vector<int> new_assignment;

    if (_use_predictive) {
        new_assignment = computePredictiveAssignment(agent_positions, predicted_horizons, goal_positions, Ts);
    } else {
        new_assignment = computeOptimalAssignment(agent_positions, goal_positions);
    }

    // Initialize current_assignment_ with the input assignment on first call
    if (current_assignment_.empty()) {
        current_assignment_ = assignment;
    }

    // Check if assignment has changed
    bool changed = false;
    for (size_t i = 0; i < new_assignment.size(); i++) {
        if (new_assignment[i] != current_assignment_[i]) {
            changed = true;
            break;
        }
    }

    if (changed) {
        reallocation_count_++;
        std::cout << "\n=== Reallocation #" << reallocation_count_ << " at time " << current_time << "s ===" << std::endl;

        // Log changes
        for (size_t i = 0; i < new_assignment.size(); i++) {  
            int old_goal = current_assignment_.empty() ? -1 : current_assignment_[i];
            int new_goal = new_assignment[i];

            if (old_goal != new_goal) {
                double distance = (agent_positions[i] - goal_positions[new_goal]).norm();
                
                std::cout << "Agent " << i << ": Goal changed from " << old_goal 
                            << " to " << new_goal << " (Distance: " << distance << " m)" << std::endl;
                
                log_file_ << current_time << "," 
                        << reallocation_count_ << "," 
                        << i << "," 
                        << old_goal << "," 
                        << new_goal << "," 
                        << distance << ","
                        << (_use_predictive ? "predictive" : "reactive") << "\n";
            }
        }
        log_file_.flush();

        // Update stored assignment
        current_assignment_ = new_assignment;
        assignment = new_assignment;
        last_reallocation_time_ = current_time;

        return true;

    }

    return false;
}