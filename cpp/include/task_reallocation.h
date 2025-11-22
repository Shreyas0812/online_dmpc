#ifndef TASK_REALLOCATION_H
#define TASK_REALLOCATION_H

#include <vector>
#include <Eigen/Dense>
#include "Hungarian.h"
#include <iostream>
#include <fstream>
#include <cmath>

class TaskReallocationManager {
    private:
        double reallocation_period_;
        double last_reallocation_time_;
        int reallocation_count_;
        std::vector<int> current_assignment_;
        std::ofstream log_file_;
    
    public:
        TaskReallocationManager(double reallocation_period = 2.0)
            : reallocation_period_(reallocation_period), last_reallocation_time_(0.0), reallocation_count_(0)
        {
            log_file_.open("reallocation_log.csv");
            log_file_ << "timestamp,reallocation_id,agent_id,old_goal,new_goal,distance\n";
        }

        ~TaskReallocationManager() {
            if (log_file_.is_open()) {
                log_file_.close();
            }
        }

        bool shouldReallocate(double current_time) {
            return (current_time - last_reallocation_time_) >= reallocation_period_;
        }

        std::vector<int> computeOptimalAssignment(
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

        bool updateAssignment(
            double current_time,
            const std::vector<Eigen::Vector3d>& agent_positions,
            const std::vector<Eigen::Vector3d>& goal_positions,
            std::vector<int>& assignment
        ) {
            if (!shouldReallocate(current_time)) {
                return false;
            }

            std::vector<int> new_assignment = computeOptimalAssignment(agent_positions, goal_positions);

            bool changed = false;
            if (current_assignment_.empty()) {
                current_assignment_ = assignment;
            }

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
                                << distance << "\n";
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

        int getReallocationCount() const {
            return reallocation_count_;
        }
};

#endif // TASK_REALLOCATION_H