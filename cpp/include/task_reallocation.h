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
        bool _use_predictive;
        double _prediction_horizon;
    
    public:
        TaskReallocationManager(double reallocation_period = 2.0,
                                bool use_predictive = false,
                                double prediction_horizon = 1.0);

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
        );
        
        std::vector<int> computePredictiveAssignment(
            const std::vector<Eigen::Vector3d>& current_positions,
            const std::vector<Eigen::MatrixXd>& predicted_horizons,
            const std::vector<Eigen::Vector3d>& goal_positions,
            double Ts
        );

        bool updateAssignment(
            double current_time,
            const std::vector<Eigen::Vector3d>& agent_positions,
            const std::vector<Eigen::MatrixXd>& predicted_horizons,
            const std::vector<Eigen::Vector3d>& goal_positions,
            std::vector<int>& assignment,
            double Ts
        );

        int getReallocationCount() const {
            return reallocation_count_;
        }
};

#endif // TASK_REALLOCATION_H