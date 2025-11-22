//
// Created by carlos on 09/04/19.
//

#ifndef ONLINE_PLANNING_SIMULATOR_H
#define ONLINE_PLANNING_SIMULATOR_H

#include "generator.h"
#include "task_reallocation.h"
#include "json.hpp"
#include <random>
#include <fstream>

class Simulator {
public:
    struct Params {
        Generator::Params generator_params;
        float position_noise_std;
        float velocity_noise_std;
    };

//    explicit Simulator(const Simulator::Params& p);
    explicit Simulator(std::ifstream& config_file);
    ~Simulator(){};


    void run(int duration);
    void run();  // Run with duration from config
    void saveDataToFile(char const* pathAndName);
    void saveGoalDataToFile(char const* goalPathAndName);
    void saveDataToFile();  // Save to paths from config
    void saveGoalDataToFile();  // Save to paths from config
    
    // Getters for config parameters
    int getSimulationDuration() const { return _simulation_duration; }
    const std::vector<std::string>& getOutputTrajectoriesPaths() const { return _output_trajectories_paths; }
    const std::vector<std::string>& getOutputGoalsPaths() const { return _output_goals_paths; }

private:

    // Members
    BezierCurve::Params _bezier_params;
    DoubleIntegrator3D::Params _model_params;
    MpcParams _mpc_params;
    std::unique_ptr<Generator> _generator;
    std::unique_ptr<DoubleIntegrator3D> _sim_model;
    std::vector<Ellipse> _ellipses;
    std::vector<Eigen::MatrixXd> _inputs;
    std::vector<Eigen::MatrixXd> _goals;
    std::vector<State3D> _current_states;
    std::vector<Eigen::MatrixXd> _trajectories;
    std::vector<Eigen::MatrixXd> _goal_trajectories;
    float _pos_std;
    float _vel_std;
    Eigen::MatrixXd _po;
    Eigen::MatrixXd _pf;
    Eigen::VectorXd _pmin;
    Eigen::VectorXd _pmax;
    int _Ncmd;
    int _N;
    float _h;
    float _Ts;
    
    // Simulation config parameters
    int _simulation_duration;
    std::vector<std::string> _output_trajectories_paths;
    std::vector<std::string> _output_goals_paths;
    
    // Collision check parameters
    float _collision_check_rmin;
    int _collision_check_order;
    float _collision_check_height_scaling;
    float _goal_tolerance;

    // Methods
    Generator::Params parseJSON(std::ifstream& config_file);
    State3D addRandomNoise(const State3D& states);
    bool collisionCheck(const std::vector<Eigen::MatrixXd>& trajectories);
    bool goalCheck(const std::vector<State3D>& states);
    Eigen::MatrixXd generateRandomPoints(int N, const Eigen::Vector3d &pmin,
                                         const Eigen::Vector3d &pmax, float rmin);
    
    // Task reallocation
    std::unique_ptr<TaskReallocationManager> _reallocation_manager;
    bool _reallocation_enabled;
    double _reallocation_period;
    std::vector<int> _current_assignment;
    std::vector<Eigen::Vector3d> _original_goals_vec;
};

#endif //ONLINE_PLANNING_SIMULATOR_H
