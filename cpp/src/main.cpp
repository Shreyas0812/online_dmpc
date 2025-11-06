#include <iostream>
#include <Eigen/Dense>
#include "simulator.h"

using namespace std;
using namespace Eigen;
using namespace std::chrono;

int main() {
	cout << "Solving multi-robot motion planning problem..." << endl;

    std::ifstream my_config_file("../config/config.json");
    assert(my_config_file && "Couldn't find the config file");

    Simulator sim(my_config_file);

    int T = 25; // simulation duration
    sim.run(T);

    // Save data to file
    string file = "../results/trajectories.txt";
    sim.saveDataToFile(file.c_str());

    string file2 = "../../../SwarmSync/trajectories.txt";
    sim.saveDataToFile(file2.c_str());

    string goal_file = "../results/goals.txt";
    sim.saveGoalDataToFile(goal_file.c_str());

    string goal_file2 = "../../../SwarmSync/goals.txt";
    sim.saveGoalDataToFile(goal_file2.c_str());
    
	return 0;
}


