#include <iostream>
#include <Eigen/Dense>
#include "simulator.h"
#include "task_reallocation.h"

using namespace std;
using namespace Eigen;
using namespace std::chrono;

int main() {
	cout << "Solving multi-robot motion planning problem..." << endl;

    std::ifstream my_config_file("../config/config.json");
    assert(my_config_file && "Couldn't find the config file");

    Simulator sim(my_config_file);

    // Run simulation with duration from config
    sim.run();

    // Save data to files specified in config
    sim.saveDataToFile();
    sim.saveGoalDataToFile();
    
	return 0;
}


