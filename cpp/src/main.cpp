#include <iostream>
#include <Eigen/Dense>
#include "simulator.h"

using namespace std;
using namespace Eigen;
using namespace std::chrono;

int main(int argc, char** argv) {
	cout << "Solving multi-robot motion planning problem..." << endl;

    // default config file
    std::string config_path = "../config/config.json";

    if (argc > 1) {
        config_path = argv[1];
    }

    std::ifstream my_config_file(config_path);
    assert(my_config_file && "Couldn't find the config file");

    Simulator sim(my_config_file);

    // Run simulation with duration from config
    sim.run();

    // Save data to files specified in config
    sim.saveDataToFile();
    sim.saveGoalDataToFile();
    
	return 0;
}


