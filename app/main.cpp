/// @file main.cpp
/// @brief Commandline utility for sbmpo
///
/// # Usage
/// ~~~
/// planner config.json results.json
/// ~~~
#include <fstream>
#include <stdexcept>
#include <string>


#include "debug.h"
#include "docopt.h"
#include "config.h"
#include "sbmpo.h"
#include "model.h"
#include "model/energy.h"
#include "model/1D.h"


// Specify usage string, from which we specify how command-line arguments should
// be parsed.  See instructions at https://github.com/docopt/docopt.cpp/
static const char USAGE[] =
R"(Usage: sbmpo [-hvq] <config> [-m FILE] [-o FILE]

Options:
    -h --help              Show this screen.
    -v --verbose           Output diagnostic messages to std err
    -q --quiet             Silence output to std out
    -m FILE --model=FILE   Provide a seperate model config file
    -o FILE --output=FILE  Specify json output file [default: .results/result.json]

COPYRIGHT CISCOR 2017, all rights reserved
)";


/// @brief load model from config json
///
/// Throws an exception when necessary key isn't found or if the modle type
/// in the config doesn't match any known model type in the code.
model::Model* load_model(const config::json& model_config);
void write_obst(const string filename, const vector<Obstacle> & obst_vec);


int main(int argc, char* argv[]) {
    // Parse arguments using docopt library
    // https://github.com/docopt/docopt.cpp
    auto args = docopt::docopt(USAGE, {argv+1, argv+argc});

    #ifdef DEBUG_BUILD
    ofstream debug_file("results/debug_log");
    std::clog.rdbuf(debug_file.rdbuf());
    for (auto arg : args) {
        DEBUG("ARGS => parsed, " << arg.first << " => " << arg.second);
    }
    #endif

    // Load config json from command-line arguments
    config::json config;
    std::ifstream config_file(args["<config>"].asString());
    config_file >> config;

    // Declare model
    model::Model* model;

    // Read model from config file, providing user-friendly error if we fail
    // to read the config
    try {
        // Handle command-line arguments to find the model config
        config::json model_config;
        if (args["--model"].isString()) {
            std::ifstream model_file(args["--model"].asString());
            model_file >> model_config;
        }
        else {
            model_config = config.at("model");
        }

        // Load model
        model = load_model(model_config);
    }
    // When attempting to read model from json, if we search for a key that
    // doesn't exist, it will throw a std::out_of_range exception.
    catch (std::out_of_range& oa) {
        std::cerr
            << "Failed to read model from config: "
            << oa.what()
            << std::endl;
        return EXIT_FAILURE;
    }
    // If the specified model type doesn't match any of the supported models,
    // we throw a config error that must be caught.
    catch (config::ConfigError& ce) {
        std::cerr
            << ce.what()
            << std::endl;
        return EXIT_FAILURE;
    }

    // Create json to capture output
    config::json output;

    // Initialize planner from config
    sbmpo::Planner planner = config::Tuner()
            .verbose(args["--verbose"].asBool()) // output debug information
            .load_config(config)
            .init(model);

    // MCQ
    Obstacle man_obst = {2.0, 2.0, 0.5};
    vector<float> man_vel = {1.0, 0.0};
    
    int n_iter = 3;  // nof iterations
    float search_delta_t = 0.5;
    int n_search_steps = 1;
    float dT = n_search_steps * search_delta_t;  // iteration time, should be a multiple of search's dt

    State start = {0.0, 0.0, 0.0};
    State goal = {10.0, 10.0, 0.0};
    State current_state = start;
    cout << "Model control dof: " << model->control_dof() << endl;
    for (int iter = 0; iter < n_iter; iter++)
    {
        cout << "iter " << iter << args["--output"].asString() << endl;
        cout << config.at("model").at("obstacle file") << endl;
        // update obstacles
        for (int i = 0; i < 2; i++) man_obst[i] += dT * man_vel[i];
        vector<Obstacle> obst_vec = {man_obst};
        string filename = "./results/obstacles_" + to_string(iter) + ".json";
        write_obst(filename, obst_vec);

        // pass map to the model
        model->set_obstacles(obst_vec);

        // init planner with current pose
        planner.setModel(model);
        cout << "current state: ";
        for (auto v : current_state) cout << v << ", ";
        cout << endl;

        Trajectory results = planner.compute_trajectory(current_state, goal);
        output = results;

        filename = args["--output"].asString();
        size_t lastindex = filename.find_last_of("."); 
        filename = filename.substr(0, lastindex);
        filename = filename + "_" + to_string(iter) + ".json";
        cout << filename << endl;

        ofstream output_file(filename);
        // if (!args["--quiet"].asBool()) {
        //     std::cout << output << std::endl;
        // }
        output_file << output << std::endl;
        output_file.close();
    
        // move along the path, get a new pose
        current_state = results.trajectory[n_search_steps].state;
    }
}

void write_obst(const string filename, const vector<Obstacle> & obst_vec)
{
    ofstream file(filename);
    for (const Obstacle obst : obst_vec)
    {
        for (float v : obst)
        {
            file << v << " ";
        }
        file << endl;
    }
}

model::Model* load_model(const config::json& model_config) {

    bool model_is_initialized = false;
    model::Model* m;

    #ifdef DEBUG_BUILD
    config::json debug_model;
    #endif

    // Read model type
    std::string model_type = model_config
        .at("type")
        .get<std::string>();

    // Initialize energy model from config
    if (model_type == "energy") {
        // Allocate memory for model and assign it
        m = new model::EnergyModel(model_config);
        model_is_initialized = true;

        #ifdef DEBUG_BUILD
        debug_model = model_config.get<model::EnergyModel>();
        #endif
    }
    if (model_type == "double integrator") {
        // Allocate memory for model and assign it
        m = new model::DoubleIntegrator(model_config);
        model_is_initialized = true;

        #ifdef DEBUG_BUILD
        debug_model = model_config.get<model::DoubleIntegrator>();
        #endif
    }
    
    // Add new model initialization here...

    // Throw user friendly error if the model isn't found
    if (!model_is_initialized) {
        throw config::ConfigError(
            "model \"\033[0;31m"
            + model_type
            + "\033[0m\" not found."
        );
    }
    // Check that model parsed properly by logging to the console
    DEBUG("CONFIG => model, type => "
        << model_type
    );

    return m;
}
