#include <getopt.h>

#include <default_params.hpp>
#include <iostream>
#include <mapf_problem.hpp>
#include <mapf_solver.hpp>
#include <getopt.h>
#include <random>
#include <vector>
#include <fstream>

void printHelp()
{
  std::cout << "\nUsage: ./lamapf [OPTIONS] [SOLVER-OPTIONS]\n"
            << "\n**instance file is necessary to run LA-MAPF simulator**\n\n"
            << "  -i --instance [FILE_PATH]     instance file path\n"
            << "  -o --output [FILE_PATH]       ouptut file path\n"
            << "  -h --help                     help\n"
            << "  -s --solver [SOLVER_NAME]     solver (LAPIBT)\n"
            << "  -n --trials [int]             number of trials to execute\n"
            << std::endl;
}

int main(int argc, char *argv[])
{
  std::string instance_file;
  std::string output_file = DEFAULT_OUTPUT_FILE;
  std::string solver_name;
  int number_of_trials = 10;

  bool verbose = false;
  char *argv_copy[argc + 1];

  for (int i = 0; i < argc; ++i)
    argv_copy[i] = argv[i];

  struct option longopts[] = {
      {"instance", required_argument, 0, 'i'},
      {"output", required_argument, 0, 'o'},
      {"solver", required_argument, 0, 's'},
      {"trials", required_argument, 0, 'n'},
      {"verbose", no_argument, 0, 'v'},
      {0, 0, 0, 0},
  };

  int inheritanceDepth = DEFAULT_INHERITANCE_DEPTH;
  // command line args
  int opt, longindex;

  opterr = 0; // ignore getopt error

  while ((opt = getopt_long(argc, argv, "i:o:s:vn:", longopts,
                            &longindex)) != -1)
  {
    switch (opt)
    {
    case 'i':
      instance_file = std::string(optarg);
      break;
    case 's':
      solver_name = std::string(optarg);
      break;
    case 'o':
      output_file = std::string(optarg);
      break;
    case 'h':
      printHelp();
      return 0;
    case 'v':
      verbose = true;
      break;
    case 'n':
      std::sscanf(optarg, "%d", &number_of_trials);
      break;
    default:
      break;
    }
  }
  
  if (instance_file.length() == 0)
  {
    std::cout << "specify instance file using -i [INSTANCE-FILE], e.g.,"
              << std::endl;
    std::cout << "> ./large-agents-mapf -i ../instance/sample.txt" << std::endl;
    return 0;
  }

  auto P = LargeAgentsMapfProblem(instance_file);
  int number_of_agents = P.getNum();
  std::vector<float> random_radiuses = P.getMinMaxRadiuses();
  int while_iterations = number_of_trials;
  int succeses = 0; // Number of succeses;
  int failures = 0; // Number of failures;
  std::vector<int> costs = {}; // Costs, e.g steps from begginig to goal;
  std::vector<int> lower_bound_costs = {}; // Lower bound of costs, e.g steps from begginig to goal;
  std::vector<int> steps = {}; // Max of timesteps spend computing;
  std::vector<int> lower_bound_steps = {}; // Lower bound of max of timesteps spend computing;
  std::vector<int> elapsed_time = {}; // Elapsed time spend computing;

  std::cout << "Progress: " ;
  std::cout.flush();
  
  while (while_iterations--) {
    std::cout << int( (number_of_trials - while_iterations) * 100 / number_of_trials )
              << "% | ";
  std::cout.flush();
  

    auto P = LargeAgentsMapfProblem(instance_file, while_iterations);
    //   solve
    auto solver = getSolver(solver_name, &P, inheritanceDepth, false, argc, argv_copy);
    solver->solve();
    if (solver->succeed())
    {
      if (!solver->getSolution().validate(&P)){
        failures++;
      } else {
        succeses++;

        costs.push_back(solver->getSolution().getSOC());
        lower_bound_costs.push_back(solver->getLowerBoundSOC());

        steps.push_back(solver->getSolution().getMakespan());
        lower_bound_steps.push_back(solver->getLowerBoundMakespan());
        elapsed_time.push_back(int(solver->getSolverElapsedTime()));
      }
    }
    else
      failures++;
  }
  std::cout << std::endl;
  
  std::ofstream log;
  log.open(output_file, std::ios::out);
  log << "num_agents=" << number_of_agents << std::endl;
  log << "min_radius=" << random_radiuses[0] << std::endl;
  log << "max_radius=" << random_radiuses[1] << std::endl;
  log << "iterations=" << number_of_trials << std::endl;
  log << "succeses=" << succeses << std::endl;
  log << "failures=" << failures << std::endl;
 
  log << "costs=";
  for (auto iterator = costs.begin(); iterator != costs.end() - 1; iterator++) {
    log << (*iterator) << ",";
  }
  log << *(costs.end() - 1) << std::endl;

  log << "lb_costs=";
  for (auto iterator = lower_bound_costs.begin(); iterator != lower_bound_costs.end() - 1; iterator++) {
    log << (*iterator) << ",";
  }
  log << *(lower_bound_costs.end() - 1) << std::endl;
   

  log << "steps=";
  for (auto iterator = steps.begin(); iterator != steps.end() - 1; iterator++) {
    log << (*iterator) << ",";
  }
  log << *(steps.end() - 1) << std::endl;

  log << "lb_steps=";
  for (auto iterator = lower_bound_steps.begin(); iterator != lower_bound_steps.end() - 1; iterator++) {
    log << (*iterator) << ",";
  }
  log << *(steps.end() - 1) << std::endl;

  log << "time=";
  for (auto iterator = elapsed_time.begin(); iterator != elapsed_time.end() - 1; iterator++) {
    log << (*iterator) << ",";
  }
  log << *(elapsed_time.end() - 1) << std::endl;

  log.close();

  return 0;
}