#include <getopt.h>

#include <default_params.hpp>
#include <iostream>
#include <mapf_problem.hpp>
#include <mapf_solver.hpp>
#include <getopt.h>
#include <random>
#include <vector>


int main(int argc, char *argv[])
{
  std::string instance_file;
  std::string output_file = DEFAULT_OUTPUT_FILE;
  std::string solver_name;
  bool verbose = false;
  char *argv_copy[argc + 1];

  for (int i = 0; i < argc; ++i)
    argv_copy[i] = argv[i];

  struct option longopts[] = {
      {"instance", required_argument, 0, 'i'},
      {"output", required_argument, 0, 'o'},
      {"solver", required_argument, 0, 's'},
      {"verbose", no_argument, 0, 'v'},
      {"time-limit", required_argument, 0, 'T'},
      {"log-short", no_argument, 0, 'L'},
      {"make-scen", no_argument, 0, 'P'},
      {0, 0, 0, 0},
  };

  bool make_scen = false;
  bool log_short = false;
  int max_comp_time = -1;

  // command line args
  int opt, longindex;

  opterr = 0; // ignore getopt error

  while ((opt = getopt_long(argc, argv, "i:o:s:vhPT:L", longopts,
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
    case 'v':
      verbose = true;
      break;
    case 'P':
      make_scen = true;
      break;
    case 'L':
      log_short = true;
      break;
    case 'T':
      max_comp_time = std::atoi(optarg);
      break;
    default:
      break;
    }
  }

  if (instance_file.length() == 0)
  {
    std::cout << "specify instance file using -i [INSTANCE-FILE], e.g.,"
              << std::endl;
    std::cout << "> ./mapf -i ../instance/sample.txt" << std::endl;
    return 0;
  }

  //  set problem
  auto P = FreeSpaceMapfProblem(instance_file);

  // set max computation time (otherwise, use param in instance_file)
  if (max_comp_time != -1)
    P.setMaxCompTime(max_comp_time);

  // create scenario
  if (make_scen)
  {
    P.makeScenFile(output_file);
    return 0;
  }

  //   solve
  auto solver = getSolver(solver_name, &P, verbose, argc, argv_copy);
  solver->setLogShort(log_short);
  solver->solve();

  if (solver->succeed() && !solver->getSolution().validate(&P))
  {
    solver->makeLog(output_file);
    std::cout << "error@mapf: invalid results" << std::endl;
    solver->makeLog(output_file);
    return 0;
  }

  if (solver->succeed())
  {
    std::cout << "Solved\n";
    solver->printResult();
    solver->makeLog(output_file);
  }
  else
  {
    std::cout << "Failed to converge\n";
    //        P.makeScenFile(output_file);
    solver->makeLog(output_file);
  }
  //     output result
  if (verbose)
  {
    std::cout << "save result as " << output_file << std::endl;
  }

  return 0;
}