#include <iostream>
#include "mapf_solver.hpp"
#include "mapf_problem.hpp"
#include "utils.hpp"
#include "fspibt.hpp"
#include <fstream>
#include <iomanip>
#include <cmath>

MinimumSolver::MinimumSolver(MapfProblem* _P)
    : solver_name(""),
      G(_P->getG()),
      MT(_P->getMT()),
      max_timestep(_P->getMaxTimestep()),
      max_comp_time(_P->getMaxCompTime()),
      solved(false),
      comp_time(0),
      verbose(false),
      log_short(false) {}

void MinimumSolver::solve() {start();exec();end();}

void MinimumSolver::start() { t_start = Time::now(); }

void MinimumSolver::end() { comp_time = getSolverElapsedTime(); }

// -------------------------------
// utilities for time
// -------------------------------

int MinimumSolver::getSolverElapsedTime() const
{
  return int(getElapsedTime(t_start));
}

int MinimumSolver::getRemainedTime() const
{
  return std::max(0, max_comp_time - getSolverElapsedTime());
}

bool MinimumSolver::overCompTime() const
{
  return getSolverElapsedTime() >= max_comp_time;
}
// -------------------------------
// utilities for debug
// -------------------------------
void MinimumSolver::info() const
{
  if (verbose) std::cout << std::endl;
}

void MinimumSolver::halt(const std::string& msg) const
{
  std::cout << "error@" << solver_name << ": " << msg << std::endl;
  this->~MinimumSolver();
  std::exit(1);
}

void MinimumSolver::warn(const std::string& msg) const
{
  std::cout << "warn@ " << solver_name << ": " << msg << std::endl;
}

// -------------------------------
// print
// -------------------------------
void MinimumSolver::printHelpWithoutOption(const std::string& solver_name)
{
  std::cout << solver_name << "\n"
            << "  (no option)" << std::endl;
}

// -----------------------------------------------
// base class for Free Space Agent
// -----------------------------------------------
FreeSpaceMAPFSolver::FreeSpaceMAPFSolver(FreeSpaceMapfProblem* problem)
        : MinimumSolver(problem),
          P(problem),
          LB_soc(0),
          LB_makespan(0),
          distance_table(problem->getNum(),
                         std::vector<int>(G->getNodesSize(), max_timestep + 1)),
          distance_table_p(nullptr) {}

void FreeSpaceMAPFSolver::exec()
{
    // create distance table
    if (distance_table_p == nullptr) {
        info("  pre-processing, create distance table by BFS");
        createDistanceTable();
        preprocessing_comp_time = getSolverElapsedTime();
        info("  done, elapsed: ", preprocessing_comp_time);
    }

    run();
}

void FreeSpaceMAPFSolver::createDistanceTable(){

    Grid* grid = reinterpret_cast<Grid*>(G);

    for (int i = 0; i < P->getNum(); ++i) {
        // breadth first search
        std::queue<Node*> OPEN;
        Node* n = P->getGoal(i);
//        int g_x = n->id % grid->getWidth();
//        int g_y = std::floor( n->id / grid->getWidth());

        float r = P->getRadius(i);
        OPEN.push(n);
        distance_table[i][n->id] = 0;
        while (!OPEN.empty()) {

            n = OPEN.front();
            OPEN.pop();
            const int d_n = distance_table[i][n->id];
            for (auto m : n->neighbor) {
                int x = m->id % grid->getWidth();
                int y = std::floor( m->id / grid->getWidth());
                if (checkIfNodeExistInRadiusOnGrid(G, x, y, r)) {
                    const int d_m = distance_table[i][m->id];

                    if (d_n + 1 >= d_m) continue;
                    distance_table[i][m->id] = d_n + 1;
                    OPEN.push(m);
                }
            }
        }
    }

    distance_table_p = &distance_table;
}

void FreeSpaceMAPFSolver::printResult()
{
    std::cout << "solved=" << solved << ", solver=" << std::right << std::setw(8)
              << solver_name << ", comp_time(ms)=" << std::right << std::setw(8)
              << getCompTime() << ", soc=" << std::right << std::setw(6)
              << solution.getSOC() << " (LB=" << std::right << std::setw(6)
              << getLowerBoundSOC() << ")"
              << ", makespan=" << std::right << std::setw(4)
              << solution.getMakespan() << " (LB=" << std::right << std::setw(6)
              << getLowerBoundMakespan() << ")" << std::endl;
}


int FreeSpaceMAPFSolver::getLowerBoundMakespan()
{
    if (LB_makespan == 0) computeLowerBounds();
    return LB_makespan;
}

int FreeSpaceMAPFSolver::getLowerBoundSOC()
{
    if (LB_soc == 0) computeLowerBounds();
    return LB_soc;
}

void FreeSpaceMAPFSolver::computeLowerBounds()
{
    LB_soc = 0;
    LB_makespan = 0;

    for (int i = 0; i < P->getNum(); ++i) {
        int d = pathDist(i);
        LB_soc += d;
        if (d > LB_makespan) LB_makespan = d;
    }
}

void FreeSpaceMAPFSolver::makeLogBasicInfo(std::ofstream& log)
{

    int size = int(P->getRadiuses().size());
    Grid* grid = reinterpret_cast<Grid*>(P->getG());
    log << "instance=" << P->getInstanceFileName() << "\n";
    log << "agents=" << P->getNum() << "\n";
    log << "radiuses=";
    for (int i = 0; i < size-1; i++) {
        log << P->getRadius(i) << ", ";
    }
    log << P->getRadius(size-1) << "\n";
    log << "map_file=" << grid->getMapFileName() << "\n";
    log << "solver=" << solver_name << "\n";
    log << "solved=" << solved << "\n";
    log << "soc=" << solution.getSOC() << "\n";
    log << "lb_soc=" << getLowerBoundSOC() << "\n";
    log << "makespan=" << solution.getMakespan() << "\n";
    log << "lb_makespan=" << getLowerBoundMakespan() << "\n";
    log << "comp_time=" << getCompTime() << "\n";
    log << "preprocessing_comp_time=" << preprocessing_comp_time << "\n";
}

void FreeSpaceMAPFSolver::makeLogSolution(std::ofstream& log)
{
    if (log_short) return;
    log << "starts=";
    for (int i = 0; i < P->getNum(); ++i) {
        Node* v = P->getStart(i);
        log << "(" << v->pos.x << "," << v->pos.y << "),";
    }
    log << "\ngoals=";
    for (int i = 0; i < P->getNum(); ++i) {
        Node* v = P->getGoal(i);
        log << "(" << v->pos.x << "," << v->pos.y << "),";
    }
    log << "\n";
    log << "solution=\n";
    for (int t = 0; t <= solution.getMakespan(); ++t) {
        log << t << ":";
        auto c = solution.get(t);
        for (auto v : c) {
            log << "(" << v->pos.x << "," << v->pos.y << "),";
        }
        log << "\n";
    }
}

int FreeSpaceMAPFSolver::pathDist(const int i, Node* const s) const
{
    if (distance_table_p != nullptr) {
        return distance_table_p->at(i)[s->id];
    }
    return distance_table[i][s->id];
}

int FreeSpaceMAPFSolver::pathDist(const int i) const
{
    return pathDist(i, P->getStart(i));
}

void FreeSpaceMAPFSolver::makeLog(const std::string& logfile)
{
    std::ofstream log;
    log.open(logfile, std::ios::out);
    makeLogBasicInfo(log);
    makeLogSolution(log);
    log.close();
}

FreeSpaceMAPFSolver::~FreeSpaceMAPFSolver() = default;

void FreeSpaceMAPFSolver::clearSizedPathTable(const PathsWithRadius& paths)
{
    const int makespan = paths.getMakespan();
    const int num_agents = paths.size();
    for (int i = 0; i < num_agents; ++i) {
        if (paths.empty(i)) continue;
        auto p = paths.get(i);
        for (int t = 0; t <= makespan; ++t) PATH_TABLE[t][p[t].node->id] = NIL;
    }
}

void FreeSpaceMAPFSolver::updateSizedPathTable(const PathsWithRadius& paths, const int id)
{
    const int makespan = paths.getMakespan();
    const int num_agents = paths.size();
    const int nodes_size = G->getNodesSize();
    // extend PATH_TABLE
    while ((int)PATH_TABLE.size() < makespan + 1)
        PATH_TABLE.push_back(std::vector<int>(nodes_size, NIL));
    // update locations
    for (int i = 0; i < num_agents; ++i) {
        if (i == id || paths.empty(i)) continue;
        auto p = paths.get(i);
        for (int t = 0; t <= makespan; ++t) PATH_TABLE[t][p[t].node->id] = i;
    }
}

void FreeSpaceMAPFSolver::updateSizedPathTableWithoutClear(const int id, const PathWithRadius& path,
                                              const PathsWithRadius& paths)
{
    if (path.empty()) return;

    const int makespan = PATH_TABLE.size() - 1;
    const int nodes_size = G->getNodesSize();
    const int p_makespan = path.size() - 1;

    // extend PATH_TABLE
    if (p_makespan > makespan) {
        while ((int)PATH_TABLE.size() < p_makespan + 1)
            PATH_TABLE.push_back(std::vector<int>(nodes_size, NIL));
        for (int i = 0; i < P->getNum(); ++i) {
            if (paths.empty(i)) continue;
            auto v_id = paths.get(i, makespan).node->id;
            for (int t = makespan + 1; t <= p_makespan; ++t) PATH_TABLE[t][v_id] = i;
        }
    }

    // register new path
    for (int t = 0; t <= p_makespan; ++t) PATH_TABLE[t][path[t].node->id] = id;
    if (makespan > p_makespan) {
        auto v_id = path[p_makespan].node->id;
        for (int t = p_makespan + 1; t <= makespan; ++t) PATH_TABLE[t][v_id] = id;
    }
}

Plan FreeSpaceMAPFSolver::sizedPathsToPlan(const PathsWithRadius& paths)
{
    Plan plan;
    if (paths.empty()) return plan;
    int makespan = paths.getMakespan();
    int num_agents = paths.size();
    for (int t = 0; t <= makespan; ++t) {
        Config c;
        for (int i = 0; i < num_agents; ++i) {
            c.push_back(paths.get(i, t).node);
        }
        plan.add(c);
    }
    return plan;
}

std::unique_ptr<FreeSpaceMAPFSolver> getSolver(const std::string &solver_name,
                                               FreeSpaceMapfProblem *P, bool verbose, int argc,
                                               char *argv[])
{
    std::unique_ptr<FreeSpaceMAPFSolver> solver;
    if (solver_name == "FSPIBT")
    {
        solver = std::make_unique<FSPIBT>(P);
    }
    else
    {
        std::cout << "warn@mapf: "
                  << "unknown solver name, " << solver_name
                  << ", availible options are ['FSPIBT']."
                  << std::endl;
        exit(1);
    }

    solver->setParams(argc, argv);
    solver->setVerbose(verbose);
    return solver;
}