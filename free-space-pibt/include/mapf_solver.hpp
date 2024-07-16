#pragma once
#include <getopt.h>

#include <chrono>
#include <functional>
#include <memory>
#include <queue>
#include <unordered_map>
#include <iostream>

class MinimumSolver
{
protected:
    std::string solver_name; // solver name
    Graph *const G;          // graph
    std::mt19937 *const MT;  // seed for randomness
    const int max_timestep;  // maximum makespan
    const int max_comp_time; // time limit for computation, ms
    Plan solution;           // solution
    bool solved;             // success -> true, failed -> false (default)

private:
    int comp_time;            // computation time
    Time::time_point t_start; // when to start solving

protected:
    bool verbose;   // true -> print additional info
    bool log_short; // true -> cannot visualize the result, default: false

    // -------------------------------
    // utilities for time
public:
    int getRemainedTime() const; // get remained time
    bool overCompTime() const;   // check time limit

    // -------------------------------
    // utilities for debug
protected:
    // print debug info (only when verbose=true)
    void info() const;
    template <class Head, class... Tail>
    void info(Head &&head, Tail &&...tail) const
    {
        if (!verbose)
            return;
        std::cout << head << " ";
        info(std::forward<Tail>(tail)...);
    }
    void halt(const std::string &msg) const; // halt program
    void warn(const std::string &msg) const; // just printing msg

    // -------------------------------
    // utilities for solver options
public:
    virtual void setParams(int argc, char *argv[]) {};
    void setVerbose(bool _verbose) { verbose = _verbose; }
    void setLogShort(bool _log_short) { log_short = _log_short; }

    // -------------------------------
    // print help
protected:
    static void printHelpWithoutOption(const std::string &solver_name);

    // -------------------------------
    // utilities for computing path

    // space-time A*
    struct AstarNode
    {
        Node *v;          // location
        int g;            // time
        int f;            // f-value
        AstarNode *p;     // parent
        std::string name; // name
        AstarNode(Node *_v, int _g, int _f, AstarNode *_p)
            : v(_v), g(_g), f(_f), p(_p), name(getName(_v, _g)) {};

        static std::string getName(Node *_v, int _g)
        {
            return std::to_string(_v->id) + "-" + std::to_string(_g);
        };
    };
    using CompareAstarNode = std::function<bool(AstarNode *, AstarNode *)>;
    using CheckAstarFin = std::function<bool(AstarNode *)>;
    using CheckInvalidAstarNode = std::function<bool(AstarNode *)>;
    using AstarHeuristics = std::function<int(AstarNode *)>;
    using AstarNodes = std::vector<AstarNode *>;
    /*
     * Template of Space-Time A*.
     * See the following reference.
     *
     * Cooperative Pathﬁnding.
     * D. Silver.
     * AI Game Programming Wisdom 3, pages 99–111, 2006.
     */
    //  static Path getPathBySpaceTimeAstar(
    //      Node* const s,                 // start
    //      Node* const g,                 // goal
    //      AstarHeuristics& fValue,       // func: f-value
    //      CompareAstarNode& compare,     // func: compare two nodes
    //      CheckAstarFin& checkAstarFin,  // func: check goal
    //      CheckInvalidAstarNode&
    //          checkInvalidAstarNode,  // func: check invalid nodes
    //      const int time_limit = -1   // time limit
    //  );
    //
    // typical functions
    static CompareAstarNode compareAstarNodeBasic;

public:
    virtual void solve(); // call start -> run -> end
protected:
    void start();
    void end();
    virtual void exec() {}; // main

public:
    MinimumSolver(Problem *_P);
    virtual ~MinimumSolver() {};

    // getter
    Plan getSolution() const { return solution; };
    bool succeed() const { return solved; };
    std::string getSolverName() const { return solver_name; };
    int getMaxTimestep() const { return max_timestep; };
    int getCompTime() const { return comp_time; }
    int getSolverElapsedTime() const; // get elapsed time from start
};

// -----------------------------------------------
// base class for Free Space Agent
// -----------------------------------------------
class FSMAPF_Solver : public MinimumSolver
{
public:
    int getLowerBoundSOC();
    int getLowerBoundMakespan();
    int getLowerBoundSOC();
    int getLowerBoundMakespan();
    void makeLog(const std::string &logfile = "./result.txt");
    void printResult();
    int pathDist(int i, Node *s) const;
    int pathDist(int i) const;
    void createDistanceTable();
    explicit FSMAPF_Solver(FSMAPF_Instance *P);
    ~FSMAPF_Solver() override;

    // used for checking conflicts
    void updateSizedPathTable(const SizedPaths &paths, const int id);
    void clearSizedPathTable(const SizedPaths &paths);
    void updateSizedPathTableWithoutClear(const int id, const SizedPath &p,
                                          const SizedPaths &paths);

    static SizedPaths planToSizedPaths(const Plan &plan);  // plan -> paths
    static Plan sizedPathsToPlan(const SizedPaths &paths); // paths -> plan

protected:
    FSMAPF_Instance *const P;
    using DistanceTable = std::vector<std::vector<int>>;
    DistanceTable distance_table;
    DistanceTable *distance_table_p;
    int preprocessing_comp_time;
    virtual void run() {}
    virtual void makeLogBasicInfo(std::ofstream &log);
    virtual void makeLogSolution(std::ofstream &log);
    static constexpr int NIL = -1;
    std::vector<std::vector<int>> PATH_TABLE;

private:
    int LB_soc;
    int LB_makespan;
    void exec() override;
    void computeLowerBounds();
};

std::unique_ptr<FSMAPF_Solver> getSolver(const std::string &solver_name,
                                         FSMAPF_Instance *P, bool verbose, int argc,
                                         char *argv[]);
