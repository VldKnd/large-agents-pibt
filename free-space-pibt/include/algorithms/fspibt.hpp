#pragma once
#include "mapf_solver.hpp"
#include <unordered_set>

struct State
{
    int id;
    int moves_made;
    Node *last_move;
};

class FSPIBT : public FreeSpaceMAPFSolver
{
public:
    static const std::string SOLVER_NAME;

private:
    // FS PIBT agent
    struct Agent
    {
        int id;
        Node *v_now;       // current location
        Node *v_next;      // next location
        Node *g;           // goal
        int elapsed;       // eta
        double init_d;     // initial distance
        float tie_breaker; // epsilon, tie-breaker
        float r;           // Size (radius) of an agent
    };
    using Agents = std::vector<Agent *>;
    using ConfigsT = std::vector<std::vector<Node *>>;

    ConfigsT occupation;
    std::unordered_set<int> activeConflicts;
    std::vector<int> stepTable;

    // option
    bool disable_dist_init = false;

    bool funcFSPIBT(Agent *ai, Agent *aj, const Agents &A);                                 // Change Parameters
    void funcFSPIBT(Agent *ai, const Agents &A, const std::unordered_set<Node *> &visited); // Change Parameters

    void run() override;

public:
    explicit FSPIBT(FreeSpaceMapfProblem *P);

    Nodes getNodes(const Agent &ai, const Agent &aj);
    bool collisionConflict(const Agents &A, Agent *x_pt);
    bool inheritanceConflict(const Agents &A, Agent *x_pt);
    void reverseToState(const std::vector<State> &states);
    bool collisionConflict(const Agents &A, Agent *a_i, Agent *a_j);
};
