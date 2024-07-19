#include "mapf_solver.hpp"
#include <unordered_set>

class OSFSPIBT : public FreeSpaceMAPFSolver
{
public:
    static const std::string SOLVER_NAME;

private:
    struct Agent
    {
        int id;
        Node *v_now;       // current location
        Node *v_next;      // next location
        Node *g;           // goal
        int elapsed;       // eta
        double init_d;     // initial distance
        float tie_breaker;  // epsilon, tie-breaker
        float r;            // Size (radius) of an agent
    };
    using Agents = std::vector<Agent *>;
    using ConfigsT = std::vector<std::vector<Node *>>;

    std::unordered_set<int> activeConflicts;
    std::vector<int> stepTable;

    // option
    bool disable_dist_init = false;

    bool funcOSFSPIBT(Agent *child_agent, Agent *parent_agent, const Agents &A);
    void funcOSFSPIBT(Agent *agent, const Agents &A);

    void run() override;

public:
    explicit OSFSPIBT(FreeSpaceMapfProblem *P);

    Nodes getNodesToAvoidInheritanceConflict(const Agent *a_child, const Agent *a_parent);
    bool collisionConflict(Agent *agent, const Agents &A);
    bool inheritanceConflict(Agent *agent, const Agents &A);
    bool collisionConflict(const Agents &A, Agent *a_i, Agent *a_j);
};
