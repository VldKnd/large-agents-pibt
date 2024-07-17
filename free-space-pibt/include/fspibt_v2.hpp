#include "mapf_solver.hpp"
#include <unordered_set>

class FSPIBTV2 : public FreeSpaceMAPFSolver
{
public:
    static const std::string SOLVER_NAME;

private:
    struct Agent
    {
        int id;
        Node *v_now;            // current location
        std::deque<Node*> *planned_path;    // next locations
        Node *goal;                // goal
        int elapsed;            // eta
        double init_d;          // initial distance
        float tie_breaker;       // epsilon, tie-breaker
        float radius;                 // Size (radius) of an agent
    };
    // using Agents = std::vector<Agent *>;
    // using ConfigsT = std::vector<std::vector<Node *>>;

    std::unordered_set<Agent> setOfAgentsInConflict;
    
    // option
    bool disable_dist_init = false;

    void funcFSPIBTV2(Agent *agent, const std::vector<Agent *> &allAgents);
    
    bool funcFSPIBTV2(Agent *child_agent, Agent *parent_agent, const std::vector<Agent *> &allAgents);

    void run() override;

public:
    explicit FSPIBTV2(FreeSpaceMapfProblem *P);

    Nodes getNodesToAvoidInheritanceConflict(const Agent *child_agent, const Agent *parent_agent);
    
    bool collisionConflict(Agent *agent, const std::vector<Agent *> &allAgents);
    bool inheritanceConflict(Agent *agent, const std::vector<Agent *> &allAgents);

    bool collisionConflict(Agent *child_agent, Agent *parent_agent, const std::vector<Agent *> &allAgents);
};
