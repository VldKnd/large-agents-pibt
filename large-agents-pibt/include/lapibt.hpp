#include "mapf_solver.hpp"
#include <unordered_set>

class LAPIBT : public LargeAgentsMAPFSolver
{
public:
    static const std::string SOLVER_NAME;

private:
    struct Agent
    {
        int id;                                        // Agents id
        std::deque<Node*> path;                        // next locations starts with  v_t0, and v_t1, v_t2
        Node * goal;                                   // goal
        int elapsed;                                   // eta
        double init_d;                                 // initial distance
        float tie_breaker;                              // epsilon, tie-breaker
        float radius;                                   // Size (radius) of an agent
    };

    std::unordered_set<Agent*> setOfAgentsInConflict;
    int inheritanceDepth;

    // option
    bool disable_dist_init = false;

    void mainLAPIBT(Agent *agent, const std::vector<Agent *> &allAgents);

    void run() override;

public:
    explicit LAPIBT(LargeAgentsMapfProblem *P);
    explicit LAPIBT(LargeAgentsMapfProblem *P, int iheritanceDepth);

    Nodes getNodesToAvoidInheritanceConflict(const Agent *child_agent, const Agent *parent_agent);
    
    bool collisionConflict(Agent *agent, const std::vector<Agent *> &allAgents);
    bool inheritanceConflict(Agent *agent, const std::vector<Agent *> &allAgents);

    int solveInheritanceConflict(Agent *child_agent, Agent *parent_agent, const std::vector<Agent *> &allAgents);
    bool collisionConflictWithAgentsInConflict(Agent *child_agent, Agent* parent_agent, const std::vector<Agent *> &allAgents);
};
