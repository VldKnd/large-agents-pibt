#include <algorithm>
#include <unordered_set>

#include "../include/osfspibt.hpp"

const std::string OSFSPIBT::SOLVER_NAME = "One Step Free Space Priority Inheritance With Backtracking (OSFSPIBT)";

OSFSPIBT::OSFSPIBT(FreeSpaceMapfProblem *problem)
    : FreeSpaceMAPFSolver(problem),
      activeConflicts()
{
    solver_name = OSFSPIBT::SOLVER_NAME;
}

void OSFSPIBT::run()
{
    auto compare = [](Agent *a, const Agent *b)
    {
        if (a->elapsed != b->elapsed)
            return a->elapsed > b->elapsed;
        if (a->init_d != b->init_d)
            return a->init_d > b->init_d;
        return a->tie_breaker > b->tie_breaker;
    };

    Agents A;

    for (int i = 0; i < P->getNum(); ++i)
    {
        Node *g = P->getGoal(i);
        double d = disable_dist_init ? 0 : pathDist(i, P->getStart(i));

        if (d == P->getMaxTimestep())
        {
            std::cout << "Goal for agent " << i << " is unreachable";
            solved = false;
            return;
        }

        auto *agent = new Agent{
            i,                        // id
            P->getStart(i),           // current location
            nullptr,                  // next location
            g,                        // goal
            0,                        // eta
            d,                        // initial distance
            getRandomFloat(0, 1, MT), // epsilon, tie-breaker
            P->getRadius(i)           // Size (radius) of an agent
        };
        A.push_back(agent);
    }

    solution.add(P->getConfigStart());

    int timestep = 0;
    while (true)
    {
        ++timestep;
        info(" ", "elapsed:", getSolverElapsedTime(), ", timestep:", timestep);

        std::sort(A.begin(), A.end(), compare);

        for (auto agent : A)
        {
            if (!agent->v_next)
                funcOSFSPIBT(agent, A);
        }

        bool check_goal_cond = true;

        Config config(P->getNum(), nullptr);

        for (auto agent : A)
        {
            config[agent->id] = agent->v_next;
            agent->v_now = agent->v_next;
            agent->v_next = nullptr;
            bool elapsed = (agent->v_now == agent->g);
            agent->elapsed = elapsed ? 0 : agent->elapsed + 1;
            check_goal_cond &= elapsed;
        }

        solution.add(config);

        if (check_goal_cond)
        {
            solved = true;
            break;
        }

        if (timestep >= max_timestep)
        {
            std::cout << "Too much steps!\n";
            break;
        }
    }

    for (auto a : A)
        delete a;
}

void OSFSPIBT::funcOSFSPIBT(Agent *agent, const Agents &A)
{
    if (agent->g == agent->v_now)
    {
        agent->v_next = agent->v_now;
        return;
    }

    auto compareToReachGoal = [&, agent](Node *const v, Node *const u)
    {
        auto d_v = pathDist(agent->id, v);
        auto d_u = pathDist(agent->id, u);
        return d_v < d_u;
    };

    Nodes C = agent->v_now->neighbor;
    std::sort(C.begin(), C.end(), compareToReachGoal);

    for (auto v_next : C)
    {
        if (pathDist(agent->id, v_next) == max_timestep + 1)
            continue;

        agent->v_next = v_next;

        if (collisionConflict(agent, A))
        {
            agent->v_next = nullptr;
            continue;
        }

        if (inheritanceConflict(agent, A))
        {
            agent->v_next = nullptr;
            continue;
        }

        return;
    }

    agent->v_next = agent->v_now;
}

bool OSFSPIBT::collisionConflict(Agent *agent, const Agents &A)
{
    for (auto other_agent : A)
    {
        if (
            other_agent->id != agent->id &&
            other_agent->v_next &&
            other_agent->v_next->euclideanDist(agent->v_next) < agent->r + other_agent->r)
            return true;
    }
    return false;
}

bool OSFSPIBT::inheritanceConflict(Agent *agent, const Agents &A)
{
    for (auto other_agent : A)
    {
        if (
            other_agent->id != agent->id &&
            !other_agent->v_next &&
            other_agent->v_now->euclideanDist(agent->v_next) < agent->r + other_agent->r &&
            !funcOSFSPIBT(other_agent, agent, A))
            return true;
    }
    return false;
}

bool OSFSPIBT::funcOSFSPIBT(Agent *child_agent, Agent *parent_agent, const Agents &A)
{
    auto compareToEvadeInheritanceConflict = [&, child_agent](Node *node_lhs, Node *node_rhs)
    {
        auto distance_node_lhs = child_agent->g->euclideanDist(node_lhs);
        auto distance_node_rhs = child_agent->g->euclideanDist(node_rhs);
        return distance_node_lhs < distance_node_rhs;
    };

    Nodes C = getNodesToAvoidInheritanceConflict(child_agent, parent_agent);
    std::sort(C.begin(), C.end(), compareToEvadeInheritanceConflict);

    for (Node *node : C)
    {

        if (pathDist(child_agent->id, node) == max_timestep + 1)
        {
            continue;
        }

        auto compareLocal = [node](const Node *a, const Node *b)
        {
            float d_a = node->euclideanDist(a);
            float d_b = node->euclideanDist(b);
            return d_a < d_b;
        };

        Nodes neighbors = child_agent->v_now->neighbor;
        std::sort(neighbors.begin(), neighbors.end(), compareLocal);

        auto find = std::find_if(neighbors.begin(), neighbors.end(),
                                 [&, child_agent, parent_agent](Node *node)
                                 {
                                     return parent_agent->v_now->euclideanDist(node) > child_agent->r + parent_agent->r &&
                                            checkIfNodeExistInRadiusOnGrid(G, node->pos.x, node->pos.y, child_agent->r);
                                 });

        if (find == neighbors.end())
            continue;

        child_agent->v_next = *find;

        if (collisionConflict(A, child_agent, parent_agent))
        {
            child_agent->v_next = nullptr;
            continue;
        }

        if (inheritanceConflict(child_agent, A))
        {
            child_agent->v_next = nullptr;
            continue;
        }

        parent_agent->v_next = parent_agent->v_now;
        return true;
    }

    return false;
}

Nodes OSFSPIBT::getNodesToAvoidInheritanceConflict(const Agent *a_child, const Agent *a_parent)
{
    int x = (a_parent->v_next->pos).x;
    int y = (a_parent->v_next->pos).y;
    double r = a_child->r + a_parent->r;
    int dx = std::ceil(r);
    int dy = 0;
    Nodes nodes;

    do
    {
        if (G->existNode(x + dx, y + dy))
            nodes.push_back(G->getNode(x + dx, y + dy));
        if (G->existNode(x - dy, y + dx))
            nodes.push_back(G->getNode(x - dy, y + dx));
        if (G->existNode(x - dx, y - dy))
            nodes.push_back(G->getNode(x - dx, y - dy));
        if (G->existNode(x + dy, y - dx))
            nodes.push_back(G->getNode(x + dy, y - dx));
        if ((dx - 1) * (dx - 1) + dy * dy > r * r)
            --dx;
        else
            ++dy;
    } while (dx != 0);

    return nodes;
}

bool OSFSPIBT::collisionConflict(const Agents &A, Agent *a_child, Agent *a_parent)
{
    for (auto agent : A)
    {
        if (agent->id != a_child->id &&
            agent->id != a_parent->id &&
            agent->v_next &&
            agent->v_next->euclideanDist(a_child->v_next) < a_child->r + agent->r)
        {
            return true;
        }
    }
    return false;
}