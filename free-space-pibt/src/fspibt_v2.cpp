#include <algorithm>
#include <unordered_set>

#include "../include/fspibt_v2.hpp"

const std::string FSPIBTV2::SOLVER_NAME = "Free Space Priority Inheritance With Backtracking V2 (FSPIBTV2)";

FSPIBTV2::FSPIBTV2(FreeSpaceMapfProblem *problem)
    : FreeSpaceMAPFSolver(problem),
      setOfAgentsInConflict()
{
    solver_name = FSPIBTV2::SOLVER_NAME;
}

void FSPIBTV2::run()
{
    auto compareAllAgents = [](Agent *agent_lhs, const Agent *agent_rhs)
    {
        if (agent_lhs->elapsed != agent_rhs->elapsed)
            return agent_lhs->elapsed > agent_rhs->elapsed;
        if (agent_lhs->init_d != agent_rhs->init_d)
            return agent_lhs->init_d > agent_rhs->init_d;
        return agent_lhs->tie_breaker > agent_rhs->tie_breaker;
    };

    std::vector<Agent *> allAgents;

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
            {},                       // List of next locations
            g,                        // goal
            0,                        // eta
            d,                        // initial distance
            getRandomFloat(0, 1, MT), // epsilon, tie-breaker
            P->getRadius(i)           // Size (radius) of an agent
        };
        allAgents.push_back(agent);
    }

    solution.add(P->getConfigStart());

    int timestep = 0;
    while (true)
    {
        ++timestep;
        info(" ", "elapsed:", getSolverElapsedTime(), ", timestep:", timestep);

        std::sort(allAgents.begin(), allAgents.end(), compareAllAgents);

        for (auto agent : allAgents)
        {
            if (agent->planned_path->empty())
                funcFSPIBTV2(agent, allAgents);
        }

        bool check_goal_condition = true;

        std::vector<Node *> configuration(P->getNum(), nullptr);

        for (auto agent : allAgents)
        {
            Node *agents_next_node = agent->planned_path->front();
            bool elapsed = (agents_next_node == agent->goal);

            configuration[agent->id] = agents_next_node;
            agent->v_now = agents_next_node;
            agent->planned_path->pop_front();

            agent->elapsed = elapsed ? 0 : agent->elapsed + 1;
            check_goal_condition &= elapsed;
        }

        solution.add(configuration);

        if (check_goal_condition)
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

    for (auto a : allAgents)
        delete a;
}

void FSPIBTV2::funcFSPIBTV2(Agent *agent, const std::vector<Agent *> &A)
{
    if (agent->goal == agent->v_now)
    {
        agent->planned_path->push_back(agent->v_now);
        return;
    }

    auto closestNodeToTheGoal = [&, agent](Node *const node_lhs, Node *const node_rhs)
    {
        auto distance_node_lhs = pathDist(agent->id, node_lhs);
        auto distance_node_rhs = pathDist(agent->id, node_rhs);
        return distance_node_lhs < distance_node_rhs;
    };

    std::vector<Node *> current_nodes_neighbours = agent->v_now->neighbor;
    std::sort(current_nodes_neighbours.begin(), current_nodes_neighbours.end(), closestNodeToTheGoal);

    for (auto perpective_next_node : current_nodes_neighbours)
    {
        if (pathDist(agent->id, perpective_next_node) == max_timestep + 1)
            continue;

        agent->planned_path->push_back(perpective_next_node);

        if (collisionConflict(agent, A))
        {
            agent->planned_path->pop_back();
            continue;
        }


        // ### CONTINUE HERE! When does the inheritance of confilct should occure? Right before the agent meets other in the future? Or for all the timesteps?
        if (inheritanceConflict(agent, A))
        {
            agent->planned_path->pop_back();
            continue;
        }

        return;
    }

    agent->planned_path->push_front(agent->v_now);
}

bool FSPIBTV2::collisionConflict(Agent *agent, const std::vector<Agent *> &allAgents)
{
    for (auto other_agent : allAgents)
    {
        if (
            other_agent->id != agent->id &&
            other_agent->planned_path->size() >= agent->planned_path->size())
        {
            for (auto node_in_planned_path = (*(other_agent->planned_path)).begin() + (agent->planned_path->size() - 1); node_in_planned_path != (*(other_agent->planned_path)).end();)
            {
                if ((*node_in_planned_path)->euclideanDist(agent->planned_path->front()) < agent->radius + other_agent->radius)
                    return true;
            }
        }
    }
    return false;
}


// ### CONTINUE HERE!
bool FSPIBTV2::inheritanceConflict(Agent *agent, const std::vector<Agent *> &allAgents)
{
    for (auto other_agent : allAgents)
    {
        if (
            other_agent->id != agent->id &&
            !other_agent->v_next &&
            other_agent->v_now->euclideanDist(agent->v_next) < agent->r + other_agent->r &&
            !funcFSPIBTV2(other_agent, agent, A))
            return true;
    }
    return false;
}

bool FSPIBTV2::funcFSPIBTV2(Agent *child_agent, Agent *parent_agent, const Agents &A)
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

Nodes FSPIBTV2::getNodesToAvoidInheritanceConflict(const Agent *a_child, const Agent *a_parent)
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

bool FSPIBTV2::collisionConflict(const Agents &A, Agent *a_child, Agent *a_parent)
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