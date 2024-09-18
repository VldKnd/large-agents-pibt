#include <algorithm>
#include <map>
#include <exception>
#include <unordered_set>
#include <stdexcept>

#include "../include/lapibt.hpp"
#include "../include/exceptions.hpp"

const std::string LAPIBT::SOLVER_NAME = "Free Space Priority Inheritance With Backtracking V2 (LAPIBT)";


LAPIBT::LAPIBT(LargeAgentsMapfProblem *problem, int inheritanceDepth)
    : LargeAgentsMAPFSolver(problem), inheritanceDepth(inheritanceDepth),
      setOfAgentsInConflict()
{
    solver_name = LAPIBT::SOLVER_NAME;
}


LAPIBT::LAPIBT(LargeAgentsMapfProblem *problem)
    : LargeAgentsMAPFSolver(problem), setOfAgentsInConflict(), inheritanceDepth(5)
{
    solver_name = LAPIBT::SOLVER_NAME;
}

void LAPIBT::run()
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
            {P->getStart(i)},         // List of next locations
            g,                        // goal
            0,                        // eta
            d,                        // initial distance
            getRandomFloat(0, 1, MT), // epsilon, tie-breaker
            P->getSize(i)             // Size (radius) of an agent
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
            if ((agent->path).size() == 1){
                try {
                    mainLAPIBT(agent, allAgents);
                } catch (too_high_compute_time_exception& e) {
                    info("Exception caught, Message: ", e.message());
                    for (auto a : allAgents)
                        delete a;
                    return;
                }
            }
        }

        bool check_goal_condition = true;

        std::vector<Node *> configuration(P->getNum(), nullptr);

        for (auto agent : allAgents)
        {
            Node *agents_next_node = *((agent->path).begin() + 1);
            bool elapsed = (agents_next_node == agent->goal);

            configuration[agent->id] = agents_next_node;
            (agent->path).pop_front();

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
            info("Too much steps!");
            break;
        }

        if (getSolverElapsedTime() > max_comp_time)
        {
            info("Computation time is too big!");
            break;
        }

    }

    for (auto a : allAgents)
        delete a;
}

void LAPIBT::mainLAPIBT(Agent *agent, const std::vector<Agent *> &allAgents)
{
    if (agent->goal == (agent->path).back())
    {
        (agent->path).push_back((agent->path).back());
        return;
    };

    auto closestNodeToTheGoal = [&, agent](Node *const node_lhs, Node *const node_rhs)
    {
        auto distance_node_lhs = pathDist(agent->id, node_lhs);
        auto distance_node_rhs = pathDist(agent->id, node_rhs);
        return distance_node_lhs < distance_node_rhs;
    };

    std::vector<Node *> current_nodes_neighbours = (agent->path).back()->neighbor;
    std::sort(current_nodes_neighbours.begin(), current_nodes_neighbours.end(), closestNodeToTheGoal);
    
    for (auto perpective_next_node : current_nodes_neighbours)
    {
        if (pathDist(agent->id, perpective_next_node) == max_timestep + 1)
            continue;

        (agent->path).push_back(perpective_next_node);

        if (collisionConflict(agent, allAgents))
        {
            (agent->path).pop_back();
            continue;
        }

        if (
            inheritanceConflict(agent, allAgents) &&
            solveInheritanceConflict(agent, allAgents).empty())
        {
            (agent->path).pop_back();
            continue;
        }

        return;
    }

    (agent->path).push_back((agent->path).back());
}

bool LAPIBT::collisionConflict(Agent *child_agent, Agent* parent_agent, const std::vector<Agent *> &allAgents)
{
    checkIfComputationTimeExceeded();

    int child_agent_pos_x = child_agent->path.back()->pos.x;
    int child_agent_pos_y = child_agent->path.back()->pos.y;
    float child_agent_size = ceil(child_agent->size);

    for (auto other_agent : setOfAgentsInConflict)
    {
        int offset = other_agent->id == parent_agent->id;
        for (
            auto node_in_other_agents_path = (other_agent->path).begin();
            node_in_other_agents_path != (other_agent->path).end() - offset;
            node_in_other_agents_path++)
        {
            int other_agent_pos_x = (*node_in_other_agents_path)->pos.x;
            int other_agent_pos_y = (*node_in_other_agents_path)->pos.y;
            float other_agent_size = ceil(other_agent->size);

            if (
                child_agent_pos_x > other_agent_pos_x - child_agent_size &&
                child_agent_pos_x < other_agent_pos_x + other_agent_size &&
                child_agent_pos_y > other_agent_pos_y - child_agent_size &&
                child_agent_pos_y < other_agent_pos_y + other_agent_size
            )
                return true;
        }
    }
    return false;
}

bool LAPIBT::collisionConflict(Agent *agent, const std::vector<Agent *> &allAgents)
{
    checkIfComputationTimeExceeded();

    int agent_pos_x = agent->path.back()->pos.x;
    int agent_pos_y = agent->path.back()->pos.y;
    float agent_size = ceil(agent->size);

    for (auto other_agent : allAgents)
    {
        if (
            other_agent->id != agent->id &&
            setOfAgentsInConflict.find(other_agent) == setOfAgentsInConflict.end() &&
            (other_agent->path).size() >= (agent->path).size())
        {
            for (
                auto node_in_other_agents_path = (other_agent->path).begin() + ((agent->path).size() - 1);
                node_in_other_agents_path != (other_agent->path).end();
                node_in_other_agents_path++)
            {
                int other_agent_pos_x = (*node_in_other_agents_path)->pos.x;
                int other_agent_pos_y = (*node_in_other_agents_path)->pos.y;
                float other_agent_size = ceil(other_agent->size);

                if (
                    agent_pos_x > other_agent_pos_x - agent_size &&
                    agent_pos_x < other_agent_pos_x + other_agent_size &&
                    agent_pos_y > other_agent_pos_y - agent_size &&
                    agent_pos_y < other_agent_pos_y + other_agent_size
                )
                    return true;
            }
        }
    }
    return false;
}

bool LAPIBT::inheritanceConflict(Agent *agent, const std::vector<Agent *> &allAgents)
{
    checkIfComputationTimeExceeded();

    int agent_pos_x = agent->path.back()->pos.x;
    int agent_pos_y = agent->path.back()->pos.y;
    float agent_size = ceil(agent->size);
    
    for (auto other_agent : allAgents)
    {
        int other_agent_pos_x = ((other_agent->path).back())->pos.x;
        int other_agent_pos_y = ((other_agent->path).back())->pos.y;
        float other_agent_size = ceil(other_agent->size);
        if (
            other_agent->id != agent->id &&
            (other_agent->path).size() < (agent->path).size() &&
            (
                agent_pos_x > other_agent_pos_x - agent_size &&
                agent_pos_x < other_agent_pos_x + other_agent_size &&
                agent_pos_y > other_agent_pos_y - agent_size &&
                agent_pos_y < other_agent_pos_y + other_agent_size
            )
        )
        {
            return true;
        }
    }

    return false;
}

std::map<LAPIBT::Agent*, LAPIBT::PathState> LAPIBT::solveInheritanceConflict(Agent *agent, const std::vector<Agent *> &allAgents)
{
    checkIfComputationTimeExceeded();

    int agent_pos_x = agent->path.back()->pos.x;
    int agent_pos_y = agent->path.back()->pos.y;
    float agent_size = ceil(agent->size);
    
    setOfAgentsInConflict.insert(agent);

    std::map<Agent*, PathState> path_states_before_conflict = {};

    auto agent_iterator = allAgents.begin();
    while (agent_iterator != allAgents.end())
    {
        Agent *other_agent = *agent_iterator;
        agent_iterator++;

        int other_agent_pos_x = ((other_agent->path).back())->pos.x;
        int other_agent_pos_y = ((other_agent->path).back())->pos.y;
        float other_agent_size = ceil(other_agent->size);

        if (
            other_agent->id != agent->id &&
            (other_agent->path).size() < (agent->path).size() &&
            (
                agent_pos_x > other_agent_pos_x - agent_size &&
                agent_pos_x < other_agent_pos_x + other_agent_size &&
                agent_pos_y > other_agent_pos_y - agent_size &&
                agent_pos_y < other_agent_pos_y + other_agent_size
            )
        )
        {
            std::map<Agent*, PathState> new_path_states_before_conflict = escapeInheritanceConflict(other_agent, agent, allAgents);

            if (new_path_states_before_conflict.empty())
            {
                for (auto const& [_agent, path_state] : path_states_before_conflict)
                {
                    (_agent->path).resize(path_state.size - 1);
                    (_agent->path).push_back(path_state.last_node_in_path);
                }

                setOfAgentsInConflict.erase(agent);
                return {};
            }
            else
            {
                for (auto const& [_agent, path_state] : new_path_states_before_conflict)
                {
                    if (path_states_before_conflict.find(_agent) == path_states_before_conflict.end()) {
                        path_states_before_conflict[_agent] = path_state;
                    }
                }
                agent_iterator = allAgents.begin();
            }
        }
    }

    setOfAgentsInConflict.erase(agent);
    return path_states_before_conflict;
}

std::map<LAPIBT::Agent*, LAPIBT::PathState> LAPIBT::escapeInheritanceConflict(Agent *child_agent, Agent *parent_agent, const std::vector<Agent *> &allAgents)
{
    if (setOfAgentsInConflict.size() > inheritanceDepth)
        return {};

    Nodes nodes_outside_of_inheritance_conflict = getNodesToAvoidInheritanceConflict(child_agent, parent_agent);

    std::sort(
        nodes_outside_of_inheritance_conflict.begin(),
        nodes_outside_of_inheritance_conflict.end(),
        [&, child_agent](Node *const v, Node *const u)
    {
        auto d_v = (child_agent->path).back()->euclideanDist(v);
        auto d_u = (child_agent->path).back()->euclideanDist(u);
        return d_v < d_u;
    });

    std::map<Agent*, PathState> path_states_before_conflict = {};
    std::unordered_set<int> ids_of_visited_nodes = {};
    bool next_node_found_during_greedy_bfs = false;
    int max_steps_allowed = 3*std::ceil(std::max(child_agent->size, parent_agent->size));

    for (auto node_to_reach : nodes_outside_of_inheritance_conflict)
    {
        if (
            pathDist(child_agent->id, node_to_reach) == max_timestep + 1 ||
            getRandomFloat(0., 1, MT) < 0.175 /// Is needed to prevent deadlocks
        ) continue;

        ids_of_visited_nodes.clear();
        ids_of_visited_nodes.insert((child_agent->path).back()->id);

        auto compareLocal = [node_to_reach](const Node *node_lhs, const Node *node_rhs)
        {
            float distance_node_lhs = node_to_reach->euclideanDist(node_lhs);
            float distance_node_rhs = node_to_reach->euclideanDist(node_rhs);
            return distance_node_lhs < distance_node_rhs;
        };

        path_states_before_conflict[child_agent] = {
            (child_agent->path).size(),
            (child_agent->path).back()
        };
        int step_counter = 0;

        while ((child_agent->path).back()->id != node_to_reach->id)
        {
            next_node_found_during_greedy_bfs = false;
            Nodes neighbours = (child_agent->path).back()->neighbor;
            std::sort(neighbours.begin(), neighbours.end(), compareLocal);

            for (auto neighbour_node : neighbours)
            {
                if (
                    ids_of_visited_nodes.find(neighbour_node->id) != ids_of_visited_nodes.end() ||
                    !checkIfNodeExistInRadiusOnGrid(
                        G,
                        neighbour_node->pos.x,
                        neighbour_node->pos.y,
                        child_agent->size
                    ))
                {
                    ids_of_visited_nodes.insert(neighbour_node->id);
                    continue;
                }

                if (step_counter > max_steps_allowed) {
                    break;
                }

                (child_agent->path).push_back(neighbour_node);
                for (auto conflicting_agent: setOfAgentsInConflict) {
                    if (conflicting_agent->path.size() <= child_agent->path.size()) {
                        for (auto conflicting_agent: setOfAgentsInConflict) {
                            if (
                                path_states_before_conflict.find(conflicting_agent) ==
                                path_states_before_conflict.end()
                            ){
                                path_states_before_conflict[conflicting_agent] = {
                                    (conflicting_agent->path).size(),
                                    (conflicting_agent->path).back()
                                };
                            }
                            conflicting_agent->wait();
                        }
                    }
                }
                
                if (
                    collisionConflict(child_agent, allAgents) ||
                    collisionConflict(child_agent, parent_agent, allAgents)
                    )
                {
                    ids_of_visited_nodes.insert(neighbour_node->id);
                    (child_agent->path).pop_back();
                    continue;
                }

                if (inheritanceConflict(child_agent, allAgents)){
                    std::map<Agent*, PathState> new_path_states_before_conflict = solveInheritanceConflict(
                        child_agent, allAgents
                    );

                    if (new_path_states_before_conflict.empty())
                    {
                        ids_of_visited_nodes.insert(neighbour_node->id);
                        (child_agent->path).pop_back();
                        break;
                    } else {
                        for (auto const& [agent, path_state] : new_path_states_before_conflict)
                        {
                            if (path_states_before_conflict.find(agent) == path_states_before_conflict.end()) {
                                path_states_before_conflict[agent] = path_state;
                            }
                        }
                    }
                }
                
                for (auto const& [moved_agent, _] : path_states_before_conflict) {
                    if (
                        setOfAgentsInConflict.find(moved_agent) == setOfAgentsInConflict.end() &&
                        moved_agent->id != child_agent->id
                    ) {
                        moved_agent->path.push_back(moved_agent->path.back());
                    }
                }
                ids_of_visited_nodes.insert(neighbour_node->id);
                next_node_found_during_greedy_bfs = true;
                step_counter++;
                break;
            }

            if (!next_node_found_during_greedy_bfs)
            {
                for (auto const& [agent, path_state] : path_states_before_conflict)
                {
                    (agent->path).resize(path_state.size - 1);
                    (agent->path).push_back(path_state.last_node_in_path);
                }
                path_states_before_conflict.clear();
                break;
            }
        }

        if ((child_agent->path).back()->id == node_to_reach->id)
            return path_states_before_conflict;
    }
    return {};
}

Nodes LAPIBT::getNodesToAvoidInheritanceConflict(const Agent *child_agent, const Agent *parent_agent)
{
    int x = ((parent_agent->path).back()->pos).x;
    int y = ((parent_agent->path).back()->pos).y;

    int parent_agent_size = ceil(parent_agent->size);
    int child_agent_size = ceil(child_agent->size);

    Nodes border;

    int deltaIncrement = std::max(1, int(std::floor((parent_agent_size+child_agent_size)/4)));

    for (int delta = 0; delta < parent_agent_size + child_agent_size; delta += deltaIncrement) {
        if (G->existNode(x + delta, y - child_agent_size))
            border.push_back(G->getNode(x + delta, y - child_agent_size));

        if (G->existNode(x + delta, y + parent_agent_size))
            border.push_back(G->getNode(x + delta, y + parent_agent_size));

        if (G->existNode(x - child_agent_size, y + delta))
            border.push_back(G->getNode(x - child_agent_size, y + delta));

        if (G->existNode(x + parent_agent_size, y + delta))
            border.push_back(G->getNode(x + parent_agent_size, y + delta));
    }
    
    return border;
}