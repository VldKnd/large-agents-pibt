#include <algorithm>
#include <map>
#include <exception>
#include <unordered_set>


#include "../include/fspibt_v2.hpp"

const std::string FSPIBTV2::SOLVER_NAME = "Free Space Priority Inheritance With Backtracking V2 (FSPIBTV2)";

FSPIBTV2::FSPIBTV2(FreeSpaceMapfProblem *problem)
    : FreeSpaceMAPFSolver(problem),
      setOfAgentIdsInConflict()
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
            {P->getStart(i)},         // List of next locations
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
            if ((agent->path).size() == 1) // run if its just { v_t0 }
                funcFSPIBTV2(agent, allAgents);
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
            std::cout << "Too much steps!\n";
            break;
        }
    }

    for (auto a : allAgents)
        delete a;
}

void FSPIBTV2::funcFSPIBTV2(Agent *agent, const std::vector<Agent *> &allAgents)
{
    if (agent->goal == (agent->path).back())
    {
        (agent->path).push_back((agent->path).back());
        return;
    }

    auto closestNodeToTheGoal = [&, agent](Node *const node_lhs, Node *const node_rhs)
    {
        auto distance_node_lhs = pathDist(agent->id, node_lhs);
        auto distance_node_rhs = pathDist(agent->id, node_rhs);
        return distance_node_lhs < distance_node_rhs;
    };

    std::vector<Node*> current_nodes_neighbours = (agent->path).back()->neighbor;
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

        if (inheritanceConflict(agent, allAgents))
        {
            (agent->path).pop_back();
            continue;
        }

        return;
    }

    (agent->path).push_back((agent->path).back());
}

bool FSPIBTV2::collisionConflict(Agent *agent, const std::vector<Agent *> &allAgents)
{
    for (auto other_agent : allAgents)
    {
        if (
            other_agent->id != agent->id &&
            (other_agent->path).size() >= (agent->path).size())
        {
            int isConflictingAgent = setOfAgentIdsInConflict.contains(other_agent);

            for (
                auto node_in_other_agents_path = (other_agent->path).begin() + ((agent->path).size() - 1);
                node_in_other_agents_path != (other_agent->path).end() - isConflictingAgent;
                /// The - isConflictingAgent is needed, since all the agents in the conflict are waiting for it to escape the inheritance conflict.
                /// So each step it takes will make everyone else wait one extra step up to when it escapes.
                /// The step it escapes, it will free the possible space of confict with previous agent, so the agent that has initiated the search will be able to make a step.
                /// For it to not block other agetns it has to escape to place, where there is no conflict with intention of any agent.
                node_in_other_agents_path++)
            {
                if ((*node_in_other_agents_path)->euclideanDist((agent->path).back()) < agent->radius + other_agent->radius)
                    return true;
            }
        }
    }
    return false;
}

bool FSPIBTV2::inheritanceConflict(Agent *agent, const std::vector<Agent *> &allAgents)
{
    setOfAgentIdsInConflict.insert(agent);
    std::vector<std::tuple<Agent*, int>> vector_of_agents_and_agents_steps_during_iheritance = {};

    auto agent_iterator = allAgents.begin();
    while (agent_iterator != allAgents.end()) {
        Agent* other_agent = *agent_iterator;
        agent_iterator++;
        if (
            other_agent->id != agent->id &&
            (other_agent->path).size() < (agent->path).size() &&
            (other_agent->path).back()->euclideanDist((agent->path).back()) < agent->radius + other_agent->radius)
        {
            int numberOfStepsDuringIheritanceConflict = solveInheritanceConflict(other_agent, allAgents);

            if (numberOfStepsDuringIheritanceConflict != 0){
                vector_of_agents_and_agents_steps_during_iheritance.push_back({
                    other_agent,
                    numberOfStepsDuringIheritanceConflict
                });
            } else {
                while (vector_of_agents_and_agents_steps_during_iheritance.size()) {
                    std::tuple<Agent*, int> agents_and_agents_steps_during_iheritance = \
                    vector_of_agents_and_agents_steps_during_iheritance.back();
                    vector_of_agents_and_agents_steps_during_iheritance.pop_back();
                    Agent* agent_in_iheritance_conflict = std::get<0>(agents_and_agents_steps_during_iheritance);
                    int agents_steps_during_iheritance = std::get<1>(agents_and_agents_steps_during_iheritance);

                    for(int i = agents_steps_during_iheritance; i--;) {
                        (agent_in_iheritance_conflict->path).pop_back();
                        for (auto agent_in_conflict : setOfAgentIdsInConflict) {
                            (agent_in_conflict->path).erase((agent_in_conflict->path).end() - 2);
                        }
                    }
                }

                setOfAgentIdsInConflict.erase(agent);
                return true;
            }
        }
    }

    setOfAgentIdsInConflict.erase(agent);
    return false;
}

int FSPIBTV2::solveInheritanceConflict(Agent *agent, const std::vector<Agent *> &allAgents)
{
    if (setOfAgentIdsInConflict.size() > 5)
        return 0;

    auto compareToReachGoal = [&, agent](Node *const v, Node *const u)
    {
        auto d_v = (agent->path).back()->euclideanDist(v);
        auto d_u = (agent->path).back()->euclideanDist(u);
        return d_v < d_u;
    };

    Nodes nodesOutsideOfCurrentInheritanceConflict = getNodesToAvoidInheritanceConflict(agent, allAgents);
    std::sort(
        nodesOutsideOfCurrentInheritanceConflict.begin(),
        nodesOutsideOfCurrentInheritanceConflict.end(),
        compareToReachGoal
    );


    std::unordered_set<int> ids_of_visited_nodes = {};
    bool next_node_found_during_greedy_bfs = false;
    int counter_of_steps_made = 0;

    for (auto node_to_reach : nodesOutsideOfCurrentInheritanceConflict)
    {
        if (
            pathDist(agent->id, node_to_reach) == max_timestep + 1 ||
            getRandomFloat(0., 1, MT) < 0.8 /// Is needed to prevent deadlocks
        )
        {
            continue;
        }

        counter_of_steps_made = 0;
        ids_of_visited_nodes.clear();
        ids_of_visited_nodes.insert((agent->path).back()->id);

        auto compareLocal = [node_to_reach](const Node *node_lhs, const Node *node_rhs)
        {
            float distance_node_lhs = node_to_reach->euclideanDist(node_lhs);
            float distance_node_rhs = node_to_reach->euclideanDist(node_rhs);
            return distance_node_lhs < distance_node_rhs;
        };

        while ((agent->path).back()->id != node_to_reach->id)
        {
            next_node_found_during_greedy_bfs = false;
            Nodes neighbours = (agent->path).back()->neighbor;
            std::sort(neighbours.begin(), neighbours.end(), compareLocal);

            for (auto neighbour_node : neighbours) {
                if (
                    ids_of_visited_nodes.find(neighbour_node->id) != ids_of_visited_nodes.end() ||
                    !checkIfNodeExistInRadiusOnGrid(G, neighbour_node->pos.x, neighbour_node->pos.y, agent->radius)
                ) {
                    continue;

                }

                (agent->path).push_back(neighbour_node);
                for (auto agent : setOfAgentIdsInConflict) {
                    Node* second_to_last_element = *((agent->path).end() - 2);
                    (agent->path).insert(
                        (agent->path).end() - 1,
                        second_to_last_element
                    );
                }

                if (collisionConflict(agent, allAgents))
                {
                    ids_of_visited_nodes.insert(neighbour_node->id);
                    (agent->path).pop_back();
                    for (auto agent : setOfAgentIdsInConflict) {
                        (agent->path).erase((agent->path).end() - 2);
                    }
                    continue;
                }

                if (inheritanceConflict(agent, allAgents))
                {
                    ids_of_visited_nodes.insert(neighbour_node->id);
                    (agent->path).pop_back();
                    for (auto agent : setOfAgentIdsInConflict) {
                        (agent->path).erase((agent->path).end() - 2);
                    }
                    continue;
                }

                ids_of_visited_nodes.insert(neighbour_node->id);
                next_node_found_during_greedy_bfs = true;
                counter_of_steps_made++;
                break;
            }

            if (!next_node_found_during_greedy_bfs || counter_of_steps_made > 5 * setOfAgentIdsInConflict.size()) {
                for(int i = counter_of_steps_made; i--;) {
                    (agent->path).pop_back();
                    for (auto agent_in_conflict : setOfAgentIdsInConflict) {
                        (agent_in_conflict->path).erase((agent_in_conflict->path).end() - 2);
                    }
                }
                break;
            }
        }

        if ((agent->path).back()->id == node_to_reach->id) {
            return counter_of_steps_made;
        }
    }

    return 0;
}

void updateMinMaxMap(int key, int value, std::map<int, std::tuple<int, int>>* dictionary) {
    if (dictionary->find(key) == dictionary->end()) {
        (*dictionary)[key] = std::tuple<int, int>({ value, value });
    } else {
        std::get<0>(dictionary->at(key)) = std::min<int>(std::get<0>(dictionary->at(key)), value);
        std::get<1>(dictionary->at(key)) = std::max<int>(std::get<1>(dictionary->at(key)), value);
    }
}

Nodes FSPIBTV2::getNodesToAvoidInheritanceConflict(const Agent *child_agent, const std::vector<Agent *> &allAgents)
{
    // outerBorder[x] = <min_y, max_y>; for each x min and max y values outside of conflict.
    std::map<int, std::tuple<int, int>> outerBorderX = {};

    // outerBorder[y] = <min_x, max_x>; for each y min and max x values outside of conflict.
    std::map<int, std::tuple<int, int>> outerBorderY = {};

    for (auto conflicting_agent : setOfAgentIdsInConflict){

        int x = ((conflicting_agent->path).back()->pos).x;
        int y = ((conflicting_agent->path).back()->pos).y;

        double r = child_agent->radius + conflicting_agent->radius;
        int dx = std::ceil(r + 1e-3);
        int dy = 0;

        do
        {
            if (G->existNode(x + dx, y + dy)) {
                updateMinMaxMap(x + dx, y + dy, &outerBorderX);
                updateMinMaxMap(y + dy, x + dx, &outerBorderY);
            }

            if (G->existNode(x - dx, y + dy)) {
                updateMinMaxMap(x - dx, y + dy, &outerBorderX);
                updateMinMaxMap(y + dy, x - dx, &outerBorderY);
            }

            if (G->existNode(x - dx, y - dy)) {
                updateMinMaxMap(x - dx, y - dy, &outerBorderX);
                updateMinMaxMap(y - dy, x - dx, &outerBorderY);
            }

            if (G->existNode(x + dx, y - dy)) {
                updateMinMaxMap(x + dx, y - dy, &outerBorderX);
                updateMinMaxMap(y - dy, x + dx, &outerBorderY);
            }
            if ((dx - 1) * (dx - 1) + dy * dy > r * r)
                --dx;
            else
                ++dy;
        } while (dx != 0);
    }

    std::unordered_set<Node*> border = {};

    for (auto const& x_minmax_y : outerBorderX) {
        border.insert(
            G->getNode(x_minmax_y.first, std::get<0>(x_minmax_y.second))
        );
        border.insert(
            G->getNode(x_minmax_y.first, std::get<1>(x_minmax_y.second))
        );
    }

    for (auto const& y_minmax_x : outerBorderY) {
        border.insert(
            G->getNode(std::get<0>(y_minmax_x.second), y_minmax_x.first)
        );
        border.insert(
            G->getNode(std::get<1>(y_minmax_x.second), y_minmax_x.first)
        );
    }
        
    std::vector<Node*> nodes;

    std::copy_if(
        border.begin(), border.end(),
        std::back_inserter(nodes),
        [&](Node* node){
            if (!checkIfNodeExistInRadiusOnGrid(G, node->pos.x, node->pos.y, child_agent->radius))
                return false;

            for (auto conflicting_agent: setOfAgentIdsInConflict) {
                for (
                    auto node_in_conflicting_agent_path = (conflicting_agent->path).begin();
                    node_in_conflicting_agent_path != (conflicting_agent->path).end() - 1;
                    node_in_conflicting_agent_path++)
                {
                    if ((*node_in_conflicting_agent_path)->euclideanDist(node) < conflicting_agent->radius + child_agent->radius)
                        return false;
                }
            }
            return true;
        }
    );

    return nodes;
}
