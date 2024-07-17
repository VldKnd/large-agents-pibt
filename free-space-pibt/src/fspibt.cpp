#include <algorithm>
#include <unordered_set>

#include "../include/fspibt.hpp"

const std::string FSPIBT::SOLVER_NAME = "Free Space Priority Inheritance With Backtracking (FSPIBT)";

FSPIBT::FSPIBT(FreeSpaceMapfProblem *problem)
    : FreeSpaceMAPFSolver(problem),
      activeConflicts()
{
    solver_name = FSPIBT::SOLVER_NAME;
    int N = problem->getNum();
    for (int i = 0; i < N; i++)
    {
        occupation.push_back({});
    }
}

void FSPIBT::run()
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

        occupation[i].push_back(P->getStart(i));
        auto *a = new Agent{i,
                            nullptr,
                            nullptr,
                            g,
                            0,
                            d,
                            getRandomFloat(0, 1, MT),
                            P->getRadius(i)};

        A.push_back(a);
    }

    solution.add(P->getConfigStart());

    int timestep = 0;
    while (true)
    {
        ++timestep;
        info(" ", "elapsed:", getSolverElapsedTime(), ", timestep:", timestep);

        std::sort(A.begin(), A.end(), compare);
        for (auto a : A)
        {
            if (occupation[a->id].size() == timestep)
            {
                std::unordered_set<Node *> visited;
                int steps = 0;
                while (steps < 1)
                {
                    funcFSPIBT(a, A, visited);
                    steps++;
                    if (a->g != occupation[a->id].back())
                    {
                        visited.insert(occupation[a->id].back());
                    }
                }
            }
        }

        bool check_goal_cond = true;
        Config config(P->getNum(), nullptr);

        for (auto a : A)
        {
            config[a->id] = occupation[a->id][timestep];

            bool elapsed = (config[a->id] == a->g);
            a->elapsed = elapsed ? 0 : a->elapsed + 1;

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

void FSPIBT::funcFSPIBT(Agent *ai, const Agents &A, const std::unordered_set<Node *> &visited)
{
    if (ai->g == occupation[ai->id].back())
    {
        occupation[ai->id].push_back(occupation[ai->id].back());
        return;
    }

    auto compareToReachGoal = [&, ai](Node *const v, Node *const u)
    {
        auto d_v = pathDist(ai->id, v);
        auto d_u = pathDist(ai->id, u);
        return d_v < d_u;
    };

    Nodes C = occupation[ai->id].back()->neighbor;
    std::sort(C.begin(), C.end(), compareToReachGoal);

    for (auto v_next : C)
    {
        if (pathDist(ai->id, v_next) == max_timestep + 1 || visited.count(v_next))
        {
            continue;
        }

        occupation[ai->id].push_back(v_next);

        if (collisionConflict(A, ai))
        {
            occupation[ai->id].pop_back();
            occupation[ai->id].push_back(occupation[ai->id].back());
            return;
        }

        if (inheritanceConflict(A, ai))
        {
            occupation[ai->id].pop_back();
            continue;
        }

        if (collisionConflict(A, ai))
        {
            occupation[ai->id].pop_back();

            occupation[ai->id].push_back(occupation[ai->id].back());
            return;
        }

        return;
    }

    occupation[ai->id].push_back(occupation[ai->id].back());
}

bool FSPIBT::funcFSPIBT(Agent *ai, Agent *aj, const Agents &A)
{
    if (activeConflicts.size() > 3)
    {
        return false;
    }
    activeConflicts.insert(aj->id);

    auto compareToEvadeCol = [&, ai](Node *v, Node *u)
    {
        auto d_v = pathDist(ai->id, v);
        auto d_u = pathDist(ai->id, u);
        return d_v < d_u;
    };

    Nodes C = getNodes(*ai, *aj);
    std::sort(C.begin(), C.end(), compareToEvadeCol);

    std::unordered_set<int> visited;
    std::vector<State> snap_shot;

    for (int i = 0; i < occupation.size(); ++i)
    {
        snap_shot.push_back({i, int(occupation[i].size()), occupation[i].back()});
    }

    for (Node *u : C)
    {

        if (pathDist(ai->id, u) == max_timestep + 1)
        {
            continue;
        }

        visited.clear();
        reverseToState(snap_shot);

        auto compareLocal = [u](const Node *a, const Node *b)
        {
            float d_a = u->euclideanDist(a);
            float d_b = u->euclideanDist(b);
            return d_a < d_b;
        };

        while (occupation[ai->id].back()->id != u->id)
        {
            if (float(visited.size()) > (ai->r + aj->r) * 6)
            {
                break;
            }

            Nodes N = occupation[ai->id].back()->neighbor;
            std::sort(N.begin(), N.end(), compareLocal);

            auto find = std::find_if(N.begin(), N.end(), [&, ai, aj, visited](Node *a)
                                     {
                if (visited.find(a->id) == visited.end() &&
                    checkIfNodeExistInRadiusOnGrid(G, a->pos.x, a->pos.y, ai->r)) {
                     for (size_t t_ai = occupation[ai->id].size() - 1; t_ai < occupation[aj->id].size() - 1; ++t_ai) {
                         if (occupation[aj->id][t_ai]->euclideanDist(a) < ai->r + aj->r) {
                             return false;
                         }
                     }
                     return true;
                } else {
                    return false;
                } });

            if (find == N.end())
            {
                break;
            }

            visited.insert((*find)->id);
            occupation[ai->id].push_back(*find);

            if (collisionConflict(A, ai, aj))
            {
                break;
            }

            if (inheritanceConflict(A, ai))
            {
                break;
            }

            if (occupation[ai->id].back()->id != u->id)
            {
                for (int id : activeConflicts)
                {
                    if (occupation[ai->id].size() >= occupation[id].size())
                    {
                        auto last = occupation[id].back();
                        occupation[id].pop_back();
                        occupation[id].push_back(occupation[id].back());
                        occupation[id].push_back(last);
                    }
                }
            }
            else
            {
                activeConflicts.erase(aj->id);
                return true;
            }
        }
    }

    reverseToState(snap_shot);
    activeConflicts.erase(aj->id);
    return false;
}

bool FSPIBT::collisionConflict(const Agents &A, Agent *a_i, Agent *a_j)
{
    for (auto a_pt : A)
    {
        if (a_pt->id != a_i->id &&
            a_pt->id != a_j->id &&
            occupation[a_pt->id].size() >= occupation[a_i->id].size())
        {
            int offset = activeConflicts.count(a_pt->id);
            for (size_t t_ai = occupation[a_i->id].size() - 1 - offset; t_ai < occupation[a_pt->id].size(); ++t_ai)
            {
                if (occupation[a_pt->id][t_ai]->euclideanDist(occupation[a_i->id].back()) <
                    a_i->r + a_pt->r)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

void FSPIBT::reverseToState(const std::vector<State> &states)
{
    for (const auto &state : states)
    {
        if (occupation[state.id].size() != state.moves_made)
        {
            occupation[state.id].resize(state.moves_made);
            if (occupation[state.id].back() != state.last_move)
            {
                occupation[state.id].pop_back();
                occupation[state.id].push_back(state.last_move);
            }
        }
    }
}

bool FSPIBT::collisionConflict(const Agents &A, Agent *x_pt)
{
    for (auto a_pt : A)
    {
        if (a_pt->id != x_pt->id &&
            occupation[a_pt->id].size() >= occupation[x_pt->id].size())
        {
            for (size_t t_ai = occupation[x_pt->id].size() - 1; t_ai < occupation[a_pt->id].size(); ++t_ai)
            {
                if (occupation[a_pt->id][t_ai]->euclideanDist(occupation[x_pt->id].back()) <
                    x_pt->r + a_pt->r)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool FSPIBT::inheritanceConflict(const Agents &A, Agent *x_pt)
{
    auto agent = A.begin();
    while (agent != A.end())
    {
        if (occupation[(*agent)->id].size() < occupation[x_pt->id].size() &&
            occupation[(*agent)->id].back()->euclideanDist(occupation[x_pt->id].back()) < x_pt->r + (*agent)->r)
        {
            if (!funcFSPIBT((*agent), x_pt, A))
            {
                return true;
            }
            else
            {
                agent = A.begin();
            }
        }
        else
        {
            ++agent;
        }
    }
    return false;
}

Nodes FSPIBT::getNodes(const Agent &ai, const Agent &aj)
{
    int x = (occupation[aj.id].back()->pos).x;
    int y = (occupation[aj.id].back()->pos).y;
    double r = ai.r + aj.r;
    int dx = std::ceil(r);
    int dy = 0;
    Nodes res;

    do
    {
        if (G->existNode(x + dx, y + dy))
            res.push_back(G->getNode(x + dx, y + dy));
        if (G->existNode(x - dy, y + dx))
            res.push_back(G->getNode(x - dy, y + dx));
        if (G->existNode(x - dx, y - dy))
            res.push_back(G->getNode(x - dx, y - dy));
        if (G->existNode(x + dy, y - dx))
            res.push_back(G->getNode(x + dy, y - dx));
        if ((dx - 1) * (dx - 1) + dy * dy > r * r)
            --dx;
        else
            ++dy;
    } while (dx != 0);

    return res;
}
