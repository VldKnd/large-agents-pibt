
#include <iostream>
#include <fstream>
#include <regex>
#include <queue>
#include <unordered_set>

#include "../include/mapf_problem.hpp"
#include "../include/graph_utils.hpp"

MapfProblem::MapfProblem(std::string _instance, Graph *_G, std::mt19937 *_MT,
                 Config _config_s, Config _config_g, int _num_agents,
                 int _max_timestep, int _max_comp_time)
        : instance(_instance),
          G(_G),
          MT(_MT),
          config_s(_config_s),
          config_g(_config_g),
          num_agents(_num_agents),
          max_timestep(_max_timestep),
          max_comp_time(_max_comp_time) {};

Node *MapfProblem::getStart(int i) const {

    if (!(0 <= i && i < (int) config_s.size())) halt("invalid index");
    return config_s[i];
}

Node *MapfProblem::getGoal(int i) const {
    if (!(0 <= i && i < (int) config_g.size())) halt("invalid index");
    return config_g[i];
}

void MapfProblem::halt(const std::string &msg) const {
    std::cout << "error@Problem: " << msg << std::endl;
    this->~MapfProblem();
    std::exit(1);
}

void MapfProblem::warn(const std::string &msg) const {
    std::cout << "warn@Problem: " << msg << std::endl;
}

// -------------------------------------------
// Large Agents MAPF

LargeAgentsMapfProblem::LargeAgentsMapfProblem(const std::string& _instance, const int seed)
    : MapfProblem(_instance), instance_initialized(true), sizes(std::vector<float>(0))
{
    MT = new std::mt19937(seed);
    readInstanceFile(_instance);
}

LargeAgentsMapfProblem::LargeAgentsMapfProblem(const std::string& _instance)
    : MapfProblem(_instance), instance_initialized(true), sizes(std::vector<float>(0))
{
    readInstanceFile(_instance);
}

void LargeAgentsMapfProblem::readInstanceFile(const std::string &_instance) {
    // read instance file
    std::ifstream file(instance);
    if (!file) halt("file " + instance + " is not found.");
    
    std::string line;
    std::smatch results;
    std::regex r_comment = std::regex(R"(#.+)");
    std::regex r_map = std::regex(R"(map_file=(.+))");
    std::regex r_agents = std::regex(R"(agents=(\d+))");
    std::regex r_well_formed = std::regex(R"(well_formed=(\d+))");
    std::regex r_sizes = std::regex("sizes=(\\(?(\\d*[.]?\\d*,? ?)*\\)?)");
    std::regex r_sizes_random_uniform = std::regex(R"(sizes_random_uniform=(\d*[.]?\d*),(\d*[.]?\d*))");
    std::regex r_seed = std::regex(R"(seed=(\d+))");
    std::regex r_random_problem = std::regex(R"(random_problem=(\d+))");
    std::regex r_max_timestep = std::regex(R"(max_timestep=(\d+))");
    std::regex r_max_comp_time = std::regex(R"(max_comp_time=(\d+))");
    std::regex r_sg = std::regex(R"((\d+),(\d+),(\d+),(\d+))");

    bool read_scen = true;
    bool well_formed = false;
    bool radius_done = false;

    while (getline(file, line)) {
        if (*(line.end() - 1) == 0x0d) line.pop_back();

        // comment
        if (std::regex_match(line, results, r_comment)) {
            continue;
        }
        // read map
        if (std::regex_match(line, results, r_map)) {
            G = new Grid(results[1].str());
            continue;
        }
        // set agent num
        if (std::regex_match(line, results, r_agents)) {
            num_agents = std::stoi(results[1].str());
            continue;
        }
        // set sizes
        if (std::regex_match(line, results, r_sizes_random_uniform) && !radius_done) {
          float r_min = std::stof(results[1].str());
		  float r_max = std::stof(results[2].str());
          std::random_device rand_dev;
          std::mt19937 generator(rand_dev());
          std::uniform_real_distribution<float> distr(r_min, r_max);
          for (size_t i = 0; i < num_agents; i++){
            sizes.push_back(distr(generator));
          }
		  radius_done = true;
		  continue;
		}
        else if (std::regex_match(line, results, r_sizes) && !radius_done) {
            std::string result = results[1].str();
            std::string::iterator end_pos_ws = std::remove(result.begin(), result.end(), ' ');
            result.erase(end_pos_ws, result.end());

            std::string::iterator end_pos_lbr = std::remove(result.begin(), result.end(), '(');
            result.erase(end_pos_lbr, result.end());

            std::string::iterator end_pos_rbr = std::remove(result.begin(), result.end(), ')');
            result.erase(end_pos_rbr, result.end());

            std::string token;
            std::size_t pos = 0;
            while ((pos = result.find(",")) != std::string::npos) {
                token = result.substr(0, pos);
                sizes.push_back(std::stof(token));
                result.erase(0, pos + 1);
            }
            sizes.push_back(std::stof(result));

            // check sizes initialized
            if (sizes.size() < num_agents) {
                std::string warn_text = (
                    "only " + std::to_string(sizes.size()) +
                     " sizes given for " + std::to_string(num_agents) +
                     " agents, set remaining agents with default radius 0.45"
                );
                warn(warn_text);
    
                for (size_t i = sizes.size(); i < num_agents; i++) {
                    sizes.push_back(0.45);
                }
            }

            sizes.resize(num_agents);
            radius_done = true;
            
            continue;
        }
        // set random seed
        if (std::regex_match(line, results, r_seed)) {
            MT = new std::mt19937(std::stoi(results[1].str()));
            continue;
        }
        // skip reading initial/goal nodes
        if (std::regex_match(line, results, r_random_problem)) {
            if (std::stoi(results[1].str())) {
                read_scen = false;
                config_s.clear();
                config_g.clear();
            }
            continue;
        }
        // set max timestep
        if (std::regex_match(line, results, r_max_timestep)) {
            max_timestep = std::stoi(results[1].str());
            continue;
        }
        // set max computation time
        if (std::regex_match(line, results, r_max_comp_time)) {
            max_comp_time = std::stoi(results[1].str());
            continue;
        }
        // read initial/goal nodes
        if (std::regex_match(line, results, r_sg) && read_scen &&
            (int) config_s.size() < (int) sizes.size() && (int) config_g.size() < (int) sizes.size() &&
            (int) config_s.size() < num_agents) {
            int x_s = std::stoi(results[1].str());
            int y_s = std::stoi(results[2].str());
            int x_g = std::stoi(results[3].str());
            int y_g = std::stoi(results[4].str());
            if (!(checkIfNodeExistInRadiusOnGrid(G, x_s, y_s, sizes[config_s.size()]))) {
                halt("start node (" + std::to_string(x_s) + ", " + std::to_string(y_s) +
                     ") does not exist, or there are object in its radius " +
                     std::to_string(sizes[config_s.size()]) + ", invalid scenario");
            }
            if (!checkIfNodeExistInRadiusOnGrid(G, x_g, y_g, sizes[config_g.size()])) {
                halt("goal node (" + std::to_string(x_g) + ", " + std::to_string(y_g) +
                     ") does not exist, or there are object in its radius " +
                     std::to_string(sizes[config_g.size()]) + ", invalid scenario");
            }

            Node *s = G->getNode(x_s, y_s);
            Node *g = G->getNode(x_g, y_g);
            config_s.push_back(s);
            config_g.push_back(g);
        }


        if (std::regex_match(line, results, r_well_formed)) {
            if (std::stoi(results[1].str())) well_formed = true;
            continue;
        }
    }

    // set default value not identified params
    if (MT == nullptr) MT = new std::mt19937(DEFAULT_SEED);
    if (max_timestep == 0) max_timestep = DEFAULT_MAX_TIMESTEP;
    if (max_comp_time == 0) max_comp_time = DEFAULT_MAX_COMP_TIME;

    // check starts/goals
    if (num_agents <= 0) halt("invalid number of agents");


    const int config_s_size = config_s.size();
    if (!config_s.empty() && num_agents > config_s_size) {
        warn("given starts/goals are not sufficient\nrandomly create instances");
    }

    if (num_agents > config_s_size) {
        if (well_formed) {
            setWellFormedInstance();
        } else {
            setRandomStartsGoals();
        }
    }

    // trimming
    config_s.resize(num_agents);
    config_g.resize(num_agents);
}

LargeAgentsMapfProblem::LargeAgentsMapfProblem(LargeAgentsMapfProblem *P, Config _config_s,
                                 Config _config_g, int _max_comp_time,
                                 int _max_timestep, std::vector<float> *_sizes)
        : MapfProblem(P->getInstanceFileName(), P->getG(), P->getMT(), _config_s,
                  _config_g, P->getNum(), _max_timestep, _max_comp_time),
          instance_initialized(false),
          sizes(*_sizes) {
}

LargeAgentsMapfProblem::LargeAgentsMapfProblem(LargeAgentsMapfProblem *P, int _max_comp_time)
        : MapfProblem(P->getInstanceFileName(), P->getG(), P->getMT(),
                  P->getConfigStart(), P->getConfigGoal(), P->getNum(),
                  P->getMaxTimestep(), _max_comp_time),
          instance_initialized(false),
          sizes(P->getSizes()) {
}

LargeAgentsMapfProblem::~LargeAgentsMapfProblem() {
    if (instance_initialized) {
        if (G != nullptr) delete G;
        if (MT != nullptr) delete MT;
    }
}

bool LargeAgentsMapfProblem::isInCollision(Config *C, int id, float r) {
    Grid *grid = reinterpret_cast<Grid *>(G);
    int y = std::ceil(id / grid->getHeight());
    int x = (id - y);
    return isInCollision(C, x, y, r);
}

bool LargeAgentsMapfProblem::isInCollision(Config *C, int x, int y, float r) {
    for (int i = 0; i < (*C).size(); i++) {
        if ((*C)[i]->pos.euclideanDist(Pos(x, y)) < r + sizes[i]) return true;
    }
    return false;
}

void LargeAgentsMapfProblem::setRandomStartsGoals() {
    setRandomStarts();
    setRandomGoals();
}


void LargeAgentsMapfProblem::setRandomStarts() {
    config_s.clear();
    Grid *grid = reinterpret_cast<Grid *>(G);
    const int N = grid->getWidth() * grid->getHeight();

    std::vector<int> starts(N);
    std::iota(starts.begin(), starts.end(), 0);
    std::shuffle(starts.begin(), starts.end(), *MT);

    int x, y;
    int i = 0;
    while ((int) config_s.size() != num_agents) {
        do {
            x = starts[i] % grid->getWidth();
            y = int(starts[i] / grid->getWidth());
            ++i;
            if (i >= N) halt("number of agents is too large.");
        } while (!checkIfNodeExistInRadiusOnGrid(G, x, y, sizes[config_s.size()]) ||
                 isInCollision(&config_s, x, y, sizes[config_s.size()]));
        config_s.push_back(G->getNode(starts[i - 1]));
    }
}

void LargeAgentsMapfProblem::setRandomGoals() {
    config_g.clear();
    Grid *grid = reinterpret_cast<Grid *>(G);
    const int N = grid->getWidth() * grid->getHeight();

    std::vector<int> goals(N);
    std::iota(goals.begin(), goals.end(), 0);
    std::shuffle(goals.begin(), goals.end(), *MT);

    int x, y;
    int i = 0;
    while ((int) config_g.size() != num_agents) {
        do {
            if (i >= N) halt("number of agents is too large.");
            x = goals[i] % grid->getWidth();
            y = int(goals[i] / grid->getWidth());
            ++i;
        } while (!checkIfNodeExistInRadiusOnGrid(G, x, y, sizes[config_g.size()]) ||
                 isInCollision(&config_g, x, y, sizes[config_g.size()]));
        config_g.push_back(G->getNode(goals[i - 1]));
    }
}

/*
 * Note: Just as with MAPF and even more it is hard to generate
 * well-formed instances with dense situations
 */
void LargeAgentsMapfProblem::setWellFormedInstance() {
    setWellFormedStarts();
    setWellFormedGoals();
}

void LargeAgentsMapfProblem::setWellFormedStarts() {
    setRandomStarts();
};

void LargeAgentsMapfProblem::setWellFormedGoals() {
    Grid *grid = reinterpret_cast<Grid *>(G);
    const int N = grid->getWidth() * grid->getHeight();

    while ((int) config_g.size() != num_agents) {
        Node* start = config_s[config_g.size()];
        std::unordered_set<Node*> reachable_nodes;

        /// BFS to find accessible nodes;
        std::queue<Node*> OPEN;
        Node* n = start;
        float r = sizes[config_g.size()];

        OPEN.push(n);
        while (!OPEN.empty()) {
            n = OPEN.front();
            OPEN.pop();

            for (auto m : n->neighbor) {
                if (reachable_nodes.count(m)) continue;

                int x = m->id % grid->getWidth();
                int y = std::floor( m->id / grid->getWidth());
                if (checkIfNodeExistInRadiusOnGrid(G, x, y, r)) {
                    reachable_nodes.insert(m);
                    OPEN.push(m);
                }
            }
        }


        int x, y;
        int j = 0;

        std::vector<int> goals(N);
        std::iota(goals.begin(), goals.end(), 0);
        std::shuffle(goals.begin(), goals.end(), *MT);

        do {
            if (j >= N) halt("number of agents is too large.");
            x = goals[j] % grid->getWidth();
            y = int(goals[j] / grid->getWidth());
            ++j;
        } while (!reachable_nodes.count(G->getNode(goals[j - 1])) ||
                 isInCollision(&config_g, x, y, sizes[config_g.size()]));

        config_g.push_back(G->getNode(goals[j - 1]));
    }
};

/*
 * end of wellformed snippet
 */

void LargeAgentsMapfProblem::makeScenFile(const std::string &output_file) {
    Grid *grid = reinterpret_cast<Grid *>(G);
    std::ofstream log;
    log.open(output_file, std::ios::out);
    log << "map_file=" << grid->getMapFileName() << "\n";
    log << "agents=" << num_agents << "\n";
    log << "sizes=" << sizes[0];
    for (int it = 1; it < sizes.size(); it++) {
        log << ", " << sizes[it];
    }
    log << "\n";
    log << "seed=0\n";
    log << "random_problem=0\n";
    log << "max_timestep=" << max_timestep << "\n";
    log << "max_comp_time=" << max_comp_time << "\n";
    for (int i = 0; i < num_agents; ++i) {
        log << config_s[i]->pos.x << "," << config_s[i]->pos.y << ","
            << config_g[i]->pos.x << "," << config_g[i]->pos.y << "\n";
    }
    log.close();
}