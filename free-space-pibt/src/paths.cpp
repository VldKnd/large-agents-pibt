#include <iostream>

#include "../include/paths.hpp"

Paths::Paths(int num_agents) {
    std::vector<Path> tmp(num_agents, Path(0));
    paths = tmp;
    makespan = 0;
}

Path Paths::get(int i) const {
    const int paths_size = paths.size();
    if (!(0 <= i && i < paths_size)) halt("invalid index");
    return paths[i];
}

Node *Paths::get(int i, int t) const {
    if (!(0 <= i && i < (int) paths.size()) || !(0 <= t && t <= makespan)) {
        halt("invalid index, i=" + std::to_string(i) + ", t=" + std::to_string(t));
    }
    return paths[i][t];
}

Node *Paths::last(int i) const { return get(i, makespan); }

bool Paths::empty() const { return paths.empty(); }

bool Paths::empty(const int i) const {
    const int paths_size = paths.size();
    if (!(0 <= i && i < paths_size))
        halt("invalid index, i=" + std::to_string(i));
    return paths[i].empty();
}

void Paths::insert(int i, const Path &path) {
    const int paths_size = paths.size();
    if (!(0 <= i && i < paths_size)) halt("invalid index");
    if (path.empty()) halt("path must not be empty");
    const int old_len = paths[i].size();
    paths[i] = path;
    const int path_size = path.size();
    if (path_size - 1 == getMakespan()) return;
    format();                           // align each path size
    if (path_size < old_len) shrink();  // cutoff additional configs
    makespan = getMaxLengthPaths();     // update makespan
}

void Paths::clear(int i) { paths[i].clear(); }

int Paths::size() const { return paths.size(); }

void Paths::operator+=(const Paths &other) {
    if (other.empty()) return;
    if (paths.size() != other.paths.size()) halt("invalid operation");
    const int paths_size = paths.size();
    if (makespan == 0) {
        for (int i = 0; i < paths_size; ++i) insert(i, other.get(i));
    } else {
        std::vector<Path> new_paths(paths_size);
        for (int i = 0; i < paths_size; ++i) {
            if (paths[i].empty() || *(paths[i].end() - 1) != other.paths[i][0]) {
                halt("invalid operation");
            }
            Path tmp;

            for (int t = 0; t <= getMakespan(); ++t) {
                tmp.push_back(get(i, t));
            }

            for (int t = 1; t <= other.getMakespan(); ++t) {
                tmp.push_back(other.get(i, t));
            }
            new_paths[i] = tmp;
        }
        for (int i = 0; i < paths_size; ++i) insert(i, new_paths[i]);
    }
}

int Paths::getMaxLengthPaths() const {
    int max_val = 0;
    for (auto p: paths) {
        if (p.empty()) continue;
        const int p_size = p.size();
        max_val = (p_size - 1 > max_val) ? p_size - 1 : max_val;
    }
    return max_val;
}

int Paths::getMakespan() const { return makespan; }

int Paths::costOfPath(int i) const {
    const int paths_size = paths.size();
    if (!(0 <= i && i < paths_size)) {
        halt("invalid index " + std::to_string(i));
    }
    return getPathCost(get(i));
}

int Paths::getSOC() const {
    int soc = 0;
    const int paths_size = paths.size();
    for (int i = 0; i < paths_size; ++i) soc += costOfPath(i);
    return soc;
}

void Paths::format() {
    const int paths_size = paths.size();
    int len = getMaxLengthPaths();
    for (int i = 0; i < paths_size; ++i) {
        if (paths[i].empty()) continue;
        int p_size = paths[i].size();
        while (p_size - 1 != len) {
            paths[i].push_back(*(paths[i].end() - 1));
            ++p_size;
        }
    }
}

void Paths::shrink() {
    const int paths_size = paths.size();
    while (true) {
        bool shrinkable = true;
        for (auto p: paths) {
            if (p.size() <= 1 || *(p.end() - 1) != *(p.end() - 2)) {
                shrinkable = false;
                break;
            }
        }
        if (!shrinkable) break;

        for (int i = 0; i < paths_size; ++i) {
            paths[i].resize(paths[i].size() - 1);
        }
    }
}

bool Paths::conflicted(int i, int j, int t) const {
    if (get(i, t) == get(j, t)) return true;
    if (get(i, t) == get(j, t - 1) && get(j, t) == get(i, t - 1)) return true;
    return false;
}

int Paths::countConflict() const {
    int cnt = 0;
    int num_agents = size();
    int makespan = getMakespan();
    for (int i = 0; i < num_agents; ++i) {
        for (int j = i + 1; j < num_agents; ++j) {
            for (int t = 1; t <= makespan; ++t) {
                if (conflicted(i, j, t)) ++cnt;
            }
        }
    }
    return cnt;
}

int Paths::countConflict(const std::vector<int> &sample) const {
    int cnt = 0;
    int makespan = getMakespan();
    const int sample_size = sample.size();
    for (int i = 0; i < sample_size; ++i) {
        for (int j = i + 1; j < sample_size; ++j) {
            for (int t = 1; t <= makespan; ++t) {
                if (conflicted(sample[i], sample[j], t)) ++cnt;
            }
        }
    }
    return cnt;
}

int Paths::countConflict(int id, const Path &path) const {
    int cnt = 0;
    int makespan = getMakespan();
    int num_agents = size();
    const int path_size = path.size();
    for (int i = 0; i < num_agents; ++i) {
        if (i == id) continue;
        for (int t = 1; t < path_size; ++t) {
            if (t > makespan) {
                if (path[t] == get(i, makespan)) {
                    ++cnt;
                    break;
                }
                continue;
            }
            // vertex conflict
            if (get(i, t) == path[t]) {
                ++cnt;
                continue;
            }
            // swap conflict
            if (get(i, t) == path[t - 1] && get(i, t - 1) == path[t]) ++cnt;
        }
    }
    return cnt;
}

void Paths::halt(const std::string &msg) const {
    std::cout << "error@Paths: " << msg << std::endl;
    this->~Paths();
    std::exit(1);
}

void Paths::warn(const std::string &msg) const {
    std::cout << "warn@Paths: " << msg << std::endl;
}

PathsWithRadius::PathsWithRadius(int num_agents) {
    std::vector<PathWithRadius> tmp(num_agents);
    paths = tmp;
    makespan = 0;
}

PathWithRadius PathsWithRadius::get(int i) const {
    const int paths_size = paths.size();
    if (!(0 <= i && i < paths_size)) halt("invalid index");
    return paths[i];
}

NodeWithRadius PathsWithRadius::get(int i, int t) const {
    if (!(0 <= i && i < (int) paths.size()) || !(0 <= t && t <= makespan)) {
        halt("invalid index, i=" + std::to_string(i) + ", t=" + std::to_string(t));
    }
    return paths[i][t];
}

NodeWithRadius PathsWithRadius::last(int i) const { return get(i, makespan); }

bool PathsWithRadius::empty() const { return paths.empty(); }

bool PathsWithRadius::empty(const int i) const {
    const int paths_size = paths.size();
    if (!(0 <= i && i < paths_size))
        halt("invalid index, i=" + std::to_string(i));
    return paths[i].empty();
}

void PathsWithRadius::insert(int i, const PathWithRadius &path) {
    const int paths_size = paths.size();
    if (!(0 <= i && i < paths_size)) halt("invalid index");
    if (path.empty()) halt("path must not be empty");
    const int old_len = paths[i].size();
    paths[i] = path;
    const int path_size = path.size();
    if (path_size - 1 == getMakespan()) return;
    format();                           // align each path size
    if (path_size < old_len) shrink();  // cutoff additional configs
    makespan = getMaxLengthPaths();     // update makespan
}

void PathsWithRadius::clear(int i) { paths[i].clear(); }

int PathsWithRadius::size() const { return paths.size(); }

int PathsWithRadius::getMaxLengthPaths() const {
    int max_val = 0;
    for (auto p: paths) {
        if (p.empty()) continue;
        const int p_size = p.size();
        max_val = (p_size - 1 > max_val) ? p_size - 1 : max_val;
    }
    return max_val;
}

int PathsWithRadius::getMakespan() const { return makespan; }

int PathsWithRadius::getPathWithRadiusCost(const PathWithRadius &path) const {
    int cost = path.size() - 1;
    auto itr = path.end() - 1;
    while (itr != path.begin() && itr->node == (itr - 1)->node) {
        --cost;
        --itr;
    }
    return cost;

}

int PathsWithRadius::costOfPath(int i) const {
    const int paths_size = paths.size();
    if (!(0 <= i && i < paths_size)) {
        halt("invalid index " + std::to_string(i));
    }
    return getPathWithRadiusCost(get(i));
}

int PathsWithRadius::getSOC() const {
    int soc = 0;
    const int paths_size = paths.size();
    for (int i = 0; i < paths_size; ++i) soc += costOfPath(i);
    return soc;
}

void PathsWithRadius::format() {
    const int paths_size = paths.size();
    int len = getMaxLengthPaths();
    for (int i = 0; i < paths_size; ++i) {
        if (paths[i].empty()) continue;
        int p_size = paths[i].size();
        while (p_size - 1 != len) {
            paths[i].push_back(*(paths[i].end() - 1));
            ++p_size;
        }
    }
}

void PathsWithRadius::shrink() {
    const int paths_size = paths.size();
    while (true) {
        bool shrinkable = true;
        for (auto p: paths) {
            if (p.size() <= 1 || (p.end() - 1)->node != (p.end() - 2)->node) {
                shrinkable = false;
                break;
            }
        }
        if (!shrinkable) break;

        for (int i = 0; i < paths_size; ++i) {
            paths[i].resize(paths[i].size() - 1);
        }
    }
}


void PathsWithRadius::halt(const std::string &msg) const {
    std::cout << "error@Paths: " << msg << std::endl;
    this->~PathsWithRadius();
    std::exit(1);
}

void PathsWithRadius::warn(const std::string &msg) const {
    std::cout << "warn@Paths: " << msg << std::endl;
}
