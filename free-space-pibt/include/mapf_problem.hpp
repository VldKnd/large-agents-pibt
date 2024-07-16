#pragma once
#include <graph.hpp>
#include <random>

#include "default_params.hpp"
#include "utils.hpp"

using Config = std::vector<Node *>; // < loc_0[t], loc_1[t], ... >
using Configs = std::vector<Config>;

// check two configurations are same or not
[[maybe_unused]] static bool sameConfig(const Config &config_i,
                                        const Config &config_j)
{
  if (config_i.size() != config_j.size())
    return false;
  const int size_i = config_i.size();
  for (int k = 0; k < size_i; ++k)
  {
    if (config_i[k] != config_j[k])
      return false;
  }
  return true;
}

[[maybe_unused]] static int getPathCost(const Path &path)
{
  int cost = path.size() - 1;
  auto itr = path.end() - 1;
  while (itr != path.begin() && *itr == *(itr - 1))
  {
    --cost;
    --itr;
  }
  return cost;
}

class MapfProblem
{
protected:
  std::string instance; // instance name
  Graph *G;             // graph
  std::mt19937 *MT;     // seed
  Config config_s;      // initial configuration
  Config config_g;      // goal configuration
  int num_agents;       // number of agents
  int max_timestep;     // timestep limit
  int max_comp_time;    // comp_time limit, ms

  // utilities
  void halt(const std::string &msg) const;
  void warn(const std::string &msg) const;

public:
  MapfProblem() {};
  MapfProblem(const std::string &_instance) : instance(_instance) {}
  MapfProblem(std::string _instance, Graph *_G, std::mt19937 *_MT, Config _config_s,
              Config _config_g, int _num_agents, int _max_timestep,
              int _max_comp_time);
  ~MapfProblem() {};

  Graph *getG() { return G; }
  int getNum() { return num_agents; }
  std::mt19937 *getMT() { return MT; }
  Node *getStart(int i) const; // return start of a_i
  Node *getGoal(int i) const;  // return  goal of a_i
  Config getConfigStart() const { return config_s; };
  Config getConfigGoal() const { return config_g; };
  int getMaxTimestep() { return max_timestep; };
  int getMaxCompTime() { return max_comp_time; };
  std::string getInstanceFileName() { return instance; };

  void setMaxCompTime(const int t) { max_comp_time = t; }
};

class FreeSpaceMapfProblem : public MapfProblem
{
private:
  const bool instance_initialized; // for memory manage
  std::vector<float> radiuses;     // To collect radiuses of robots

  // set starts and goals randomly
  void setRandomStartsGoals();
  void setRandomStarts();
  void setRandomGoals();
  void setWellFormedStarts();
  void setWellFormedGoals();
  // set well-formed instance
  void setWellFormedInstance();

public:
  std::vector<float> getRadiuses() { return radiuses; }
  float getRadius(int i) { return radiuses[i]; }
  FreeSpaceMapfProblem(const std::string &_instance);
  FreeSpaceMapfProblem(FreeSpaceMapfProblem *P, Config _config_s, Config _config_g,
                       int _max_comp_time, int _max_timestep, std::vector<float> *_radiuses);
  FreeSpaceMapfProblem(FreeSpaceMapfProblem *P, int _max_comp_time);
  ~FreeSpaceMapfProblem();

  bool isInitializedInstance() const { return instance_initialized; }

  // used when making new instance file
  void makeScenFile(const std::string &output_file);
  bool isInCollision(Config *C, int id, float r);
  bool isInCollision(Config *C, int x, int y, float r);
};