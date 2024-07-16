#include <iostream>
#include "mapf_solver.hpp"
#include "mapf_problem.hpp"

using Time = std::chrono::steady_clock;

template <typename T>
static bool inArray(const T a, const std::vector<T> &arr);

[[maybe_unused]] static bool getRandomBoolean(std::mt19937 *const MT);

[[maybe_unused]] static int getRandomInt(int from, int to,
                                         std::mt19937 *const MT);

[[maybe_unused]] static float getRandomFloat(float from, float to,
                                             std::mt19937 *const MT);

template <typename T>
static T randomChoose(const std::vector<T> &arr, std::mt19937 *const MT);

[[maybe_unused]] static double getElapsedTime(const Time::time_point &t_start);
bool checkIfNodeExistInRadiusOnGrid(Graph *G, int x, int y, float r);