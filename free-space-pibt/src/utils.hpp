#include <iostream>
#include "mapf_solver.hpp"
#include "mapf_problem.hpp"

using Time = std::chrono::steady_clock;

template <typename T>
static bool inArray(const T a, const std::vector<T> &arr)
{
    auto itr = std::find(arr.begin(), arr.end(), a);
    return itr != arr.end();
}

[[maybe_unused]] static bool getRandomBoolean(std::mt19937 *const MT)
{
    std::uniform_int_distribution<int> r(0, 1);
    return r(*MT);
}

[[maybe_unused]] static int getRandomInt(int from, int to,
                                         std::mt19937 *const MT)
{
    std::uniform_int_distribution<int> r(from, to);
    return r(*MT);
}

[[maybe_unused]] static float getRandomFloat(float from, float to,
                                             std::mt19937 *const MT)
{
    std::uniform_real_distribution<float> r(from, to);
    return r(*MT);
}

template <typename T>
static T randomChoose(const std::vector<T> &arr, std::mt19937 *const MT)
{
    return arr[getRandomInt(0, arr.size() - 1, MT)];
}

[[maybe_unused]] static double getElapsedTime(const Time::time_point &t_start)
{
    auto t_end = Time::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
        .count();
}

bool checkIfNodeExistInRadiusOnGrid(Grid* G, int x, int y, float r) {
    int dx = r;
    int dy = 0;

    if (dx == 0) {
        return G->existNode(x, y);
    }

    do {
        if (!G->existNode(x + dx, y + dy) ||
            !G->existNode(x - dy, y + dx) ||
            !G->existNode(x - dx, y - dy) ||
            !G->existNode(x + dy, y - dx) )  {
            return false;
        }
        if (dx*dx + (dy+1)*(dy+1) <= r*r ) ++dy;
        else --dx;
    } while (dx != 0);

    return true;
};