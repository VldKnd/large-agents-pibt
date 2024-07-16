#include <iostream>
#include "../include/graph_utils.hpp"

bool checkIfNodeExistInRadiusOnGrid(Graph* G, int x, int y, float r) {
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