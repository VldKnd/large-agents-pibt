#include <iostream>
#include "../include/graph_utils.hpp"

bool checkIfNodeExistInRadiusOnGrid(Graph* G, int x, int y, float s) {
    int size = floor(s);

    if (size == 0) {
        return G->existNode(x, y);
    }

    for (int delta = 0; delta <= size; delta ++) {
        if (!G->existNode(x + delta, y) ||
            !G->existNode(x + delta, y + size) ||
            !G->existNode(x , y + delta) ||
            !G->existNode(x + size, y + delta)
            )  {
            return false;
        }
    }
    
    return true;
};