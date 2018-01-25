#ifndef MAP_SEARCH_STATE_H
#define MAP_SEARCH_STATE_H

#include <cmath>
#include <iostream>
#include "../AStarSearch.h"

class MapSearchState : public AStarState<MapSearchState> {

public:

    static const int MAP_WIDTH = 20;
    static const int MAP_HEIGHT = 20;

    int worldMap[MAP_WIDTH * MAP_HEIGHT] = {
            1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   // 00
            1,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,1,   // 01
            1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 02
            1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 03
            1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 04
            1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 05
            1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 06
            1,9,9,9,9,9,9,9,9,1,1,1,9,9,9,9,9,9,9,1,   // 07
            1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 08
            1,9,1,9,9,9,9,9,9,9,1,1,9,9,9,9,9,9,9,1,   // 09
            1,9,1,1,1,1,9,1,1,9,1,1,1,1,1,1,1,1,1,1,   // 10
            1,9,9,9,9,9,1,9,1,9,1,9,9,9,9,9,1,1,1,1,   // 11
            1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 12
            1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 13
            1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 14
            1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 15
            1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 16
            1,1,9,9,9,9,9,9,9,1,1,1,9,9,9,1,9,9,9,9,   // 17
            1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 18
            1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1    // 19
    };

    int getMap(int x, int y);

    MapSearchState();

    MapSearchState(int x, int y);

    // Here's the heuristic function that estimates the distance from a Node to the Goal.
    float goalDistanceEstimate(MapSearchState &nodeGoal) override;

    bool isGoal(MapSearchState &nodeGoal) override;

    /* This generates the successors to the given Node. It uses a helper function called addSuccessor to give the
     * successors to the AStar class. The A* specific initialisation is done for each node internally, so here you just
     * set the state information that is specific to the application. */
    bool getSuccessors(AStarSearch<MapSearchState> *aStarSearch, MapSearchState *parentNode) override;

    /* Given this node, what does it cost to move to successor. In the case of our map the answer is the map terrain
     * value at this node since that is conceptually where we're moving. */
    float getCost(MapSearchState &successor) override;

    bool isSameState(MapSearchState &rhs) override;

    void printNodeInfo();

private:

    int x;
    int y;
};

MapSearchState::MapSearchState() {
    x = y = 0;
}

MapSearchState::MapSearchState(int x, int y) {
    this->x = x;
    this->y = y;
}

float MapSearchState::goalDistanceEstimate(MapSearchState &nodeGoal) {
    return fabsf(x - nodeGoal.x) + fabsf(y - nodeGoal.y);
}

bool MapSearchState::isGoal(MapSearchState &nodeGoal) {
    return x == nodeGoal.x && y == nodeGoal.y;
}

int MapSearchState::getMap(int x, int y) {
    if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) return 9;
    return worldMap[(y * MAP_WIDTH) + x];
}

bool MapSearchState::getSuccessors(AStarSearch<MapSearchState> *aStarSearch, MapSearchState *parentNode) {
    int parentX = -1;
    int parentY = -1;
    if (parentNode) {
        parentX = parentNode->x;
        parentY = parentNode->y;
    }
    MapSearchState newNode;
    if (getMap(x-1, y) < 9 && !(parentX == x-1 && parentY == y)) {
        newNode = MapSearchState(x-1, y);
        aStarSearch->addSuccessor(newNode);
    }
    if (getMap(x, y-1 < 9) && !(parentX == x && parentY == y-1)) {
        newNode = MapSearchState(x, y-1);
        aStarSearch->addSuccessor(newNode);
    }
    if (getMap(x+1, y) < 9 && !(parentX == x+1 && parentY == y)) {
        newNode = MapSearchState(x+1, y);
        aStarSearch->addSuccessor(newNode);
    }
    if (getMap(x, y+1) < 9 && !(parentX == x && parentY == y+1)) {
        newNode = MapSearchState(x, y+1);
        aStarSearch->addSuccessor(newNode);
    }
    return true;
}

float MapSearchState::getCost(MapSearchState &successor) {
    return (float) getMap(x, y);
}

bool MapSearchState::isSameState(MapSearchState &rhs) {
    return x == rhs.x && y == rhs.y;
}

void MapSearchState::printNodeInfo() {
    char str[100];
    sprintf(str, "Node position: (%d,%d)", x, y);
    cout << str << endl;
}

#endif
