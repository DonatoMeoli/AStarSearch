#ifndef PATH_SEARCH_STATE_H
#define PATH_SEARCH_STATE_H

#include <string>
#include <iostream>
#include "../AStarSearch.h"

const int MAX_CITIES = 20;

enum ENUM_CITIES {
    Arad = 0,
    Bucharest,
    Craiova,
    Drobeta,
    Eforie,
    Fagaras,
    Giurgiu,
    Hirsova,
    Iasi,
    Lugoj,
    Mehadia,
    Neamt,
    Oradea,
    Pitesti,
    RimnicuVilcea,
    Sibiu,
    Timisoara,
    Urziceni,
    Vaslui,
    Zerind
};

vector<string> CityNames(MAX_CITIES);

float RomaniaMap[MAX_CITIES][MAX_CITIES];

class PathSearchState : public AStarState<PathSearchState> {

public:

    ENUM_CITIES city;

    PathSearchState();

    explicit PathSearchState(ENUM_CITIES in);

    // Euclidean distance between "this" node city and Bucharest. Numbers are taken from the book.
    float goalDistanceEstimate(PathSearchState &nodeGoal) override;

    bool isGoal(PathSearchState &nodeGoal) override;

    bool getSuccessors(AStarSearch<PathSearchState> *aStarSearch, PathSearchState *parentNode) override;

    float getCost(PathSearchState &successor) override;

    bool isSameState(PathSearchState &rhs) override;

    void printNodeInfo();
};

PathSearchState::PathSearchState() {
    city = Arad;
}

PathSearchState::PathSearchState(ENUM_CITIES in) {
    city = in;
}

float PathSearchState::goalDistanceEstimate(PathSearchState &nodeGoal) {
    switch(city) {
        case Arad:return 366;
        case Bucharest: return 0;
        case Craiova: return 160;
        case Drobeta: return 242;
        case Eforie: return 161;
        case Fagaras: return 176;
        case Giurgiu: return 77;
        case Hirsova: return 151;
        case Iasi: return 226;
        case Lugoj: return 244;
        case Mehadia: return 241;
        case Neamt: return 234;
        case Oradea: return 380;
        case Pitesti: return 100;
        case RimnicuVilcea: return 193;
        case Sibiu: return 253;
        case Timisoara: return 329;
        case Urziceni: return 80;
        case Vaslui: return 199;
        case Zerind: return 374;
        default: return 0.0f;
    }
}

bool PathSearchState::isGoal(PathSearchState &nodeGoal) {
    return city == Bucharest;
}

bool PathSearchState::getSuccessors(AStarSearch<PathSearchState> *aStarSearch, PathSearchState *parentNode) {
    PathSearchState newNode;
    for (int c = 0; c < MAX_CITIES; c++) {
        if (RomaniaMap[city][c] < 0) continue;
        newNode = PathSearchState((ENUM_CITIES)c);
        aStarSearch->addSuccessor(newNode);
    }
    return true;
}

float PathSearchState::getCost(PathSearchState &successor) {
    return RomaniaMap[city][successor.city];
}

bool PathSearchState::isSameState(PathSearchState &rhs) {
    return city == rhs.city;
}

void PathSearchState::printNodeInfo() {
    cout << " " << CityNames[city] << endl;
}

#endif
