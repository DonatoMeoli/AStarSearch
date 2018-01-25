/**
 * A* Search implementation to find the minimum path between two cities given a map. The example is taken from the
 * book AI: A Modern Approach, 3rd Ed., by Russel and Norvig, where a map of Romania is given. The target node is
 * Bucharest, and the user can specify the initial city from which the search algorithm will start looking for the
 * minimum path to Bucharest.
 *
 * Usage: ./min-path-to-Bucharest.o {Arad|Bucharest|Craiova|Drobeta|Eforie|Fagaras|Giurgiu|Hirsova|Iasi|Lugoj|Mehadia|
 *                                  |Neamt|Oradea|Pitesti|RimnicuVilcea|Sibiu|Timisoara|Urziceni|Vaslui|Zerind}
 *
 * Example: ./min-path-to-Bucharest Arad
 *
 * @author Donato Meoli
 */

#include "PathSearchState.h"

int main(int argc, char *argv[]) {
    for (auto &i : RomaniaMap) {
        for (float &j : i) {
            j = static_cast<float>(-1.0);
        }
    }
    RomaniaMap[Arad][Sibiu] = 140;
    RomaniaMap[Arad][Zerind] = 75;
    RomaniaMap[Arad][Timisoara] = 118;
    RomaniaMap[Bucharest][Giurgiu] = 90;
    RomaniaMap[Bucharest][Urziceni] = 85;
    RomaniaMap[Bucharest][Fagaras] = 211;
    RomaniaMap[Bucharest][Pitesti] = 101;
    RomaniaMap[Craiova][Drobeta] = 120;
    RomaniaMap[Craiova][RimnicuVilcea] = 146;
    RomaniaMap[Craiova][Pitesti] = 138;
    RomaniaMap[Drobeta][Craiova] = 120;
    RomaniaMap[Drobeta][Mehadia] = 75;
    RomaniaMap[Eforie][Hirsova] = 75;
    RomaniaMap[Fagaras][Bucharest] = 211;
    RomaniaMap[Fagaras][Sibiu] = 99;
    RomaniaMap[Giurgiu][Bucharest] = 90;
    RomaniaMap[Hirsova][Eforie] = 86;
    RomaniaMap[Hirsova][Urziceni] = 98;
    RomaniaMap[Iasi][Vaslui] = 92;
    RomaniaMap[Iasi][Neamt] = 87;
    RomaniaMap[Lugoj][Timisoara] = 111;
    RomaniaMap[Lugoj][Mehadia] = 70;
    RomaniaMap[Mehadia][Lugoj] = 70;
    RomaniaMap[Mehadia][Drobeta] = 75;
    RomaniaMap[Neamt][Iasi] = 87;
    RomaniaMap[Oradea][Zerind] = 71;
    RomaniaMap[Oradea][Sibiu] = 151;
    RomaniaMap[Pitesti][Bucharest] = 101;
    RomaniaMap[Pitesti][RimnicuVilcea] = 97;
    RomaniaMap[Pitesti][Craiova] = 138;
    RomaniaMap[RimnicuVilcea][Pitesti] = 97;
    RomaniaMap[RimnicuVilcea][Craiova] = 146;
    RomaniaMap[RimnicuVilcea][Sibiu] = 80;
    RomaniaMap[Sibiu][RimnicuVilcea] = 80;
    RomaniaMap[Sibiu][Fagaras] = 99;
    RomaniaMap[Sibiu][Oradea] = 151;
    RomaniaMap[Sibiu][Arad] = 140;
    RomaniaMap[Timisoara][Arad] = 118;
    RomaniaMap[Timisoara][Lugoj] = 111;
    RomaniaMap[Urziceni][Bucharest] = 85;
    RomaniaMap[Urziceni][Hirsova] = 98;
    RomaniaMap[Urziceni][Vaslui] = 142;
    RomaniaMap[Vaslui][Urziceni] = 142;
    RomaniaMap[Vaslui][Iasi] = 92;
    RomaniaMap[Zerind][Arad] = 75;
    RomaniaMap[Zerind][Oradea] = 71;

    CityNames[Arad].assign("Arad");
    CityNames[Bucharest].assign("Bucharest");
    CityNames[Craiova].assign("Craiova");
    CityNames[Drobeta].assign("Drobeta");
    CityNames[Eforie].assign("Eforie");
    CityNames[Fagaras].assign("Fagaras");
    CityNames[Giurgiu].assign("Giurgiu");
    CityNames[Hirsova].assign("Hirsova");
    CityNames[Iasi].assign("Iasi");
    CityNames[Lugoj].assign("Lugoj");
    CityNames[Mehadia].assign("Mehadia");
    CityNames[Neamt].assign("Neamt");
    CityNames[Oradea].assign("Oradea");
    CityNames[Pitesti].assign("Pitesti");
    CityNames[RimnicuVilcea].assign("RimnicuVilcea");
    CityNames[Sibiu].assign("Sibiu");
    CityNames[Timisoara].assign("Timisoara");
    CityNames[Urziceni].assign("Urziceni");
    CityNames[Vaslui].assign("Vaslui");
    CityNames[Zerind].assign("Zerind");

    ENUM_CITIES initCity = Arad;
    if (argc == 2) {
        bool found = false;
        for (size_t i = 0; i < CityNames.size(); i++) {
            if (CityNames[i] == argv[1]) {
                initCity = (ENUM_CITIES) i;
                found = true;
                break;
            }
        }
        if (!found) {
            cout << "There is no city named " << argv[1] << " in the map!" << endl;
            return EXIT_FAILURE;
        }
    }
    AStarSearch<PathSearchState> aStarSearch;
    PathSearchState nodeStart;
    nodeStart.city = initCity;
    PathSearchState nodeEnd;
    nodeEnd.city = Bucharest;
    aStarSearch.setStartAndGoalStates(nodeStart, nodeEnd);
    unsigned int SearchState;
    unsigned int SearchSteps = 0;
    do {
        SearchState = aStarSearch.searchStep();
        SearchSteps++;
    } while (SearchState == AStarSearch<PathSearchState>::SEARCH_STATE_SEARCHING);
    if (SearchState == AStarSearch<PathSearchState>::SEARCH_STATE_SUCCEEDED) {
        cout << "Search found the goal state..." << endl;
        PathSearchState *node = aStarSearch.getSolutionStart();
        cout << "Displaying solution:" << endl;
        int steps = 0;
        node->printNodeInfo();
        for ( ; ; ) {
            node = aStarSearch.getSolutionNext();
            if (!node) break;
            node->printNodeInfo();
            steps ++;
        }
        cout << "Solution steps " << steps << endl;
        aStarSearch.freeSolutionNodes();
    } else if (SearchState == AStarSearch<PathSearchState>::SEARCH_STATE_FAILED) {
        cout << "Search terminated. Did not find goal state!" << endl;
    }
    cout << "SearchSteps: " << SearchSteps << endl;
    return EXIT_SUCCESS;
}

