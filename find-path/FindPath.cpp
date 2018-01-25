/**
 * A* Search implementation to find a path on a simple grid maze.
 *
 * @author Donato Meoli
 */

#include "MapSearchState.h"

int main(int argc, char *argv[]) {
    AStarSearch<MapSearchState> aStarSearch;
    unsigned int searchCount = 0;
    const unsigned int numSearches = 1;
    while (searchCount < numSearches) {
        MapSearchState startState(rand() % MapSearchState::MAP_HEIGHT, rand() % MapSearchState::MAP_HEIGHT);
        MapSearchState goalState(rand() % MapSearchState::MAP_WIDTH, rand() % MapSearchState::MAP_HEIGHT);
        aStarSearch.setStartAndGoalStates(startState, goalState);
        unsigned int searchState;
        unsigned int searchSteps = 0;
        do {
            searchState = aStarSearch.searchStep();
            searchSteps++;
        } while (searchState == AStarSearch<MapSearchState>::SEARCH_STATE_SEARCHING);
        if (searchState == AStarSearch<MapSearchState>::SEARCH_STATE_SUCCEEDED) {
            cout << "Search found goal state..." << endl;
            MapSearchState *node = aStarSearch.getSolutionStart();
            cout << "Displaying solution:" << endl;
            int steps = 0;
            node->printNodeInfo();
            for ( ; ; ) {
                node = aStarSearch.getSolutionNext();
                if (!node) break;
                node->printNodeInfo();
                steps++;
            };
            cout << "Solution steps " << steps << endl;
            aStarSearch.freeSolutionNodes();
        } else if (searchState == AStarSearch<MapSearchState>::SEARCH_STATE_FAILED) {
            cout << "Search terminated. Did not find goal state!" << endl;
        }
        cout << "SearchSteps: " << searchSteps << endl;
        searchCount++;
    }
    return EXIT_SUCCESS;
}

