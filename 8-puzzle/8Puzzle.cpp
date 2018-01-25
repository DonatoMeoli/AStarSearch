/**
 * A* Search implementation to solve 8 puzzle.
 *
 * Goal:        Easy:        Medium:      Hard:        Worst:
 *
 * 1 2 3        1 3 4        2 8 1        2 8 1        5 6 7
 * 8   4        8 6 2          4 3        4 6 3        4   8
 * 7 6 5        7   5        7 6 5          7 5        3 2 1
 *
 * Usage: ./8-puzzle {134862705|281043765|281463075|567408321|etc.}
 *
 * Example: ./8-puzzle 281463075
 *
 * @author Donato Meoli
 */

#include "PuzzleState.h"

int main(int argc, char *argv[]) {
    if (argc > 1) {
        int i = 0, c;
        while ((c = argv[1][i])) {
            if (isdigit(c)) {
                int n = (c - '0');
                PuzzleState::start[i] = static_cast<PuzzleState::TILE>(n);
            }
            i++;
        }
    }
    AStarSearch<PuzzleState> aStarSearch;
    PuzzleState startState(PuzzleState::start);
    PuzzleState goalState(PuzzleState::goal);
    aStarSearch.setStartAndGoalStates(startState, goalState);
    unsigned int SearchState;
    unsigned int SearchSteps = 0;
    do {
        SearchState = aStarSearch.searchStep();
        SearchSteps++;
    } while (SearchState == AStarSearch<PuzzleState>::SEARCH_STATE_SEARCHING);
    if (SearchState == AStarSearch<PuzzleState>::SEARCH_STATE_SUCCEEDED) {
        cout << "Search found goal state..." << endl;
        PuzzleState *node = aStarSearch.getSolutionStart();
        cout << "Displaying solution:" << endl;
        int steps = 0;
        node->printNodeInfo();
        cout << endl;
        for ( ; ; ) {
            node = aStarSearch.getSolutionNext();
            if (!node) break;
            node->printNodeInfo();
            cout << endl;
            steps++;
        }
        cout << "Solution steps " << steps << endl;
        node = aStarSearch.getSolutionEnd();
        steps = 0;
        node->printNodeInfo();
        cout << endl;
        for ( ; ; ) {
            node = aStarSearch.getSolutionPrev();
            if (!node) break;
            steps++;
        }
        aStarSearch.freeSolutionNodes();
    } else if (SearchState == AStarSearch<PuzzleState>::SEARCH_STATE_FAILED) {
        cout << "Search terminated. Did not find goal state!" << endl;
    } else if (SearchState == AStarSearch<PuzzleState>::SEARCH_STATE_OUT_OF_MEMORY) {
        cout << "Search terminated. Out of memory!" << endl;
    }
    cout << "SearchSteps: " << aStarSearch.getStepCount() << endl;
    return EXIT_SUCCESS;
}

