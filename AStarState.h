#ifndef A_STAR_STATE_H
#define A_STAR_STATE_H

#include "AStarSearch.h"

template <class S>
class AStarState {

public:

    // Heuristic function which computes the estimated cost to the goal node.
    virtual float goalDistanceEstimate(S &nodeGoal) = 0;

    // Returns true if this node is the goal node.
    virtual bool isGoal(S &nodeGoal) = 0;

    // Retrieves all successors to this node and adds them via aStarSearch.addSuccessor().
    virtual bool getSuccessors(AStarSearch<S> *aStarSearch, S *parentNode) = 0;

    // Computes the cost of travelling from this node to the successor node.
    virtual float getCost(S &successor) = 0;

    // Returns true if this node is the same as the rhs node.
    virtual bool isSameState(S &rhs) = 0;
};

#endif
