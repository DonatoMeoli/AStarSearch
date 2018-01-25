#ifndef A_STAR_SEARCH_H
#define A_STAR_SEARCH_H

#include <algorithm>
#include <vector>

using namespace std;

template <class State>
class AStarSearch {

public:

    enum {
        SEARCH_STATE_NOT_INITIALISED,
        SEARCH_STATE_SEARCHING,
        SEARCH_STATE_SUCCEEDED,
        SEARCH_STATE_FAILED,
        SEARCH_STATE_OUT_OF_MEMORY
    };

    class Node {

    public:

        Node *parent;
        Node *child;

        float g;
        float h;
        float f;

        State state;

        Node();
    };

    class HeapCompare {
    public:
        bool operator()(const Node *x, const Node *y) const;
    };

    AStarSearch();

    void setStartAndGoalStates(State &start, State &goal);

    unsigned int searchStep();

    bool addSuccessor(State &state);

    void freeSolutionNodes();

    State *getSolutionStart();
    State *getSolutionNext();
    State *getSolutionEnd();
    State *getSolutionPrev();

    int getStepCount();

private:

    vector<Node*> openList;
    vector<Node*> closedList;
    vector<Node*> successors;

    unsigned int state;
    int steps;

    Node *start;
    Node *goal;
    Node *currentSolutionNode;

    int allocateNodeCount;
    bool cancelRequest;

    Node *allocateNode();

    void freeNode(Node *node);
    void freeAllNodes();
    void freeUnusedNodes();
};

template <class State>
class AStarState {
public:
    // Heuristic function which computes the estimated cost to the goal node.
    virtual float goalDistanceEstimate(State &nodeGoal) = 0;
    // Returns true if this node is the goal node.
    virtual bool isGoal(State &nodeGoal) = 0;
    // Retrieves all successors to this node and adds them via aStarSearch.addSuccessor().
    virtual bool getSuccessors(AStarSearch<State> *aStarSearch, State *parentNode) = 0;
    // Computes the cost of travelling from this node to the successor node.
    virtual float getCost(State &successor) = 0;
    // Returns true if this node is the same as the rhs node.
    virtual bool isSameState(State &rhs) = 0;
};

template <class State>
AStarSearch<State>::Node::Node() {
    parent = nullptr;
    child = nullptr;
    g = 0.0f;
    h = 0.0f;
    h = 0.0f;
}

template <class State>
bool AStarSearch<State>::HeapCompare::operator()(const Node *x, const Node *y) const {
    return x->f > y->f;
}


template <class State>
AStarSearch<State>::AStarSearch() {
    state = SEARCH_STATE_NOT_INITIALISED;
    currentSolutionNode = nullptr;
    allocateNodeCount = 0;
    cancelRequest = false;
}

template <class State>
void AStarSearch<State>::setStartAndGoalStates(State &start, State &goal) {
    cancelRequest = false;
    this->start = allocateNode();
    this->goal = allocateNode();
    this->start->state = start;
    this->goal->state = goal;
    state = SEARCH_STATE_SEARCHING;
    this->start->g = 0;
    this->start->h = this->start->state.goalDistanceEstimate(this->goal->state);
    this->start->f = this->start->g + this->start->h;
    this->start->parent = 0;
    openList.push_back(this->start);
    push_heap(openList.begin(), openList.end(), HeapCompare());
    steps = 0;
}

template <class State>
unsigned int AStarSearch<State>::searchStep() {
    if (state == SEARCH_STATE_SUCCEEDED || state == SEARCH_STATE_FAILED) {
        return state;
    }
    if (openList.empty() || cancelRequest) {
        freeAllNodes();
        state = SEARCH_STATE_FAILED;
        return state;
    }
    steps++;
    Node *n = openList.front();
    pop_heap(openList.begin(), openList.end(), HeapCompare());
    openList.pop_back();
    if (n->state.isGoal(goal->state)) {
        goal->parent = n->parent;
        goal->g = n->g;
        if (n->state.isSameState(start->state) == false) {
            freeNode(n);
            Node *nodeChild = goal;
            Node *nodeParent = goal->parent;
            do {
                nodeParent->child = nodeChild;
                nodeChild = nodeParent;
                nodeParent = nodeParent->parent;
            } while (nodeChild != start);
        }
        freeUnusedNodes();
        state = SEARCH_STATE_SUCCEEDED;
        return state;
    } else {
        successors.clear();
        bool ret = n->state.getSuccessors(this, n->parent ? &n->parent->state : nullptr);
        if (!ret) {
            typename vector< Node * >::iterator successor;
            for (successor = successors.begin(); successor != successors.end(); successor++) {
                freeNode(*successor);
            }
            successors.clear();
            freeAllNodes();
            state = SEARCH_STATE_OUT_OF_MEMORY;
            return state;
        }
        typename vector<Node*>::iterator successor;
        for (successor = successors.begin(); successor != successors.end(); successor++) {
            float newG = n->g + n->state.getCost((*successor)->state);
            typename vector<Node*>::iterator openListResult;
            for (openListResult = openList.begin(); openListResult != openList.end(); openListResult++) {
                if ((*openListResult)->state.isSameState((*successor)->state)) {
                    break;
                }
            }
            if (openListResult != openList.end()) {
                if ((*openListResult)->g <= newG) {
                    freeNode(*successor);
                    continue;
                }
            }
            typename vector<Node*>::iterator closedListResult;
            for (closedListResult = closedList.begin(); closedListResult != closedList.end(); closedListResult++) {
                if ((*closedListResult)->state.isSameState((*successor)->state)) {
                    break;
                }
            }
            if (closedListResult != closedList.end()) {
                if ((*closedListResult)->g <= newG) {
                    freeNode(*successor);
                    continue;
                }
            }
            (*successor)->parent = n;
            (*successor)->g = newG;
            (*successor)->h = (*successor)->state.goalDistanceEstimate(goal->state);
            (*successor)->f = (*successor)->g + (*successor)->h;
            if (closedListResult != closedList.end()) {
                freeNode(*closedListResult);
                closedList.erase(closedListResult);
            }
            if (openListResult != openList.end()) {
                freeNode(*openListResult);
                openList.erase(openListResult);
                make_heap(openList.begin(), openList.end(), HeapCompare());
            }
            openList.push_back(*successor);
            push_heap(openList.begin(), openList.end(), HeapCompare());
        }
        closedList.push_back(n);
    }
    return state;
}

template <class State>
bool AStarSearch<State>::addSuccessor(State &state) {
    Node *node = allocateNode();
    if (node) {
        node->state = state;
        successors.push_back(node);
        return true;
    }
    return false;
}

template <class State>
void AStarSearch<State>::freeSolutionNodes() {
    Node *n = start;
    if (start->child) {
        do {
            Node *del = n;
            n = n->child;
            freeNode(del);
        } while (n != goal);
        freeNode(n);
    } else {
        freeNode(start);
        freeNode(goal);
    }
}

template <class State>
State* AStarSearch<State>::getSolutionStart() {
    currentSolutionNode = start;
    if (start) {
        return &start->state;
    } else {
        return nullptr;
    }
}

template <class State>
State* AStarSearch<State>::getSolutionNext() {
    if (currentSolutionNode) {
        if (currentSolutionNode->child) {
            Node *child = currentSolutionNode->child;
            currentSolutionNode = currentSolutionNode->child;
            return &child->state;
        }
    }
    return nullptr;
}

template <class State>
State* AStarSearch<State>::getSolutionEnd() {
    currentSolutionNode = goal;
    if (goal) {
        return &goal->state;
    } else {
        return nullptr;
    }
}

template <class State>
State* AStarSearch<State>::getSolutionPrev() {
    if (currentSolutionNode) {
        if (currentSolutionNode->parent) {
            Node *parent = currentSolutionNode->parent;
            currentSolutionNode = currentSolutionNode->parent;
            return &parent->state;
        }
    }
    return nullptr;
}

template <class State>
int AStarSearch<State>::getStepCount() {
    return steps;
}

template <class State>
typename AStarSearch<State>::Node* AStarSearch<State>::allocateNode() {
    auto *p = new Node;
    return p;
}

template <class State>
void AStarSearch<State>::freeNode(Node *node) {
    allocateNodeCount--;
    delete node;
}

template <class State>
void AStarSearch<State>::freeAllNodes() {
    typename vector<Node*>::iterator iterOpen = openList.begin();
    while (iterOpen != openList.end()) {
        Node *n = *iterOpen;
        freeNode(n);
        iterOpen++;
    }
    openList.clear();
    typename vector<Node*>::iterator iterClosed;
    for (iterClosed = closedList.begin(); iterClosed != closedList.end(); iterClosed++) {
        Node *n = *iterClosed;
        freeNode(n);
    }
    closedList.clear();
    freeNode(goal);
}

template <class State>
void AStarSearch<State>::freeUnusedNodes() {
    typename vector<Node*>::iterator iterOpen = openList.begin();
    while (iterOpen != openList.end()) {
        Node *n = *iterOpen;
        if (!n->child) {
            freeNode(n);
        }
        iterOpen++;
    }
    openList.clear();
    typename vector<Node*>::iterator iterClosed;
    for (iterClosed = closedList.begin(); iterClosed != closedList.end(); iterClosed++) {
        Node *n = *iterClosed;
        if (!n->child) {
            freeNode(n);
        }
    }
    closedList.clear();
}

#endif