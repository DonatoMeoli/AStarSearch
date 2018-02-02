/**
 * Artificial Intelligence: A Modern Approach, 3rd Edition, by Russel and Norvig.
 *
 * The most widely known form of best-first search is called A* Search. It evaluates nodes by combining g(n), the cost
 * to reach the node, and h(n), the cost to get from the node to the goal: f(n) = g(n) + h(n).
 * Since g(n) gives the path cost from the start node to node n, and h(n) is the estimated cost of the cheapest path
 * from n to the goal, we have f(n) = estimated cost of the cheapest solution through n.
 *
 * @author Donato Meoli
 */

#ifndef A_STAR_SEARCH_H
#define A_STAR_SEARCH_H

#include <vector>
#include <algorithm>

using namespace std;

template <class AStarState>
class AStarSearch {

public:

    enum {
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

        AStarState aStarState;

        Node();
    };

    class HeapCompare {
    public:
        bool operator()(const Node *x, const Node *y) const;
    };

    AStarSearch();

    void setStartAndGoalStates(AStarState &startState, AStarState &goalState);

    unsigned int searchStep();

    bool addSuccessor(AStarState &state);

    void freeSolutionNodes();

    AStarState *getSolutionStart();
    AStarState *getSolutionNext();
    AStarState *getSolutionEnd();
    AStarState *getSolutionPrev();

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

    Node *allocateNode();

    void freeNode(Node *node);
    void freeUnusedNodes();
    void freeAllNodes();
};

template <class AStarState>
AStarSearch<AStarState>::Node::Node() {
    parent = nullptr;
    child = nullptr;
    g = 0.0f;
    h = 0.0f;
    h = 0.0f;
}

template <class AStarState>
bool AStarSearch<AStarState>::HeapCompare::operator()(const Node *x, const Node *y) const {
    return x->f > y->f;
}

template <class AStarState>
AStarSearch<AStarState>::AStarSearch() {
    currentSolutionNode = nullptr;
}

template <class AStarState>
void AStarSearch<AStarState>::setStartAndGoalStates(AStarState &startState, AStarState &goalState) {
    start = allocateNode();
    goal = allocateNode();
    start->aStarState = startState;
    goal->aStarState = goalState;
    state = SEARCH_STATE_SEARCHING;
    start->g = 0.0f;
    start->h = start->aStarState.goalDistanceEstimate(goal->aStarState);
    start->f = start->g + start->h;
    start->parent = nullptr;
    openList.push_back(start);
    push_heap(openList.begin(), openList.end(), HeapCompare());
    steps = 0;
}

template <class AStarState>
unsigned int AStarSearch<AStarState>::searchStep() {
    if (state == SEARCH_STATE_SUCCEEDED || state == SEARCH_STATE_FAILED) return state;
    if (openList.empty()) {
        freeAllNodes();
        state = SEARCH_STATE_FAILED;
        return state;
    }
    steps++;
    Node *first = openList.front();
    pop_heap(openList.begin(), openList.end(), HeapCompare());
    openList.pop_back();
    if (first->aStarState.isGoal(goal->aStarState)) {
        goal->parent = first->parent;
        goal->g = first->g;
        if (!first->aStarState.isSameState(start->aStarState)) {
            freeNode(first);
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
        if (!first->aStarState.getSuccessors(this, first->parent ? &first->parent->aStarState : nullptr)) {
            typename vector<Node*>::iterator iterSucc;
            for (iterSucc = successors.begin(); iterSucc != successors.end(); iterSucc++) freeNode(*iterSucc);
            successors.clear();
            freeAllNodes();
            state = SEARCH_STATE_OUT_OF_MEMORY;
            return state;
        }
        typename vector<Node*>::iterator iterSucc;
        for (iterSucc = successors.begin(); iterSucc != successors.end(); iterSucc++) {
            float g = first->g + first->aStarState.getCost((*iterSucc)->aStarState);
            typename vector<Node*>::iterator iterOpen;
            for (iterOpen = openList.begin(); iterOpen != openList.end(); iterOpen++) {
                if ((*iterOpen)->aStarState.isSameState((*iterSucc)->aStarState)) break;
            }
            if (iterOpen != openList.end()) {
                if ((*iterOpen)->g <= g) {
                    freeNode(*iterSucc);
                    continue;
                }
            }
            typename vector<Node*>::iterator iterClosed;
            for (iterClosed = closedList.begin(); iterClosed != closedList.end(); iterClosed++) {
                if ((*iterClosed)->aStarState.isSameState((*iterSucc)->aStarState)) break;
            }
            if (iterClosed != closedList.end()) {
                if ((*iterClosed)->g <= g) {
                    freeNode(*iterSucc);
                    continue;
                }
            }
            (*iterSucc)->parent = first;
            (*iterSucc)->g = g;
            (*iterSucc)->h = (*iterSucc)->aStarState.goalDistanceEstimate(goal->aStarState);
            (*iterSucc)->f = (*iterSucc)->g + (*iterSucc)->h;
            if (iterClosed != closedList.end()) {
                freeNode(*iterClosed);
                closedList.erase(iterClosed);
            }
            if (iterOpen != openList.end()) {
                freeNode(*iterOpen);
                openList.erase(iterOpen);
                make_heap(openList.begin(), openList.end(), HeapCompare());
            }
            openList.push_back(*iterSucc);
            push_heap(openList.begin(), openList.end(), HeapCompare());
        }
        closedList.push_back(first);
    }
    return state;
}

template <class AStarState>
bool AStarSearch<AStarState>::addSuccessor(AStarState &state) {
    Node *node = allocateNode();
    if (node) {
        node->aStarState = state;
        successors.push_back(node);
        return true;
    }
    return false;
}

template <class AStarState>
void AStarSearch<AStarState>::freeSolutionNodes() {
    Node *node = start;
    if (start->child) {
        do {
            Node *del = node;
            node = node->child;
            freeNode(del);
        } while (node != goal);
        freeNode(node);
    } else {
        freeNode(start);
        freeNode(goal);
    }
}

template <class AStarState>
AStarState* AStarSearch<AStarState>::getSolutionStart() {
    currentSolutionNode = start;
    if (start) {
        return &start->aStarState;
    } else {
        return nullptr;
    }
}

template <class AStarState>
AStarState* AStarSearch<AStarState>::getSolutionNext() {
    if (currentSolutionNode) {
        if (currentSolutionNode->child) {
            Node *child = currentSolutionNode->child;
            currentSolutionNode = currentSolutionNode->child;
            return &child->aStarState;
        }
    }
    return nullptr;
}

template <class AStarState>
AStarState* AStarSearch<AStarState>::getSolutionEnd() {
    currentSolutionNode = goal;
    if (goal) {
        return &goal->aStarState;
    } else {
        return nullptr;
    }
}

template <class AStarState>
AStarState* AStarSearch<AStarState>::getSolutionPrev() {
    if (currentSolutionNode) {
        if (currentSolutionNode->parent) {
            Node *parent = currentSolutionNode->parent;
            currentSolutionNode = currentSolutionNode->parent;
            return &parent->aStarState;
        }
    }
    return nullptr;
}

template <class AStarState>
int AStarSearch<AStarState>::getStepCount() {
    return steps;
}

template <class AStarState>
typename AStarSearch<AStarState>::Node* AStarSearch<AStarState>::allocateNode() {
    return new Node;
}

template <class AStarState>
void AStarSearch<AStarState>::freeNode(Node *node) {
    delete node;
}

template <class AStarState>
void AStarSearch<AStarState>::freeUnusedNodes() {
    typename vector<Node*>::iterator iterOpen;
    for (iterOpen = openList.begin(); iterOpen != openList.end(); iterOpen++) {
        if (!(*iterOpen)->child) {
            freeNode(*iterOpen);
        }
    }
    openList.clear();
    typename vector<Node*>::iterator iterClosed;
    for (iterClosed = closedList.begin(); iterClosed != closedList.end(); iterClosed++) {
        if (!(*iterClosed)->child) {
            freeNode(*iterClosed);
        }
    }
    closedList.clear();
}

template <class AStarState>
void AStarSearch<AStarState>::freeAllNodes() {
    typename vector<Node*>::iterator iterOpen;
    for (iterOpen = openList.begin(); iterOpen != openList.end(); iterOpen++) {
        freeNode(*iterOpen);
    }
    openList.clear();
    typename vector<Node*>::iterator iterClosed;
    for (iterClosed = closedList.begin(); iterClosed != closedList.end(); iterClosed++) {
        freeNode(*iterClosed);
    }
    closedList.clear();
    freeNode(goal);
}

#endif