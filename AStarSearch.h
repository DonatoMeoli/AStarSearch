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
#include <cassert>

using namespace std;

template <class AStarState>
class AStarSearch {

public:

    enum {
        SEARCH_STATE_NOT_INITIALISED,
        SEARCH_STATE_SEARCHING,
        SEARCH_STATE_SUCCEEDED,
        SEARCH_STATE_FAILED,
        SEARCH_STATE_OUT_OF_MEMORY,
        SEARCH_STATE_INVALID
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

    void setStartAndGoalStates(AStarState &start, AStarState &goal);

    unsigned int searchStep();

    bool addSuccessor(AStarState &state);

    void cancelSearch();

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

    int allocateNodeCount;
    bool cancelRequest;

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
    state = SEARCH_STATE_NOT_INITIALISED;
    currentSolutionNode = nullptr;
    allocateNodeCount = 0;
    cancelRequest = false;
}

template <class AStarState>
void AStarSearch<AStarState>::setStartAndGoalStates(AStarState &start, AStarState &goal) {

    cancelRequest = false;

    this->start = allocateNode();
    this->goal = allocateNode();

    assert(start != nullptr && goal != nullptr);
    
    this->start->aStarState = start;
    this->goal->aStarState = goal;

    state = SEARCH_STATE_SEARCHING;

    this->start->g = 0.0f;
    this->start->h = this->start->aStarState.goalDistanceEstimate(this->goal->aStarState);
    this->start->f = this->start->g + this->start->h;
    this->start->parent = nullptr;

    openList.push_back(this->start);
    push_heap(openList.begin(), openList.end(), HeapCompare());

    steps = 0;
}

template <class AStarState>
unsigned int AStarSearch<AStarState>::searchStep() {

    assert(state > SEARCH_STATE_NOT_INITIALISED && state < SEARCH_STATE_INVALID);

    if (state == SEARCH_STATE_SUCCEEDED || state == SEARCH_STATE_FAILED) {
        return state;
    }

    if (openList.empty() || cancelRequest) {
        freeAllNodes();
        state = SEARCH_STATE_FAILED;
        return state;
    }

    steps++;

    Node *node = openList.front();
    pop_heap(openList.begin(), openList.end(), HeapCompare());
    openList.pop_back();

    if (node->aStarState.isGoal(goal->aStarState)) {

        goal->parent = node->parent;
        goal->g = node->g;

        if (!node->aStarState.isSameState(start->aStarState)) {
            freeNode(node);

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

        if (!node->aStarState.getSuccessors(this, node->parent ? &node->parent->aStarState : nullptr)) {
            typename vector<Node*>::iterator iterSucc;
            for (iterSucc = successors.begin(); iterSucc != successors.end(); iterSucc++) {
                freeNode(*iterSucc);
            }

            successors.clear();
            freeAllNodes();
            state = SEARCH_STATE_OUT_OF_MEMORY;
            return state;
        }

        typename vector<Node*>::iterator iterSucc;
        for (iterSucc = successors.begin(); iterSucc != successors.end(); iterSucc++) {
            float g = node->g + node->aStarState.getCost((*iterSucc)->aStarState);

            typename vector<Node*>::iterator iterOpen;
            for (iterOpen = openList.begin(); iterOpen != openList.end(); iterOpen++) {
                if ((*iterOpen)->aStarState.isSameState((*iterSucc)->aStarState)) {
                    break;
                }
            }
            if (iterOpen != openList.end()) {
                if ((*iterOpen)->g <= g) {
                    freeNode(*iterSucc);
                    continue;
                }
            }

            typename vector<Node*>::iterator iterClosed;
            for (iterClosed = closedList.begin(); iterClosed != closedList.end(); iterClosed++) {
                if ((*iterClosed)->aStarState.isSameState((*iterSucc)->aStarState)) {
                    break;
                }
            }
            if (iterClosed != closedList.end()) {
                if ((*iterClosed)->g <= g) {
                    freeNode(*iterSucc);
                    continue;
                }
            }

            (*iterSucc)->parent = node;
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
        closedList.push_back(node);
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
void AStarSearch<AStarState>::cancelSearch() {
    cancelRequest = true;
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
    Node *node = new Node;
    return node;
}

template <class AStarState>
void AStarSearch<AStarState>::freeNode(Node *node) {
    allocateNodeCount--;
    delete node;
}

template <class AStarState>
void AStarSearch<AStarState>::freeUnusedNodes() {
    typename vector<Node*>::iterator iterOpen;
    for (iterOpen = openList.begin(); iterOpen != openList.end(); iterOpen++) {
        Node *node = *iterOpen;
        if (!node->child) {
            freeNode(node);
        }
    }
    openList.clear();
    typename vector<Node*>::iterator iterClosed;
    for (iterClosed = closedList.begin(); iterClosed != closedList.end(); iterClosed++) {
        Node *node = *iterClosed;
        if (!node->child) {
            freeNode(node);
        }
    }
    closedList.clear();
}

template <class AStarState>
void AStarSearch<AStarState>::freeAllNodes() {
    typename vector<Node*>::iterator iterOpen;
    for (iterOpen = openList.begin(); iterOpen != openList.end(); iterOpen++) {
        Node *node = *iterOpen;
        freeNode(node);
    }
    openList.clear();
    typename vector<Node*>::iterator iterClosed;
    for (iterClosed = closedList.begin(); iterClosed != closedList.end(); iterClosed++) {
        Node *node = *iterClosed;
        freeNode(node);
    }
    closedList.clear();
    freeNode(goal);
}

#endif