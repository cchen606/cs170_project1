/*
Clement Chen, cchen606@ucr.edu 862321584
cs 170 Project 1
Resources used: cplusplus.com, geeksforgeeks.org, w3schools.com, cppreference.com, stackoverflow.com
*/


#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <cmath>
#include <algorithm>
#include <iomanip>

using namespace std;

class Node {
public:
    vector<vector<int>> state;
    Node* parent;
    int g; // Cost to reach this node
    int h; // Heuristic cost
    int depth;

    Node(vector<vector<int>> s, Node* p = nullptr) : state(s), parent(p), g(0), h(0), depth(0) {}

    // Calculate f(n) = g(n) + h(n)
    int f() const {
        return g + h;
    }

    // Operators for priority queue to sort nodes based on f(n)
    bool operator>(const Node& other) const {
        return f() > other.f();
    }
};

class Problem {
public:
    vector<vector<int>> initial_state;
    vector<vector<int>> goal_state;

    Problem(vector<vector<int>> init, vector<vector<int>> goal) : initial_state(init), goal_state(goal) {}

    // Check if the state is the goal state
    bool isGoal(const vector<vector<int>>& state) const {
        return state == goal_state;
    }

    // Generate all possible moves from the current state
    vector<Node> getSuccessors(const Node& node) {
        vector<Node> successors;
        int blank_row, blank_col;

        // Find the position of the blank (0)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (node.state[i][j] == 0) {
                    blank_row = i;
                    blank_col = j;
                }
            }
        }

        // Define possible moves (up, down, left, right)
        vector<pair<int, int>> moves = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1} // up, down, left, right
        };

        for (auto move : moves) {
            int new_row = blank_row + move.first;
            int new_col = blank_col + move.second;

            // Check if the new position is within bounds
            if (new_row >= 0 && new_row < 3 && new_col >= 0 && new_col < 3) {
                // Create a new state
                vector<vector<int>> new_state = node.state;
                swap(new_state[blank_row][blank_col], new_state[new_row][new_col]);
                successors.emplace_back(new_state, new Node(node));
            }
        }
        return successors;
    }

    // Misplaced Tile heuristic
    int misplacedTiles(const vector<vector<int>>& state) {
        int count = 0;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (state[i][j] != 0 && state[i][j] != goal_state[i][j]) {
                    ++count;
                }
            }
        }
        return count;
    }

    // Euclidean Distance heuristic
    double euclideanDistance(const vector<vector<int>>& state) {
        double distance = 0.0;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (state[i][j] != 0) {
                    int goal_x = (state[i][j] - 1) / 3;
                    int goal_y = (state[i][j] - 1) % 3;
                    distance += sqrt(pow(i - goal_x, 2) + pow(j - goal_y, 2));
                }
            }
        }
        return distance;
    }

    // Uniform Cost Search
    void uniformCostSearch() {
        priority_queue<Node, vector<Node>, greater<Node>> frontier;
        set<vector<vector<int>>> explored;

        Node root(initial_state);
        root.h = 0; //  h is always 0 in UCS
        frontier.push(root);

        int maxNodesInQueue = 0;

        while (!frontier.empty()) {
            Node current = frontier.top();
            frontier.pop();

            if (isGoal(current.state)) {
                printSolution(current);
                cout << "Goal!!!" << endl;
                cout << "Total nodes expanded: " << explored.size() + 1 << endl;
                cout << "Maximum nodes in queue: " << maxNodesInQueue << endl;
                cout << "Depth of goal node: " << current.depth << endl;
                return;
            }

            explored.insert(current.state);
            for (auto& successor : getSuccessors(current)) {
                successor.g = current.g + 1; // Cost from root to successor
                successor.h = 0; // Uniform Cost Search heuristic is always 0
                successor.depth = current.depth + 1;

                if (explored.find(successor.state) == explored.end()) {
                    frontier.push(successor);
                    maxNodesInQueue = max(maxNodesInQueue, (int)frontier.size());
                }
            }
        }
        cout << "No solution found." << endl;
    }

    // A* Search with Misplaced Tile heuristic
    void aStarMisplacedTile() {
        priority_queue<Node, vector<Node>, greater<Node>> frontier;
        set<vector<vector<int>>> explored;

        Node root(initial_state);
        root.g = 0;
        root.h = misplacedTiles(root.state);
        frontier.push(root);

        int maxNodesInQueue = 0;

        while (!frontier.empty()) {
            Node current = frontier.top();
            frontier.pop();

            if (isGoal(current.state)) {
                printSolution(current);
                cout << "Goal!!!" << endl;
                cout << "Total nodes expanded: " << explored.size() + 1 << endl;
                cout << "Maximum nodes in queue: " << maxNodesInQueue << endl;
                cout << "Depth of goal node: " << current.depth << endl;
                return;
            }

            explored.insert(current.state);
            for (auto& successor : getSuccessors(current)) {
                successor.g = current.g + 1;
                successor.h = misplacedTiles(successor.state);
                successor.depth = current.depth + 1;

                if (explored.find(successor.state) == explored.end()) {
                    frontier.push(successor);
                    maxNodesInQueue = max(maxNodesInQueue, (int)frontier.size());
                }
            }
        }
        cout << "No solution found." << endl;
    }

    // A* Search with Euclidean Distance heuristic
    void aStarEuclideanDistance() {
        priority_queue<Node, vector<Node>, greater<Node>> frontier;
        set<vector<vector<int>>> explored;

        Node root(initial_state);
        root.g = 0;
        root.h = static_cast<int>(euclideanDistance(root.state));
        frontier.push(root);

        int maxNodesInQueue = 0;

        while (!frontier.empty()) {
            Node current = frontier.top();
            frontier.pop();

            if (isGoal(current.state)) {
                printSolution(current);
                cout << "Goal!!!" << endl;
                cout << "Total nodes expanded: " << explored.size() + 1 << endl;
                cout << "Maximum nodes in queue: " << maxNodesInQueue << endl;
                cout << "Depth of goal node: " << current.depth << endl;
                return;
            }

            explored.insert(current.state);
            for (auto& successor : getSuccessors(current)) {
                successor.g = current.g + 1;
                successor.h = static_cast<int>(euclideanDistance(successor.state));
                successor.depth = current.depth + 1;

                if (explored.find(successor.state) == explored.end()) {
                    frontier.push(successor);
                    maxNodesInQueue = max(maxNodesInQueue, (int)frontier.size());
                }
            }
        }
        cout << "No solution found." << endl;
    }

    // Print the solution path
    void printSolution(const Node& node) {
        vector<Node> path;
        const Node* current = &node;

        while (current) {
            path.push_back(*current);
            current = current->parent;
        }

        reverse(path.begin(), path.end());
        cout << "Solution path: " << endl;
        for (const auto& n : path) {
            for (const auto& row : n.state) {
                for (int val : row) {
                    cout << (val == 0 ? "b" : to_string(val)) << " "; // Use "b" for blank
                }
                cout << endl;
            }
            cout << "-----" << endl;
        }
    }
};

void printWelcomeMessage() {
    cout << "Welcome to 862321584 8-puzzle solver." << endl;
}

int main() {
    printWelcomeMessage();

    int choice;
    cout << "Type '1' to use a default puzzle, or '2' to enter your own puzzle." << endl;
    cin >> choice;

    vector<vector<int>> initial_state;
    if (choice == 1) {
        initial_state = {
            {1, 2, 3},
            {4, 8, 0},
            {7, 6, 5}
        };
        // initial_state = {
        //     {8, 7, 1},
        //     {6, 0, 2},
        //     {5, 4, 3}
        // };
    } else {
        initial_state.resize(3, vector<int>(3));
        cout << "Enter your puzzle. Enter numbers from top left to bottom right seperated by a space or new line. Use a zero to represent the blank:" << endl;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                cin >> initial_state[i][j];
            }
        }
    }

    vector<vector<int>> goal_state = {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 0}
    };

    Problem problem(initial_state, goal_state);
    while(1){
        int algorithm_choice;
        cout << "Enter your choice of algorithm:" << endl;
        cout << "1. Uniform Cost Search" << endl;
        cout << "2. A* with Misplaced Tile heuristic" << endl;
        cout << "3. A* with Euclidean Distance heuristic" << endl;
        cout << "4. Quit" << endl;
        cin >> algorithm_choice;

        switch (algorithm_choice) {
            case 1:
                problem.uniformCostSearch();
                break;
            case 2:
                problem.aStarMisplacedTile();
                break;
            case 3:
                problem.aStarEuclideanDistance();
                break;
            case 4:
                return 0;
            default:
                cout << "Invalid choice." << endl;
                break;
        }
    }
    return 0;
}
