#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <cassert>

using namespace std;

/**
 * @brief Implementation of the Shortest Path Faster Algorithm (SPFA).
 * 
 * This function calculates the shortest path from a starting node to all other nodes in a weighted directed graph using SPFA.
 * It also detects negative cycles in the graph.
 * 
 * To find the longest path, you can negate all the weights in the graph.
 * The shortest path found in this modified graph will correspond to the 
 * longest path in the original graph.
 * Then, you just have to negate the result to get the real path length.
 *
 * @param start The starting node for calculating shortest paths.
 * @param n The total number of nodes in the graph.
 * @param adj The adjacency list representation of the graph, where adj[u] is a vector of pairs (v, weight) 
 *            representing an edge from node u to node v with the given weight.
 * @return A pair containing:
 *         - A boolean indicating whether a negative cycle was detected (true if a negative cycle exists, false otherwise).
 *         - A vector of integers representing the shortest distances from the starting node to all other nodes.
 * @note Time Complexity: O(V*E) in the worst case, where V is the number of vertices and E is the number of edges.
 *       Difference with Bellman-Ford:
 *       - Bellman-Ford relaxes all edges in each of V-1 iterations, regardless of whether a node's distance has changed.
 *       - SPFA, on the other hand, uses a queue to only process nodes whose distances have been updated, leading to potential early termination and efficiency in many cases.
 *       - However, SPFA's worst-case time complexity is still O(V*E), and in some pathological cases, it can be slower than Bellman-Ford.
 *       - Bellman-Ford is more stable in the worst case, but SPFA performs much better in practice on average.
 *       - Both algorithm can be used to find negative cycle
 *       Space Complexity: O(V), for storing distances, counts, and the queue.
 * 
 *       System of Difference Constraints (SDC):
 *       - SPFA can be used to solve a System of Difference Constraints.
 *       - An SDC is a set of inequalities of the form x_i - x_j <= c_ij, where x_i and x_j are variables and c_ij is a constant.
 *       - To solve an SDC using SPFA:
 *         1. Create a graph where each variable x_i corresponds to a node i in the graph.
 *         2. For each constraint x_i - x_j <= c_ij, add a directed edge from node j to node i with weight c_ij (from i to j with weight -c_ij if x_i - x_j >= c_ij).
 *         3. Add a virtual source node (node s) and add edges of weight 0 from s to all other nodes.
 *         4. Run SPFA from the source node s.
 *         5. If a negative cycle is detected, the system of constraints is inconsistent (no solution).
 *         6. Otherwise, the shortest distance dist[i] from the source node to node i represents a solution for x_i.
 *         7. If there is no path from the virtual source to node i, the solution for x_i can be any value.
 *         8. Because of the way the edges are created, we have dist[i] - dist[j] <= w_ji
 */
pair<bool, vector<int>> spfa(int start, int n, const vector<vector<pair<int, int>>>& adj) {
    // dist[i] stores the shortest distance from the starting node to node i. Initialized to infinity.
    vector<int> dist(n, numeric_limits<int>::max());
    // cnt[i] stores the number of times node i has been relaxed (updated) in the queue.
    // Used to detect negative cycles: if a node is relaxed n or more times, a negative cycle exists.
    vector<int> cnt(n, 0);
    // in_queue[i] indicates whether node i is currently in the queue.
    vector<bool> in_queue(n, false);
    // The queue used for the SPFA algorithm.
    queue<int> q;

    dist[start] = 0;
    q.push(start);
    in_queue[start] = true;

    while (!q.empty()) {
        int u = q.front();
        q.pop();
        in_queue[u] = false;

        for (const auto& edge : adj[u]) {
            int v = edge.first;
            int weight = edge.second;
            if (dist[u] != numeric_limits<int>::max() && dist[v] > dist[u] + weight) {
                dist[v] = dist[u] + weight;
                if (!in_queue[v]){
                    cnt[v]++;
                    if (cnt[v] >= n) {
                        return {true, dist};
                    }
                    q.push(v);
                    in_queue[v] = true;
                }
                

            }
        }
    }
    return {false, dist};
}

void test_spfa() {
    // Test case 1: Simple graph with no negative cycle
    vector<vector<pair<int, int>>> adj1(4);
    adj1[0].push_back({1, 1});
    adj1[0].push_back({2, 4});
    adj1[1].push_back({2, 2});
    adj1[1].push_back({3, 3});
    adj1[2].push_back({3, 1});

    auto result1 = spfa(0, 4, adj1);
    assert(result1.first == false);
    assert(result1.second[0] == 0);
    assert(result1.second[1] == 1);
    assert(result1.second[2] == 3);
    assert(result1.second[3] == 4);

    // Test case 2: Graph with a negative cycle
    vector<vector<pair<int, int>>> adj2(4);
    adj2[0].push_back({1, 1});
    adj2[1].push_back({2, -2});
    adj2[2].push_back({0, -2});
    adj2[1].push_back({3, 3});

    auto result2 = spfa(0, 4, adj2);
    assert(result2.first == true);

    // Test case 3: Disconnected graph
    vector<vector<pair<int, int>>> adj3(4);
    adj3[0].push_back({1, 1});
    adj3[2].push_back({3, 1});

    auto result3 = spfa(0, 4, adj3);
    assert(result3.first == false);
    assert(result3.second[0] == 0);
    assert(result3.second[1] == 1);
    assert(result3.second[2] == numeric_limits<int>::max());
    assert(result3.second[3] == numeric_limits<int>::max());
    
    // Test case 4 : empty graph
     vector<vector<pair<int, int>>> adj4(1);

    auto result4 = spfa(0, 1, adj4);
    assert(result4.first == false);
    assert(result4.second[0] == 0);
    
    // Test case 5: Graph with negative edges but no negative cycle
    vector<vector<pair<int, int>>> adj5(4);
    adj5[0].push_back({1, -1});
    adj5[1].push_back({2, -2});
    adj5[2].push_back({3, 1});
    adj5[0].push_back({3, 3});

    auto result5 = spfa(0, 4, adj5);
    assert(result5.first == false);
    assert(result5.second[0] == 0);
    assert(result5.second[1] == -1);
    assert(result5.second[2] == -3);
    assert(result5.second[3] == -2);

}

void run_spfa_sample() {
    int n = 4;
    vector<vector<pair<int, int>>> adj(n);
    adj[0].push_back({1, 1});
    adj[0].push_back({2, 4});
    adj[1].push_back({2, 2});
    adj[1].push_back({3, 3});
    adj[2].push_back({3, 1});

    auto result = spfa(0, n, adj);
    if (result.first) {
        cout << "Negative cycle detected!" << endl;
    } else {
        cout << "Shortest distances from node 0:" << endl;
        for (int i = 0; i < n; ++i) {
            cout << "Node " << i << ": " << result.second[i] << endl;
        }
    }
}


int main() {
    test_spfa();
    run_spfa_sample();
    return 0;
}
