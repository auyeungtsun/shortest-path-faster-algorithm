#include <iostream>
#include <vector>
#include <cassert>
#include <queue>
#include <climits>

using namespace std;

struct Edge {
    /**
     * @brief Represents a directed edge in a flow network.
     * @param to The destination vertex of the edge.
     * @param cap The capacity of the edge.
     * @param cost The cost associated with sending one unit of flow along the edge.
     * @param flow The current flow through the edge.
     * @param rev The index of the reverse edge in the adjacency list of the destination vertex.
     */
    int to, cap, cost, flow, rev;
};

class MinCostFlow {
public:
    /** @brief adj[i] is the list of edges from i */
    vector<vector<Edge>> adj;
    /** @brief dist[i] is the minimum cost from the start to i */
    vector<int> dist;
    /** @brief prevv[i] is the previous vertex on the shortest path to i */
    vector<int> prevv;
    /** @brief preve[i] is the index of the edge used to reach i */
    vector<int> preve;
    /** @brief in_queue[i] is true if the node i is in queue */
    vector<bool> in_queue;

    MinCostFlow(int n) : adj(n), dist(n), prevv(n), preve(n) {}

    /**
    * @brief Adds a directed edge to the flow network.
    * 
    * This function adds a directed edge from 'from' to 'to' with a given 
    * capacity 'cap' and cost 'cost'. It also adds a reverse edge with 
    * zero capacity and negative cost for flow calculation purposes.
    * 
    * @param from The starting node of the edge.
    * @param to The ending node of the edge.
    * @param cap The capacity of the edge.
    * @param cost The cost associated with sending one unit of flow along the edge.
    */
    void add_edge(int from, int to, int cap, int cost) {
        adj[from].push_back({to, cap, cost, 0, (int)adj[to].size()});
        adj[to].push_back({from, 0, -cost, 0, (int)adj[from].size() - 1});
    }

    /**
    * @brief Finds the shortest path in a graph using the Shortest Path Faster Algorithm (SPFA).
    *
    * This function uses SPFA to find the shortest path from a source node 's' to a target node 't'
    * in a graph with edge weights (costs). It also calculates the predecessors of each node in the shortest path.
    * The shortest path is determined by the minimum sum of edge costs along the path.
    *
    * @param s The source node.
    * @param t The target node.
    *
    * @return True if a path exists from 's' to 't', false otherwise.
    * @note Time Complexity: O(V * E) in the worst case, where V is the number of vertices and E is the number of edges.
    * @note Space Complexity: O(V + E), where V is the number of vertices and E is the number of edges.
    */
    bool spfa(int s, int t) {
        dist.assign(adj.size(), INT_MAX);
        prevv.assign(adj.size(), -1);
        preve.assign(adj.size(), -1);
        in_queue.assign(adj.size(), false);
        dist[s] = 0;
        queue<int> q;
        q.push(s);
        in_queue[s] = true;
        while (!q.empty()) {
            int v = q.front();
            q.pop();
            in_queue[v] = false;
            for (int i = 0; i < adj[v].size(); ++i) {
                Edge &e = adj[v][i];
                if (e.cap > e.flow && dist[e.to] > dist[v] + e.cost) {
                    dist[e.to] = dist[v] + e.cost;
                    prevv[e.to] = v;
                    preve[e.to] = i;
                    if (!in_queue[e.to]) {
                       q.push(e.to);
                       in_queue[e.to] = true;
                    }
                }
            }
        }
        return dist[t] != INT_MAX;
    }

    /**
    * @brief Calculates the minimum cost maximum flow in a flow network.
    * 
    * This function computes the minimum cost to transport a certain amount of flow 
    * from a source node 's' to a sink node 't' in a directed graph. The graph is 
    * represented by an adjacency list where each edge has a capacity and a cost.
    * 
    * @param s The source node.
    * @param t The sink node.
    * @param f The desired amount of flow to be transported.
    * 
    * @return A pair where the first element is the actual flow achieved (which 
    *         may be less than 'f' if the maximum flow is less than 'f'), and the 
    *         second element is the minimum cost to achieve that flow.
    * @note Time Complexity: O(F * E * V), where F is the maximum flow, E is the number of edges, and V is the number of vertices.
    * @note Space Complexity: O(V + E), where V is the number of vertices and E is the number of edges.
    */
    pair<int, int> min_cost_flow(int s, int t, int f) {
        int flow = 0, cost = 0;
        while(flow < f && spfa(s, t)){
            int d = f - flow;
            for (int v = t; v != s; v = prevv[v]) {
                d = min(d, adj[prevv[v]][preve[v]].cap - adj[prevv[v]][preve[v]].flow);
            }
            flow += d;
            cost += d * dist[t];
            for (int v = t; v != s; v = prevv[v]) {
                Edge &e = adj[prevv[v]][preve[v]];
                e.flow += d;
                adj[v][e.rev].flow -= d;
            }
        }
        return {flow, cost};
    }
};

void test_min_cost_flow() {
    // Test case 1: Basic flow with multiple paths
    int n = 4;
    MinCostFlow mcf(n);
    mcf.add_edge(0, 1, 10, 2);
    mcf.add_edge(0, 2, 2, 4);
    mcf.add_edge(1, 3, 3, 6);
    mcf.add_edge(2, 3, 8, 1);
    mcf.add_edge(1, 2, 6, 3);
    
    pair<int,int> res = mcf.min_cost_flow(0,3,10);
    assert(res.first == 10);
    assert(res.second == 62);

    // Test case 2: Simple flow with two edges
    MinCostFlow mcf2(3);
    mcf2.add_edge(0, 1, 10, 1);
    mcf2.add_edge(1, 2, 5, 2);
    pair<int,int> res2 = mcf2.min_cost_flow(0,2,5);
    assert(res2.first == 5);
    assert(res2.second == 15);

    // Test case 3: Max flow request exceeding capacity
    MinCostFlow mcf3(3);
    mcf3.add_edge(0, 1, 10, 1);
    mcf3.add_edge(1, 2, 5, 2);
    pair<int,int> res3 = mcf3.min_cost_flow(0,2,INT_MAX);
    assert(res3.first == 5);
    assert(res3.second == 15);
    
    // Test case 4: Negative cost edge
    MinCostFlow mcf4(3);
    mcf4.add_edge(0, 1, 5, -1);
    mcf4.add_edge(1, 2, 5, 2);
    mcf4.add_edge(0, 2, 10, 5);
    pair<int,int> res4 = mcf4.min_cost_flow(0,2,5);
    assert(res4.first == 5);
    assert(res4.second == 5);

}

void run_min_cost_flow_sample() {
    int n = 6; 
    MinCostFlow mcf(n);
    mcf.add_edge(0, 1, 10, 1);
    mcf.add_edge(0, 2, 5, 2);
    mcf.add_edge(1, 3, 4, 2);
    mcf.add_edge(1, 4, 6, 3);
    mcf.add_edge(2, 4, 2, 1);
    mcf.add_edge(3, 5, 8, 1);
    mcf.add_edge(4, 5, 7, 2);

    pair<int, int> res = mcf.min_cost_flow(0, 5, 10);
    cout << "Flow: " << res.first << ", Cost: " << res.second << endl;
}

int main() {
    test_min_cost_flow();
    run_min_cost_flow_sample();
    return 0;
}



