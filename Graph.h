#pragma once

#include <vector>
#include <optional>
#include <iostream>
#include <exception>
#include <queue>
#include <limits>
#include <utility>


// define 1 if you want to implement it
#define BONUS 1

// Edge can be constructed with syntax
// Edge e{ 1, 5, 1.89 };
struct Edge
{
    int v;
    int u;
    double weight;
};

/* Graph represents one abstract graph.
 *
 * Vertices are always numbered from 0.
 *
 * Between two vertices,  there can only be one edge.   If edges is in
 * form of  (v, v), so  both ends are  the same, e.g.   AddEdge({1, 1,
 * 0.5}) throw an exception.
 *
 * Also throw excepton if one or both vertices are out of bounds.
 *
 * Graph is not directed so (u, v) is the same edge as (v, u).
 *
 * Weights are  limited to positive  values, so 0 and  negative values
 * are forbidden.  If you encounter such weights throw an exception.
 */

class Graph
{
    // Do not modify public interface
public:
    // Construct graph with n vertices and no edges
    explicit Graph(size_t n) {
        
        if (n == 0) {
            throw std::out_of_range("Graph needs to have at least 1 vertex ...\n");
        }
        m_graph_size = static_cast<int>(n);
        m_adj_list.resize(n);
    }


    // Will construct graph with given  edges, the vertices are from 0 to the highest number in the vector.
    explicit Graph(const std::vector<Edge>& edges) {

        if (edges.empty()) {
            throw std::out_of_range("Edges vector is empty ...\n");
        }
        set_size(edges);

        for (auto& edge : edges) {
            AddEdge(edge);
        }
    }


    // Add  edge to  graph. If  the edge  already exists,  replace the weight.
    void AddEdge(const Edge& edge) {

        check_if_in_bounds(edge.v, edge.u);
        check_weight(edge.weight);
        check_if_looped(edge.v, edge.u);

        insert_edge(edge);
    }


    /* Same as AddEdge, but can insert  multiple edges. If one edge is
     * there more  than once,  use later one  (edge with  higher index
     * overwrites edge with lower index) */
    void AddEdges(const std::vector<Edge>& edges) {

        // Maybe end function without throwing exception instead
        if (edges.empty()) {
            throw std::out_of_range("Edges vector is empty ...\n");
            //std::cout << "Edges vector is empty ...\n";
            //return;
        }

        for (auto& edge : edges) {
            AddEdge(edge);
        }
    }


    // Return  weight between  vertices  u  and v.  If  edge does  not exists, behaviour is undefined.
    double operator()(int u, int v) const {

        // if out of bounds return -1    
        if ((v < 0 || v >= m_graph_size) || ((u < 0 || u >= m_graph_size))) {
            return -1;
        }

        for (auto& i : m_adj_list[v]) {
            if (i.first == u) {
                return i.second;
            }
        }
        return -1;
    }


    // Return weight between vertices u and v, if edge does not exists, throw an exception.
    double At(int u, int v) const {
        
        check_if_in_bounds(v, u);
        check_if_looped(v, u);

        for (auto& i : m_adj_list[v]) {
            if (i.first == u) {
                return i.second;
            }
        }
        throw std::invalid_argument("This edge does not exist ...\n");
    }


    // Return true  if there  is an  edge between  u and  v, otherwise false
    bool Connected(int u, int v) const noexcept {

        // return false if looped or out of bounds
        if (u == v) { return false; }
        
        if ((v < 0 || v >= m_graph_size) || ((u < 0 || u >= m_graph_size))) {
            return false;
        }

        for (auto& i : m_adj_list[v]) {
            if (i.first == u) {
                return true;
            }
        }
        return false;
    }


    /* Return shortest path  from u to v (path with  minimal cost). If
     * there is no path return nullopt.   You can use dijkstra, or any
     * other algorithm. Path should start with u and ends with v.
     *
     * https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
     *
     * 1 BONUS point will be for  speed of this function.
     * First 5 implementations will be awarded with the bonus point.
     */
    std::optional<std::vector<int>> Path(int u, int v) const {

        // first we check if both vertices are in bounds, if not return nullopt
        if ((v < 0 || v > m_graph_size) || ((u < 0 || u > m_graph_size))) {
            return std::nullopt;
        }
        // if it's path from u to u, return empty vector (or vector containing only u maybe)
        if (u == v) { 
            std::vector<int> path;
            return path;
        }

        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>,
                            std::greater<std::pair<double, int>>> min_heap;
    
        // vector used for storing total distance and previous vertex from starting vertex
        // sets all distances as infinite and vertices as unvisited (-1)
        std::vector<std::pair<double, int>> distance(m_graph_size, std::make_pair(std::numeric_limits<double>::infinity(), -1));

        min_heap.push(std::make_pair(0, u));
        distance[u].first = 0;

        while (!min_heap.empty()) {
            
            int curr = min_heap.top().second;
            min_heap.pop();

            for (auto& i : m_adj_list[curr]) {
                int vertex = i.first;
                double weight = i.second;

                if (distance[vertex].first > distance[curr].first + weight) {
                    distance[vertex].first = distance[curr].first + weight;
                    distance[vertex].second = curr;
                    min_heap.push(std::make_pair(distance[vertex].first, vertex));
                }
            }
        }
        // if path exist, build a path, otherwise return nullopt
        if (distance[v].second != -1) {

            std::vector<int> final_path;
            int to_push = v;
            final_path.push_back(to_push);
            
            while (distance[to_push].second != -1) {
                to_push = distance[to_push].second;
                final_path.insert(final_path.begin(), to_push);
            }
            //std::cout << "Shortes path from " << u << " to " << v << ": ";
            //for (auto& i : final_path) { std::cout << i << " "; }
            //std::cout << "\n";
            return final_path;
        }
        return std::nullopt;
    }


#if BONUS == 1
    /* Returns  minimum spanning  tree  for this  graph.  You can  use
     * kruskal's algorithm
     * https://en.wikipedia.org/wiki/Kruskal%27s_algorithm
     */
    Graph SpannigTree() const {

        // minheap containing all edges and their weights
        std::priority_queue<std::pair<double, std::pair<int, int>>, std::vector<std::pair<double, std::pair<int, int>>>,
            std::greater<std::pair<double, std::pair<int, int>>>> min_heap;
        
        for (int i = 0; i < m_graph_size; i++) {
            for (auto& v : m_adj_list[i]) {
                min_heap.push(std::make_pair(v.second, std::make_pair(i, v.first)));
            }
        }
        std::vector<int> ranks (m_graph_size, 0);
        std::vector<int> parents (m_graph_size, 0);

        for (int i = 0; i < m_graph_size; i++) {
            parents[i] = i;
        }
        // contains MST of this graph
        std::vector<Edge> edges;

        while (!min_heap.empty()) {
            
            auto edge = min_heap.top();
            int v = find_parent_vertex(parents, edge.second.first);
            int u = find_parent_vertex(parents, edge.second.second);

            if (v != u) {
                //std::cout << edge.second.first << " -> " << edge.second.second << ", " << edge.first << "\n";
                edges.push_back(Edge{edge.second.first, edge.second.second, edge.first });
                if (ranks[v] < ranks[u]) {
                    parents[v] = u;
                    ranks[u]++;
                }
                else {
                    parents[u] = v;
                    ranks[v]++;
                }
            }
            min_heap.pop();
        }
        return Graph(edges);
    }
#endif

    // DELETE LATER !!!!!!!!!!!!
    /*void print_graph() {
        
        int edge_counter = 0;
        std::cout << "\n...............................................\n";
        std::cout << "Adjacency list: \n";
        for (int i = 0; i < m_graph_size; i++) {
            for (auto& v : m_adj_list[i]) {
                ++edge_counter;
                std::cout << "(" << i << " -> " << v.first << ", weight: " << v.second << ") ";
            }
            std::cout << std::endl;
        }
        std::cout << "Size of adjacency list: " << m_adj_list.size() << "\n";
        std::cout << "Number of vertices: " << m_graph_size << "\n";
        std::cout << "Number of edges: " << edge_counter/2 << "\n";
        std::cout << "...............................................\n\n";
    }*/

private:
    // Class members ............................................................
    // Representation of graph using adjacency list
    std::vector<std::vector<std::pair<int, double>>> m_adj_list;

    // contains number of vertices in a graph
    int m_graph_size;    


    // Functions ................................................................
    // throws exception if invalid weight
    void check_weight(double weight) {
        if (weight <= 0) {
            throw std::out_of_range("Edge weight needs to be higher than 0 ...\n");
        }
    }


    // throws exception if the edge is looped
    void check_if_looped(int v, int u) const  {
        if (v == u) {
            throw std::invalid_argument("No loops allowed ...\n");
        }
    }


    // throws exception if vertex is out of bounds
    void check_if_in_bounds(int v, int u) const {
        
        if ((v < 0 || v >= m_graph_size) || ((u < 0 || u >= m_graph_size))) {
            throw std::out_of_range("Vertex is out of bounds ...\n");
        }
    }


    // inserts edge into the graph or overwrites its weight
    void insert_edge(const Edge& edge) {
        
        // checks if we need to overwrite some edge
        // first we replace weight in (v, u, w)
        for (auto& i : m_adj_list[edge.v]) {
            if (i.first == edge.u) {
                i.second = edge.weight;

                // then replace weight in (u, v, w)
                for (auto& j : m_adj_list[edge.u]) {
                    if (j.first == edge.v) {
                        j.second = edge.weight;
                    }
                }
                return;
            }
        }
        // if we don't need to overwrite anything we add new element to the vector
        m_adj_list[edge.v].push_back(std::make_pair(edge.u, edge.weight));
        m_adj_list[edge.u].push_back(std::make_pair(edge.v, edge.weight));
    }
    

    // finds the vertex with max value, resize the adjacency list to this value + 1
    void set_size(const std::vector<Edge>& edges) {

        int temp_max;

        for (auto& edge : edges) {
            temp_max = (edge.v > edge.u) ? edge.v : edge.u;
            ++temp_max;
            if (temp_max > m_graph_size) {
                m_graph_size = temp_max;
            }
        }
        m_adj_list.resize(m_graph_size);
    }


    // finds parent vertex of v, used for Kruskal's algo
    int find_parent_vertex(std::vector<int>& parents, int v) const {
        
        if (parents[v] == v) {
            return v;
        } else {
            v = find_parent_vertex(parents, parents[v]);
        }
        return v;
    }
};
