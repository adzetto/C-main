/**
 * Routing and Mapping Toolkit
 * Author: adzetto
 */
#ifndef ROUTE_MAPPING_H
#define ROUTE_MAPPING_H

#include <vector>
#include <string>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <limits>

namespace routing
{
    struct Node { int id; double x, y; };
    struct Edge { int from, to; double cost; bool on_highway{false}; };

    class Graph
    {
    public:
        int add_node(double x, double y) { int id = (int)nodes_.size(); nodes_.push_back({id, x, y}); return id; }
        void add_edge(int a, int b, double cost, bool hwy=false) { edges_.push_back({a, b, cost, hwy}); adj_[a].push_back((int)edges_.size()-1); }
        const std::vector<Node>& nodes() const { return nodes_; }
        const std::vector<Edge>& edges() const { return edges_; }
        const std::vector<int>& out_edges(int n) const { auto it=adj_.find(n); static const std::vector<int> empty; return it==adj_.end()? empty: it->second; }
    private:
        std::vector<Node> nodes_{}; std::vector<Edge> edges_{}; std::unordered_map<int,std::vector<int>> adj_{};
    };

    inline double heuristic(const Node& a, const Node& b) { double dx=a.x-b.x, dy=a.y-b.y; return std::sqrt(dx*dx+dy*dy); }

    struct RouteStep { int node; double g; double f; int parent; };

    class AStar
    {
    public:
        std::vector<int> find_path(const Graph& g, int start, int goal) {
            auto cmp = [](const RouteStep& a, const RouteStep& b){ return a.f > b.f; };
            std::priority_queue<RouteStep,std::vector<RouteStep>,decltype(cmp)> open(cmp);
            std::unordered_map<int, RouteStep> best;
            const auto& ns = g.nodes();
            open.push({start, 0.0, heuristic(ns[start], ns[goal]), -1});
            while (!open.empty()) {
                RouteStep cur = open.top(); open.pop();
                if (best.count(cur.node) && cur.g >= best[cur.node].g) continue;
                best[cur.node] = cur;
                if (cur.node == goal) break;
                for (int ei : g.out_edges(cur.node)) {
                    const auto& e = g.edges()[ei];
                    double ng = cur.g + e.cost;
                    double nf = ng + heuristic(ns[e.to], ns[goal]);
                    open.push({e.to, ng, nf, cur.node});
                }
            }
            std::vector<int> path;
            if (!best.count(goal)) return path;
            int at = goal; while (at != -1) { path.push_back(at); at = best[at].parent; }
            std::reverse(path.begin(), path.end());
            return path;
        }
    };
}

#endif // ROUTE_MAPPING_H

