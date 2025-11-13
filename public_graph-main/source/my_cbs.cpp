#include "my_cbs.hpp"
#include <chrono>
#include <limits>
#include <algorithm>
#include <iostream>
#include <map>
#include <tuple>

namespace raplab
{

    // CBSNode implementation
    CBSNode::CBSNode() : cost(0.0), id(-1)
    {
        constraints.clear();
        solution.clear();
    }

    CBSNode::~CBSNode() {}

    // CBS implementation
    CBS::CBS() : node_counter_(0)
    {
        stats_["nodes_expanded"] = 0;
        stats_["nodes_generated"] = 0;
        stats_["runtime"] = 0;
        stats_["low_level_calls"] = 0;
    }

    CBS::~CBS() {}

    int CBS::Solve(std::vector<long> &starts, std::vector<long> &goals,
                   double time_limit, double eps)
    {
        auto start_time = std::chrono::steady_clock::now();

        starts_ = starts;
        goals_ = goals;
        time_limit_ = time_limit;
        eps_ = eps;

        std::cout << "CBS: Starting with " << starts.size() << " agents" << std::endl;

        // Create root node
        auto root = createRootNode();
        if (root == nullptr)
        {
            std::cout << "CBS: Failed to create root node" << std::endl;
            stats_["runtime"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
            return -1; // Failed to find initial solution
        }

        open_list_.push(root);
        stats_["nodes_generated"] = 1;

        std::cout << "CBS: Root node created, cost = " << root->cost << std::endl;

        while (!open_list_.empty())
        {
            // Check time limit
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - start_time).count();
            if (elapsed > time_limit)
            {
                std::cout << "CBS: Timeout after " << elapsed << " seconds" << std::endl;
                stats_["runtime"] = elapsed;
                return -2; // Timeout
            }

            // Get best node
            auto current = open_list_.top();
            open_list_.pop();
            stats_["nodes_expanded"] = static_cast<double>(stats_["nodes_expanded"]) + 1;

            std::cout << "CBS: Expanding node " << current->id << " with cost " << current->cost << std::endl;

            // Validate solution
            Conflict conflict;
            if (validateSolution(current, conflict))
            {
                // No conflict found - solution is valid
                std::cout << "CBS: Found valid solution!" << std::endl;
                final_solution_ = current->solution;
                final_cost_ = {current->cost};
                stats_["runtime"] = elapsed;
                return 0; // Success
            }

            if (conflict.is_edge_conflict) {
                std::cout << "CBS: EDGE Conflict found between agents " << conflict.agent1
                          << " and " << conflict.agent2 << " at edge (" << conflict.vertex 
                          << "," << conflict.vertex2 << ") time " << conflict.time << std::endl;
            } else {
                std::cout << "CBS: VERTEX Conflict found between agents " << conflict.agent1
                          << " and " << conflict.agent2 << " at vertex " << conflict.vertex
                          << " time " << conflict.time << std::endl;
            }

            // Generate child nodes
            auto children = generateChildNodes(current, conflict);
            std::cout << "CBS: Generated " << children.size() << " child nodes" << std::endl;

            for (auto &child : children)
            {
                open_list_.push(child);
                stats_["nodes_generated"] = static_cast<double>(stats_["nodes_generated"]) + 1;
            }

            if (stats_["nodes_expanded"] > 1000)
            {
                std::cout << "CBS: Node expansion limit reached" << std::endl;
                break; // Safety limit
            }
        }

        auto end_time = std::chrono::steady_clock::now();
        stats_["runtime"] = std::chrono::duration<double>(end_time - start_time).count();
        std::cout << "CBS: No solution found after " << stats_["runtime"] << " seconds" << std::endl;
        return -1; // No solution found
    }

    std::shared_ptr<CBSNode> CBS::createRootNode()
    {
        auto node = std::make_shared<CBSNode>();
        node->id = node_counter_++;

        // Initialize constraints for each agent
        node->constraints.resize(starts_.size());

        // Find individual paths for all agents
        node->solution.resize(starts_.size());
        bool all_paths_found = true;

        for (int i = 0; i < starts_.size(); i++)
        {
            std::cout << "CBS: Finding path for agent " << i << " from " << starts_[i] << " to " << goals_[i] << std::endl;
            node->solution[i] = findPathForAgent(i, node->constraints[i], time_limit_ / 10.0); // Use portion of time limit

            if (node->solution[i].empty())
            {
                std::cout << "CBS: Failed to find path for agent " << i << std::endl;
                all_paths_found = false;
                // Don't break immediately, try to see if other agents can find paths
            }
            else
            {
                std::cout << "CBS: Agent " << i << " path: ";
                for (auto v : node->solution[i]) {
                    std::cout << v << " ";
                }
                std::cout << " (length: " << node->solution[i].size() << ")" << std::endl;
            }
        }

        if (!all_paths_found)
        {
            std::cout << "CBS: Some agents failed to find initial paths" << std::endl;
            return nullptr;
        }

        node->cost = calculateSIC(node->solution);
        return node;
    }

    bool CBS::findFirstConflict(std::shared_ptr<CBSNode> node, Conflict &conflict)
    {
        if (node->solution.empty())
        {
            return false;
        }

        // Find maximum path length
        size_t max_length = 0;
        for (const auto &path : node->solution)
        {
            if (path.size() > max_length)
            {
                max_length = path.size();
            }
        }

        if (max_length == 0)
        {
            return false;
        }

        // Check vertex conflicts
        for (size_t t = 0; t < max_length; t++)
        {
            std::unordered_map<long, int> vertex_occupation;

            for (int i = 0; i < node->solution.size(); i++)
            {
                if (node->solution[i].empty())
                {
                    continue; // Skip agents with no path
                }

                long vertex = -1;
                if (t < node->solution[i].size())
                {
                    vertex = node->solution[i][t];
                }
                else
                {
                    // Agent has reached goal - stay at goal position
                    vertex = node->solution[i].back();
                }

                if (vertex == -1)
                {
                    continue; // Invalid vertex
                }

                if (vertex_occupation.find(vertex) != vertex_occupation.end())
                {
                    std::cout << "CBS: Vertex conflict at t=" << t << ", v=" << vertex 
                              << " between agents " << vertex_occupation[vertex] << " and " << i << std::endl;
                    conflict = Conflict(vertex_occupation[vertex], i, vertex, t);
                    return true;
                }
                vertex_occupation[vertex] = i;
            }
        }

        // Check for edge conflicts (swapping conflicts)
        for (size_t t = 0; t < max_length - 1; t++)
        {
            for (int i = 0; i < node->solution.size(); i++)
            {
                for (int j = i + 1; j < node->solution.size(); j++)
                {
                    if (node->solution[i].empty() || node->solution[j].empty())
                    {
                        continue;
                    }

                    long vi1 = (t < node->solution[i].size()) ? node->solution[i][t] : node->solution[i].back();
                    long vi2 = (t + 1 < node->solution[i].size()) ? node->solution[i][t + 1] : node->solution[i].back();
                    long vj1 = (t < node->solution[j].size()) ? node->solution[j][t] : node->solution[j].back();
                    long vj2 = (t + 1 < node->solution[j].size()) ? node->solution[j][t + 1] : node->solution[j].back();

                    if (vi1 == vj2 && vi2 == vj1 && vi1 != -1 && vi2 != -1 && vj1 != -1 && vj2 != -1)
                    {
                        std::cout << "CBS: Edge conflict at t=" << t 
                                  << " between agents " << i << " and " << j 
                                  << " on edge (" << vi1 << "," << vi2 << ")" << std::endl;
                        conflict = Conflict(i, j, vi1, vi2, t);
                        return true;
                    }
                }
            }
        }

        std::cout << "CBS: No conflicts found in solution!" << std::endl;
        return false;
    }

    std::vector<std::shared_ptr<CBSNode>> CBS::generateChildNodes(
        std::shared_ptr<CBSNode> parent, const Conflict &conflict)
    {
        std::vector<std::shared_ptr<CBSNode>> children;

        if (conflict.agent1 < 0 || conflict.agent1 >= parent->solution.size() ||
            conflict.agent2 < 0 || conflict.agent2 >= parent->solution.size())
        {
            std::cout << "CBS: Invalid conflict agents" << std::endl;
            return children;
        }

        // 简化策略：对于边冲突总是使用边约束，对于顶点冲突使用顶点约束
        for (int i = 0; i < 2; i++) {
            int agent = (i == 0) ? conflict.agent1 : conflict.agent2;
            
            auto child = std::make_shared<CBSNode>();
            child->id = node_counter_++;
            child->constraints = parent->constraints;
            child->solution = parent->solution; // 关键：复制所有路径

            if (conflict.is_edge_conflict) {
                // 边冲突：添加边约束
                long v1 = (i == 0) ? conflict.vertex : conflict.vertex2;
                long v2 = (i == 0) ? conflict.vertex2 : conflict.vertex;
                child->constraints[agent].push_back(Constraint(agent, v1, v2, conflict.time));
                std::cout << "CBS: Edge constraint for agent " << agent << " at (" << v1 << "," << v2 << ") t=" << conflict.time << std::endl;
            } else {
                // 顶点冲突：添加顶点约束
                child->constraints[agent].push_back(Constraint(agent, conflict.vertex, conflict.time));
                std::cout << "CBS: Vertex constraint for agent " << agent << " at " << conflict.vertex << " t=" << conflict.time << std::endl;
            }

            // 只重新规划冲突agent的路径
            std::cout << "CBS: Replanning for agent " << agent << " with new constraint" << std::endl;
            child->solution[agent] = findPathForAgent(agent, child->constraints[agent], time_limit_ / 10.0);

            if (!child->solution[agent].empty()) {
                child->cost = calculateSIC(child->solution);
                children.push_back(child);
                std::cout << "CBS: Child node " << child->id << " created with cost " << child->cost << std::endl;
                
                // 输出新路径详情
                std::cout << "CBS: Agent " << agent << " new path: ";
                for (auto v : child->solution[agent]) {
                    std::cout << v << " ";
                }
                std::cout << " (length: " << child->solution[agent].size() << ")" << std::endl;
            } else {
                std::cout << "CBS: Replanning failed for agent " << agent << std::endl;
            }
        }

        return children;
    }

    std::vector<long> CBS::findPathForAgent(int agent,
                                            const std::vector<Constraint> &constraints,
                                            double time_limit)
    {
        stats_["low_level_calls"] = static_cast<double>(stats_["low_level_calls"]) + 1;

        // 首先检查图指针是否有效
        if (_graph == nullptr)
        {
            std::cout << "CBS: Graph pointer is null!" << std::endl;
            return std::vector<long>();
        }

        // 检查起点和终点是否在图中
        if (!_graph->HasVertex(starts_[agent]) || !_graph->HasVertex(goals_[agent]))
        {
            std::cout << "CBS: Start or goal vertex not in graph! Start: " << starts_[agent]
                      << ", Goal: " << goals_[agent] << std::endl;
            return std::vector<long>();
        }

        // 如果没有约束，使用普通A*
        if (constraints.empty())
        {
            raplab::Astar astar_planner;
            astar_planner.SetGraphPtr(_graph);

            auto path = astar_planner.PathFinding(starts_[agent], goals_[agent], time_limit, 0);
            return path;
        }

        // 有约束时使用时空A*
        try
        {
            raplab::AstarSTGrid2d low_level_planner;
            low_level_planner.SetGraphPtr(_graph);

            // 输出约束信息
            std::cout << "CBS: Agent " << agent << " constraints: ";
            for (const auto &constraint : constraints)
            {
                if (constraint.agent == agent)
                {
                    if (constraint.is_edge_constraint && constraint.vertex2 != -1) {
                        std::cout << "Edge(" << constraint.vertex << "," << constraint.vertex2 << ",t=" << constraint.time << ") ";
                        low_level_planner.AddEdgeCstr(constraint.vertex, constraint.vertex2, constraint.time);
                    } else {
                        std::cout << "Vertex(" << constraint.vertex << ",t=" << constraint.time << ") ";
                        low_level_planner.AddNodeCstr(constraint.vertex, constraint.time);
                    }
                }
            }
            std::cout << std::endl;

            // 寻找路径
            auto path = low_level_planner.PathFinding(starts_[agent], goals_[agent], time_limit, 0);
            return path;
        }
        catch (const std::exception &e)
        {
            std::cout << "CBS: Exception in low-level planner: " << e.what() << std::endl;
            return std::vector<long>();
        }
        catch (...)
        {
            std::cout << "CBS: Unknown exception in low-level planner" << std::endl;
            return std::vector<long>();
        }
    }

    double CBS::calculateSIC(const std::vector<std::vector<long>> &solution)
    {
        double cost = 0.0;
        for (const auto &path : solution)
        {
            if (!path.empty())
            {
                cost += path.size() - 1; // Path length as cost
            }
        }
        return cost;
    }

    bool CBS::validateSolution(std::shared_ptr<CBSNode> node, Conflict &conflict)
    {
        if (!findFirstConflict(node, conflict)) {
            std::cout << "CBS: ✅ Valid solution found with cost " << node->cost << std::endl;
            // 输出最终路径详情
            for (int i = 0; i < node->solution.size(); i++) {
                std::cout << "Agent " << i << " final path: ";
                for (auto v : node->solution[i]) {
                    std::cout << v << " ";
                }
                std::cout << " (length: " << node->solution[i].size() << ")" << std::endl;
            }
            return true;
        }
        
        // 输出冲突时的路径状态
        std::cout << "CBS: Conflict detected at time " << conflict.time << ", current paths:" << std::endl;
        for (int i = 0; i < node->solution.size(); i++) {
            std::cout << "Agent " << i << ": ";
            for (size_t t = 0; t < std::min(node->solution[i].size(), (size_t)(conflict.time + 3)); t++) {
                if (t < node->solution[i].size()) {
                    std::cout << "t" << t << "=" << node->solution[i][t] << " ";
                }
            }
            std::cout << std::endl;
        }
        
        return false;
    }

    PathSet CBS::GetPlan(long nid)
    {
        return final_solution_;
    }

    CostVec CBS::GetPlanCost(long nid)
    {
        return final_cost_;
    }

    std::unordered_map<std::string, double> CBS::GetStats()
    {
        return stats_;
    }

} // namespace raplab