#include "my_cbs_multi.hpp"
#include <chrono>
#include <limits>
#include <algorithm>
#include <iostream>
#include <map>
#include <tuple>
#include <set>

namespace raplab
{

    // MultiCBSNode implementation
    MultiCBSNode::MultiCBSNode() : cost(0.0), id(-1)
    {
        constraints.clear();
        solution.clear();
    }

    MultiCBSNode::~MultiCBSNode() {}

    // MultiCBS implementation
    MultiCBS::MultiCBS() : node_counter_(0)
    {
        stats_["nodes_expanded"] = 0;
        stats_["nodes_generated"] = 0;
        stats_["runtime"] = 0;
        stats_["low_level_calls"] = 0;
    }

    MultiCBS::~MultiCBS() {}

    void MultiCBS::SetAgentSizes(const std::vector<std::pair<int, int>> &sizes)
    {
        agents_.clear();
        for (size_t i = 0; i < sizes.size(); ++i)
        {
            agents_.emplace_back(-1, -1, sizes[i].first, sizes[i].second);
        }
    }

    int MultiCBS::Solve(std::vector<long> &starts, std::vector<long> &goals,
                        double time_limit, double eps)
    {
        auto start_time = std::chrono::steady_clock::now();

        starts_ = starts;
        goals_ = goals;
        time_limit_ = time_limit;
        eps_ = eps;

        // Initialize agents if not already done
        if (agents_.empty())
        {
            for (size_t i = 0; i < starts.size(); ++i)
            {
                agents_.emplace_back(starts[i], goals[i], 1, 1);
            }
        }
        else
        {
            // Update starts and goals for existing agents
            for (size_t i = 0; i < std::min(agents_.size(), starts.size()); ++i)
            {
                agents_[i].start = starts[i];
                agents_[i].goal = goals[i];
            }
        }

        std::cout << "MultiCBS: Starting with " << agents_.size() << " agents" << std::endl;
        for (size_t i = 0; i < agents_.size(); ++i)
        {
            std::cout << "Agent " << i << ": size " << agents_[i].width << "x" << agents_[i].height
                      << ", from " << agents_[i].start << " to " << agents_[i].goal << std::endl;
        }

        // Create root node
        auto root = createRootNode();
        if (root == nullptr)
        {
            std::cout << "MultiCBS: Failed to create root node" << std::endl;
            stats_["runtime"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
            return -1; // Failed to find initial solution
        }

        open_list_.push(root);
        stats_["nodes_generated"] = 1;

        std::cout << "MultiCBS: Root node created, cost = " << root->cost << std::endl;

        while (!open_list_.empty())
        {
            // Check time limit
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - start_time).count();
            if (elapsed > time_limit)
            {
                std::cout << "MultiCBS: Timeout after " << elapsed << " seconds" << std::endl;
                stats_["runtime"] = elapsed;
                return -2; // Timeout
            }

            // Get best node
            auto current = open_list_.top();
            open_list_.pop();
            stats_["nodes_expanded"] = static_cast<double>(stats_["nodes_expanded"]) + 1;

            std::cout << "MultiCBS: Expanding node " << current->id << " with cost " << current->cost << std::endl;

            // Validate solution
            MultiConflict conflict;
            if (validateSolution(current, conflict))
            {
                // No conflict found - solution is valid
                std::cout << "MultiCBS: Found valid solution!" << std::endl;
                final_solution_ = current->solution;
                final_cost_ = {current->cost};
                stats_["runtime"] = elapsed;
                return 0; // Success
            }

            if (conflict.is_edge_conflict)
            {
                std::cout << "MultiCBS: EDGE Conflict found between agents " << conflict.agent1
                          << " and " << conflict.agent2 << " at time " << conflict.time << std::endl;
            }
            else
            {
                std::cout << "MultiCBS: VERTEX Conflict found between agents " << conflict.agent1
                          << " and " << conflict.agent2 << " at time " << conflict.time << std::endl;
            }

            // Generate child nodes
            auto children = generateChildNodes(current, conflict);
            std::cout << "MultiCBS: Generated " << children.size() << " child nodes" << std::endl;

            for (auto &child : children)
            {
                open_list_.push(child);
                stats_["nodes_generated"] = static_cast<double>(stats_["nodes_generated"]) + 1;
            }

            if (stats_["nodes_expanded"] > 1000)
            {
                std::cout << "MultiCBS: Node expansion limit reached" << std::endl;
                break; // Safety limit
            }
        }

        auto end_time = std::chrono::steady_clock::now();
        stats_["runtime"] = std::chrono::duration<double>(end_time - start_time).count();
        std::cout << "MultiCBS: No solution found after " << stats_["runtime"] << " seconds" << std::endl;
        return -1; // No solution found
    }

    std::shared_ptr<MultiCBSNode> MultiCBS::createRootNode()
    {
        auto node = std::make_shared<MultiCBSNode>();
        node->id = node_counter_++;

        // Initialize constraints for each agent
        node->constraints.resize(agents_.size());

        // Find individual paths for all agents
        node->solution.resize(agents_.size());
        bool all_paths_found = true;

        for (int i = 0; i < agents_.size(); i++)
        {
            std::cout << "MultiCBS: Finding path for agent " << i << " from " << agents_[i].start
                      << " to " << agents_[i].goal << std::endl;
            node->solution[i] = findPathForAgent(i, node->constraints[i], time_limit_ / 10.0);

            if (node->solution[i].empty())
            {
                std::cout << "MultiCBS: Failed to find path for agent " << i << std::endl;
                all_paths_found = false;
                // 继续尝试其他agent，让CBS在后续处理中解决冲突
            }
            else
            {
                std::cout << "MultiCBS: Agent " << i << " path: ";
                for (auto v : node->solution[i])
                {
                    std::cout << v << " ";
                }
                std::cout << " (length: " << node->solution[i].size() << ")" << std::endl;
            }
        }

        if (!all_paths_found)
        {
            std::cout << "MultiCBS: Some agents failed to find initial paths, but continuing for CBS to resolve conflicts" << std::endl;
            // 不返回nullptr，让CBS继续处理
            // 空的路径会在validateSolution中产生冲突
        }

        node->cost = calculateSIC(node->solution);
        return node;
    }

    bool MultiCBS::findFirstConflict(std::shared_ptr<MultiCBSNode> node, MultiConflict &conflict)
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

        // Check vertex conflicts (overlaps at same timestep)
        for (size_t t = 0; t < max_length; t++)
        {
            for (int i = 0; i < node->solution.size(); i++)
            {
                for (int j = i + 1; j < node->solution.size(); j++)
                {
                    if (node->solution[i].empty() || node->solution[j].empty())
                    {
                        continue;
                    }

                    long base_i = (t < node->solution[i].size()) ? node->solution[i][t] : node->solution[i].back();
                    long base_j = (t < node->solution[j].size()) ? node->solution[j][t] : node->solution[j].back();

                    if (checkCollision(i, base_i, j, base_j))
                    {
                        std::cout << "MultiCBS: Vertex conflict at t=" << t
                                  << " between agents " << i << " and " << j << std::endl;
                        auto vertices_i = getAgentVertices(i, base_i);
                        auto vertices_j = getAgentVertices(j, base_j);
                        conflict = MultiConflict(i, j, vertices_i, vertices_j, t);
                        return true;
                    }
                }
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

                    long base_i_from = (t < node->solution[i].size()) ? node->solution[i][t] : node->solution[i].back();
                    long base_i_to = (t + 1 < node->solution[i].size()) ? node->solution[i][t + 1] : node->solution[i].back();
                    long base_j_from = (t < node->solution[j].size()) ? node->solution[j][t] : node->solution[j].back();
                    long base_j_to = (t + 1 < node->solution[j].size()) ? node->solution[j][t + 1] : node->solution[j].back();

                    // 检查是否发生交换冲突：两个agent在相邻时间步交换位置并发生碰撞
                    if (base_i_to == base_j_from && base_i_from == base_j_to)
                    {
                        // 检查在交换过程中是否会发生碰撞
                        if (checkCollision(i, base_i_to, j, base_j_from))
                        {
                            std::cout << "MultiCBS: Edge conflict at t=" << t
                                      << " between agents " << i << " and " << j << std::endl;
                            auto vertices_i_from = getAgentVertices(i, base_i_from);
                            auto vertices_i_to = getAgentVertices(i, base_i_to);
                            auto vertices_j_from = getAgentVertices(j, base_j_from);
                            auto vertices_j_to = getAgentVertices(j, base_j_to);
                            conflict = MultiConflict(i, j, vertices_i_from, vertices_i_to,
                                                     vertices_j_from, vertices_j_to, t);
                            return true;
                        }
                    }
                }
            }
        }

        std::cout << "MultiCBS: No conflicts found in solution!" << std::endl;
        return false;
    }

    std::vector<std::shared_ptr<MultiCBSNode>> MultiCBS::generateChildNodes(
        std::shared_ptr<MultiCBSNode> parent, const MultiConflict &conflict)
    {
        std::vector<std::shared_ptr<MultiCBSNode>> children;

        // 处理与障碍物的冲突 (agent2 == -1)
        if (conflict.agent2 == -1)
        {
            // 这是agent与障碍物的冲突，只为该agent生成约束
            auto child = std::make_shared<MultiCBSNode>();
            child->id = node_counter_++;
            child->constraints = parent->constraints;
            child->solution = parent->solution;

            child->constraints[conflict.agent1].push_back(
                MultiConstraint(conflict.agent1, conflict.vertices1, conflict.time));

            std::cout << "MultiCBS: Obstacle constraint for agent " << conflict.agent1
                      << " at time " << conflict.time << std::endl;

            std::cout << "MultiCBS: Replanning for agent " << conflict.agent1 << " with obstacle constraint" << std::endl;
            child->solution[conflict.agent1] = findPathForAgent(conflict.agent1, child->constraints[conflict.agent1], time_limit_ / 10.0);

            if (!child->solution[conflict.agent1].empty())
            {
                child->cost = calculateSIC(child->solution);
                children.push_back(child);
                std::cout << "MultiCBS: Child node " << child->id << " created with cost " << child->cost << std::endl;
            }
            else
            {
                std::cout << "MultiCBS: Replanning failed for agent " << conflict.agent1 << std::endl;
            }

            return children;
        }

        // 原有的agent间冲突处理逻辑
        if (conflict.agent1 < 0 || conflict.agent1 >= parent->solution.size() ||
            conflict.agent2 < 0 || conflict.agent2 >= parent->solution.size())
        {
            std::cout << "MultiCBS: Invalid conflict agents" << std::endl;
            return children;
        }

        for (int i = 0; i < 2; i++)
        {
            int agent = (i == 0) ? conflict.agent1 : conflict.agent2;

            auto child = std::make_shared<MultiCBSNode>();
            child->id = node_counter_++;
            child->constraints = parent->constraints;
            child->solution = parent->solution;

            if (conflict.is_edge_conflict)
            {
                auto from_vertices = (i == 0) ? std::vector<long>(conflict.vertices1.begin(), conflict.vertices1.begin() + conflict.vertices1.size() / 2) : std::vector<long>(conflict.vertices2.begin(), conflict.vertices2.begin() + conflict.vertices2.size() / 2);
                auto to_vertices = (i == 0) ? std::vector<long>(conflict.vertices1.begin() + conflict.vertices1.size() / 2, conflict.vertices1.end()) : std::vector<long>(conflict.vertices2.begin() + conflict.vertices2.size() / 2, conflict.vertices2.end());

                child->constraints[agent].push_back(MultiConstraint(agent, from_vertices, to_vertices, conflict.time));
                std::cout << "MultiCBS: Edge constraint for agent " << agent << " at time " << conflict.time << std::endl;
            }
            else
            {
                auto vertices = (i == 0) ? conflict.vertices1 : conflict.vertices2;
                child->constraints[agent].push_back(MultiConstraint(agent, vertices, conflict.time));
                std::cout << "MultiCBS: Vertex constraint for agent " << agent << " at time " << conflict.time << std::endl;
            }

            std::cout << "MultiCBS: Replanning for agent " << agent << " with new constraint" << std::endl;
            child->solution[agent] = findPathForAgent(agent, child->constraints[agent], time_limit_ / 10.0);

            if (!child->solution[agent].empty())
            {
                child->cost = calculateSIC(child->solution);
                children.push_back(child);
                std::cout << "MultiCBS: Child node " << child->id << " created with cost " << child->cost << std::endl;
            }
            else
            {
                std::cout << "MultiCBS: Replanning failed for agent " << agent << std::endl;
            }
        }

        return children;
    }

    std::vector<long> MultiCBS::findPathForAgent(int agent,
                                                 const std::vector<MultiConstraint> &constraints,
                                                 double time_limit)
    {
        stats_["low_level_calls"] = static_cast<double>(stats_["low_level_calls"]) + 1;

        if (_graph == nullptr)
        {
            std::cout << "MultiCBS: Graph pointer is null!" << std::endl;
            return std::vector<long>();
        }

        // 检查起点和终点是否有效
        if (!isValidPosition(agent, agents_[agent].start))
        {
            std::cout << "MultiCBS: Start position invalid for agent " << agent
                      << "! Start: " << agents_[agent].start << std::endl;
            return std::vector<long>();
        }

        if (!isValidPosition(agent, agents_[agent].goal))
        {
            std::cout << "MultiCBS: Goal position invalid for agent " << agent
                      << "! Goal: " << agents_[agent].goal << std::endl;
            return std::vector<long>();
        }

        try
        {
            raplab::AstarSTGrid2d low_level_planner;
            low_level_planner.SetGraphPtr(_graph);

            // 添加多单元格约束到低层规划器
            std::cout << "MultiCBS: Agent " << agent << " constraints: ";
            for (const auto &constraint : constraints)
            {
                if (constraint.agent == agent)
                {
                    if (constraint.is_edge_constraint)
                    {
                        std::cout << "Edge(t=" << constraint.time << ", vertices=[";
                        for (long vertex : constraint.vertices)
                        {
                            std::cout << vertex << " ";
                            low_level_planner.AddNodeCstr(vertex, constraint.time);
                            // 对于边约束，通常还需要约束下一个时间步
                            low_level_planner.AddNodeCstr(vertex, constraint.time + 1);
                        }
                        std::cout << "]) ";
                    }
                    else
                    {
                        std::cout << "Vertex(t=" << constraint.time << ", vertices=[";
                        for (long vertex : constraint.vertices)
                        {
                            std::cout << vertex << " ";
                            low_level_planner.AddNodeCstr(vertex, constraint.time);
                        }
                        std::cout << "]) ";
                    }
                }
            }
            std::cout << std::endl;

            auto path = low_level_planner.PathFinding(agents_[agent].start, agents_[agent].goal, time_limit, 0);

            if (path.empty())
            {
                std::cout << "MultiCBS: Low-level planner returned empty path for agent " << agent << std::endl;
                return std::vector<long>();
            }

            // 验证路径的每个位置是否有效
            for (size_t t = 0; t < path.size(); ++t)
            {
                if (!isValidPosition(agent, path[t]))
                {
                    std::cout << "MultiCBS: Path position invalid at time " << t
                              << ", vertex " << path[t]  << std::endl;
                    // 这里不直接返回失败，而是让高层CBS检测到这个冲突并创建约束
                    // 路径仍然返回，冲突会在validateSolution中检测到
                }
            }

            return path;
        }
        catch (const std::exception &e)
        {
            std::cout << "MultiCBS: Exception in low-level planner: " << e.what() << std::endl;
            return std::vector<long>();
        }
        catch (...)
        {
            std::cout << "MultiCBS: Unknown exception in low-level planner" << std::endl;
            return std::vector<long>();
        }
    }

    double MultiCBS::calculateSIC(const std::vector<std::vector<long>> &solution)
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

    bool MultiCBS::validateSolution(std::shared_ptr<MultiCBSNode> node, MultiConflict &conflict)
    {
        // 首先检查是否有agent没有路径
        for (int i = 0; i < node->solution.size(); i++)
        {
            if (node->solution[i].empty())
            {
                std::cout << "MultiCBS: Agent " << i << " has no path" << std::endl;
                // 这应该被处理为一种冲突
                conflict = MultiConflict(i, -1, std::vector<long>(), std::vector<long>(), 0);
                return false;
            }
        }

        // 然后检查agent之间的冲突
        if (!findFirstConflict(node, conflict))
        {
            // 最后检查agent与障碍物的冲突
            for (int i = 0; i < node->solution.size(); i++)
            {
                for (size_t t = 0; t < node->solution[i].size(); t++)
                {
                    if (!isValidPosition(i, node->solution[i][t]))
                    {
                        std::cout << "MultiCBS: Agent " << i << " collides with obstacle at time "
                                  << t << ", vertex " << node->solution[i][t] << std::endl;
                        auto vertices = getAgentVertices(i, node->solution[i][t]);
                        conflict = MultiConflict(i, -1, vertices, std::vector<long>(), t);
                        return false;
                    }
                }
            }

            std::cout << "MultiCBS: ✅ Valid solution found with cost " << node->cost << std::endl;
            return true;
        }

        std::cout << "MultiCBS: Conflict detected at time " << conflict.time << std::endl;
        return false;
    }

    std::vector<long> MultiCBS::getAgentVertices(int agent, long base_vertex, long time) const
    {
        std::vector<long> vertices;

        // 获取图的指针并转换为Grid2d类型
        Grid2d *grid_ptr = dynamic_cast<Grid2d *>(_graph);
        if (grid_ptr == nullptr)
        {
            std::cout << "MultiCBS: Graph is not a Grid2d, cannot compute agent vertices" << std::endl;
            vertices.push_back(base_vertex); // 回退到单单元格
            return vertices;
        }

        // 获取基点的坐标
        long base_r = grid_ptr->_k2r(base_vertex);
        long base_c = grid_ptr->_k2c(base_vertex);

        int width = agents_[agent].width;
        int height = agents_[agent].height;

        // 假设基点在矩形的左上角，向右和向下扩展
        for (int dr = 0; dr < height; dr++)
        {
            for (int dc = 0; dc < width; dc++)
            {
                long r = base_r + dr;
                long c = base_c + dc;

                // 检查坐标是否在网格范围内 - 修复边界检查
                if (r >= 0 && r < grid_ptr->GetOccuGridPtr()->size() &&
                    c >= 0 && c < grid_ptr->GetOccuGridPtr()->at(0).size())
                {
                    long vertex_id = grid_ptr->_rc2k(r, c);
                    vertices.push_back(vertex_id);
                }
                else
                {
                    // 如果部分超出边界，返回空向量表示无效位置
                    std::cout << "MultiCBS: Agent " << agent << " partially outside grid at ("
                              << r << "," << c << "), grid size: "
                              << grid_ptr->GetOccuGridPtr()->size() << "x"
                              << grid_ptr->GetOccuGridPtr()->at(0).size() << std::endl;
                    return std::vector<long>(); // 返回空向量表示无效
                }
            }
        }

        return vertices;
    }

    bool MultiCBS::checkCollision(int agent1, long base1, int agent2, long base2) const
    {
        // 如果两个agent的基点相同，肯定碰撞
        if (base1 == base2)
        {
            return true;
        }

        auto vertices1 = getAgentVertices(agent1, base1);
        auto vertices2 = getAgentVertices(agent2, base2);

        // 如果任一agent的位置无效（如部分超出边界），认为发生碰撞
        if (vertices1.empty() || vertices2.empty())
        {
            return true;
        }

        // 检查是否有共享的顶点
        std::unordered_set<long> vertex_set1(vertices1.begin(), vertices1.end());
        for (long v2 : vertices2)
        {
            if (vertex_set1.find(v2) != vertex_set1.end())
            {
                return true;
            }
        }

        return false;
    }

    bool MultiCBS::checkObstacleCollision(int agent, long base_vertex) const
    {
        auto vertices = getAgentVertices(agent, base_vertex);

        // 如果位置无效（如部分超出边界），认为与障碍物碰撞
        if (vertices.empty())
        {
            return true;
        }

        Grid2d *grid_ptr = dynamic_cast<Grid2d *>(_graph);
        if (grid_ptr == nullptr)
        {
            std::cout << "MultiCBS: Graph is not a Grid2d, cannot check obstacle collision" << std::endl;
            return false;
        }

        auto occu_grid_ptr = grid_ptr->GetOccuGridPtr();
        if (occu_grid_ptr == nullptr)
        {
            std::cout << "MultiCBS: Occupancy grid pointer is null" << std::endl;
            return false;
        }

        // 检查所有占据的顶点是否都是障碍物
        for (long vertex : vertices)
        {
            long r = grid_ptr->_k2r(vertex);
            long c = grid_ptr->_k2c(vertex);

            // 障碍物检查：cell value > 0 表示障碍物
            if ((*occu_grid_ptr)[r][c] > 0)
            {
                return true;
            }
        }

        return false;
    }

    bool MultiCBS::isValidPosition(int agent, long base_vertex) const
    {
        if (!_graph->HasVertex(base_vertex))
        {
            return false;
        }

        return !checkObstacleCollision(agent, base_vertex);
    }

    PathSet MultiCBS::GetPlan(long nid)
    {
        return final_solution_;
    }

    CostVec MultiCBS::GetPlanCost(long nid)
    {
        return final_cost_;
    }

    std::unordered_map<std::string, double> MultiCBS::GetStats()
    {
        return stats_;
    }

    

} // namespace raplab