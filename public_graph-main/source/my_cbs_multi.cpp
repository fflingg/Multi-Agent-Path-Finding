#include "my_cbs_multi.hpp"
#include <chrono>
#include <limits>
#include <algorithm>
#include <iostream>
#include <set>
#include <queue>
#include <unordered_set>

namespace raplab
{

    // CBSNodeMulti implementation
    CBSNodeMulti::CBSNodeMulti()
    {
        constraints.clear();
        solution.clear();
        rect_constraints.clear();
        rect_solution.clear();
        cost = 0.0;
        id = -1;
    }

    CBSNodeMulti::~CBSNodeMulti() {}

    // CBSMulti implementation
    CBSMulti::CBSMulti() : CBS() // 调用基类构造函数
    {
        // 基类CBS的构造函数已经初始化了stats_
    }

    CBSMulti::~CBSMulti() {}

    void CBSMulti::SetRectangleAgents(const std::vector<RectangleAgent> &agents)
    {
        rect_agents_ = agents;
    }

    std::vector<long> CBSMulti::getOccupiedVertices(long base_vertex, int width, int height) const
    {
        std::vector<long> occupied;

        if (_graph == nullptr)
            return occupied;

        auto grid_ptr = dynamic_cast<Grid2d *>(_graph);
        if (grid_ptr == nullptr)
            return occupied;

        auto occ_grid_ptr = grid_ptr->GetOccuGridPtr();
        if (occ_grid_ptr == nullptr)
            return occupied;

        int grid_width = occ_grid_ptr->at(0).size();
        int grid_height = occ_grid_ptr->size();

        int base_row = base_vertex / grid_width;
        int base_col = base_vertex % grid_width;

        // 检查边界 - 如果超出边界，返回空向量
        if (base_row < 0 || base_col < 0 ||
            base_row + height > grid_height || base_col + width > grid_width)
        {
            return occupied; // 返回空向量表示无效位置
        }

        for (int r = base_row; r < base_row + height; r++)
        {
            for (int c = base_col; c < base_col + width; c++)
            {
                occupied.push_back(r * grid_width + c);
            }
        }

        return occupied;
    }

    bool CBSMulti::checkRectangleCollision(const std::vector<long> &rect1, const std::vector<long> &rect2) const
    {
        std::set<long> rect1_set(rect1.begin(), rect1.end());
        for (long v : rect2)
        {
            if (rect1_set.find(v) != rect1_set.end())
            {
                return true;
            }
        }
        return false;
    }

    bool CBSMulti::isObstacle(long vertex) const
    {
        if (_graph == nullptr)
            return true;

        auto grid_ptr = dynamic_cast<Grid2d *>(_graph);
        if (grid_ptr == nullptr)
            return true;

        auto occ_grid_ptr = grid_ptr->GetOccuGridPtr();
        if (occ_grid_ptr == nullptr)
            return true;

        int grid_width = occ_grid_ptr->at(0).size();
        int grid_height = occ_grid_ptr->size();

        int row = vertex / grid_width;
        int col = vertex % grid_width;

        // 检查边界
        if (row < 0 || col < 0 || row >= grid_height || col >= grid_width)
        {
            return true; // 超出边界视为障碍物
        }

        // 检查是否是障碍物 (假设1.0表示障碍物)
        return (occ_grid_ptr->at(row)[col] >= 1.0);
    }

    bool CBSMulti::checkRectangleObstacleCollision(const std::vector<long> &rect_vertices) const
    {
        // 检查矩形中的每个顶点是否是障碍物
        for (long vertex : rect_vertices)
        {
            if (isObstacle(vertex))
            {
                return true;
            }
        }
        return false;
    }

    bool CBSMulti::validateRectanglePosition(long base_vertex, const RectangleAgent &agent) const
    {
        auto occupied = getOccupiedVertices(base_vertex, agent.width, agent.height);
        return !occupied.empty() && !checkRectangleObstacleCollision(occupied);
    }

    int CBSMulti::Solve(std::vector<long> &starts, std::vector<long> &goals, double time_limit, double eps)
    {
        auto start_time = std::chrono::steady_clock::now();

        starts_ = starts;
        goals_ = goals;
        time_limit_ = time_limit;
        eps_ = eps;

        std::cout << "CBSMulti: Starting with " << rect_agents_.size() << " rectangle agents" << std::endl;

        // 如果没有设置矩形智能体，使用默认的1x1智能体
        if (rect_agents_.empty())
        {
            for (int i = 0; i < starts.size(); i++)
            {
                rect_agents_.push_back(RectangleAgent(i, 1, 1, starts[i], goals[i]));
            }
        }

        // Create root node
        auto root = createRootNodeMulti();
        if (root == nullptr)
        {
            std::cout << "CBSMulti: Failed to create root node" << std::endl;
            stats_["runtime"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
            return -1;
        }

        // 使用优先队列
        auto node_compare = [](const std::shared_ptr<CBSNodeMulti> &a, const std::shared_ptr<CBSNodeMulti> &b)
        {
            return a->cost > b->cost; // min-heap
        };
        std::priority_queue<std::shared_ptr<CBSNodeMulti>,
                            std::vector<std::shared_ptr<CBSNodeMulti>>,
                            decltype(node_compare)>
            open_list(node_compare);

        open_list.push(root);
        stats_["nodes_generated"] = 1;

        std::cout << "CBSMulti: Root node created, cost = " << root->cost << std::endl;

        while (!open_list.empty())
        {
            // Check time limit
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - start_time).count();
            if (elapsed > time_limit)
            {
                std::cout << "CBSMulti: Timeout after " << elapsed << " seconds" << std::endl;
                stats_["runtime"] = elapsed;
                return -2;
            }

            // Get best node
            auto current = open_list.top();
            open_list.pop();
            stats_["nodes_expanded"] = static_cast<double>(stats_["nodes_expanded"]) + 1;

            std::cout << "CBSMulti: Expanding node " << current->id << " with cost " << current->cost << std::endl;

            // Validate solution
            RectangleConflict conflict;
            if (validateRectangleSolution(current, conflict))
            {
                // No conflict found - solution is valid
                std::cout << "CBSMulti: Found valid solution!" << std::endl;
                final_solution_ = current->solution;
                final_rect_solution_ = current->rect_solution;
                final_cost_ = {current->cost};
                stats_["runtime"] = elapsed;
                return 0;
            }

            std::cout << "CBSMulti: Conflict found - agent1: " << conflict.agent1
                      << ", agent2: " << conflict.agent2 << ", time: " << conflict.time << std::endl;

            // Generate child nodes
            auto children = generateChildNodesMulti(current, conflict);
            std::cout << "CBSMulti: Generated " << children.size() << " child nodes" << std::endl;

            for (auto &child : children)
            {
                open_list.push(child);
                stats_["nodes_generated"] = static_cast<double>(stats_["nodes_generated"]) + 1;
            }

            if (stats_["nodes_expanded"] > 1000)
            {
                std::cout << "CBSMulti: Node expansion limit reached" << std::endl;
                break;
            }
        }

        auto end_time = std::chrono::steady_clock::now();
        stats_["runtime"] = std::chrono::duration<double>(end_time - start_time).count();
        std::cout << "CBSMulti: No solution found after " << stats_["runtime"] << " seconds" << std::endl;
        return -1;
    }

    std::shared_ptr<CBSNodeMulti> CBSMulti::createRootNodeMulti()
    {
        auto node = std::make_shared<CBSNodeMulti>();
        node->id = 0;

        // Initialize constraints
        node->constraints.resize(rect_agents_.size());
        node->rect_constraints.resize(rect_agents_.size());
        node->solution.resize(rect_agents_.size());
        node->rect_solution.resize(rect_agents_.size());

        bool all_paths_found = true;

        for (int i = 0; i < rect_agents_.size(); i++)
        {
            const auto &agent = rect_agents_[i];
            std::cout << "CBSMulti: Finding path for rectangle agent " << i
                      << " (" << agent.width << "x" << agent.height << ")"
                      << " from " << agent.start << " to " << agent.goal << std::endl;

            auto rect_path = findPathForRectangleAgent(i, node->rect_constraints[i], time_limit_ / 10.0);

            if (rect_path.empty())
            {
                std::cout << "CBSMulti: Failed to find path for rectangle agent " << i << std::endl;
                all_paths_found = false;
            }
            else
            {
                node->rect_solution[i] = rect_path;
                // 提取基点的路径（用于兼容原有接口）
                std::vector<long> base_path;
                for (const auto &positions : rect_path)
                {
                    if (!positions.empty())
                    {
                        base_path.push_back(positions[0]); // 第一个顶点是基点
                    }
                }
                node->solution[i] = base_path;
                std::cout << "CBSMulti: Agent " << i << " path length: " << rect_path.size() << std::endl;
            }
        }

        if (!all_paths_found)
        {
            return nullptr;
        }

        node->cost = calculateSIC(node->solution);
        return node;
    }

    bool CBSMulti::validateRectangleSolution(std::shared_ptr<CBSNodeMulti> node, RectangleConflict &conflict)
    {
        return !findFirstRectangleConflict(node, conflict);
    }

    bool CBSMulti::findFirstRectangleConflict(std::shared_ptr<CBSNodeMulti> node, RectangleConflict &conflict)
    {
        if (node->rect_solution.empty())
        {
            return false;
        }

        // Find maximum path length
        size_t max_length = 0;
        for (const auto &path : node->rect_solution)
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

        // 检查所有时间步的冲突
        for (size_t t = 0; t < max_length; t++)
        {
            // 首先检查与障碍物的冲突和边界冲突
            for (int i = 0; i < node->rect_solution.size(); i++)
            {
                if (node->rect_solution[i].empty())
                    continue;

                std::vector<long> rect_i;
                if (t < node->rect_solution[i].size())
                {
                    rect_i = node->rect_solution[i][t];
                }
                else
                {
                    rect_i = node->rect_solution[i].back(); // Stay at goal
                }

                // 检查边界
                if (rect_i.empty())
                {
                    long base_vertex = (t < node->solution[i].size()) ? node->solution[i][t] : node->solution[i].back();
                    conflict = RectangleConflict(i, -1, {base_vertex}, t);
                    std::cout << "CBSMulti: Agent " << i << " out of bounds at time " << t
                              << " (base vertex: " << base_vertex << ")" << std::endl;
                    return true;
                }

                // 检查障碍物碰撞
                if (checkRectangleObstacleCollision(rect_i))
                {
                    long base_vertex = (t < node->solution[i].size()) ? node->solution[i][t] : node->solution[i].back();
                    conflict = RectangleConflict(i, -1, {base_vertex}, t);
                    std::cout << "CBSMulti: Agent " << i << " collides with obstacle at time " << t
                              << " (base vertex: " << base_vertex << ")" << std::endl;
                    return true;
                }
            }

            // 然后检查智能体间的冲突
            for (int i = 0; i < node->rect_solution.size(); i++)
            {
                for (int j = i + 1; j < node->rect_solution.size(); j++)
                {
                    if (node->rect_solution[i].empty() || node->rect_solution[j].empty())
                    {
                        continue;
                    }

                    // Get occupied positions at time t
                    std::vector<long> rect_i, rect_j;

                    if (t < node->rect_solution[i].size())
                    {
                        rect_i = node->rect_solution[i][t];
                    }
                    else
                    {
                        rect_i = node->rect_solution[i].back(); // Stay at goal
                    }

                    if (t < node->rect_solution[j].size())
                    {
                        rect_j = node->rect_solution[j][t];
                    }
                    else
                    {
                        rect_j = node->rect_solution[j].back(); // Stay at goal
                    }

                    // Check for collision between agents
                    if (checkRectangleCollision(rect_i, rect_j))
                    {
                        // 创建冲突对象 - 只约束基点
                        std::vector<long> base_conflict_vertices;
                        long base_i = (t < node->solution[i].size()) ? node->solution[i][t] : node->solution[i].back();
                        long base_j = (t < node->solution[j].size()) ? node->solution[j][t] : node->solution[j].back();

                        base_conflict_vertices.push_back(base_i); // 智能体i的基点
                        base_conflict_vertices.push_back(base_j); // 智能体j的基点

                        conflict = RectangleConflict(i, j, base_conflict_vertices, t);

                        // 输出调试信息
                        std::cout << "CBSMulti: Agent collision at time " << t << std::endl;
                        std::cout << "  Agent " << i << " base: " << base_i
                                  << ", occupies: ";
                        for (auto v : rect_i)
                            std::cout << v << " ";
                        std::cout << std::endl;
                        std::cout << "  Agent " << j << " base: " << base_j
                                  << ", occupies: ";
                        for (auto v : rect_j)
                            std::cout << v << " ";
                        std::cout << std::endl;
                        std::cout << "  Constraint base vertices: ";
                        for (auto v : conflict.vertices)
                            std::cout << v << " ";
                        std::cout << std::endl;

                        return true;
                    }
                }
            }
        }

        return false;
    }

    std::vector<std::shared_ptr<CBSNodeMulti>> CBSMulti::generateChildNodesMulti(
        std::shared_ptr<CBSNodeMulti> parent, const RectangleConflict &conflict)
    {
        std::vector<std::shared_ptr<CBSNodeMulti>> children;

        // 处理与障碍物的冲突（agent2 == -1）
        if (conflict.agent2 == -1)
        {
            // 只有与障碍物的冲突，只需要为冲突的智能体添加约束
            if (conflict.agent1 >= 0 && conflict.agent1 < parent->rect_solution.size())
            {
                auto child = std::make_shared<CBSNodeMulti>();
                child->id = parent->id + 1;
                child->constraints = parent->constraints;
                child->rect_constraints = parent->rect_constraints;
                child->solution = parent->solution;
                child->rect_solution = parent->rect_solution;

                // 检查是否已经存在相同的约束
                bool constraint_exists = false;
                for (const auto &existing_constraint : child->rect_constraints[conflict.agent1])
                {
                    if (existing_constraint.time == conflict.time &&
                        existing_constraint.vertices == conflict.vertices)
                    {
                        constraint_exists = true;
                        break;
                    }
                }

                if (!constraint_exists)
                {
                    // 添加新的约束
                    child->rect_constraints[conflict.agent1].push_back(
                        RectangleConstraint(conflict.agent1, conflict.vertices, conflict.time));

                    std::cout << "CBSMulti: Replanning for agent " << conflict.agent1
                              << " with NEW obstacle constraint at time " << conflict.time
                              << " (base vertex: " << conflict.vertices[0] << ")" << std::endl;

                    auto new_rect_path = findPathForRectangleAgent(
                        conflict.agent1, child->rect_constraints[conflict.agent1], time_limit_ / 10.0);

                    if (!new_rect_path.empty())
                    {
                        child->rect_solution[conflict.agent1] = new_rect_path;
                        // Update base path
                        std::vector<long> base_path;
                        for (const auto &positions : new_rect_path)
                        {
                            if (!positions.empty())
                            {
                                base_path.push_back(positions[0]);
                            }
                        }
                        child->solution[conflict.agent1] = base_path;
                        child->cost = calculateSIC(child->solution);
                        children.push_back(child);
                        std::cout << "CBSMulti: Child created with cost " << child->cost << std::endl;
                    }
                    else
                    {
                        std::cout << "CBSMulti: Replanning failed for agent " << conflict.agent1 << std::endl;
                    }
                }
                else
                {
                    std::cout << "CBSMulti: Constraint already exists for agent " << conflict.agent1
                              << " at time " << conflict.time << ", skipping..." << std::endl;
                }
            }
        }
        else
        {
            // 处理智能体间的冲突
            // Create child for agent1
            auto child1 = std::make_shared<CBSNodeMulti>();
            child1->id = parent->id + 1;
            child1->constraints = parent->constraints;
            child1->rect_constraints = parent->rect_constraints;
            child1->solution = parent->solution;
            child1->rect_solution = parent->rect_solution;

            if (conflict.agent1 < child1->rect_constraints.size())
            {
                // 检查约束是否已存在
                bool constraint_exists = false;
                for (const auto &existing_constraint : child1->rect_constraints[conflict.agent1])
                {
                    if (existing_constraint.time == conflict.time &&
                        existing_constraint.vertices.size() == 1 &&
                        existing_constraint.vertices[0] == conflict.vertices[0])
                    {
                        constraint_exists = true;
                        break;
                    }
                }

                if (!constraint_exists)
                {
                    // 只为智能体1的基点添加约束
                    std::vector<long> agent1_constraint = {conflict.vertices[0]};
                    child1->rect_constraints[conflict.agent1].push_back(
                        RectangleConstraint(conflict.agent1, agent1_constraint, conflict.time));

                    std::cout << "CBSMulti: Replanning for agent " << conflict.agent1
                              << " with NEW constraint at time " << conflict.time
                              << " (base vertex: " << conflict.vertices[0] << ")" << std::endl;

                    auto new_rect_path = findPathForRectangleAgent(
                        conflict.agent1, child1->rect_constraints[conflict.agent1], time_limit_ / 10.0);

                    if (!new_rect_path.empty())
                    {
                        child1->rect_solution[conflict.agent1] = new_rect_path;
                        std::vector<long> base_path;
                        for (const auto &positions : new_rect_path)
                        {
                            if (!positions.empty())
                            {
                                base_path.push_back(positions[0]);
                            }
                        }
                        child1->solution[conflict.agent1] = base_path;
                        child1->cost = calculateSIC(child1->solution);
                        children.push_back(child1);
                        std::cout << "CBSMulti: Child1 created with cost " << child1->cost << std::endl;
                    }
                }
            }

            // Create child for agent2
            auto child2 = std::make_shared<CBSNodeMulti>();
            child2->id = parent->id + 2;
            child2->constraints = parent->constraints;
            child2->rect_constraints = parent->rect_constraints;
            child2->solution = parent->solution;
            child2->rect_solution = parent->rect_solution;

            if (conflict.agent2 < child2->rect_constraints.size())
            {
                // 检查约束是否已存在
                bool constraint_exists = false;
                for (const auto &existing_constraint : child2->rect_constraints[conflict.agent2])
                {
                    if (existing_constraint.time == conflict.time &&
                        existing_constraint.vertices.size() == 1 &&
                        existing_constraint.vertices[0] == conflict.vertices[1])
                    {
                        constraint_exists = true;
                        break;
                    }
                }

                if (!constraint_exists)
                {
                    // 只为智能体2的基点添加约束
                    std::vector<long> agent2_constraint = {conflict.vertices[1]};
                    child2->rect_constraints[conflict.agent2].push_back(
                        RectangleConstraint(conflict.agent2, agent2_constraint, conflict.time));

                    std::cout << "CBSMulti: Replanning for agent " << conflict.agent2
                              << " with NEW constraint at time " << conflict.time
                              << " (base vertex: " << conflict.vertices[1] << ")" << std::endl;

                    auto new_rect_path = findPathForRectangleAgent(
                        conflict.agent2, child2->rect_constraints[conflict.agent2], time_limit_ / 10.0);

                    if (!new_rect_path.empty())
                    {
                        child2->rect_solution[conflict.agent2] = new_rect_path;
                        std::vector<long> base_path;
                        for (const auto &positions : new_rect_path)
                        {
                            if (!positions.empty())
                            {
                                base_path.push_back(positions[0]);
                            }
                        }
                        child2->solution[conflict.agent2] = base_path;
                        child2->cost = calculateSIC(child2->solution);
                        children.push_back(child2);
                        std::cout << "CBSMulti: Child2 created with cost " << child2->cost << std::endl;
                    }
                }
            }
        }

        return children;
    }

    std::vector<std::vector<long>> CBSMulti::findPathForRectangleAgent(int agent,
                                                                       const std::vector<RectangleConstraint> &constraints, double time_limit)
    {
        const auto &rect_agent = rect_agents_[agent];

        std::cout << "=== CBSMulti: Planning for agent " << agent << " ===" << std::endl;
        std::cout << "Start: " << rect_agent.start << ", Goal: " << rect_agent.goal << std::endl;
        std::cout << "Constraints count: " << constraints.size() << std::endl;

        // 详细输出所有约束
        for (size_t i = 0; i < constraints.size(); i++)
        {
            const auto &c = constraints[i];
            std::cout << "  Constraint[" << i << "]: time=" << c.time << ", vertices=[";
            for (auto v : c.vertices)
                std::cout << v << " ";
            std::cout << "]" << std::endl;
        }

        raplab::AstarSTGrid2d low_level_planner;
        raplab::StateSpaceST stateSpace;
        auto grid_ptr = dynamic_cast<Grid2d *>(_graph);
        if (grid_ptr)
        {
            stateSpace.SetOccuGridPtr(grid_ptr->GetOccuGridPtr());
        }
        low_level_planner.SetGraphPtr(&stateSpace);

        // 添加时空约束
        int constraints_added = 0;
        for (const auto &constraint : constraints)
        {
            if (constraint.agent == agent)
            {
                for (long forbidden_v : constraint.vertices)
                {
                    std::cout << "ADDING CONSTRAINT: agent " << agent << " cannot be at "
                              << forbidden_v << " at time " << constraint.time << std::endl;
                    low_level_planner.AddNodeCstr(forbidden_v, constraint.time);
                    constraints_added++;
                }
            }
        }
        std::cout << "Total constraints added to planner: " << constraints_added << std::endl;

        // 寻找路径
        auto base_path = low_level_planner.PathFinding(rect_agent.start, rect_agent.goal, time_limit, 0);

        if (base_path.empty())
        {
            std::cout << "CBSMulti: No path found!" << std::endl;
            return {};
        }

        // 详细检查路径是否违反约束
        std::cout << "Found path (length=" << base_path.size() << "):" << std::endl;
        bool constraint_violated = false;

        for (size_t t = 0; t < base_path.size(); t++)
        {
            long current_vertex = base_path[t];
            std::cout << "  Time " << t << ": vertex " << current_vertex;

            // 检查这个时间步是否违反任何约束
            bool violates = false;
            for (const auto &constraint : constraints)
            {
                if (constraint.agent == agent && constraint.time == static_cast<long>(t))
                {
                    for (long forbidden_v : constraint.vertices)
                    {
                        if (current_vertex == forbidden_v)
                        {
                            std::cout << " [VIOLATES: should not be at " << forbidden_v << " at time " << t << "]";
                            violates = true;
                            constraint_violated = true;
                        }
                    }
                }
            }

            if (!violates)
            {
                std::cout << " [OK]";
            }
            std::cout << std::endl;
        }

        if (constraint_violated)
        {
            std::cout << "*** CRITICAL ERROR: Path violates constraints! ***" << std::endl;
            // 即使违反约束，也继续处理以观察行为
            // return {};
        }

        // 转换为矩形路径
        std::vector<std::vector<long>> rect_path;
        for (long base_vertex : base_path)
        {
            auto occupied = getOccupiedVertices(base_vertex, rect_agent.width, rect_agent.height);
            rect_path.push_back(occupied);
        }

        std::cout << "CBSMulti: Path returned for agent " << agent << std::endl;
        return rect_path;
    }

    // 必须实现这些纯虚函数
    PathSet CBSMulti::GetPlan(long nid)
    {
        return final_solution_;
    }

    CostVec CBSMulti::GetPlanCost(long nid)
    {
        return final_cost_;
    }

    std::unordered_map<std::string, double> CBSMulti::GetStats()
    {
        return stats_;
    }

} // namespace raplab