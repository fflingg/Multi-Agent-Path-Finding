#include "my_cbs_multi.hpp"
#include <chrono>
#include <limits>
#include <algorithm>
#include <iostream>
#include <set>
#include <queue>

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
CBSMulti::CBSMulti() : CBS()  // 调用基类构造函数
{
    // 基类CBS的构造函数已经初始化了stats_
}

CBSMulti::~CBSMulti() {}

void CBSMulti::SetRectangleAgents(const std::vector<RectangleAgent>& agents)
{
    rect_agents_ = agents;
}

std::vector<long> CBSMulti::getOccupiedVertices(long base_vertex, int width, int height) const
{
    std::vector<long> occupied;
    
    if (_graph == nullptr) return occupied;
    
    auto grid_ptr = dynamic_cast<Grid2d*>(_graph);
    if (grid_ptr == nullptr) return occupied;
    
    auto occ_grid_ptr = grid_ptr->GetOccuGridPtr();
    if (occ_grid_ptr == nullptr) return occupied;
    
    int grid_width = occ_grid_ptr->at(0).size();
    int grid_height = occ_grid_ptr->size();
    
    int base_row = base_vertex / grid_width;
    int base_col = base_vertex % grid_width;
    
    // 检查边界
    if (base_row < 0 || base_col < 0 || 
        base_row + height > grid_height || base_col + width > grid_width) {
        return occupied;
    }
    
    for (int r = base_row; r < base_row + height; r++) {
        for (int c = base_col; c < base_col + width; c++) {
            occupied.push_back(r * grid_width + c);
        }
    }
    
    return occupied;
}

bool CBSMulti::checkRectangleCollision(const std::vector<long>& rect1, const std::vector<long>& rect2) const
{
    std::set<long> rect1_set(rect1.begin(), rect1.end());
    for (long v : rect2) {
        if (rect1_set.find(v) != rect1_set.end()) {
            return true;
        }
    }
    return false;
}

int CBSMulti::Solve(std::vector<long>& starts, std::vector<long>& goals, double time_limit, double eps)
{
    auto start_time = std::chrono::steady_clock::now();

    starts_ = starts;
    goals_ = goals;
    time_limit_ = time_limit;
    eps_ = eps;

    std::cout << "CBSMulti: Starting with " << rect_agents_.size() << " rectangle agents" << std::endl;

    // 如果没有设置矩形智能体，使用默认的1x1智能体
    if (rect_agents_.empty()) {
        for (int i = 0; i < starts.size(); i++) {
            rect_agents_.push_back(RectangleAgent(i, 1, 1, starts[i], goals[i]));
        }
    }

    // Create root node
    auto root = createRootNodeMulti();
    if (root == nullptr) {
        std::cout << "CBSMulti: Failed to create root node" << std::endl;
        stats_["runtime"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        return -1;
    }

    // 使用优先队列
    auto node_compare = [](const std::shared_ptr<CBSNodeMulti>& a, const std::shared_ptr<CBSNodeMulti>& b) {
        return a->cost > b->cost; // min-heap
    };
    std::priority_queue<std::shared_ptr<CBSNodeMulti>, 
                       std::vector<std::shared_ptr<CBSNodeMulti>>,
                       decltype(node_compare)> open_list(node_compare);
    
    open_list.push(root);
    stats_["nodes_generated"] = 1;

    std::cout << "CBSMulti: Root node created, cost = " << root->cost << std::endl;

    while (!open_list.empty()) {
        // Check time limit
        auto current_time = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - start_time).count();
        if (elapsed > time_limit) {
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
        if (validateRectangleSolution(current, conflict)) {
            // No conflict found - solution is valid
            std::cout << "CBSMulti: Found valid solution!" << std::endl;
            final_solution_ = current->solution;
            final_rect_solution_ = current->rect_solution;
            final_cost_ = {current->cost};
            stats_["runtime"] = elapsed;
            return 0;
        }

        std::cout << "CBSMulti: Rectangle conflict found between agents " << conflict.agent1
                  << " and " << conflict.agent2 << " at time " << conflict.time << std::endl;

        // Generate child nodes
        auto children = generateChildNodesMulti(current, conflict);
        std::cout << "CBSMulti: Generated " << children.size() << " child nodes" << std::endl;

        for (auto& child : children) {
            open_list.push(child);
            stats_["nodes_generated"] = static_cast<double>(stats_["nodes_generated"]) + 1;
        }

        if (stats_["nodes_expanded"] > 1000) {
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

    for (int i = 0; i < rect_agents_.size(); i++) {
        const auto& agent = rect_agents_[i];
        std::cout << "CBSMulti: Finding path for rectangle agent " << i 
                  << " (" << agent.width << "x" << agent.height << ")"
                  << " from " << agent.start << " to " << agent.goal << std::endl;
        
        auto rect_path = findPathForRectangleAgent(i, node->rect_constraints[i], time_limit_ / 10.0);
        
        if (rect_path.empty()) {
            std::cout << "CBSMulti: Failed to find path for rectangle agent " << i << std::endl;
            all_paths_found = false;
        } else {
            node->rect_solution[i] = rect_path;
            // 提取基点的路径（用于兼容原有接口）
            std::vector<long> base_path;
            for (const auto& positions : rect_path) {
                if (!positions.empty()) {
                    base_path.push_back(positions[0]); // 第一个顶点是基点
                }
            }
            node->solution[i] = base_path;
            std::cout << "CBSMulti: Agent " << i << " path length: " << rect_path.size() << std::endl;
        }
    }

    if (!all_paths_found) {
        return nullptr;
    }

    node->cost = calculateSIC(node->solution);
    return node;
}

bool CBSMulti::validateRectangleSolution(std::shared_ptr<CBSNodeMulti> node, RectangleConflict& conflict)
{
    return !findFirstRectangleConflict(node, conflict);
}

bool CBSMulti::findFirstRectangleConflict(std::shared_ptr<CBSNodeMulti> node, RectangleConflict& conflict)
{
    if (node->rect_solution.empty()) {
        return false;
    }

    // Find maximum path length
    size_t max_length = 0;
    for (const auto& path : node->rect_solution) {
        if (path.size() > max_length) {
            max_length = path.size();
        }
    }

    if (max_length == 0) {
        return false;
    }

    // Check for rectangle collisions at each timestep
    for (size_t t = 0; t < max_length; t++) {
        for (int i = 0; i < node->rect_solution.size(); i++) {
            for (int j = i + 1; j < node->rect_solution.size(); j++) {
                if (node->rect_solution[i].empty() || node->rect_solution[j].empty()) {
                    continue;
                }

                // Get occupied positions at time t
                std::vector<long> rect_i, rect_j;
                
                if (t < node->rect_solution[i].size()) {
                    rect_i = node->rect_solution[i][t];
                } else {
                    rect_i = node->rect_solution[i].back(); // Stay at goal
                }
                
                if (t < node->rect_solution[j].size()) {
                    rect_j = node->rect_solution[j][t];
                } else {
                    rect_j = node->rect_solution[j].back(); // Stay at goal
                }

                // Check for collision
                if (checkRectangleCollision(rect_i, rect_j)) {
                    // 创建冲突对象
                    conflict = RectangleConflict(i, j, {}, t);
                    
                    // 记录两个智能体在冲突时间的所有占据顶点
                    std::vector<long> all_conflict_vertices;
                    
                    // 添加智能体i的所有顶点
                    all_conflict_vertices.insert(all_conflict_vertices.end(), rect_i.begin(), rect_i.end());
                    
                    // 添加智能体j的所有顶点  
                    all_conflict_vertices.insert(all_conflict_vertices.end(), rect_j.begin(), rect_j.end());
                    
                    // 去重并排序
                    std::sort(all_conflict_vertices.begin(), all_conflict_vertices.end());
                    auto last = std::unique(all_conflict_vertices.begin(), all_conflict_vertices.end());
                    all_conflict_vertices.erase(last, all_conflict_vertices.end());
                    
                    conflict.vertices = all_conflict_vertices;
                    
                    // 输出调试信息
                    std::cout << "CBSMulti: Detailed conflict at time " << t << std::endl;
                    std::cout << "  Agent " << i << " occupies: ";
                    for (auto v : rect_i) std::cout << v << " ";
                    std::cout << std::endl;
                    std::cout << "  Agent " << j << " occupies: ";
                    for (auto v : rect_j) std::cout << v << " ";
                    std::cout << std::endl;
                    std::cout << "  Constraint vertices: ";
                    for (auto v : conflict.vertices) std::cout << v << " ";
                    std::cout << std::endl;
                    
                    return true;
                }
            }
        }
    }

    return false;
}

std::vector<std::shared_ptr<CBSNodeMulti>> CBSMulti::generateChildNodesMulti(
    std::shared_ptr<CBSNodeMulti> parent, const RectangleConflict& conflict)
{
    std::vector<std::shared_ptr<CBSNodeMulti>> children;

    if (conflict.agent1 < 0 || conflict.agent1 >= parent->rect_solution.size() ||
        conflict.agent2 < 0 || conflict.agent2 >= parent->rect_solution.size()) {
        std::cout << "CBSMulti: Invalid conflict agents" << std::endl;
        return children;
    }

    // Create child for agent1
    auto child1 = std::make_shared<CBSNodeMulti>();
    child1->id = parent->id + 1;
    child1->constraints = parent->constraints;
    child1->rect_constraints = parent->rect_constraints;
    child1->solution = parent->solution;
    child1->rect_solution = parent->rect_solution;

    // Add rectangle constraint for agent1
    if (conflict.agent1 < child1->rect_constraints.size()) {
        child1->rect_constraints[conflict.agent1].push_back(
            RectangleConstraint(conflict.agent1, conflict.vertices, conflict.time));

        std::cout << "CBSMulti: Replanning for rectangle agent " << conflict.agent1 
                  << " with new constraint at time " << conflict.time << std::endl;
        
        auto new_rect_path = findPathForRectangleAgent(
            conflict.agent1, child1->rect_constraints[conflict.agent1], time_limit_ / 10.0);

        if (!new_rect_path.empty()) {
            child1->rect_solution[conflict.agent1] = new_rect_path;
            // Update base path
            std::vector<long> base_path;
            for (const auto& positions : new_rect_path) {
                if (!positions.empty()) {
                    base_path.push_back(positions[0]);
                }
            }
            child1->solution[conflict.agent1] = base_path;
            child1->cost = calculateSIC(child1->solution);
            children.push_back(child1);
            std::cout << "CBSMulti: Child1 created with cost " << child1->cost << std::endl;
        } else {
            std::cout << "CBSMulti: Child1 replanning failed" << std::endl;
        }
    }

    // Create child for agent2 (similar logic)
    auto child2 = std::make_shared<CBSNodeMulti>();
    child2->id = parent->id + 2;
    child2->constraints = parent->constraints;
    child2->rect_constraints = parent->rect_constraints;
    child2->solution = parent->solution;
    child2->rect_solution = parent->rect_solution;

    if (conflict.agent2 < child2->rect_constraints.size()) {
        child2->rect_constraints[conflict.agent2].push_back(
            RectangleConstraint(conflict.agent2, conflict.vertices, conflict.time));

        std::cout << "CBSMulti: Replanning for rectangle agent " << conflict.agent2 
                  << " with new constraint at time " << conflict.time << std::endl;
        
        auto new_rect_path = findPathForRectangleAgent(
            conflict.agent2, child2->rect_constraints[conflict.agent2], time_limit_ / 10.0);

        if (!new_rect_path.empty()) {
            child2->rect_solution[conflict.agent2] = new_rect_path;
            std::vector<long> base_path;
            for (const auto& positions : new_rect_path) {
                if (!positions.empty()) {
                    base_path.push_back(positions[0]);
                }
            }
            child2->solution[conflict.agent2] = base_path;
            child2->cost = calculateSIC(child2->solution);
            children.push_back(child2);
            std::cout << "CBSMulti: Child2 created with cost " << child2->cost << std::endl;
        } else {
            std::cout << "CBSMulti: Child2 replanning failed" << std::endl;
        }
    }

    return children;
}

std::vector<std::vector<long>> CBSMulti::findPathForRectangleAgent(int agent,
    const std::vector<RectangleConstraint>& constraints, double time_limit)
{
    const auto& rect_agent = rect_agents_[agent];
    
    // 使用时空A*而不是普通A*
    raplab::AstarSTGrid2d low_level_planner;
    
    // 创建StateSpaceST适配器
    raplab::StateSpaceST stateSpace;
    auto grid_ptr = dynamic_cast<Grid2d*>(_graph);
    if (grid_ptr) {
        stateSpace.SetOccuGridPtr(grid_ptr->GetOccuGridPtr());
    }
    low_level_planner.SetGraphPtr(&stateSpace);
    
    // 添加时空约束
    for (const auto& constraint : constraints) {
        if (constraint.agent == agent) {
            for (long forbidden_v : constraint.vertices) {
                // 禁止在特定时间占据特定顶点
                low_level_planner.AddNodeCstr(forbidden_v, constraint.time);
            }
        }
    }
    
    // 寻找路径
    auto base_path = low_level_planner.PathFinding(rect_agent.start, rect_agent.goal, time_limit, 0);
    
    if (base_path.empty()) {
        return {};
    }
    
    // 转换为矩形路径
    std::vector<std::vector<long>> rect_path;
    for (long base_vertex : base_path) {
        auto occupied = getOccupiedVertices(base_vertex, rect_agent.width, rect_agent.height);
        if (occupied.empty()) {
            return {};
        }
        rect_path.push_back(occupied);
    }
    
    return rect_path;
}

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
