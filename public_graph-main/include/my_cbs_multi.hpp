#ifndef MY_CBS_MULTI_HPP
#define MY_CBS_MULTI_HPP

#include "my_cbs.hpp"
#include <vector>
#include <memory>
#include <unordered_map>

namespace raplab
{

/**
 * @brief 矩形智能体定义
 */
struct RectangleAgent
{
    int id;
    int width;    // 占据的宽度（列数）
    int height;   // 占据的高度（行数）
    long start;   // 左上角起始位置
    long goal;    // 左上角目标位置
    
    RectangleAgent(int id = -1, int w = 1, int h = 1, long s = -1, long g = -1)
        : id(id), width(w), height(h), start(s), goal(g) {}
};

/**
 * @brief 矩形智能体冲突
 */
struct RectangleConflict
{
    int agent1;
    int agent2;
    std::vector<long> vertices; // 冲突涉及的所有顶点
    long time;
    
    RectangleConflict(int a1 = -1, int a2 = -1, const std::vector<long>& v = {}, long t = -1)
        : agent1(a1), agent2(a2), vertices(v), time(t) {}
};

/**
 * @brief 矩形智能体约束
 */
struct RectangleConstraint
{
    int agent;
    std::vector<long> vertices; // 约束涉及的所有顶点
    long time;
    
    RectangleConstraint(int a = -1, const std::vector<long>& v = {}, long t = -1)
        : agent(a), vertices(v), time(t) {}
};

/**
 * @brief 支持矩形智能体的CBS节点
 */
class CBSNodeMulti : public CBSNode
{
public:
    CBSNodeMulti();
    ~CBSNodeMulti();
    
    std::vector<std::vector<RectangleConstraint>> rect_constraints; // 矩形约束
    std::vector<std::vector<std::vector<long>>> rect_solution;     // 矩形解决方案（每个位置的所有顶点）
};

/**
 * @brief 支持矩形智能体的CBS算法
 */
class CBSMulti : public CBS  // 改为继承CBS而不是MAPFPlanner
{
public:
    CBSMulti();
    virtual ~CBSMulti();

    // 设置矩形智能体
    void SetRectangleAgents(const std::vector<RectangleAgent>& agents);
    
    virtual int Solve(std::vector<long>& starts, std::vector<long>& goals,
                     double time_limit, double eps) override;
    virtual PathSet GetPlan(long nid = -1) override;
    virtual CostVec GetPlanCost(long nid = -1) override;
    virtual std::unordered_map<std::string, double> GetStats() override;

private:
    // 矩形相关方法
    std::vector<long> getOccupiedVertices(long base_vertex, int width, int height) const;
    bool checkRectangleCollision(const std::vector<long>& rect1, const std::vector<long>& rect2) const;
    bool validateRectangleSolution(std::shared_ptr<CBSNodeMulti> node, RectangleConflict& conflict);
    bool findFirstRectangleConflict(std::shared_ptr<CBSNodeMulti> node, RectangleConflict& conflict);
    
    // 障碍物碰撞检测
    bool isObstacle(long vertex) const;
    bool checkRectangleObstacleCollision(const std::vector<long>& rect_vertices) const;
    bool validateRectanglePosition(long base_vertex, const RectangleAgent& agent) const; // 添加这行
    
    // 重写的高层方法
    std::shared_ptr<CBSNodeMulti> createRootNodeMulti();
    std::vector<std::shared_ptr<CBSNodeMulti>> generateChildNodesMulti(
        std::shared_ptr<CBSNodeMulti> parent, const RectangleConflict& conflict);
    
    // 重写的低层方法
    std::vector<std::vector<long>> findPathForRectangleAgent(int agent,
        const std::vector<RectangleConstraint>& constraints, double time_limit);

    // 成员变量
    std::vector<RectangleAgent> rect_agents_;
    std::vector<std::vector<std::vector<long>>> final_rect_solution_;
};

} // namespace raplab

#endif // MY_CBS_MULTI_HPP