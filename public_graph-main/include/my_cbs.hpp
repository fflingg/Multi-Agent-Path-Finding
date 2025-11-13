#ifndef MY_CBS_HPP
#define MY_CBS_HPP

#include "graph.hpp"
#include "search_astar_st.hpp"
#include "mapf_util.hpp"
#include "avltree.hpp"
#include <unordered_map>
#include <vector>
#include <memory>
#include <queue>

namespace raplab
{

    /**
     * @brief Conflict structure for CBS
     */
    struct Conflict
    {
        int agent1;
        int agent2;
        long vertex; // conflict vertex
        long time;   // conflict time

        Conflict(int a1 = -1, int a2 = -1, long v = -1, long t = -1)
            : agent1(a1), agent2(a2), vertex(v), time(t) {}
    };

    /**
     * @brief Constraint structure for CBS
     */
    struct Constraint
    {
        int agent;
        long vertex;
        long time;

        Constraint(int a = -1, long v = -1, long t = -1)
            : agent(a), vertex(v), time(t) {}
    };

    /**
     * @brief CBS Node for high-level search
     */
    class CBSNode
    {
    public:
        CBSNode();
        ~CBSNode();

        std::vector<std::vector<Constraint>> constraints; // constraints per agent
        std::vector<std::vector<long>> solution;          // paths for all agents
        double cost;                                      // sum of individual costs
        int id;

        bool operator<(const CBSNode &other) const
        {
            return cost > other.cost; // for min-heap
        }
    };

    // 比较函数用于优先队列
    struct CBSNodeCompare
    {
        bool operator()(const std::shared_ptr<CBSNode> &a, const std::shared_ptr<CBSNode> &b) const
        {
            return a->cost > b->cost; // min-heap
        }
    };

    /**
     * @brief Conflict-Based Search (CBS) algorithm
     */
    class CBS : public MAPFPlanner
    {
    public:
        CBS();
        virtual ~CBS();

        virtual int Solve(std::vector<long> &starts, std::vector<long> &goals,
                          double time_limit, double eps) override;
        virtual PathSet GetPlan(long nid = -1) override;
        virtual CostVec GetPlanCost(long nid = -1) override;
        virtual std::unordered_map<std::string, double> GetStats() override;

    protected:  // 将protected放在这里，让派生类可以访问
        // High-level methods
        std::shared_ptr<CBSNode> createRootNode();
        bool findFirstConflict(std::shared_ptr<CBSNode> node, Conflict &conflict);
        std::vector<std::shared_ptr<CBSNode>> generateChildNodes(
            std::shared_ptr<CBSNode> parent, const Conflict &conflict);

        // Low-level methods
        std::vector<long> findPathForAgent(int agent,
                                           const std::vector<Constraint> &constraints,
                                           double time_limit);

        // Helper methods
        double calculateSIC(const std::vector<std::vector<long>> &solution);
        bool validateSolution(std::shared_ptr<CBSNode> node, Conflict &conflict);
        bool hasConflict(const std::vector<long> &path1, const std::vector<long> &path2,
                         int agent1, int agent2, Conflict &conflict);

        // Member variables - 改为protected
        std::vector<long> starts_;
        std::vector<long> goals_;
        double time_limit_;
        double eps_;

        std::priority_queue<std::shared_ptr<CBSNode>,
                            std::vector<std::shared_ptr<CBSNode>>,
                            CBSNodeCompare>
            open_list_;
        std::unordered_map<std::string, double> stats_;
        PathSet final_solution_;
        CostVec final_cost_;

        int node_counter_;
    };

} // namespace raplab

#endif // MY_CBS_HPP
