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
        long vertex;
        long vertex2; // a second vertex to indicate edge conflict, -1 for node conflict
        long time;
        bool is_edge_conflict;

        // node conflict
        Conflict(int a1 = -1, int a2 = -1, long v = -1, long t = -1)
            : agent1(a1), agent2(a2), vertex(v), vertex2(-1), time(t), is_edge_conflict(false) {}

        // edge conflict
        Conflict(int a1, int a2, long v1, long v2, long t)
            : agent1(a1), agent2(a2), vertex(v1), vertex2(v2), time(t), is_edge_conflict(true) {}
    };

    /**
     * @brief Constraint structure for CBS
     */
    struct Constraint
    {
        int agent;
        long vertex;
        long vertex2; // a second vertex to indicate edge conflict, -1 for node conflict
        long time;
        bool is_edge_constraint;

        // node conflict
        Constraint(int a = -1, long v = -1, long t = -1)
            : agent(a), vertex(v), vertex2(-1), time(t), is_edge_constraint(false) {}

        // edge conflict
        Constraint(int a, long v1, long v2, long t)
            : agent(a), vertex(v1), vertex2(v2), time(t), is_edge_constraint(true) {}
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
            return cost > other.cost;
        }
    };

    struct CBSNodeCompare
    {
        bool operator()(const std::shared_ptr<CBSNode> &a, const std::shared_ptr<CBSNode> &b) const
        {
            return a->cost > b->cost; 
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

    protected:
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