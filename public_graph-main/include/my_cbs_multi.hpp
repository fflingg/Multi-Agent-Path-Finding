#ifndef MY_CBS_MULTI_HPP
#define MY_CBS_MULTI_HPP

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
     * @brief Agent definition with size information
     */
    struct Agent
    {
        long start;
        long goal;
        int width;   // size in x-direction
        int height;  // size in y-direction
        
        Agent(long s = -1, long g = -1, int w = 1, int h = 1) 
            : start(s), goal(g), width(w), height(h) {}
    };

    /**
     * @brief Conflict structure for CBS with multi-cell agents
     */
    struct MultiConflict
    {
        int agent1;
        int agent2;
        std::vector<long> vertices1; // all vertices occupied by agent1 at conflict time
        std::vector<long> vertices2; // all vertices occupied by agent2 at conflict time
        long time;
        bool is_edge_conflict;

        // vertex conflict (overlap at same time)
        MultiConflict(int a1 = -1, int a2 = -1, const std::vector<long>& v1 = {}, 
                     const std::vector<long>& v2 = {}, long t = -1)
            : agent1(a1), agent2(a2), vertices1(v1), vertices2(v2), time(t), is_edge_conflict(false) {}

        // edge conflict (swapping conflict)
        MultiConflict(int a1, int a2, const std::vector<long>& v1_from, const std::vector<long>& v1_to,
                     const std::vector<long>& v2_from, const std::vector<long>& v2_to, long t)
            : agent1(a1), agent2(a2), time(t), is_edge_conflict(true) 
        {
            vertices1 = v1_from;
            vertices1.insert(vertices1.end(), v1_to.begin(), v1_to.end());
            vertices2 = v2_from;
            vertices2.insert(vertices2.end(), v2_to.begin(), v2_to.end());
        }
    };

    /**
     * @brief Constraint structure for CBS with multi-cell agents
     */
    struct MultiConstraint
    {
        int agent;
        std::vector<long> vertices; // all vertices that are constrained
        long time;
        bool is_edge_constraint;

        // vertex constraint
        MultiConstraint(int a = -1, const std::vector<long>& v = {}, long t = -1)
            : agent(a), vertices(v), time(t), is_edge_constraint(false) {}

        // edge constraint
        MultiConstraint(int a, const std::vector<long>& from_vertices, 
                       const std::vector<long>& to_vertices, long t)
            : agent(a), time(t), is_edge_constraint(true) 
        {
            vertices = from_vertices;
            vertices.insert(vertices.end(), to_vertices.begin(), to_vertices.end());
        }
    };

    /**
     * @brief CBS Node for high-level search with multi-cell agents
     */
    class MultiCBSNode
    {
    public:
        MultiCBSNode();
        ~MultiCBSNode();

        std::vector<std::vector<MultiConstraint>> constraints; // constraints per agent
        std::vector<std::vector<long>> solution;               // base point paths for all agents
        double cost;                                           // sum of individual costs
        int id;

        bool operator<(const MultiCBSNode &other) const
        {
            return cost > other.cost;
        }
    };

    struct MultiCBSNodeCompare
    {
        bool operator()(const std::shared_ptr<MultiCBSNode> &a, const std::shared_ptr<MultiCBSNode> &b) const
        {
            return a->cost > b->cost; 
        }
    };

    /**
     * @brief Conflict-Based Search (CBS) algorithm for multi-cell agents
     */
    class MultiCBS : public MAPFPlanner
    {
    public:
        MultiCBS();
        virtual ~MultiCBS();

        virtual int Solve(std::vector<long> &starts, std::vector<long> &goals,
                          double time_limit, double eps) override;
        virtual PathSet GetPlan(long nid = -1) override;
        virtual CostVec GetPlanCost(long nid = -1) override;
        virtual std::unordered_map<std::string, double> GetStats() override;

        // Set agent sizes (must be called before Solve)
        void SetAgentSizes(const std::vector<std::pair<int, int>>& sizes);

    protected:
        // High-level methods
        std::shared_ptr<MultiCBSNode> createRootNode();
        bool findFirstConflict(std::shared_ptr<MultiCBSNode> node, MultiConflict &conflict);
        std::vector<std::shared_ptr<MultiCBSNode>> generateChildNodes(
            std::shared_ptr<MultiCBSNode> parent, const MultiConflict &conflict);

        // Low-level methods
        std::vector<long> findPathForAgent(int agent,
                                           const std::vector<MultiConstraint> &constraints,
                                           double time_limit);

        // Helper methods
        double calculateSIC(const std::vector<std::vector<long>> &solution);
        bool validateSolution(std::shared_ptr<MultiCBSNode> node, MultiConflict &conflict);
        
        // Multi-cell specific methods
        std::vector<long> getAgentVertices(int agent, long base_vertex, long time = -1) const;
        bool checkCollision(int agent1, long base1, int agent2, long base2) const;
        bool checkObstacleCollision(int agent, long base_vertex) const;
        bool isValidPosition(int agent, long base_vertex) const;

        std::vector<Agent> agents_;
        std::vector<long> starts_;
        std::vector<long> goals_;
        double time_limit_;
        double eps_;

        std::priority_queue<std::shared_ptr<MultiCBSNode>,
                            std::vector<std::shared_ptr<MultiCBSNode>>,
                            MultiCBSNodeCompare>
            open_list_;
        std::unordered_map<std::string, double> stats_;
        PathSet final_solution_;
        CostVec final_cost_;

        int node_counter_;
    };

} // namespace raplab

#endif // MY_CBS_MULTI_HPP