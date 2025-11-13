
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#ifndef ZHONGQIANGREN_BASIC_SEARCH_ASTAR_ST_H_
#define ZHONGQIANGREN_BASIC_SEARCH_ASTAR_ST_H_

#include "search_astar.hpp"
#include <unordered_map>
#include "avltree.hpp"

#define DEBUG_ASTAR_ST 0

namespace raplab{


/**
 * @brief
 */
struct StateST {
	long id;
	long v;
	long t;
	/**
	 * @brief
	 */
	std::string ToStr() ;
	/**
	 * @brief
	 */
	bool operator==(StateST& another);
};


/**
 * @brief
 */
std::ostream& operator<<(std::ostream& os, StateST& s) ;

/**
 * @brief
 */
class StateSpaceST : public Grid2d
{
public:
	/**
	 * @brief
	 */
	StateSpaceST();
	/**
	 * @brief
	 */
	StateSpaceST(Grid2d* ptr);
	/**
	 * @brief
	 */
	~StateSpaceST();
	/**
	 * @brief return successors of a StatePPAMO
	 */
	virtual void GetSuccs(
		StateST& s, 
		std::vector<StateST>* out, 
		std::vector<std::vector<double> >* out_costs) ;
};


/**
 *
 */
class AstarSTGrid2d : public AstarGrid2d
{
public:
	/**
	 *
	 */
	AstarSTGrid2d();
	/**
	 *
	 */
	~AstarSTGrid2d();
	/**
	 * @brief The input v is uesless here in this derived classs.
	 */	
	virtual std::vector<long> GetPath(long v, bool do_reverse) override ;
	/**
	 * @brief New method.
	 */
	virtual void AddNodeCstr(long nid, long t) ;
	/**
	 * @brief New method.
	 */
	virtual void AddEdgeCstr(long u, long v, long t) ;
	
protected:

	virtual int _search() override ;

	/**
	 * @brief 
	 */
	virtual bool _check_terminate(StateST& s) ;
	/**
	 * @brief 
	 */
	virtual long _gen_label_id() ;
	/**
	 * @brief 
	 */
	virtual double _heuristic(long v) override ;
	/**
	 * @brief 
	 */
	virtual void _init_more() override ;
	/**
	 * @brief 
	 */
	virtual bool _collide_check(long v1, long v2, long t) ;

	size_t __vec_alloc_total = 0;
	size_t __vec_alloc_batch = 1024;
	size_t __vec_alloc_batch_max = 1024*4;

	Dijkstra _dijk; // for heuristic

	long _label_id_gen = 0;

	long _ts = 0; // initial time step, usually start from 0.
	std::unordered_map<std::string, double> _g_all;
	std::vector<StateST> _states;

	long _reached_goal_state_id = -1;

	// members for node/edge constraints
	long _last_nc_t = -1;
	std::unordered_map<long, raplab::AVLTree<long> > _avl_node;
	std::unordered_map<long, std::unordered_map<long, raplab::AVLTree<long> > > _avl_edge;
};


int RunAstarSTGrid2d(
  PlannerGraph* g, long vo, long vd, double time_limit, 
  std::vector< std::vector<long> >& ncs, std::vector< std::vector<long> >& ecs, std::vector<long>* out_path) ;

} // end namespace zr

#endif  // ZHONGQIANGREN_BASIC_SEARCH_ASTAR_ST_H_
