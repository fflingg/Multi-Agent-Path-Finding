
/*******************************************
 * Author: Zhongqiang Richard Ren.
 * All Rights Reserved.
 *******************************************/

#include "search_astar_st.hpp"
#include <debug.hpp>
#include <set>
#include <limits>

namespace raplab
{

  std::string StateST::ToStr()
  {
    std::string out = std::to_string(v) + "," + std::to_string(t);
    return out;
  };

  bool StateST::operator==(StateST &another)
  {
    return (v == another.v) && (t == another.t);
  };

  std::ostream &operator<<(std::ostream &os, StateST &s)
  {
    os << "{id:" << s.id << ",v:" << s.v << ",t:" << s.t << "}";
    return os;
  };

  /////////////////////////////////////

  StateSpaceST::StateSpaceST() {};

  StateSpaceST::StateSpaceST(Grid2d *ptr) {

  };

  StateSpaceST::~StateSpaceST() {};

  void StateSpaceST::GetSuccs(
      StateST &s,
      std::vector<StateST> *out,
      std::vector<std::vector<double>> *out_costs)
  {
    auto uAll = Grid2d::GetSuccs(s.v);
    uAll.push_back(s.v); // wait in place
    for (auto &u : uAll)
    {
      int y = _k2r(u); // row
      int x = _k2c(u); // col
      if (!IsWithinBorder(y, x))
      {
        continue;
      }
      if (_occu_grid_ptr->at(y).at(x) != 0)
      { // a static obstacle
        continue;
      }
      size_t kth = out->size();
      out->resize(kth + 1);
      out->at(kth).v = u;
      out->at(kth).t = s.t + 1;
      out_costs->resize(kth + 1);
      out_costs->at(kth).resize(1);
      out_costs->at(kth)[0] = 1; // move cost
    }
    return;
  };

  //////////

  AstarSTGrid2d::AstarSTGrid2d()
  {
    _class_name = "AstarSTGrid2d";
  };

  AstarSTGrid2d::~AstarSTGrid2d() {

  };

  void AstarSTGrid2d::AddNodeCstr(long nid, long t)
  {
    if (_avl_node.find(nid) == _avl_node.end())
    {
      _avl_node[nid] = raplab::AVLTree<long>();
    }
    _avl_node[nid].Add(t); // add this unsafe interval.
    // std::cout << " add node cstr = " << nid << " t = " << t << std::endl;
    if (t > _last_nc_t)
    {
      _last_nc_t = t;
      // std::cout << " _last_nc_t is updated to " << _last_nc_t << std::endl;
    }
    return;
  };

  void AstarSTGrid2d::AddEdgeCstr(long u, long v, long t)
  {
    if (_avl_edge.find(u) == _avl_edge.end())
    {
      _avl_edge[u] = std::unordered_map<long, raplab::AVLTree<long>>();
    }
    if (_avl_edge[u].find(v) == _avl_edge[u].end())
    {
      _avl_edge[u][v] = raplab::AVLTree<long>();
    }
    _avl_edge[u][v].Add(t);
    return;
  };

  int AstarSTGrid2d::_search()
  {

    if (DEBUG_ASTAR_ST)
    {
      std::cout << "[DEBUG] AstarSTGrid2d::_search, _vs = " << _vs << " _vg = " << _vg << " _mode = "
                << _mode << " _time_limit = " << _time_limit << " _class_name = " << _class_name << std::endl;
    }
    if (!(_graph->HasVertex(_vs)))
    {
      // failed, input src does not exists in the graph.
      std::cout << "[ERROR] AstarSTGrid2d, input v_start " << _vs << " does not exist !!" << std::endl;
      throw std::runtime_error("[ERROR] AstarSTGrid2d, input v_start does not exist !!");
      return -1;
    }
    if ((_mode == 0) && !(_graph->HasVertex(_vg)))
    {
      // failed, input goal does not exists in the graph.
      std::cout << "[ERROR] AstarSTGrid2d, input v_goal " << _vg << " does not exist !!" << std::endl;
      throw std::runtime_error("[ERROR] AstarSTGrid2d, input v_goal does not exist !!");
      return -2;
    }

    _init_more();

    auto ss = dynamic_cast<StateSpaceST *>(_graph);

    // ### init ###
    SimpleTimer timer;
    timer.Start();

    // init search
    StateST s0;
    s0.v = _vs;
    s0.t = _ts;
    s0.id = _gen_label_id();
    _states[s0.id] = s0; // id should be the same as the index in _states
    _g_all[s0.ToStr()] = 0;
    _parent[s0.id] = -1;
    _reached_goal_state_id = -1;

    _open.push(Node(s0.id, 0, _heuristic(s0.v)));

    long n_exp = 0;
    long n_gen = 0;

    // main search loop
    while (!_open.empty())
    {

      // check timeout
      if (timer.GetDurationSecond() > _time_limit)
      {
        std::cout << "[INFO] AstarSTGrid2d::Search timeout !" << std::endl;
        break;
      }

      // ## select label l, lexicographic order ##
      auto cur = _open.top();
      _open.pop();
      StateST s = _states[cur.id];
      double g_s = _g_all[s.ToStr()];

      if (DEBUG_ASTAR_ST > 0)
      {
        std::cout << "[DEBUG] ### Pop state = " << s << " g=" << g_s << " h=" << _heuristic(s.v) << " f=" << cur.f() << std::endl;
      }

      if (_check_terminate(s))
      {
        _reached_goal_state_id = s.id;
        break;
      }

      if (DEBUG_ASTAR_ST > 1)
      {
        std::cout << "[DEBUG] ### Exp. " << s << std::endl;
      }

      // ## expand label l ##
      std::vector<StateST> succs;
      std::vector<std::vector<double>> cvecs;
      ss->GetSuccs(s, &succs, &cvecs);

      n_exp++;

      if (DEBUG_ASTAR_ST > 0)
      {
        std::cout << "[DEBUG] get " << succs.size() << " successors" << std::endl;
      }
      for (int idx = 0; idx < succs.size(); idx++)
      {
        auto &s2 = succs[idx];

        // move from (v,t) to (u,t+1), collision check
        if (_collide_check(s.v, s2.v, s.t))
        {
          continue; // this ngh (u,t) is in conflict, skip.
        }

        double g2 = g_s + cvecs[idx][0];

        if (DEBUG_ASTAR_ST > 0)
        {
          std::cout << "[DEBUG] --- Loop s= " << s2 << " g=" << g2 << " h=" << _heuristic(s2.v) << std::endl;
        }

        auto s2str = s2.ToStr();
        if (_g_all.find(s2str) != _g_all.end())
        {
          if (_g_all[s2str] < g2)
          {
            if (DEBUG_ASTAR_ST > 1)
            {
              std::cout << "[DEBUG] ----- pruned, cont..." << std::endl;
            }
            continue; // prune
          }
        }

        s2.id = _gen_label_id();
        _states[s2.id] = s2;
        _parent[s2.id] = s.id;
        _g_all[s2str] = g2;

        n_gen++;

        if (DEBUG_ASTAR_ST > 0)
        {
          std::cout << "[DEBUG] ----- Add to open..." << std::endl;
        }

        // cannot use Astar::_add_open() since there it is label ID rather than vertex ID.
        auto h2 = _wh * _heuristic(s2.v);
        _open.push(Node(s2.id, g2, h2));

      } // end for
    } // end while

    // if (DEBUG_ASTAR_ST > 0){
    std::cout << "[INFO] AstarSTGrid2d::Search exit after " << timer.GetDurationSecond() << " seconds with n_exp=" << n_exp << " n_gen=" << n_gen << std::endl;
    // }
    return 1;
  };

  bool AstarSTGrid2d::_check_terminate(StateST &s)
  {
    if ((s.v == _vg) && (s.t > _last_nc_t))
    {
      return true;
    }
    return false;
  };

  long AstarSTGrid2d::_gen_label_id()
  {

    long out = _label_id_gen++;

    if (_label_id_gen > __vec_alloc_total)
    {
      __vec_alloc_total += __vec_alloc_batch;
      _states.resize(__vec_alloc_total);
      _parent.resize(__vec_alloc_total);
      if (__vec_alloc_batch < __vec_alloc_batch_max)
      {
        __vec_alloc_batch += __vec_alloc_batch;
      }
    }

    return out;
  };

  /////////////

  double AstarSTGrid2d::_heuristic(long v)
  {

    // auto gg = dynamic_cast<Grid2d*>( _graph );
    // auto r = gg->_k2r(v);
    // auto c = gg->_k2c(v);
    // return abs(r - _vd_r) + abs(c - _vd_c);

    // --- Dijkstra ---
    auto ss = dynamic_cast<StateSpaceST *>(_graph);
    auto out = _dijk.GetDistValue(v);
    if (out < 0)
    {
      throw std::runtime_error("[ERROR], unavailable heuristic !?");
    }
    // std::cout << " h(" << v << ") = " << out << std::endl;
    return out;
  };

  void AstarSTGrid2d::_init_more()
  {

    AstarGrid2d::_init_more();

    // SimpleTimer timer;
    // timer.Start();
    _dijk.SetGraphPtr(_graph);
    _dijk.ExhaustiveBackwards(_vg, std::numeric_limits<double>::infinity(), 0);
    // _res.rt_initHeu = timer.GetDurationSecond();
  };

  bool AstarSTGrid2d::_collide_check(long v1, long v2, long t)
  {
    // return true if in collision; return false if not in collision
    // node constraint check
    if (_avl_node[v2].Find(t + 1).h != 0)
    {
      // the input t overlaps with exact a node constraint.
      return true;
    }
    // edge constraint check
    if (_avl_edge.find(v1) == _avl_edge.end())
    {
      return false;
    }
    if (_avl_edge[v1].find(v2) == _avl_edge[v1].end())
    {
      return false;
    }
    if (_avl_edge[v1][v2].Find(t).h == 0)
    {
      return false;
    }
    return true;
  };

  std::vector<long> AstarSTGrid2d::GetPath(long v, bool do_reverse)
  {
    // the input v is never used here...
    if ((_parent.size() == 0) || (_reached_goal_state_id == -1))
    {
      return std::vector<long>();
    }
    long sid = _reached_goal_state_id;
    std::vector<long> out;
    out.push_back(_states[sid].v);
    while (_parent[sid] != -1)
    {
      out.push_back(_states[_parent[sid]].v);
      sid = _parent[sid];
    }

    if (do_reverse)
    {
      std::vector<long> path;
      for (size_t i = 0; i < out.size(); i++)
      {
        path.push_back(out[out.size() - 1 - i]);
      }
      return path;
    }
    else
    {
      return out;
    }
  };

  int RunAstarSTGrid2d(
      PlannerGraph *g, long vo, long vd, double time_limit,
      std::vector<std::vector<long>> &ncs, std::vector<std::vector<long>> &ecs, std::vector<long> *out_path)
  {
    auto g2 = dynamic_cast<Grid2d *>(g);
    raplab::StateSpaceST ss;
    ss.SetOccuGridPtr(g2->GetOccuGridPtr());

    auto planner = raplab::AstarSTGrid2d();
    planner.SetGraphPtr(&ss);

    for (auto nc : ncs)
    {
      planner.AddNodeCstr(nc[0], nc[1]);
    }
    for (auto ec : ecs)
    {
      planner.AddEdgeCstr(ec[0], ec[1], ec[2]);
    }

    // pp.SetHeuWeight(1.2);
    *out_path = planner.PathFinding(vo, vd, time_limit);
    return out_path->size();
  };

} // end namespace zr
