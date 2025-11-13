
/*******************************************
 * Author: Zhongqiang Richard Ren.
 * All Rights Reserved.
 *******************************************/

#include "debug.hpp" 
#include "movingai_map_parser.hpp"
#include "movingai_scen_parser.hpp"
#include "search_astar.hpp"
#include <cassert>

int TestAstar();
int TestAstarGrid2d();
int TestMovingAIGrid2D();

int main() {

  TestAstar();

  //TestAstarGrid2d();
  //TestMovingAIGrid2D();

  return 0;
};

int TestAstar() {

  std::cout << "####### TestAstar() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  raplab::PlannerGraph *g_ptr;
  raplab::SparseGraph g;

  timer.Start();
  g.AddVertex(0);
  g.AddVertex(1);
  g.AddVertex(2);
  g.AddVertex(3);
  g.AddVertex(4);

  g.AddArc(0, 1, std::vector<double>({11.3}));
  g.AddArc(1, 0, std::vector<double>({0.3}));
  g.AddEdge(1, 2, std::vector<double>({15.5}));
  g.AddArc(2, 3, std::vector<double>({15.5}));
  g.AddEdge(3, 4, std::vector<double>({16}));
  g.AddEdge(4, 1, std::vector<double>({17.6}));
  g.AddEdge(1, 7, std::vector<double>({9.9}));

  g.AddArc(0, 3, std::vector<double>({6}));

  g_ptr = &g;

  auto pp = raplab::Astar();
  pp.SetGraphPtr(g_ptr);
  auto p = pp.PathFinding(0, 4);
  auto d_all = pp.GetDistAll();
  for (auto vv : p) {
    std::cout << " v = " << vv << " dist = " << d_all[vv] << std::endl;
  }
  for (size_t jj = 0; jj < d_all.size(); jj++) {
    std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
  }

  timer.PrintDuration();

  std::cout << "####### TestAstar() End #######" << std::endl;

  return 1;
};

int TestAstarGrid2d() {

  std::cout << "####### TestAstarGrid2d() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  raplab::Grid2d g;
  std::vector<std::vector<double>> occupancy_grid;
  occupancy_grid.resize(10);
  for (int i = 0; i < 10; i++) {
    occupancy_grid[i].resize(10, 0);
  }
  for (int i = 0; i < 10; i++) {
    if (i == 5) {
      continue;
    }
    occupancy_grid[5][i] = 1;
  }
  g.SetOccuGridPtr(&occupancy_grid);

  timer.Start();
  auto pp = raplab::AstarGrid2d();
  pp.SetGraphPtr(&g);
  auto p = pp.PathFinding(0, 99);

  auto d_all = pp.GetDistAll();
  for (auto vv : p) {
    std::cout << " v = " << vv << " dist = " << d_all[vv] << std::endl;
  }
  for (size_t jj = 0; jj < d_all.size(); jj++) {
    std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
  }

  timer.PrintDuration();

  std::cout << "####### TestAstarGrid2d() End #######" << std::endl;

  return 1;
};

int TestMovingAIGrid2D() {
  std::string mapfile = "../data/arena/arena.map";
  std::string scenfile = "../data/arena/arena.map.scen";
  movingai::gridmap _g(mapfile);
  movingai::scenario_manager scen_mgr;
  scen_mgr.load_scenario(scenfile);

  raplab::SimpleTimer timer;
  raplab::Grid2d g;
  std::vector<std::vector<double>> occupancy_grid;
  int maxh = _g.height_, maxw = _g.width_;
  occupancy_grid.resize(maxh);
  for (int i = 0; i < maxw; i++)
    occupancy_grid[i].resize(maxw, 0);

  for (int y = 0; y < maxh; y++)
    for (int x = 0; x < maxw; x++) {
      if (_g.is_obstacle({x, y}))
        occupancy_grid[y][x] = 1;
    }
	g.SetOccuGridPtr(&occupancy_grid);

	auto pp = raplab::AstarGrid2d();
	pp.SetGraphPtr(&g);

  auto dijk = raplab::Dijkstra();
	dijk.SetGraphPtr(&g);

	for (int i=0; i<scen_mgr.num_experiments(); i++) {
		int sx, sy, gx, gy, sid, gid;
		auto expr = scen_mgr.get_experiment(i);
		sx = expr->startx();
		sy = expr->starty();
		sid = sy * maxw + sx;

		gx = expr->goalx();
		gy = expr->goaly();
		gid = gy * maxw + gx;

		timer.Start();
		auto path = pp.PathFinding(sid, gid);
		auto cost = pp.GetDistValue(path.back());
		auto tcost = timer.GetDurationSecond();

		auto path_dij = dijk.PathFinding(sid, gid);
		auto cost_dij = dijk.GetDistValue(path_dij.back());

		assert ( fabs(cost - cost_dij) < 1e-6);
		// due to the non-corner-cutting rule in movingai benchmark,
		// cost and ref cost might be different
		// reference: https://github.com/gppc-dev/startkit-classic/blob/master/Problem_Definition.md
		std::cout << "raplab::AstarGrid2d Cost: " << cost 
			<< ", dij cost: " << cost_dij
			<< ", ref cost: " << expr->distance() 
			<< " took " << tcost << "s" << std::endl;
	}

  return 1;
}
