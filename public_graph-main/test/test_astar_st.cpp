
/*******************************************
 * Author: Zhongqiang Richard Ren.
 * All Rights Reserved.
 *******************************************/

#include "search_astar_st.hpp"
#include "debug.hpp"

int TestAstarSTGrid2d();

int main()
{

  TestAstarSTGrid2d();

  return 0;
};

int TestAstarSTGrid2d()
{

  std::cout << "####### TestAstarSTGrid2d() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  // raplab::Grid2d
  raplab::StateSpaceST g;
  std::vector<std::vector<double>> occupancy_grid;
  occupancy_grid.resize(10);
  for (int i = 0; i < 10; i++)
  {
    occupancy_grid[i].resize(10, 0);
  }
  for (int i = 0; i < 10; i++)
  {
    if (i == 5)
    {
      continue;
    }
    occupancy_grid[5][i] = 1;
  }
  g.SetOccuGridPtr(&occupancy_grid);

  timer.Start();
  auto pp = raplab::AstarSTGrid2d();
  pp.SetGraphPtr(&g);
  pp.AddNodeCstr(3, 3);
  pp.AddNodeCstr(12, 3);
  pp.AddEdgeCstr(24, 25, 6);
  pp.AddNodeCstr(99, 20);
  pp.SetHeuWeight(1.2);
  auto p = pp.PathFinding(0, 99, 1);

  // auto d_all = pp.GetDistAll();
  // for (auto vv : p) {
  //   std::cout << " v = " << vv << " dist = " << d_all[vv] << std::endl;
  // }
  // for (size_t jj = 0; jj < d_all.size(); jj++) {
  //   std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
  // }

  std::cout << " p = " << p << std::endl;

  timer.PrintDuration();

  std::cout << "####### TestAstarSTGrid2d() End #######" << std::endl;

  return 1;
};
