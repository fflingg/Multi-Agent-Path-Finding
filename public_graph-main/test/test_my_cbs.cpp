#include "my_cbs.hpp"
#include "debug.hpp"
#include <iostream>
#include <cassert>

int TestCBSBasic();
int TestCBSWithObstacles();
int TestCBSWithObstacles2();
int TestCBSMultipleAgents();
int TestCBSOnGrid();

int main()
{
  // TestCBSBasic();
  // TestCBSWithObstacles2();
  // TestCBSMultipleAgents();
  TestCBSOnGrid();

  return 0;
};

int TestCBSBasic()
{
  std::cout << "####### TestCBSBasic() Begin #######" << std::endl;

  try
  {
    std::cout << "Step 1: Creating StateSpaceST..." << std::endl;
    raplab::StateSpaceST g;

    // 设置网格大小（3x3网格）
    std::vector<std::vector<double>> occupancy_grid;
    int grid_size = 3;

    occupancy_grid.resize(grid_size);
    for (int i = 0; i < grid_size; i++)
    {
      occupancy_grid[i].resize(grid_size, 0); // 0表示空闲，1表示障碍物
    }

    g.SetOccuGridPtr(&occupancy_grid);

    std::cout << "Step 2: Graph created with " << grid_size << "x" << grid_size << " grid" << std::endl;

    // 验证图连接性
    std::cout << "Step 3: Verifying graph connectivity..." << std::endl;
    for (int i = 0; i < grid_size * grid_size; i++)
    {
      auto succs = g.Grid2d::GetSuccs(i);
      std::cout << "  Vertex " << i << " has " << succs.size() << " successors: ";
      for (auto s : succs)
        std::cout << s << " ";
      std::cout << std::endl;
    }

    // Test individual A* first
    std::cout << "Step 4: Testing individual A*..." << std::endl;
    raplab::Astar astar;
    astar.SetGraphPtr(&g);
    auto path0 = astar.PathFinding(0, 8, 5.0, 0);
    auto path1 = astar.PathFinding(8, 0, 5.0, 0);

    std::cout << "  A* path 0->8: ";
    for (auto v : path0)
      std::cout << v << " ";
    std::cout << std::endl;

    std::cout << "  A* path 8->0: ";
    for (auto v : path1)
      std::cout << v << " ";
    std::cout << std::endl;

    // Now test CBS
    std::cout << "Step 5: Initializing CBS..." << std::endl;
    raplab::CBS cbs;

    std::cout << "Step 6: Setting graph pointer..." << std::endl;
    cbs.SetGraphPtr(&g);

    std::vector<long> starts = {0, 8};
    std::vector<long> goals = {8, 0};

    std::cout << "Step 7: Starting CBS solve..." << std::endl;
    int result = cbs.Solve(starts, goals, 10.0, 1.0);

    if (result == 0)
    {
      std::cout << "SUCCESS: CBS found solution!" << std::endl;
      auto plan = cbs.GetPlan();
      auto stats = cbs.GetStats();

      for (int i = 0; i < plan.size(); i++)
      {
        std::cout << "Agent " << i << " path: ";
        for (auto v : plan[i])
          std::cout << v << " ";
        std::cout << std::endl;
      }
    }
    else
    {
      std::cout << "CBS failed with error code: " << result << std::endl;
    }
  }
  catch (const std::exception &e)
  {
    std::cout << "EXCEPTION: " << e.what() << std::endl;
  }

  std::cout << "####### TestCBSBasic() End #######" << std::endl;
  return 1;
}

int TestCBSWithObstacles()
{
  std::cout << "####### TestCBSWithObstacles() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  // 修改点1: 使用 StateSpaceST 替代 Grid2d
  raplab::StateSpaceST g;
  std::vector<std::vector<double>> occupancy_grid;
  int grid_size = 5;

  occupancy_grid.resize(grid_size);
  for (int i = 0; i < grid_size; i++)
  {
    occupancy_grid[i].resize(grid_size, 0);
  }

  // Add some obstacles
  occupancy_grid[1][1] = 1;
  occupancy_grid[1][2] = 1;
  occupancy_grid[1][3] = 1;
  occupancy_grid[3][1] = 1;
  occupancy_grid[3][2] = 1;
  occupancy_grid[3][3] = 1;

  g.SetOccuGridPtr(&occupancy_grid);

  // Test CBS with 2 agents
  raplab::CBS cbs;
  cbs.SetGraphPtr(&g);

  // Agent 0: top-left to bottom-right
  // Agent 1: top-right to bottom-left
  std::vector<long> starts = {0, 4};  // (0,0) and (0,4)
  std::vector<long> goals = {24, 20}; // (4,4) and (4,0)

  int result = cbs.Solve(starts, goals, 10.0, 1.0);

  if (result == 0)
  {
    std::cout << "CBS found solution with obstacles!" << std::endl;
    auto plan = cbs.GetPlan();
    auto cost = cbs.GetPlanCost();
    auto stats = cbs.GetStats();

    std::cout << "Plan cost: " << cost[0] << std::endl;
    std::cout << "Nodes expanded: " << stats["nodes_expanded"] << std::endl;
    std::cout << "Runtime: " << stats["runtime"] << " seconds" << std::endl;

    for (int i = 0; i < plan.size(); i++)
    {
      std::cout << "Agent " << i << " path length: " << plan[i].size() << std::endl;
      std::cout << "Agent " << i << " path: ";
      for (auto v : plan[i])
        std::cout << v << " ";
      std::cout << std::endl;
    }
  }
  else
  {
    std::cout << "CBS failed to find solution with obstacles. Error code: " << result << std::endl;
  }

  timer.PrintDuration();
  std::cout << "####### TestCBSWithObstacles() End #######" << std::endl;

  return 1;
};

int TestCBSWithObstacles2()
{
  std::cout << "####### TestCBSWithObstacles() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  // 修改点1: 使用 StateSpaceST 替代 Grid2d
  raplab::StateSpaceST g;
  std::vector<std::vector<double>> occupancy_grid;
  int grid_size = 4;

  occupancy_grid.resize(grid_size);
  for (int i = 0; i < grid_size; i++)
  {
    occupancy_grid[i].resize(grid_size, 0);
  }

  // Add some obstacles
  occupancy_grid[0][1] = 1;
  occupancy_grid[0][2] = 1;

  occupancy_grid[1][1] = 1;
  //occupancy_grid[1][2] = 1;

  occupancy_grid[2][1] = 1;
  occupancy_grid[2][2] = 1;


  g.SetOccuGridPtr(&occupancy_grid);

  // Test CBS with 2 agents
  raplab::CBS cbs;
  cbs.SetGraphPtr(&g);

  // Agent 0: top-left to bottom-right
  // Agent 1: top-right to bottom-left
  std::vector<long> starts = {0, 3};  // (0,0) and (0,4)
  std::vector<long> goals = {3, 0}; // (4,4) and (4,0)

  int result = cbs.Solve(starts, goals, 30.0, 1.0);

  if (result == 0)
  {
    std::cout << "CBS found solution with obstacles!" << std::endl;
    auto plan = cbs.GetPlan();
    auto cost = cbs.GetPlanCost();
    auto stats = cbs.GetStats();

    std::cout << "Plan cost: " << cost[0] << std::endl;
    std::cout << "Nodes expanded: " << stats["nodes_expanded"] << std::endl;
    std::cout << "Runtime: " << stats["runtime"] << " seconds" << std::endl;

    for (int i = 0; i < plan.size(); i++)
    {
      std::cout << "Agent " << i << " path length: " << plan[i].size() << std::endl;
      std::cout << "Agent " << i << " path: ";
      for (auto v : plan[i])
        std::cout << v << " ";
      std::cout << std::endl;
    }
  }
  else
  {
    std::cout << "CBS failed to find solution with obstacles. Error code: " << result << std::endl;
  }

  timer.PrintDuration();
  std::cout << "####### TestCBSWithObstacles() End #######" << std::endl;

  return 1;
};

int TestCBSMultipleAgents()
{
  std::cout << "####### TestCBSMultipleAgents() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  // 修改点2: 使用 StateSpaceST 替代 SparseGraph
  raplab::StateSpaceST g;
  std::vector<std::vector<double>> occupancy_grid;
  int grid_size = 4; // 4x4 grid

  occupancy_grid.resize(grid_size);
  for (int i = 0; i < grid_size; i++)
  {
    occupancy_grid[i].resize(grid_size, 0); // 全部空闲
  }

  g.SetOccuGridPtr(&occupancy_grid);

  // Test CBS with 3 agents
  raplab::CBS cbs;
  cbs.SetGraphPtr(&g);

  // 3 agents with different start and goal positions
  // 修改点3: 更新顶点编号为4x4网格的编号
  std::vector<long> starts = {0, 3, 12}; // Top-left (0,0), top-right (0,3), bottom-left (3,0)
  std::vector<long> goals = {15, 12, 3}; // Bottom-right (3,3), bottom-left (3,0), top-right (0,3)

  int result = cbs.Solve(starts, goals, 10.0, 1.0);

  if (result == 0)
  {
    std::cout << "CBS found solution for 3 agents!" << std::endl;
    auto plan = cbs.GetPlan();
    auto cost = cbs.GetPlanCost();
    auto stats = cbs.GetStats();

    std::cout << "Plan cost: " << cost[0] << std::endl;
    std::cout << "Nodes expanded: " << stats["nodes_expanded"] << std::endl;
    std::cout << "Nodes generated: " << stats["nodes_generated"] << std::endl;
    std::cout << "Runtime: " << stats["runtime"] << " seconds" << std::endl;

    for (int i = 0; i < plan.size(); i++)
    {
      std::cout << "Agent " << i << " path length: " << plan[i].size()
                << ", path: ";
      for (auto v : plan[i])
        std::cout << v << " ";
      std::cout << std::endl;
    }
  }
  else
  {
    std::cout << "CBS failed to find solution for 3 agents. Error code: " << result << std::endl;
  }

  timer.PrintDuration();
  std::cout << "####### TestCBSMultipleAgents() End #######" << std::endl;

  return 1;
};

int TestCBSOnGrid()
{
  std::cout << "####### TestCBSOnGrid() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  // 修改点4: 使用 StateSpaceST 替代 Grid2d
  raplab::StateSpaceST g;
  std::vector<std::vector<double>> occupancy_grid;
  int grid_size = 8;

  occupancy_grid.resize(grid_size);
  for (int i = 0; i < grid_size; i++)
  {
    occupancy_grid[i].resize(grid_size, 0);
  }

  // Add corridor-like obstacles
  for (int i = 1; i < grid_size - 1; i++)
  {
    if (i != grid_size / 2)
    {
      occupancy_grid[i][grid_size / 2] = 1;
    }
  }

  g.SetOccuGridPtr(&occupancy_grid);

  // Test CBS with agents that need to coordinate through narrow passage
  raplab::CBS cbs;
  cbs.SetGraphPtr(&g);

  // Agents on opposite sides of the narrow passage
  std::vector<long> starts = {
      0,                                            // Top-left (0,0)
      (grid_size - 1) * grid_size + 0,              // Bottom-left (7,0)
      grid_size - 1,                                // Top-right (0,7)
      (grid_size - 1) * grid_size + (grid_size - 1) // Bottom-right (7,7)
  };

  std::vector<long> goals = {
      (grid_size - 1) * grid_size + (grid_size - 1), // Bottom-right
      grid_size - 1,                                 // Top-right
      (grid_size - 1) * grid_size + 0,               // Bottom-left
      0                                              // Top-left
  };

  std::cout << "Testing CBS with 4 agents on " << grid_size << "x" << grid_size << " grid..." << std::endl;
  std::cout << "Grid layout (0=free, 1=obstacle):" << std::endl;
  for (int i = 0; i < grid_size; i++)
  {
    for (int j = 0; j < grid_size; j++)
    {
      std::cout << occupancy_grid[i][j] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << "Starts: " << starts << std::endl;
  std::cout << "Goals: " << goals << std::endl;

  int result = cbs.Solve(starts, goals, 30.0, 1.0); // Longer time limit for more complex problem

  if (result == 0)
  {
    std::cout << "CBS found solution for 4 agents on grid!" << std::endl;
    auto plan = cbs.GetPlan();
    auto cost = cbs.GetPlanCost();
    auto stats = cbs.GetStats();

    std::cout << "Plan cost: " << cost[0] << std::endl;
    std::cout << "Nodes expanded: " << stats["nodes_expanded"] << std::endl;
    std::cout << "Nodes generated: " << stats["nodes_generated"] << std::endl;
    std::cout << "Runtime: " << stats["runtime"] << " seconds" << std::endl;

    // Print path lengths and paths
    for (int i = 0; i < plan.size(); i++)
    {
      std::cout << "Agent " << i << " path length: " << plan[i].size() << std::endl;
      std::cout << "Agent " << i << " path: ";
      for (auto v : plan[i])
        std::cout << v << ", ";
      std::cout << std::endl;
    }
  }
  else
  {
    std::cout << "CBS failed to find solution for 4 agents. Error code: " << result << std::endl;
  }

  timer.PrintDuration();
  std::cout << "####### TestCBSOnGrid() End #######" << std::endl;

  return 1;
}