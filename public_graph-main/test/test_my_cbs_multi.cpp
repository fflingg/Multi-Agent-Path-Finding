#include "my_cbs_multi.hpp"
#include "debug.hpp"
#include <iostream>
#include <cassert>

int TestMultiCBSBasic();
int TestMultiCBSWithObstacles();
int TestMultiCBSMultipleSizes();
int TestMultiCBSOnLargeGrid();

int main()
{
  std::cout << "=== MultiCBS Test Suite ===" << std::endl;

  TestMultiCBSBasic();
  // TestMultiCBSWithObstacles();
  // TestMultiCBSMultipleSizes();
  // TestMultiCBSOnLargeGrid();

  return 0;
};

int TestMultiCBSBasic()
{
  std::cout << "####### TestMultiCBSBasic() Begin #######" << std::endl;

  try
  {
    std::cout << "Step 1: Creating StateSpaceST..." << std::endl;
    raplab::StateSpaceST g;

    std::vector<std::vector<double>> occupancy_grid;
    int grid_size = 4;

    occupancy_grid.resize(grid_size);
    for (int i = 0; i < grid_size; i++)
    {
      occupancy_grid[i].resize(grid_size, 0);
    }

    occupancy_grid[0][1] = 1;
    occupancy_grid[0][2] = 1;

    occupancy_grid[1][1] = 1;
    // occupancy_grid[1][2] = 1;

    occupancy_grid[2][1] = 1;
    occupancy_grid[2][2] = 1;

    g.SetOccuGridPtr(&occupancy_grid);

    std::cout << "Step 2: Graph created with " << grid_size << "x" << grid_size << " grid" << std::endl;

    std::cout << "Step 3: Initializing MultiCBS..." << std::endl;
    raplab::MultiCBS cbs;

    std::cout << "Step 4: Setting agent sizes..." << std::endl;
    std::vector<std::pair<int, int>> agent_sizes = {
        {1, 1}, 
        {1, 1}  
    };
    cbs.SetAgentSizes(agent_sizes);

    std::cout << "Step 5: Setting graph pointer..." << std::endl;
    cbs.SetGraphPtr(&g);

    std::vector<long> starts = {0, 3}; 
    std::vector<long> goals = {3, 0};  

    std::cout << "Step 6: Starting MultiCBS solve..." << std::endl;
    int result = cbs.Solve(starts, goals, 60.0, 1.0);

    if (result == 0)
    {
      std::cout << "SUCCESS: MultiCBS found solution!" << std::endl;
      auto plan = cbs.GetPlan();
      auto stats = cbs.GetStats();

      std::cout << "Nodes expanded: " << stats["nodes_expanded"] << std::endl;
      std::cout << "Nodes generated: " << stats["nodes_generated"] << std::endl;
      std::cout << "Runtime: " << stats["runtime"] << " seconds" << std::endl;

      for (int i = 0; i < plan.size(); i++)
      {
        std::cout << "Agent " << i << " path: ";
        for (auto v : plan[i])
          std::cout << v << ", ";
        std::cout << std::endl;
      }
    }
    else
    {
      std::cout << "MultiCBS failed with error code: " << result << std::endl;
    }
  }
  catch (const std::exception &e)
  {
    std::cout << "EXCEPTION: " << e.what() << std::endl;
  }

  std::cout << "####### TestMultiCBSBasic() End #######" << std::endl;
  return 1;
}

int TestMultiCBSWithObstacles()
{
  std::cout << "####### TestMultiCBSWithObstacles() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  try
  {
    raplab::StateSpaceST g;
    std::vector<std::vector<double>> occupancy_grid;
    int grid_size = 6;

    occupancy_grid.resize(grid_size);
    for (int i = 0; i < grid_size; i++)
    {
      occupancy_grid[i].resize(grid_size, 0);
    }

    occupancy_grid[1][2] = 1;
    occupancy_grid[2][1] = 1;
    occupancy_grid[2][2] = 1;
    occupancy_grid[2][4] = 1;
    occupancy_grid[3][2] = 1;

    g.SetOccuGridPtr(&occupancy_grid);

    raplab::MultiCBS cbs;


    std::vector<std::pair<int, int>> agent_sizes = {
        {1, 1}, 
        {1, 2}  
    };
    cbs.SetAgentSizes(agent_sizes);
    cbs.SetGraphPtr(&g);



    std::vector<long> starts = {0, 5};  
    std::vector<long> goals = {35, 21}; 

    std::cout << "Grid layout (0=free, 1=obstacle):" << std::endl;
    for (int i = 0; i < grid_size; i++)
    {
      for (int j = 0; j < grid_size; j++)
      {
        std::cout << occupancy_grid[i][j] << " ";
      }
      std::cout << std::endl;
    }
    std::cout << "Agent 0: " << agent_sizes[0].first << "x" << agent_sizes[0].second << std::endl;
    std::cout << "Agent 1: " << agent_sizes[1].first << "x" << agent_sizes[1].second << std::endl;
    std::cout << "Starts: [";
    for (size_t i = 0; i < starts.size(); i++)
    {
      std::cout << starts[i];
      if (i < starts.size() - 1)
        std::cout << ",";
    }
    std::cout << "]" << std::endl;
    std::cout << "Goals: [";
    for (size_t i = 0; i < goals.size(); i++)
    {
      std::cout << goals[i];
      if (i < goals.size() - 1)
        std::cout << ",";
    }
    std::cout << "]" << std::endl;

    int result = cbs.Solve(starts, goals, 30.0, 1.0);

    if (result == 0)
    {
      std::cout << "SUCCESS: MultiCBS found solution with obstacles!" << std::endl;
      auto plan = cbs.GetPlan();
      auto cost = cbs.GetPlanCost();
      auto stats = cbs.GetStats();

      std::cout << "Plan cost: " << cost[0] << std::endl;
      std::cout << "Nodes expanded: " << stats["nodes_expanded"] << std::endl;
      std::cout << "Nodes generated: " << stats["nodes_generated"] << std::endl;
      std::cout << "Low-level calls: " << stats["low_level_calls"] << std::endl;
      std::cout << "Runtime: " << stats["runtime"] << " seconds" << std::endl;

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
      std::cout << "MultiCBS failed to find solution with obstacles. Error code: " << result << std::endl;
    }
  }
  catch (const std::exception &e)
  {
    std::cout << "EXCEPTION: " << e.what() << std::endl;
  }

  timer.PrintDuration();
  std::cout << "####### TestMultiCBSWithObstacles() End #######" << std::endl;

  return 1;
};

int TestMultiCBSMultipleSizes()
{
  std::cout << "####### TestMultiCBSMultipleSizes() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  try
  {
    raplab::StateSpaceST g;
    std::vector<std::vector<double>> occupancy_grid;
    int grid_size = 6;

    occupancy_grid.resize(grid_size);
    for (int i = 0; i < grid_size; i++)
    {
      occupancy_grid[i].resize(grid_size, 0);
    }


    occupancy_grid[0][1] = 1;
    occupancy_grid[0][2] = 1;
    occupancy_grid[0][3] = 1;
    occupancy_grid[0][4] = 1;
    occupancy_grid[1][1] = 1;
    occupancy_grid[1][2] = 1;
    occupancy_grid[1][3] = 1;
    occupancy_grid[1][4] = 1;
    occupancy_grid[4][1] = 1;
    // occupancy_grid[4][2] = 1;
    occupancy_grid[4][3] = 1;
    occupancy_grid[4][4] = 1;
    occupancy_grid[5][1] = 1;
    occupancy_grid[5][2] = 1;
    occupancy_grid[5][3] = 1;
    occupancy_grid[5][4] = 1;
    g.SetOccuGridPtr(&occupancy_grid);

    raplab::MultiCBS cbs;


    std::vector<std::pair<int, int>> agent_sizes = {
        {1, 1}, 
        {1, 2}, 

    };
    cbs.SetAgentSizes(agent_sizes);
    cbs.SetGraphPtr(&g);

    std::vector<long> starts = {0, 5}; 
    std::vector<long> goals = {5, 15}; 

    std::cout << "Testing MultiCBS with 2 agents of different sizes:" << std::endl;
    for (int i = 0; i < agent_sizes.size(); i++)
    {
      std::cout << "  Agent " << i << ": " << agent_sizes[i].first
                << "x" << agent_sizes[i].second << std::endl;
    }

    std::cout << "Grid layout (0=free, 1=obstacle):" << std::endl;
    for (int i = 0; i < grid_size; i++)
    {
      for (int j = 0; j < grid_size; j++)
      {
        std::cout << occupancy_grid[i][j] << " ";
      }
      std::cout << std::endl;
    }

    int result = cbs.Solve(starts, goals, 60.0, 1.0);

    if (result == 0)
    {
      std::cout << "SUCCESS: MultiCBS found solution for multiple agent sizes!" << std::endl;
      auto plan = cbs.GetPlan();
      auto cost = cbs.GetPlanCost();
      auto stats = cbs.GetStats();

      std::cout << "Plan cost: " << cost[0] << std::endl;
      std::cout << "Nodes expanded: " << stats["nodes_expanded"] << std::endl;
      std::cout << "Nodes generated: " << stats["nodes_generated"] << std::endl;
      std::cout << "Low-level calls: " << stats["low_level_calls"] << std::endl;
      std::cout << "Runtime: " << stats["runtime"] << " seconds" << std::endl;

      for (int i = 0; i < plan.size(); i++)
      {
        std::cout << "Agent " << i << " (" << agent_sizes[i].first << "x"
                  << agent_sizes[i].second << ") path length: " << plan[i].size() << std::endl;
        std::cout << "Agent " << i << " path: ";
        for (auto v : plan[i])
          std::cout << v << ", ";
        std::cout << std::endl;
      }
    }
    else
    {
      std::cout << "MultiCBS failed to find solution for multiple agent sizes. Error code: " << result << std::endl;
    }
  }
  catch (const std::exception &e)
  {
    std::cout << "EXCEPTION: " << e.what() << std::endl;
  }

  timer.PrintDuration();
  std::cout << "####### TestMultiCBSMultipleSizes() End #######" << std::endl;

  return 1;
};

int TestMultiCBSOnLargeGrid()
{
  std::cout << "####### TestMultiCBSOnLargeGrid() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  try
  {
    raplab::StateSpaceST g;
    std::vector<std::vector<double>> occupancy_grid;
    int grid_size = 10;

    occupancy_grid.resize(grid_size);
    for (int i = 0; i < grid_size; i++)
    {
      occupancy_grid[i].resize(grid_size, 0);
    }


    for (int i = 1; i < grid_size - 1; i++)
    {
      if (i != 5)
      {
        occupancy_grid[3][i] = 1;
        occupancy_grid[6][i] = 1;
      }
    }
    for (int i = 3; i < 7; i++)
    {
      if (i != 5)
      {
        occupancy_grid[i][3] = 1;
        occupancy_grid[i][6] = 1;
      }
    }

    g.SetOccuGridPtr(&occupancy_grid);

    raplab::MultiCBS cbs;


    std::vector<std::pair<int, int>> agent_sizes = {
        {2, 2}, 
        {1, 2}, 
        //{3, 1}, 
        {1, 1} 
    };
    cbs.SetAgentSizes(agent_sizes);
    cbs.SetGraphPtr(&g);

    // std::vector<long> starts = {0, 9, 90, 99};
    // std::vector<long> goals = {76, 79, 72, 42};
    std::vector<long> starts = {0, 9, 99};
    std::vector<long> goals = {6, 44, 42};

    std::cout << "Testing MultiCBS on " << grid_size << "x" << grid_size << " grid with 4 agents:" << std::endl;
    for (int i = 0; i < agent_sizes.size(); i++)
    {
      std::cout << "  Agent " << i << ": " << agent_sizes[i].first
                << "x" << agent_sizes[i].second << std::endl;
    }

    int result = cbs.Solve(starts, goals, 180.0, 1.0);

    if (result == 0)
    {
      std::cout << "SUCCESS: MultiCBS found solution on large grid!" << std::endl;
      auto plan = cbs.GetPlan();
      auto cost = cbs.GetPlanCost();
      auto stats = cbs.GetStats();

      std::cout << "Plan cost: " << cost[0] << std::endl;
      std::cout << "Nodes expanded: " << stats["nodes_expanded"] << std::endl;
      std::cout << "Nodes generated: " << stats["nodes_generated"] << std::endl;
      std::cout << "Low-level calls: " << stats["low_level_calls"] << std::endl;
      std::cout << "Runtime: " << stats["runtime"] << " seconds" << std::endl;

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
      std::cout << "MultiCBS failed to find solution on large grid. Error code: " << result << std::endl;
    }
  }
  catch (const std::exception &e)
  {
    std::cout << "EXCEPTION: " << e.what() << std::endl;
  }

  timer.PrintDuration();
  std::cout << "####### TestMultiCBSOnLargeGrid() End #######" << std::endl;

  return 1;
}