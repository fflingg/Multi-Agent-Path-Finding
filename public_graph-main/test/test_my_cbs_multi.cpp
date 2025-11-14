#include "my_cbs_multi.hpp"
#include "debug.hpp"
#include <iostream>
#include <cassert>


int TestCBSMultiWithObstacles1();
int TestCBSMultiWithObstacles2();


int main()
{
  std::cout << "=== Testing CBS for Rectangle Agents ===" << std::endl;
  
  TestCBSMultiWithObstacles1();
  //TestCBSMultiWithObstacles2();
  return 0;
}



int TestCBSMultiWithObstacles1()
{
  std::cout << "\n####### TestCBSMultiWithObstacles() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  try
  {
    std::cout << "Step 1: Creating StateSpaceST with obstacles..." << std::endl;
    raplab::StateSpaceST g;


    std::vector<std::vector<double>> occupancy_grid;
    int grid_size = 8;

    occupancy_grid.resize(grid_size);
    for (int i = 0; i < grid_size; i++)
    {
      occupancy_grid[i].resize(grid_size, 0);
    }

    // obstacles
    for (int i = 1; i < grid_size - 1; i++) {
      if (i != 3 && i != 4) {
        occupancy_grid[i][3] = 1;
        occupancy_grid[i][4] = 1;
      }
    }

    g.SetOccuGridPtr(&occupancy_grid);


    std::cout << "Grid layout (0=free, 1=obstacle):" << std::endl;
    for (int i = 0; i < grid_size; i++) {
      for (int j = 0; j < grid_size; j++) {
        std::cout << occupancy_grid[i][j] << " ";
      }
      std::cout << std::endl;
    }

    std::cout << "Step 2: Setting up rectangle agents in narrow corridor..." << std::endl;
    raplab::CBSMulti cbs;
    cbs.SetGraphPtr(&g);


    std::vector<raplab::RectangleAgent> rect_agents;
    
    // agents
    rect_agents.push_back(raplab::RectangleAgent(0, 2, 1, 0, 46));   
    rect_agents.push_back(raplab::RectangleAgent(1, 1, 2, 6, 40));

    cbs.SetRectangleAgents(rect_agents);


    std::vector<long> starts = {0, 6};
    std::vector<long> goals = {46, 40};

    std::cout << "Step 3: Starting CBSMulti solve with obstacles..." << std::endl;
    int result = cbs.Solve(starts, goals, 30.0, 1.0);

    if (result == 0)
    {
      std::cout << "SUCCESS: CBSMulti found solution for rectangle agents with obstacles!" << std::endl;
      auto plan = cbs.GetPlan();
      auto stats = cbs.GetStats();

      for (int i = 0; i < plan.size(); i++)
      {
        std::cout << "Agent " << i << " base path (length=" << plan[i].size() << "): ";
        for (auto v : plan[i])
          std::cout << v << ", ";
        std::cout << std::endl;
      }
      
      std::cout << "Statistics:" << std::endl;
      std::cout << "  Nodes expanded: " << stats["nodes_expanded"] << std::endl;
      std::cout << "  Nodes generated: " << stats["nodes_generated"] << std::endl;
      std::cout << "  Runtime: " << stats["runtime"] << " seconds" << std::endl;
    }
    else
    {
      std::cout << "CBSMulti failed with error code: " << result << std::endl;
    }
  }
  catch (const std::exception &e)
  {
    std::cout << "EXCEPTION: " << e.what() << std::endl;
  }

  timer.PrintDuration();
  std::cout << "####### TestCBSMultiWithObstacles() End #######" << std::endl;
  return 1;
}

int TestCBSMultiWithObstacles2()
{
  std::cout << "\n####### TestCBSMultiWithObstacles() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  try
  {
    std::cout << "Step 1: Creating StateSpaceST with obstacles..." << std::endl;
    raplab::StateSpaceST g;


    std::vector<std::vector<double>> occupancy_grid;
    int grid_size = 6;

    occupancy_grid.resize(grid_size);
    for (int i = 0; i < grid_size; i++)
    {
      occupancy_grid[i].resize(grid_size, 0);
    }

    // obstacles
    occupancy_grid[0][2] = 1;
    occupancy_grid[0][3] = 1;
    occupancy_grid[1][2] = 1;
    occupancy_grid[1][3] = 1;
    //occupancy_grid[2][2] = 1;
    occupancy_grid[2][3] = 1;
    occupancy_grid[3][2] = 1;
    occupancy_grid[3][3] = 1;

    g.SetOccuGridPtr(&occupancy_grid);


    std::cout << "Grid layout (0=free, 1=obstacle):" << std::endl;
    for (int i = 0; i < grid_size; i++) {
      for (int j = 0; j < grid_size; j++) {
        std::cout << occupancy_grid[i][j] << " ";
      }
      std::cout << std::endl;
    }

    std::cout << "Step 2: Setting up rectangle agents in narrow corridor..." << std::endl;
    raplab::CBSMulti cbs;
    cbs.SetGraphPtr(&g);


    std::vector<raplab::RectangleAgent> rect_agents;
    
    // agents
    rect_agents.push_back(raplab::RectangleAgent(0, 2, 2, 0, 4));   
    rect_agents.push_back(raplab::RectangleAgent(1, 1, 1, 5, 0));

    cbs.SetRectangleAgents(rect_agents);


    std::vector<long> starts = {0, 5};
    std::vector<long> goals = {4, 0};

    std::cout << "Step 3: Starting CBSMulti solve with obstacles..." << std::endl;
    int result = cbs.Solve(starts, goals, 30.0, 1.0);

    if (result == 0)
    {
      std::cout << "SUCCESS: CBSMulti found solution for rectangle agents with obstacles!" << std::endl;
      auto plan = cbs.GetPlan();
      auto stats = cbs.GetStats();

      for (int i = 0; i < plan.size(); i++)
      {
        std::cout << "Agent " << i << " base path (length=" << plan[i].size() << "): ";
        for (auto v : plan[i])
          std::cout << v << ", ";
        std::cout << std::endl;
      }
      
      std::cout << "Statistics:" << std::endl;
      std::cout << "  Nodes expanded: " << stats["nodes_expanded"] << std::endl;
      std::cout << "  Nodes generated: " << stats["nodes_generated"] << std::endl;
      std::cout << "  Runtime: " << stats["runtime"] << " seconds" << std::endl;
    }
    else
    {
      std::cout << "CBSMulti failed with error code: " << result << std::endl;
    }
  }
  catch (const std::exception &e)
  {
    std::cout << "EXCEPTION: " << e.what() << std::endl;
  }

  timer.PrintDuration();
  std::cout << "####### TestCBSMultiWithObstacles() End #######" << std::endl;
  return 1;
}
