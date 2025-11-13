#include "my_cbs_multi.hpp"
#include "debug.hpp"
#include <iostream>
#include <cassert>

int TestCBSMultiBasic();
int TestCBSMultiRectangles();
int TestCBSMultiDifferentSizes();
int TestCBSMultiWithObstacles();

int main()
{
  std::cout << "=== Testing CBS for Rectangle Agents ===" << std::endl;
  
  //TestCBSMultiBasic();
  //TestCBSMultiRectangles();
  //TestCBSMultiDifferentSizes();
  TestCBSMultiWithObstacles();

  return 0;
}

int TestCBSMultiBasic()
{
  std::cout << "\n####### TestCBSMultiBasic() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  try
  {
    std::cout << "Step 1: Creating StateSpaceST..." << std::endl;
    raplab::StateSpaceST g;

    // 设置6x6网格
    std::vector<std::vector<double>> occupancy_grid;
    int grid_size = 6;

    occupancy_grid.resize(grid_size);
    for (int i = 0; i < grid_size; i++)
    {
      occupancy_grid[i].resize(grid_size, 0);
    }

    g.SetOccuGridPtr(&occupancy_grid);

    std::cout << "Step 2: Testing CBSMulti with 1x1 agents (compatibility test)..." << std::endl;
    raplab::CBSMulti cbs;
    cbs.SetGraphPtr(&g);

    // 使用默认的1x1智能体
    std::vector<long> starts = {0, 35};  // (0,0) and (5,5)
    std::vector<long> goals = {35, 0};   // (5,5) and (0,0)

    int result = cbs.Solve(starts, goals, 10.0, 1.0);

    if (result == 0)
    {
      std::cout << "SUCCESS: CBSMulti found solution for 1x1 agents!" << std::endl;
      auto plan = cbs.GetPlan();
      auto stats = cbs.GetStats();

      for (int i = 0; i < plan.size(); i++)
      {
        std::cout << "Agent " << i << " path: ";
        for (auto v : plan[i])
          std::cout << v << " ";
        std::cout << std::endl;
      }
      
      std::cout << "Statistics:" << std::endl;
      std::cout << "  Nodes expanded: " << stats["nodes_expanded"] << std::endl;
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
  std::cout << "####### TestCBSMultiBasic() End #######" << std::endl;
  return 1;
}

int TestCBSMultiRectangles()
{
  std::cout << "\n####### TestCBSMultiRectangles() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  try
  {
    std::cout << "Step 1: Creating StateSpaceST for rectangle agents..." << std::endl;
    raplab::StateSpaceST g;

    // 设置8x8网格
    std::vector<std::vector<double>> occupancy_grid;
    int grid_size = 8;

    occupancy_grid.resize(grid_size);
    for (int i = 0; i < grid_size; i++)
    {
      occupancy_grid[i].resize(grid_size, 0);
    }

    g.SetOccuGridPtr(&occupancy_grid);

    std::cout << "Step 2: Setting up 2x2 rectangle agents..." << std::endl;
    raplab::CBSMulti cbs;
    cbs.SetGraphPtr(&g);

    // 定义矩形智能体
    std::vector<raplab::RectangleAgent> rect_agents;
    
    // Agent 0: 2x2 从左上到右下
    rect_agents.push_back(raplab::RectangleAgent(0, 2, 2, 0, 36)); 
    // 起点: (0,0)-(1,1), 终点: (5,5)-(6,6)
    
    // Agent 1: 2x2 从右上到左下  
    rect_agents.push_back(raplab::RectangleAgent(1, 2, 2, 6, 42));
    // 起点: (0,6)-(1,7), 终点: (5,1)-(6,2)

    cbs.SetRectangleAgents(rect_agents);

    std::cout << "Agent 0: 2x2, start=" << rect_agents[0].start << " (base), goal=" << rect_agents[0].goal << std::endl;
    std::cout << "Agent 1: 2x2, start=" << rect_agents[1].start << " (base), goal=" << rect_agents[1].goal << std::endl;

    // 使用空的starts和goals，因为我们在RectangleAgent中已经定义了
    std::vector<long> starts = {0, 6};
    std::vector<long> goals = {42, 36};

    std::cout << "Step 3: Starting CBSMulti solve..." << std::endl;
    int result = cbs.Solve(starts, goals, 20.0, 1.0);

    if (result == 0)
    {
      std::cout << "SUCCESS: CBSMulti found solution for 2x2 rectangle agents!" << std::endl;
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
  std::cout << "####### TestCBSMultiRectangles() End #######" << std::endl;
  return 1;
}

int TestCBSMultiDifferentSizes()
{
  std::cout << "\n####### TestCBSMultiDifferentSizes() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  try
  {
    std::cout << "Step 1: Creating StateSpaceST for mixed-size agents..." << std::endl;
    raplab::StateSpaceST g;

    // 设置10x10网格
    std::vector<std::vector<double>> occupancy_grid;
    int grid_size = 10;

    occupancy_grid.resize(grid_size);
    for (int i = 0; i < grid_size; i++)
    {
      occupancy_grid[i].resize(grid_size, 0);
    }

    g.SetOccuGridPtr(&occupancy_grid);

    std::cout << "Step 2: Setting up mixed-size rectangle agents..." << std::endl;
    raplab::CBSMulti cbs;
    cbs.SetGraphPtr(&g);

    // 定义不同尺寸的智能体
    std::vector<raplab::RectangleAgent> rect_agents;
    
    // Agent 0: 2x3 矩形
    rect_agents.push_back(raplab::RectangleAgent(0, 2, 3, 0, 72)); 
    // 起点: (0,0)-(2,1), 终点: (7,2)-(9,3)
    
    // Agent 1: 1x2 矩形 (长条)
    rect_agents.push_back(raplab::RectangleAgent(1, 1, 2, 8, 81));
    // 起点: (0,8)-(1,8), 终点: (8,1)-(9,1)
    
    // Agent 2: 3x1 矩形 (横条)
    rect_agents.push_back(raplab::RectangleAgent(2, 3, 1, 70, 7));
    // 起点: (7,0)-(7,2), 终点: (0,7)-(0,9)

    cbs.SetRectangleAgents(rect_agents);

    for (int i = 0; i < rect_agents.size(); i++) {
      std::cout << "Agent " << i << ": " << rect_agents[i].width << "x" << rect_agents[i].height 
                << ", start=" << rect_agents[i].start << ", goal=" << rect_agents[i].goal << std::endl;
    }

    std::vector<long> starts = {0, 8, 70};
    std::vector<long> goals = {72, 81, 7};

    std::cout << "Step 3: Starting CBSMulti solve with mixed-size agents..." << std::endl;
    int result = cbs.Solve(starts, goals, 30.0, 1.0);

    if (result == 0)
    {
      std::cout << "SUCCESS: CBSMulti found solution for mixed-size rectangle agents!" << std::endl;
      auto plan = cbs.GetPlan();
      auto stats = cbs.GetStats();

      for (int i = 0; i < plan.size(); i++)
      {
        std::cout << "Agent " << i << " (" << rect_agents[i].width << "x" << rect_agents[i].height 
                  << ") base path (length=" << plan[i].size() << "): ";
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
  std::cout << "####### TestCBSMultiDifferentSizes() End #######" << std::endl;
  return 1;
}

int TestCBSMultiWithObstacles()
{
  std::cout << "\n####### TestCBSMultiWithObstacles() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  try
  {
    std::cout << "Step 1: Creating StateSpaceST with obstacles..." << std::endl;
    raplab::StateSpaceST g;

    // 设置8x8网格
    std::vector<std::vector<double>> occupancy_grid;
    int grid_size = 8;

    occupancy_grid.resize(grid_size);
    for (int i = 0; i < grid_size; i++)
    {
      occupancy_grid[i].resize(grid_size, 0);
    }

    // 添加障碍物形成走廊
    for (int i = 1; i < grid_size - 1; i++) {
      if (i != 3 && i != 4) {
        occupancy_grid[i][3] = 1;
        occupancy_grid[i][4] = 1;
      }
    }

    g.SetOccuGridPtr(&occupancy_grid);

    // 显示网格布局
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

    // 定义矩形智能体
    std::vector<raplab::RectangleAgent> rect_agents;
    
    // Agent 0: 2x1 矩形
    rect_agents.push_back(raplab::RectangleAgent(0, 2, 1, 0, 46)); 
    // 起点: (0,0)-(0,1), 终点: (5,6)-(5,7)
    
    // Agent 1: 1x2 矩形  
    rect_agents.push_back(raplab::RectangleAgent(1, 1, 2, 6, 40));
    // 起点: (0,6)-(1,6), 终点: (5,0)-(6,0)

    cbs.SetRectangleAgents(rect_agents);

    std::cout << "Agent 0: " << rect_agents[0].width << "x" << rect_agents[0].height 
              << ", start=" << rect_agents[0].start << ", goal=" << rect_agents[0].goal << std::endl;
    std::cout << "Agent 1: " << rect_agents[1].width << "x" << rect_agents[1].height 
              << ", start=" << rect_agents[1].start << ", goal=" << rect_agents[1].goal << std::endl;

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
