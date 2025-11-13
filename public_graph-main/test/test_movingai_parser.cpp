#include "movingai_map_parser.hpp"
#include "movingai_scen_parser.hpp"
#include <string>
#include <iostream>
using namespace std;

void test_graph_parser(string filename) {
  movingai::gridmap g(filename);
	cout << "[Parse map file: " << filename << "]" << endl;
  cout << "height: " << g.height_ << endl;
  cout << "width: " << g.width_ << endl;
  for (int y=0; y<g.height_; y++) {
    for (int x=0; x<g.width_; x++) 
      cout << g.is_obstacle({x, y});
    cout << endl;
  }
}

void test_scen_parser(string filename) {
  movingai::scenario_manager scenmgr;
  scenmgr.load_scenario(filename);

	cout << "[Parse scen file: " << filename << "]" << endl;
  for (int i=0; i<scenmgr.num_experiments(); i++) {
    auto expr = scenmgr.get_experiment(i);
    cout << expr->mapheight() << " " << expr->mapwidth() << " "
              << expr->startx() << " " << expr->starty() << " "
              << expr->goalx() << " " << expr->goaly() << " "
              << expr->distance() << std::endl;
        
  }
}

int main() {
  //./test_io <mapfile>
  // -> print out the content of map read from file
	string mapfile = "../data/arena/arena.map";
  test_graph_parser(mapfile);
	cout << endl;
	string scenfile = "../data/arena/arena.map.scen";
	test_scen_parser(scenfile);
}
