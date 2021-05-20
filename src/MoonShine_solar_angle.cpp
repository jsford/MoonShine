// NOTE: Ensure all dependencies in CMakeLists are correct

#include <nanoplan/nanoplan.h>
#include <fmt/format.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>

using namespace std;

int row;
int col;

std::vector<vector<int>> Obs_Map;
double theta = 3*M_PI/4 + 0.1; // Position of the sun (in radian) - 0 to 2*pi

struct State2D {
  int x;	// X coordinate
  int y;	// Y coordinate
  double a;	// Normal vector of Solar Panel

  bool operator==(const State2D& rhs) const {
    return x == rhs.x && y == rhs.y; // rhs.a is not included as it's not a necessary state space parameter but is still required to provide orientation i.e. we don't solve for "a"
  }
};
NANOPLAN_MAKE_STATE_HASHABLE(State2D, s.x, s.y);


class SearchSpace2D final : public nanoplan::SearchSpace<State2D> {
  public:
    SearchSpace2D() = default;

    // If you are in state "state", where all can you go in a single step?
    std::vector<State2D> get_successors(const State2D& state) override {
      std::vector<State2D> succs; // Stores all possible directions the rover can move based on the current location of the rover
      //Syntax:
      //State2D    up {state.x+0, state.y+1, M_PI}; -- State2D <direction of motion> {<new X-position>, <new Y-position>, <angle of normal of the solar panel while moving in that direction>};
	    
      State2D    up {state.x+0, state.y+1, M_PI};
      if( 0 <= up.x && up.x < col && 0 <= up.y && up.y < row && Obs_Map[up.x][up.y]) {  // Put GV value here
        
	// Can shift the code below into a function. Currently this section is repeated for every <direction of motion>. There are 8 <direction of motion>
	      
	// Since the rover can move in 2 directions, 2 angles are needed to account for the 
	double ang;
        double ang2;
	// determine the angle between the normal of the front of the solar panel and the solar influx angle
        // angle must be between 0 and 2*PI
        if(theta - up.a >= 0) {
          ang = theta - up.a;
        }
        else if (theta - up.a<0) {
          ang = theta - up.a + 2*M_PI;
        }

	// determine the angle between the normal of the back of the solar panel and the solar influx angle
	// angle must be between 0 and 2*PI
        if(ang-M_PI >=0){
          ang2 = ang - M_PI;
        }
        else if(ang-M_PI<0) {
          ang2 = ang + M_PI;
        }
			
	// The <direction of motion> is added to succs if the sun (theta) is in the range mentioned below. This range is derived after some basic calculations.
        if (ang>=3*M_PI/4 && ang<=5*M_PI/4){
          succs.push_back(up);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){ //Add pi
          up.a = 0; // rover is driving "backwards" 
          succs.push_back(up);
        }
      }

      State2D  down {state.x+0, state.y-1, 0};
      if( 0 <= down.x && down.x < col && 0 <= down.y && down.y < row && Obs_Map[down.x][down.y]) {
        double ang;
        double ang2;
        if(theta - down.a >= 0) {
          ang = theta - down.a;
        }
        else if (theta - down.a<0) {
          ang = theta - down.a + 2*M_PI;
        }

        if(ang-M_PI >=0){
          ang2 = ang - M_PI;
        }
        else if(ang-M_PI<0) {
          ang2 = ang + M_PI;
        }

        if (ang>=3*M_PI/4 && ang<=5*M_PI/4){
          //down.a = ang;
          succs.push_back(down);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){
          down.a = M_PI;
          succs.push_back(down);
        }
      }

      State2D  left {state.x-1, state.y+0, 3*M_PI/2};
      if( 0 <= left.x && left.x < col && 0 <= left.y && left.y < row && Obs_Map[left.x][left.y]) {
        double ang;
        double ang2;
        if(theta - left.a >= 0) {
          ang = theta - left.a;
        }
        else if (theta - left.a<0) {
          ang = theta - left.a + 2*M_PI;
        }

        if(ang-M_PI >=0){
          ang2 = ang - M_PI;
        }
        else if(ang-M_PI<0) {
          ang2 = ang + M_PI;
        }

        if (ang>=3*M_PI/4 && ang<=5*M_PI/4){
          succs.push_back(left);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){
          left.a = M_PI/2;
          succs.push_back(left);
        }
      }

      State2D right {state.x+1, state.y+0, M_PI/2};
      if( 0 <= right.x && right.x < col && 0 <= right.y && right.y < row && Obs_Map[right.x][right.y]) {
        double ang;
        double ang2;
        if(theta - right.a >= 0) {
          ang = theta - right.a;
        }
        else if (theta - right.a<0) {
          ang = theta - right.a + 2*M_PI;
        }

        if(ang-M_PI >=0){
          ang2 = ang - M_PI;
        }
        else if(ang-M_PI<0) {
          ang2 = ang + M_PI;
        }

        if (ang>=3*M_PI/4 && ang<=5*M_PI/4){
          //right.a = ang;
          succs.push_back(right);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){
          right.a = 3*M_PI/2;
          succs.push_back(right);
        }
      }

      State2D upright {state.x+1, state.y+1, 3*M_PI/4};
      if( 0 <= upright.x && upright.x < col && 0 <= upright.y && upright.y < row && Obs_Map[upright.x][upright.y]) {
        double ang;
        double ang2;
        if(theta - upright.a >= 0) {
          ang = theta - upright.a;
        }
        else if (theta - upright.a<0) {
          ang = theta - upright.a + 2*M_PI;
        }

        if(ang-M_PI >=0){
          ang2 = ang - M_PI;
        }
        else if(ang-M_PI<0) {
          ang2 = ang + M_PI;
        }

        if (ang>=3*M_PI/4 && ang<=5*M_PI/4){
          succs.push_back(upright);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){
          upright.a = 7*M_PI/4;
          succs.push_back(upright);
        }
      }

      State2D downright {state.x+1, state.y-1, M_PI/4};
      if( 0 <= downright.x && downright.x < col && 0 <= downright.y && downright.y < row && Obs_Map[downright.x][downright.y]) {
        double ang;
        double ang2;
        if(theta - downright.a >= 0) {
          ang = theta - downright.a;
        }
        else if (theta - downright.a<0) {
          ang = theta - downright.a + 2*M_PI;
        }

        if(ang-M_PI >=0){
          ang2 = ang - M_PI;
        }
        else if(ang-M_PI<0) {
          ang2 = ang + M_PI;
        }

        if (ang>=3*M_PI/4 && ang<=5*M_PI/4){
          succs.push_back(downright);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){
          downright.a = 5*M_PI/4;
          succs.push_back(downright);
        }
      }

      State2D upleft {state.x-1, state.y+1, 5*M_PI/4};
      if( 0 <= upleft.x && upleft.x < col && 0 <= upleft.y && upleft.y < row && Obs_Map[upleft.x][upleft.y]) {
        double ang;
        double ang2;
        if(theta - upleft.a >= 0) {
          ang = theta - upleft.a;
        }
        else if (theta - upleft.a<0) {
          ang = theta - upleft.a + 2*M_PI;
        }

        if(ang-M_PI >=0){
          ang2 = ang - M_PI;
        }
        else if(ang-M_PI<0) {
          ang2 = ang + M_PI;
        }

        if (ang>=3*M_PI/4 && ang<=5*M_PI/4){
          succs.push_back(upleft);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){
          upleft.a = M_PI/4;
          succs.push_back(upleft);
        }
      }

      State2D downleft {state.x-1, state.y-1, 7*M_PI/4};
      if( 0 <= downleft.x && downleft.x < col && 0 <= downleft.y && downleft.y < row && Obs_Map[downleft.x][downleft.y]) {
        double ang;
        double ang2;
        if(theta - downleft.a >= 0) {
          ang = theta - downleft.a;
        }
        else if (theta - downleft.a<0) {
          ang = theta - downleft.a + 2*M_PI;
        }

        if(ang-M_PI >=0){
          ang2 = ang - M_PI;
        }
        else if(ang-M_PI<0) {
          ang2 = ang + M_PI;
        }

        if (ang>=3*M_PI/4 && ang<=5*M_PI/4){
          succs.push_back(downleft);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){
          downleft.a = 3*M_PI/4;
          succs.push_back(downleft);
        }
      }

      return succs;
    }

    // This is the A* cost function. Given two states "from" and "to",
    // it calculates the cost to move from one to the other.
    nanoplan::Cost get_from_to_cost(const State2D& from, const State2D& to) override {
      double manhattan = std::abs(from.x-to.x) + std::abs(from.y-to.y);
      return nanoplan::Cost(manhattan);
    }

    // This is the A* heuristic function. Given two states
    // "from" and "to", it estimates the cost to move from one to the other.
    // For A* to work, this function must never over-estimate the actual cost.
    nanoplan::Cost get_from_to_heuristic(const State2D& from, const State2D& to) override {
      double euclidean = std::sqrt((from.x-to.x)*(from.x-to.x) + (from.y-to.y)*(from.y-to.y));
      return nanoplan::Cost(euclidean);
    }
};

int main(int argc, char** argv) {
  // Read the map from a file.
  if(argc < 2) {
    fmt::print("Please provide a map file.\n");
    fmt::print("usage: ./MoonShine map.txt\n");
    return -1;
  }
  fmt::print("MoonShine\n");
  fmt::print("Planning using {} map file.\n", argv[1]);

  // Stores size of map
  ifstream file { argv[1] };
  file>>row;
  file>>col;

  // Stores start and end location on map
  int Start_X, Start_Y, Goal_X, Goal_Y;

  file>>Start_X;
  file>>Start_Y;

  file>>Goal_X;
  file>>Goal_Y;

  // Stores obstacle/viable map
  for(int i = 0;i<row;i++) {
    std::vector<int> ran_arr(col);
    for(int j=0;j<col;j++) {
      file>>ran_arr[j];
    }
    Obs_Map.push_back(ran_arr);
  }

  // Construct a search space.
  auto space2d = std::make_shared<SearchSpace2D>();

  // Construct an A* planner.
  nanoplan::AStar<SearchSpace2D> planner(space2d);

  // Non essential parameters required for initialization
  double Start_Ang = 0;
  double Goal_Ang = 3*M_PI/4;

  // Set the start state.
  State2D start {Start_X, Start_Y, Start_Ang};
  fmt::print("Start: ({},{})\n", start.x, start.y);

  // Set the goal state.
  State2D  goal {Goal_X, Goal_Y, Goal_Ang};
  fmt::print("Goal: ({},{})\n", goal.x, goal.y);

  // Search for a path from start to goal.
  std::vector<State2D> path = planner.plan(start, goal);

  if(path.size() == 0) {
    fmt::print("Path not found.");
  }

  // Print out the path and make solution file.

  ofstream outdata;
  outdata.open("sol.txt");
  if( !outdata ) { // file couldn't be opened
    cerr << "Error: file could not be opened" << endl;
    exit(1);
  }
  // Creates solution file("sol.txt") and prints path taken by rover to go from start to end
  // The "sol.txt" format was based on SBPL's for visualizaion purposes. MS later got it's own visualization setup. The same format works for new visuliazation setup also.
  for(int i=0; i<path.size(); ++i) {
    outdata << path[i].x<<'\t'<<path[i].y<<'\t'<<path[i].a<<'\t'<<'\t'<<double(path[i].x)/10 + 0.05<<'\t'<<double(path[i].y)/10 + 0.05<<'\t'<<0; 
    outdata<<endl;
    fmt::print("({}, {})", path[i].x, path[i].y);
    if( i != path.size()-1 ) { fmt::print("->"); }

  }
  fmt::print("\n");

  return 0;
}
