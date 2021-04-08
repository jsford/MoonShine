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
double theta = 1.5;

struct State2D {
  int x;
  int y;
  double a;   //Normal vector of Solar Panel

  bool operator==(const State2D& rhs) const {
    return x == rhs.x && y == rhs.y && a == rhs.a;
  }
};
NANOPLAN_MAKE_STATE_HASHABLE(State2D, s.x, s.y, s.a);


class SearchSpace2D final : public nanoplan::SearchSpace<State2D> {
  public:
    SearchSpace2D() = default;

    // If you are in state "state", where all can you go in a single step?
    std::vector<State2D> get_successors(const State2D& state) override {
      std::vector<State2D> succs;

      State2D    up {state.x+0, state.y+1, M_PI};
      if( 0 <= up.x && up.x < col && 0 <= up.y && up.y < row && Obs_Map[up.x][up.y]) {  // Put GV value here
        double ang;
        double ang2;
        if(theta - up.a >= 0) {
          ang = theta - up.a;
        }
        else if (theta - up.a<0) {
          ang = theta - up.a + 2*M_PI;
        }

        if(ang-M_PI >=0){
          ang2 = ang - M_PI;
        }
        else if(ang-M_PI<0) {
          ang2 = ang + M_PI;
        }

        if (ang>=3*M_PI/4 && ang<=5*M_PI/4){
          up.a = ang
          succs.push_back(up);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){ //Add pi
          up.a = ang2
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
          down.a = ang
          succs.push_back(down);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){ //Add pi
          down.a = ang2
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
          left.a = ang
          succs.push_back(left);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){ //Add pi
          left.a = ang2
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
          right.a = ang
          succs.push_back(right);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){ //Add pi
          right.a = ang2
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
          upright.a = ang
          succs.push_back(upright);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){ //Add pi
          upright.a = ang2
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
          downright.a = ang
          succs.push_back(downright);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){ //Add pi
          downright.a = ang2
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
          upleft.a = ang
          succs.push_back(upleft);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){ //Add pi
          upleft.a = ang2
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
          downleft.a = ang
          succs.push_back(downleft);
        }
        else if (ang2>=3*M_PI/4 && ang2<=5*M_PI/4){ //Add pi
          downleft.a = ang2
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

  ifstream file { argv[1] };
  file>>row;
  file>>col;
  int Start_X, Start_Y, Goal_X, Goal_Y;

  file>>Start_X;
  file>>Start_Y;

  file>>Goal_X;
  file>>Goal_Y;

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

  // Set the start state.
  State2D start {Start_X, Start_Y};  //  Add cell coordinates
  fmt::print("Start: ({},{})\n", start.x, start.y);

  // Set the goal state.
  State2D  goal {Goal_X, Goal_Y};  //  Add cell coordinates
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

  for(int i=0; i<path.size(); ++i) {
    outdata << path[i].x<<'\t'<<path[i].y<<'\t'<<path[i].a<<'\t'<<'\t'<<double(path[i].x)/10 + 0.05<<'\t'<<double(path[i].y)/10 + 0.05<<'\t'<<0;
    outdata<<endl;
    fmt::print("({}, {})", path[i].x, path[i].y);
    if( i != path.size()-1 ) { fmt::print("->"); }

  }
  fmt::print("\n");

  return 0;
}
