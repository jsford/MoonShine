#include <nanoplan/nanoplan.h>
#include <fmt/format.h>


struct State2D {
  int x;
  int y;

  bool operator==(const State2D& rhs) const {
    return x == rhs.x && y == rhs.y;
  }
};
NANOPLAN_MAKE_STATE_HASHABLE(State2D, s.x, s.y);


class SearchSpace2D final : public nanoplan::SearchSpace<State2D> {
  public:
    SearchSpace2D() = default;

    // If you are in state "state", where all can you go in a single step?
    std::vector<State2D> get_successors(const State2D& state) override {
      std::vector<State2D> succs;

      State2D    up {state.x+0, state.y+1};
      if( 0 <= up.x && up.x < 10 && 0 <= up.y && up.y < 10) {
        succs.push_back(up);
      }

      State2D  down {state.x+0, state.y-1};
      if( 0 <= down.x && down.x < 10 && 0 <= down.y && down.y < 10) {
        succs.push_back(down);
      }

      State2D  left {state.x-1, state.y+0};
      if( 0 <= left.x && left.x < 10 && 0 <= left.y && left.y < 10) {
        succs.push_back(left);
      }

      State2D right {state.x+1, state.y+0};
      if( 0 <= right.x && right.x < 10 && 0 <= right.y && right.y < 10) {
        succs.push_back(right);
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
  // TODO: Read the map from a file.

  // Construct a search space.
  auto space2d = std::make_shared<SearchSpace2D>();

  // Construct an A* planner.
  nanoplan::AStar<SearchSpace2D> planner(space2d);

  // Set the start state.
  State2D start {0, 0};

  // Set the goal state.
  State2D  goal {9, 9};

  // Search for a path from start to goal.
  std::vector<State2D> path = planner.plan(start, goal);

  // Print out the path.
  for(int i=0; i<path.size(); ++i) {
    fmt::print("({}, {})", path[i].x, path[i].y);
    if( i != path.size()-1 ) { fmt::print("->"); }
  }
  fmt::print("\n");

  return 0;
}
