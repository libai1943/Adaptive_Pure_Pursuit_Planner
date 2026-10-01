#pragma once
#include <array>
#include <string>
#include <vector>

namespace app {
struct Point {
  double x = 0, y = 0;
};
struct State {
  double x = 0, y = 0, theta = 0, phi = 0;
};
using Polygon = std::vector<Point>;
struct Triangle {
  Polygon p;
  double xmin, xmax, ymin, ymax;
};
struct Scene {
  double xmin, xmax, ymin, ymax;
  State start, goal;
  std::vector<Triangle> obstacles;  // Disjoint CCW triangles of the obstacle union.
};
struct Config {
  double wheelbase = 2.8, width = 1.942, length = 4.689, rear_overhang = 0.929;
  double max_steer = 0.7, max_steer_rate = 2.5, buffer_left = 4, buffer_right = 4;
  double buffer_increment = 5, simulation_dt = 1, speed = 1, traceback_length = 4, nudge = 0.1;
  double grid_resolution = 0.25, reference_spacing = 0.1, lookahead = 3, extension = 7;
  int outer_iterations = 10, inner_iterations = 200, integration_steps = 100,
      max_tracking_steps = 2000;
};
struct Sample {
  std::vector<State> path, dense;
  std::vector<Point> carrots;
  bool valid = false;
};
struct Result {
  bool success = false;
  std::string status = "uninitialized";
  std::vector<Point> initial_route;
  Sample sample;
  std::vector<std::array<int, 3>> history;  // iteration, conflicting samples, segments
};
Config load_config(const std::string& file);
Scene load_scene(const std::string& file);
Polygon footprint(const State& s, const Config& c, double lower = -1, double upper = 1);
double polygon_area(const Polygon& p);
Polygon clip_polygon(Polygon subject, const Polygon& clip);
bool collision(const State& s, const Scene& scene, const Config& c);
std::array<double, 2> collision_rates(const State& s, const Scene& scene, const Config& c);
std::vector<Point> search_astar(const Scene& scene, const Config& c);
Sample sample_pure_pursuit(std::vector<Point> reference, const State& start, const State& goal,
                           const Config& c, bool dense = false);
std::vector<std::array<int, 2>> conflict_segments(const std::vector<State>& path,
                                                  const std::vector<int>& conflicts, double left,
                                                  double right);
int matched_carrot(const std::vector<Point>& carrots, int i, double traceback);
Result plan(const Scene& scene, const Config& c);
void write_result(const Result& r, const Scene& scene, const Config& c,
                  const std::string& directory);
}  // namespace app
