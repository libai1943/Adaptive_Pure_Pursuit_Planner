#include <cmath>
#include <iostream>
#include <stdexcept>

#include "app.hpp"

namespace {
void check(bool yes, const char* message) {
  if (!yes) throw std::runtime_error(message);
}
void near(double a, double b) { check(std::abs(a - b) < 1e-9, "numeric assertion"); }
void add_rectangle(app::Scene& s, double x0, double x1, double y0, double y1) {
  s.obstacles.push_back({{{x0, y0}, {x1, y0}, {x1, y1}}, x0, x1, y0, y1});
  s.obstacles.push_back({{{x0, y0}, {x1, y1}, {x0, y1}}, x0, x1, y0, y1});
}
}  // namespace
int main() {
  try {
    app::Config c;
    app::Scene s{-10, 40, -10, 10, {0, 0, 0, 0}, {25, 0, 0, 0}, {}};
    const auto p = app::footprint(s.start, c);
    near(app::polygon_area(p), c.width * c.length);
    // A full left-half overlap must be 1, not the legacy value 0.5.
    add_rectangle(s, -c.rear_overhang, c.length - c.rear_overhang, 0, c.width / 2);
    auto rates = app::collision_rates(s.start, s, c);
    near(rates[0], 1);
    near(rates[1], 0);
    check(app::collision(s.start, s, c), "half-body collision missed");
    s.obstacles.clear();
    add_rectangle(s, 3, 4, -5, 5);
    check(app::collision(s.start, s, c), "edge crossing without interior obstacle vertex missed");
    s.obstacles.clear();
    add_rectangle(s, 10, 11, -10, 10);
    check(app::search_astar(s, c).empty(), "A* crossed an occupied wall");
    check(app::plan(s, c).status == "no_astar_route", "no-route failure flag");
    s.obstacles.clear();
    add_rectangle(s, 10, 12, -1, 1);
    auto route = app::search_astar(s, c);
    check(route.size() > 2, "A* did not go around obstacle");
    double deviation = 0;
    for (auto q : route) deviation = std::max(deviation, std::abs(q.y));
    check(deviation >= 2, "A* half-width dilation missing");
    s.obstacles.clear();
    auto r = app::plan(s, c);
    check(r.success, "straight unobstructed scene failed");
    near(r.sample.dense.front().x, s.start.x);
    for (size_t i = 1; i < r.sample.dense.size(); ++i) {
      auto a = r.sample.dense[i - 1], b = r.sample.dense[i];
      near(std::hypot(b.x - a.x, b.y - a.y), c.speed * c.simulation_dt / c.integration_steps);
      near(b.y, 0);
      near(b.phi, 0);
    }
    // Steering-rate saturation must persist across controller updates.
    auto turn = app::sample_pure_pursuit({{0, 0}, {8, 0}, {12, 6}, {20, 12}}, s.start,
                                         {20, 12, 0.5, 0}, c, true);
    check(turn.valid, "turn tracking failed");
    double maxphi = 0;
    for (size_t i = 1; i < turn.dense.size(); ++i) {
      auto a = turn.dense[i - 1], b = turn.dense[i];
      check(std::abs(b.phi) <= c.max_steer + 1e-12, "steering bound");
      check(std::abs(b.phi - a.phi) <=
                c.max_steer_rate * c.simulation_dt / c.integration_steps + 1e-12,
            "slew bound");
      maxphi = std::max(maxphi, std::abs(b.phi));
    }
    check(maxphi > 0.1, "turn fixture did not exercise steering");
    std::vector<app::State> folded{{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 2}};
    auto seg = app::conflict_segments(folded, {0, 0, 1, 0, 0}, 1.5, 1.5);
    check(seg.size() == 1 && seg[0][0] == 0 && seg[0][1] == 4, "arc-length buffers");
    std::vector<app::Point> carrots{{0, 0}, {2, 0}, {2, 2}, {0, 2}};
    check(app::matched_carrot(carrots, 3, 3.9) == 1, "traceback must follow carrot arc");
    check(app::matched_carrot(carrots, 3, 0) == 3, "zero traceback");
    check(app::matched_carrot(carrots, 3, 100) == 0, "traceback clamps at start");
    s.start.x = -10;
    check(app::plan(s, c).status == "invalid_endpoint", "boundary collision");
    std::cout << "All C++ core tests passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
