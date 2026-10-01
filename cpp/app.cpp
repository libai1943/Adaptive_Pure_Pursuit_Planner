#include "app.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace app {
namespace {
constexpr double eps = 1e-10;
std::filesystem::path native_path(const std::string& name) {
#ifdef _WIN32
  // Windows main() receives command-line paths in the active ANSI code page.
  // Convert explicitly: MinGW filesystem's default C locale rejects Chinese.
  int count =
      MultiByteToWideChar(CP_ACP, MB_ERR_INVALID_CHARS, name.data(), int(name.size()), nullptr, 0);
  if (count == 0) throw std::runtime_error("Cannot decode path");
  std::wstring wide(count, L'\0');
  MultiByteToWideChar(CP_ACP, MB_ERR_INVALID_CHARS, name.data(), int(name.size()), wide.data(),
                      count);
  return std::filesystem::path(wide);
#else
  return std::filesystem::path(name);
#endif
}
double cross(Point a, Point b, Point c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}
double distance(Point a, Point b) { return std::hypot(a.x - b.x, a.y - b.y); }
Point xy(State a) { return {a.x, a.y}; }
double clamp(double x, double a, double b) { return std::max(a, std::min(b, x)); }
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
bool overlap(const Polygon& a, const Polygon& b) {
  for (const auto* p : {&a, &b})
    for (size_t i = 0; i < p->size(); ++i) {
      auto u = (*p)[i], v = (*p)[(i + 1) % p->size()];
      double ax = u.y - v.y, ay = v.x - u.x, al = 1e100, ah = -1e100, bl = 1e100, bh = -1e100;
      for (auto q : a) {
        double d = ax * q.x + ay * q.y;
        al = std::min(al, d);
        ah = std::max(ah, d);
      }
      for (auto q : b) {
        double d = ax * q.x + ay * q.y;
        bl = std::min(bl, d);
        bh = std::max(bh, d);
      }
      if (ah < bl - eps || bh < al - eps) return false;
    }
  return true;
}
bool broad(const Polygon& p, const Triangle& t) {
  double xl = 1e100, xh = -1e100, yl = 1e100, yh = -1e100;
  for (auto q : p) {
    xl = std::min(xl, q.x);
    xh = std::max(xh, q.x);
    yl = std::min(yl, q.y);
    yh = std::max(yh, q.y);
  }
  return xh >= t.xmin - eps && xl <= t.xmax + eps && yh >= t.ymin - eps && yl <= t.ymax + eps;
}
std::vector<Point> resample_reference(const std::vector<Point>& input, double spacing) {
  std::vector<Point> p;
  for (auto q : input)
    if (p.empty() || distance(p.back(), q) > eps) p.push_back(q);
  if (p.size() < 2) return {};
  std::vector<double> s(p.size(), 0);
  for (size_t i = 1; i < p.size(); ++i) s[i] = s[i - 1] + distance(p[i - 1], p[i]);
  int n = std::max(2, int(std::ceil(s.back() / spacing)) + 1);
  size_t j = 0;
  std::vector<Point> out;
  for (int i = 0; i < n; ++i) {
    double d = s.back() * i / (n - 1);
    while (j + 1 < s.size() - 1 && s[j + 1] < d) ++j;
    double f = (d - s[j]) / (s[j + 1] - s[j]);
    out.push_back({p[j].x + f * (p[j + 1].x - p[j].x), p[j].y + f * (p[j + 1].y - p[j].y)});
  }
  return out;
}
std::vector<int> identify(const std::vector<State>& p, const Scene& s, const Config& c) {
  std::vector<int> out;
  for (auto q : p) out.push_back(collision(q, s, c) ? 1 : 0);
  return out;
}
std::vector<Point> polish(const Scene& scene, const Config& c, const Sample& global,
                          std::array<int, 2> seg, const std::vector<int>& flags) {
  const int a = seg[0], b = seg[1], n = b - a + 1;
  std::vector<State> path(global.path.begin() + a, global.path.begin() + b + 1);
  std::vector<Point> carrots(global.carrots.begin() + a, global.carrots.begin() + b + 1);
  std::vector<int> bad(flags.begin() + a, flags.begin() + b + 1);
  State start = path.front(), goal = path.back();
  for (int iter = 0; iter < c.inner_iterations; ++iter) {
    for (int i = 0; i < n; ++i)
      if (bad[i]) {
        auto rate = collision_rates(path[i], scene, c);
        int id = matched_carrot(carrots, i, std::abs(rate[0] - rate[1]) * c.traceback_length);
        double sign = rate[0] >= rate[1] ? 1.0 : -1.0;
        carrots[id].x += sign * c.nudge * std::sin(path[i].theta);
        carrots[id].y -= sign * c.nudge * std::cos(path[i].theta);
      }
    auto s = sample_pure_pursuit(carrots, start, goal, c);
    if (!s.valid) break;
    // Paired nearest-index resampling preserves the tracked carrot association.
    for (int i = 0; i < n; ++i) {
      int j = int(std::floor(double(i) * (s.path.size() - 1) / (n - 1) + 0.5));
      path[i] = s.path[j];
      carrots[i] = s.carrots[j];
    }
    bad = identify(path, scene, c);
    if (std::none_of(bad.begin(), bad.end(), [](int v) { return v != 0; })) break;
  }
  return carrots;
}
}  // namespace
Config load_config(const std::string& file) {
  std::ifstream f(native_path(file));
  if (!f) throw std::runtime_error("Cannot read parameters: " + file);
  Config c;
  const std::map<std::string, double Config::*> real_fields{
      {"wheelbase", &Config::wheelbase},
      {"width", &Config::width},
      {"length", &Config::length},
      {"rear_overhang", &Config::rear_overhang},
      {"max_steer", &Config::max_steer},
      {"max_steer_rate", &Config::max_steer_rate},
      {"buffer_left", &Config::buffer_left},
      {"buffer_right", &Config::buffer_right},
      {"buffer_increment", &Config::buffer_increment},
      {"simulation_dt", &Config::simulation_dt},
      {"speed", &Config::speed},
      {"traceback_length", &Config::traceback_length},
      {"nudge", &Config::nudge},
      {"grid_resolution", &Config::grid_resolution},
      {"reference_spacing", &Config::reference_spacing},
      {"lookahead", &Config::lookahead},
      {"extension", &Config::extension}};
  const std::map<std::string, int Config::*> integer_fields{
      {"outer_iterations", &Config::outer_iterations},
      {"inner_iterations", &Config::inner_iterations},
      {"integration_steps", &Config::integration_steps},
      {"max_tracking_steps", &Config::max_tracking_steps}};
  std::set<std::string> seen;
  std::string line, key;
  double v;
  while (std::getline(f, line)) {
    std::istringstream in(line);
    if (!(in >> key) || key[0] == '#') continue;
    std::string extra;
    if (!(in >> v) || (in >> extra) || !std::isfinite(v) || v <= 0 || !seen.insert(key).second)
      throw std::runtime_error("Invalid parameter: " + line);
    if (auto field = real_fields.find(key); field != real_fields.end()) {
      c.*(field->second) = v;
    } else if (auto field = integer_fields.find(key); field != integer_fields.end()) {
      if (v != std::floor(v) || v > 1000000) throw std::runtime_error("Invalid integer: " + key);
      c.*(field->second) = int(v);
    } else {
      throw std::runtime_error("Unknown parameter: " + key);
    }
  }
  if (seen.size() != real_fields.size() + integer_fields.size())
    throw std::runtime_error("Missing parameters");
  if (c.length <= c.rear_overhang || c.max_steer >= 1.5707963267948966)
    throw std::runtime_error("Invalid vehicle geometry/steering");
  return c;
}
Scene load_scene(const std::string& file) {
  std::ifstream f(native_path(file));
  Scene s{};
  int n;
  if (!(f >> s.xmin >> s.xmax >> s.ymin >> s.ymax >> s.start.x >> s.start.y >> s.start.theta >>
        s.goal.x >> s.goal.y >> s.goal.theta >> n) ||
      n < 0 || n > 100000 || s.xmin >= s.xmax || s.ymin >= s.ymax)
    throw std::runtime_error("Invalid scene header: " + file);
  for (int i = 0; i < n; ++i) {
    Triangle t;
    t.xmin = t.ymin = 1e100;
    t.xmax = t.ymax = -1e100;
    for (int j = 0; j < 3; ++j) {
      Point p;
      if (!(f >> p.x >> p.y) || !std::isfinite(p.x) || !std::isfinite(p.y))
        throw std::runtime_error("Invalid triangle");
      t.p.push_back(p);
      t.xmin = std::min(t.xmin, p.x);
      t.xmax = std::max(t.xmax, p.x);
      t.ymin = std::min(t.ymin, p.y);
      t.ymax = std::max(t.ymax, p.y);
    }
    if (cross(t.p[0], t.p[1], t.p[2]) <= eps)
      throw std::runtime_error("Triangles must be nondegenerate and counterclockwise");
    s.obstacles.push_back(t);
  }
  for (double v : {s.xmin, s.xmax, s.ymin, s.ymax, s.start.x, s.start.y, s.start.theta, s.goal.x,
                   s.goal.y, s.goal.theta})
    if (!std::isfinite(v)) throw std::runtime_error("Nonfinite scene header");
  std::string extra;
  if (f >> extra) throw std::runtime_error("Unexpected data after scene triangles");
  return s;
}
Polygon footprint(const State& s, const Config& c, double lower, double upper) {
  Polygon out;
  double co = std::cos(s.theta), si = std::sin(s.theta);
  for (auto q : Polygon{{-c.rear_overhang, lower * c.width / 2},
                        {c.length - c.rear_overhang, lower * c.width / 2},
                        {c.length - c.rear_overhang, upper * c.width / 2},
                        {-c.rear_overhang, upper * c.width / 2}})
    out.push_back({s.x + q.x * co - q.y * si, s.y + q.x * si + q.y * co});
  return out;
}
double polygon_area(const Polygon& p) {
  double sum = 0;
  for (size_t i = 0; i < p.size(); ++i) {
    auto a = p[i], b = p[(i + 1) % p.size()];
    sum += a.x * b.y - a.y * b.x;
  }
  return std::abs(sum) * 0.5;
}
Polygon clip_polygon(Polygon p, const Polygon& clip) {
  for (size_t k = 0; k < clip.size() && !p.empty(); ++k) {
    Polygon out;
    auto a = clip[k], b = clip[(k + 1) % clip.size()], prev = p.back();
    double dp = cross(a, b, prev);
    for (auto cur : p) {
      double dc = cross(a, b, cur);
      bool ip = dp >= 0, ic = dc >= 0;
      if (ip != ic) {
        double t = dp / (dp - dc);
        out.push_back({prev.x + t * (cur.x - prev.x), prev.y + t * (cur.y - prev.y)});
      }
      if (ic) out.push_back(cur);
      prev = cur;
      dp = dc;
    }
    p = std::move(out);
  }
  return p;
}
bool collision(const State& s, const Scene& scene, const Config& c) {
  auto p = footprint(s, c);
  for (auto q : p)
    if (q.x < scene.xmin - eps || q.x > scene.xmax + eps || q.y < scene.ymin - eps ||
        q.y > scene.ymax + eps)
      return true;
  for (const auto& t : scene.obstacles)
    if (broad(p, t) && overlap(p, t.p)) return true;
  return false;
}
std::array<double, 2> collision_rates(const State& s, const Scene& scene, const Config& c) {
  std::array<double, 2> out{};
  Polygon bounds{{scene.xmin, scene.ymin},
                 {scene.xmax, scene.ymin},
                 {scene.xmax, scene.ymax},
                 {scene.xmin, scene.ymax}};
  for (int side = 0; side < 2; ++side) {
    auto p = footprint(s, c, side == 0 ? 0 : -1, side == 0 ? 1 : 0);
    auto inside = clip_polygon(p, bounds);
    double area = c.length * c.width / 2 - polygon_area(inside);
    for (const auto& t : scene.obstacles)
      if (broad(inside, t)) area += polygon_area(clip_polygon(inside, t.p));
    out[side] = clamp(area / (c.length * c.width / 2), 0, 1);
  }
  return out;
}
std::vector<Point> search_astar(const Scene& s, const Config& c) {
  const double d = c.grid_resolution;
  int nx = int(std::floor((s.xmax - s.xmin) / d)) + 1,
      ny = int(std::floor((s.ymax - s.ymin) / d)) + 1;
  if (nx < 2 || ny < 2 || double(nx) * ny > 1e7) throw std::runtime_error("Unsupported grid size");
  const int total = nx * ny;
  std::vector<unsigned char> occupied(total, 0), blocked(total, 0);
  auto id = [nx](int x, int y) { return y * nx + x; };
  auto point = [&](int k) { return Point{s.xmin + (k % nx) * d, s.ymin + (k / nx) * d}; };
  for (const auto& t : s.obstacles) {
    int x0 = std::max(0, int(std::ceil((t.xmin - s.xmin) / d))),
        x1 = std::min(nx - 1, int(std::floor((t.xmax - s.xmin) / d)));
    int y0 = std::max(0, int(std::ceil((t.ymin - s.ymin) / d))),
        y1 = std::min(ny - 1, int(std::floor((t.ymax - s.ymin) / d)));
    for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x) {
        auto p = point(id(x, y));
        if (cross(t.p[0], t.p[1], p) >= -eps && cross(t.p[1], t.p[2], p) >= -eps &&
            cross(t.p[2], t.p[0], p) >= -eps)
          occupied[id(x, y)] = 1;
      }
  }
  int radius = int(std::ceil(c.width / 2 / d));
  for (int y = 0; y < ny; ++y)
    for (int x = 0; x < nx; ++x)
      if (occupied[id(x, y)])
        for (int dy = -radius; dy <= radius; ++dy)
          for (int dx = -radius; dx <= radius; ++dx)
            if (dx * dx + dy * dy <= radius * radius && x + dx >= 0 && x + dx < nx && y + dy >= 0 &&
                y + dy < ny)
              blocked[id(x + dx, y + dy)] = 1;
  for (int y = 0; y < ny; ++y)
    for (int x = 0; x < nx; ++x)
      if (x < radius || y < radius || x >= nx - radius || y >= ny - radius) blocked[id(x, y)] = 1;
  auto index = [&](State p) {
    if (p.x < s.xmin || p.x > s.xmax || p.y < s.ymin || p.y > s.ymax) return -1;
    int x = int(std::floor((p.x - s.xmin) / d + 0.5));
    int y = int(std::floor((p.y - s.ymin) / d + 0.5));
    if (x >= nx || y >= ny) return -1;
    return id(x, y);
  };
  int start = index(s.start), goal = index(s.goal);
  if (start < 0 || goal < 0 || blocked[start] || blocked[goal]) return {};
  std::vector<double> g(total, std::numeric_limits<double>::infinity());
  std::vector<int> parent(total, -1);
  std::vector<bool> closed(total, false);
  // Quantized f only resolves floating-point ties reproducibly across languages.
  using Entry = std::tuple<long long, int, double>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  auto push = [&](int k) {
    double h = distance(point(k), point(goal));
    open.emplace(static_cast<long long>(std::floor((g[k] + h) * 1e9 + 0.5)), k, g[k]);
  };
  g[start] = 0;
  push(start);
  const int dxs[] = {1, 0, -1, 0, 1, -1, -1, 1}, dys[] = {0, 1, 0, -1, 1, 1, -1, -1};
  while (!open.empty()) {
    auto [score, u, gu] = open.top();
    open.pop();
    (void)score;
    if (closed[u] || gu > g[u] + eps) continue;
    if (u == goal) break;
    closed[u] = true;
    int x = u % nx, y = u / nx;
    for (int k = 0; k < 8; ++k) {
      int xx = x + dxs[k], yy = y + dys[k];
      if (xx < 0 || xx >= nx || yy < 0 || yy >= ny) continue;
      int v = id(xx, yy);
      if (blocked[v] || closed[v]) continue;
      if (k >= 4 && (blocked[id(xx, y)] || blocked[id(x, yy)])) continue;
      double candidate = g[u] + d * (k < 4 ? 1 : std::sqrt(2.0));
      if (candidate < g[v] - eps) {
        g[v] = candidate;
        parent[v] = u;
        push(v);
      }
    }
  }
  if (!std::isfinite(g[goal])) return {};
  std::vector<Point> path;
  for (int k = goal; k != -1; k = parent[k]) path.push_back(point(k));
  std::reverse(path.begin(), path.end());
  path.front() = xy(s.start);
  path.back() = xy(s.goal);
  return path;
}
Sample sample_pure_pursuit(std::vector<Point> reference, const State& start, const State& goal,
                           const Config& c, bool dense) {
  Sample out;
  if (reference.size() < 2) return out;
  reference.push_back(
      {goal.x + c.extension * std::cos(goal.theta), goal.y + c.extension * std::sin(goal.theta)});
  auto ref = resample_reference(reference, c.reference_spacing);
  if (ref.size() < 2) return out;
  State cur = start;
  size_t latest = 0;
  double dt = c.simulation_dt / c.integration_steps;
  if (dense) out.dense.push_back(cur);
  bool exhausted = false;
  for (int step = 0; step < c.max_tracking_steps; ++step) {
    size_t nearest = latest;
    double best = 1e100;
    for (size_t i = latest; i < ref.size(); ++i) {
      double v = distance(xy(cur), ref[i]);
      if (v < best) {
        best = v;
        nearest = i;
      }
    }
    latest = std::min(nearest + 1, ref.size() - 1);
    size_t carrot = nearest;
    while (carrot < ref.size() && distance(xy(cur), ref[carrot]) < c.lookahead) ++carrot;
    if (carrot == ref.size()) {
      exhausted = true;
      break;
    }
    auto cp = ref[carrot];
    double alpha = std::atan2(cp.y - cur.y, cp.x - cur.x) - cur.theta;
    double desired = clamp(std::atan2(2 * c.wheelbase * std::sin(alpha), c.lookahead), -c.max_steer,
                           c.max_steer);
    for (int j = 0; j < c.integration_steps; ++j) {
      // Analytic slew limit (5), midpoint integration of the bicycle model (2).
      double next =
          cur.phi + clamp(desired - cur.phi, -c.max_steer_rate * dt, c.max_steer_rate * dt);
      double midphi =
          cur.phi + clamp(desired - cur.phi, -c.max_steer_rate * dt / 2, c.max_steer_rate * dt / 2);
      double yaw = c.speed / c.wheelbase * std::tan(midphi) * dt;
      cur.x += c.speed * std::cos(cur.theta + yaw / 2) * dt;
      cur.y += c.speed * std::sin(cur.theta + yaw / 2) * dt;
      cur.theta += yaw;
      cur.phi = next;
      if (dense) out.dense.push_back(cur);
    }
    out.path.push_back(cur);
    out.carrots.push_back(cp);
  }
  if (!exhausted || out.path.empty()) return out;
  size_t end = 0;
  double best = 1e100;
  for (size_t i = 0; i < out.path.size(); ++i) {
    double score = distance(xy(out.path[i]), xy(goal));
    if (score < best) {
      best = score;
      end = i;
    }
  }
  out.path.resize(end + 1);
  out.carrots.resize(end + 1);
  if (dense) out.dense.resize(1 + (end + 1) * c.integration_steps);
  out.valid = out.path.size() >= 2;
  return out;
}
std::vector<std::array<int, 2>> conflict_segments(const std::vector<State>& path,
                                                  const std::vector<int>& bad, double left,
                                                  double right) {
  int n = int(path.size());
  std::vector<int> mask(n, 0);
  for (int i = 0; i < n; ++i)
    if (bad[i]) {
      int a = i, b = i;
      double s = 0;
      while (a > 0 && s < left) {
        s += distance(xy(path[a]), xy(path[a - 1]));
        --a;
      }
      s = 0;
      while (b + 1 < n && s < right) {
        s += distance(xy(path[b]), xy(path[b + 1]));
        ++b;
      }
      for (int j = a; j <= b; ++j) mask[j] = 1;
    }
  std::vector<std::array<int, 2>> out;
  for (int i = 0; i < n; ++i)
    if (mask[i]) {
      int a = i;
      while (i + 1 < n && mask[i + 1]) ++i;
      out.push_back({a, i});
    }
  return out;
}
int matched_carrot(const std::vector<Point>& carrots, int i, double tb) {
  int best = i;
  double s = 0, error = std::abs(tb);
  for (int j = i - 1; j >= 0; --j) {
    s += distance(carrots[j], carrots[j + 1]);
    double e = std::abs(tb - s);
    if (e < error - eps) {
      error = e;
      best = j;
    }
  }
  return best;
}
Result plan(const Scene& scene, const Config& c) {
  Result r;
  if (collision(scene.start, scene, c) || collision(scene.goal, scene, c)) {
    r.status = "invalid_endpoint";
    return r;
  }
  r.initial_route = search_astar(scene, c);
  if (r.initial_route.size() < 2) {
    r.status = "no_astar_route";
    return r;
  }
  auto carrots = r.initial_route;
  double left = c.buffer_left, right = c.buffer_right;
  for (int iter = 0; iter < c.outer_iterations; ++iter) {
    r.sample = sample_pure_pursuit(carrots, scene.start, scene.goal, c, true);
    if (!r.sample.valid) {
      r.status = "tracking_failed";
      return r;
    }
    auto bad = identify(r.sample.path, scene, c);
    auto segs = conflict_segments(r.sample.path, bad, left, right);
    int count = 0;
    for (int b : bad) count += b;
    r.history.push_back({iter + 1, count, int(segs.size())});
    if (segs.empty()) {
      for (auto p : r.sample.dense)
        if (collision(p, scene, c)) {
          r.status = "dense_collision";
          return r;
        }
      r.success = true;
      r.status = "success";
      return r;
    }
    carrots = r.sample.carrots;
    for (auto seg : segs) {
      auto local = polish(scene, c, r.sample, seg, bad);
      std::copy(local.begin(), local.end(), carrots.begin() + seg[0]);
    }
    left += c.buffer_increment;
    right += c.buffer_increment;
  }
  r.status = "iteration_limit";
  return r;
}
void write_result(const Result& r, const Scene& s, const Config& c, const std::string& directory) {
  auto destination = native_path(directory);
  std::filesystem::create_directories(destination);
  auto stream = [&](const std::string& name) {
    std::ofstream f(destination / name);
    if (!f) throw std::runtime_error("Cannot write " + name);
    f << std::setprecision(17);
    return f;
  };
  auto route = stream("initial_route.csv");
  route << "x,y\n";
  for (auto p : r.initial_route) route << p.x << ',' << p.y << '\n';
  auto path = stream("path.csv");
  path << "t,x,y,theta,phi,carrot_x,carrot_y\n";
  for (size_t i = 0; i < r.sample.path.size(); ++i) {
    auto p = r.sample.path[i];
    auto q = r.sample.carrots[i];
    path << (i + 1) * c.simulation_dt << ',' << p.x << ',' << p.y << ',' << p.theta << ',' << p.phi
         << ',' << q.x << ',' << q.y << '\n';
  }
  auto dense = stream("dense_path.csv");
  dense << "t,x,y,theta,phi\n";
  for (size_t i = 0; i < r.sample.dense.size(); ++i) {
    auto p = r.sample.dense[i];
    dense << i * c.simulation_dt / c.integration_steps << ',' << p.x << ',' << p.y << ',' << p.theta
          << ',' << p.phi << '\n';
  }
  auto history = stream("history.csv");
  history << "iteration,conflicts,segments\n";
  for (auto h : r.history) history << h[0] << ',' << h[1] << ',' << h[2] << '\n';
  double position = -1, yaw = -1;
  if (!r.sample.path.empty()) {
    auto p = r.sample.path.back();
    position = distance(xy(p), xy(s.goal));
    yaw = std::abs(wrap(p.theta - s.goal.theta));
  }
  auto summary = stream("summary.json");
  summary << "{\n  \"success\": " << (r.success ? "true" : "false") << ",\n  \"status\": \""
          << r.status << "\",\n  \"outer_iterations\": " << r.history.size()
          << ",\n  \"goal_position_error_m\": " << position
          << ",\n  \"goal_heading_error_rad\": " << yaw << "\n}\n";
}
}  // namespace app
