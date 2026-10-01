#include <chrono>
#include <iostream>

#include "app.hpp"
int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: app_demo SCENE PARAMETERS OUTPUT_DIRECTORY\n";
    return 2;
  }
  try {
    auto scene = app::load_scene(argv[1]);
    auto config = app::load_config(argv[2]);
    auto start = std::chrono::steady_clock::now();
    auto result = app::plan(scene, config);
    double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    app::write_result(result, scene, config, argv[3]);
    std::cout << result.status << ", outer iterations=" << result.history.size()
              << ", runtime=" << seconds << " s\n";
    return result.success ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
