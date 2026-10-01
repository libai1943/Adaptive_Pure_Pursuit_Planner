# Validation record

Local validation: Windows, GCC 16.2.0 (C++17, `-O2`) and MATLAB R2021b Update 7. C++ and MATLAB execute independently. `tools/compare_results.py` checks identical status, iteration count, A* coordinates and conflict history, and a maximum absolute tolerance of 1e-9 for every exported state and carrot coordinate. Timing is excluded from equality checks.

| Original case | Expected result | Outer iterations | Largest C++ / MATLAB CSV difference |
| --- | --- | ---: | ---: |
| 001 | `dense_collision` | 8 | 1.28e-13 |
| 002 | `iteration_limit` | 10 | 2.35e-13 |
| 003 | `dense_collision` | 2 | 6.10e-14 |
| 005 | `success` | 5 | 1.11e-13 |

These four cases form a small, deliberately selected regression set. They are **not** a success-rate estimate. Cases 001, 002 and 003 are retained to test failure behavior. Case 005 is selected for the README because it passes the additional dense check. No speedup ratio, paper-level success rate, or exact reproduction of the paper's experimental tables is claimed.

For case 005, the independent audit uses Shapely/GEOS rather than either planner's collision functions:

- 18,901 dense poses checked; zero footprint collisions.
- Path length: 189.0 m with constant virtual speed 1 m/s.
- Maximum absolute curvature: 0.3008172787 1/m, consistent with `tan(0.7)/2.8`.
- Maximum absolute steering rate: 2.5 rad/s, within floating-point tolerance.
- Goal position error: 0.4296936952 m; goal heading error: 0.2106162641 rad.
- Outer-loop conflicting-pose counts: 28, 6, 1, 1, 0.

The audit also checks time monotonicity, initial pose, step length, midpoint position dynamics, steering magnitude, steering rate, and the yaw-rate bound. Both native core test suites cover analytic half-body overlap, crossing-edge collisions, A* dilation, blocked-route failure, straight and turning tracking, arc-length conflict buffers, traceback saturation, and invalid boundary endpoints. C++ tests use explicit throwing checks, so Release builds do not disable assertions.

Run all four cases and compare using:

```sh
python tools/run_regression.py --cpp build/app_demo --matlab matlab
```

Floating-point libraries can differ across platforms; the reported tiny local errors are measurements, not a promise of bitwise identity on all CPUs. Every successful output is still subjected to the same collision and steering constraints. The CI workflow checks both implementations on one runner.
