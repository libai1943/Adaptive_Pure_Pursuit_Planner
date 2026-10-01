# Adaptive Pure Pursuit (APP) Planner

**Plan with a tracking controller. Repair collisions by reshaping the carrot path.**

[中文说明](README_zh.md) · [Paper](https://doi.org/10.1109/TIV.2023.3296435) · [Equation mapping](docs/ALGORITHM.md)

C++17 and native MATLAB implementations of the APP method in:

> Bai Li, Yazhou Wang, Siji Ma, Xuepeng Bian, Hu Li, Tantan Zhang, Xiaohui Li, and Youmin Zhang, **“Adaptive Pure Pursuit: A Real-Time Path Planner Using Tracking Controllers to Plan Safe and Kinematically Feasible Paths,”** *IEEE Transactions on Intelligent Vehicles*, vol. 8, no. 9, pp. 4155–4168, 2023. [Paper / DOI](https://doi.org/10.1109/TIV.2023.3296435)

本仓库同时是《非结构化场景自动驾驶轨迹规划技术》**第八章：露天矿区狭窄大曲率道路场景中的轨迹规划方法**的配套代码。C++ 与 MATLAB 采用相同的算法、参数、输入场景和输出格式；实现依据以 TIV 论文为准。

![APP path, vehicle footprints, curvature and iterative collision repair](docs/images/app_benchmark.png)

*An actual run of this repository on a curvy-road scene from the original MATLAB benchmark collection (case 005). Five outer iterations reduce conflicting sampled poses from 28 to 0. All 18,901 integration poses pass a separate GEOS/Shapely footprint audit. This is a reproduction example, not a copied paper figure or a rerun of the paper's complete benchmark study.*

## What APP does

1. **A* guidance:** rasterize the obstacle union and dilate it by half the vehicle width; search an eight-neighbor grid for an initial carrot path.
2. **Virtual tracking:** drive a bicycle-model vehicle with pure pursuit. Bound both steering angle and steering rate to obtain a kinematically consistent path.
3. **Adaptive repair:** cluster conflicting poses, measure overlap on the left and right halves of the vehicle, trace backward along the carrot path, and nudge the selected carrot point away from the more obstructed side.
4. **Stitch and repeat:** replace local carrot segments, grow the conflict buffers, and simulate the entire path again.

Both versions use **A*** for initialization. No DP initializer, numerical optimizer, AMPL installation, ROS workspace, MEX bridge, or MATLAB toolbox is required to run the planners.

## Quick start: C++

Requires a C++17 compiler and CMake 3.16 or later. From the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
./build/app_demo data/benchmark_005.txt data/paper_parameters.txt output/cpp
```

With a Visual Studio generator on Windows, the executable is `build/Release/app_demo.exe`. For MinGW, use an ASCII-only checkout path if your toolchain does not handle Unicode paths.

Or build directly:

```sh
g++ -std=c++17 -O2 cpp/app.cpp cpp/main.cpp -o app_demo
./app_demo data/benchmark_005.txt data/paper_parameters.txt output/cpp
```

Exit codes: `0` = successful checked path; `1` = planning failure; `2` = input or execution error. The C++ library entry point is `app::plan(scene, config)` in [cpp/app.hpp](cpp/app.hpp).

## Quick start: MATLAB

Tested locally with MATLAB R2021b. From the repository root:

```matlab
addpath('matlab');
result = run_demo();                % benchmark 005; writes output/matlab
assert(result.success);
plot_demo(result);                  % optional native MATLAB visualization
test_core;
```

For another case or output directory:

```matlab
result = run_demo('data/benchmark_001.txt', 'output/matlab_case1');
disp(result.status);
```

The native MATLAB entry point is [app_plan](matlab/app_plan.m). MATLAB computes the planner itself; it does not call the C++ executable.

## Reproduce the figure and check agreement

The planners export the same files:

| File | Contents |
| --- | --- |
| `initial_route.csv` | A* guide: `x,y` |
| `path.csv` | Controller-update samples: `t,x,y,theta,phi,carrot_x,carrot_y` |
| `dense_path.csv` | Initial state and every integration state: `t,x,y,theta,phi` |
| `history.csv` | Outer iteration, conflicting sampled poses, local segments |
| `summary.json` | Success, explicit failure reason, iteration count, goal-pose errors |

Compare independent runs using Python 3, with no extra packages:

```sh
python tools/compare_results.py output/cpp output/matlab
python tools/run_regression.py --cpp build/app_demo --matlab matlab
```

The four included cases cover success, the outer iteration limit, and rejection of collisions between controller-update samples. On the local validation platform, the A* guides and iteration histories match exactly; the largest path difference is below **2.4e-13**, against a test tolerance of **1e-9**. See [validation details](docs/VALIDATION.md).

Plotting and an independent geometry audit are optional Python utilities:

```sh
python -m pip install numpy matplotlib shapely
python tools/validate_result.py data/benchmark_005.txt output/cpp data/paper_parameters.txt
python tools/plot_result.py data/benchmark_005.txt output/cpp docs/images/app_benchmark.png
```

## Paper correspondence and scope

[The implementation notes](docs/ALGORITHM.md) map each algorithm and equation to both languages and distinguish published parameters from numerical choices. All Table I parameters are in [data/paper_parameters.txt](data/paper_parameters.txt), including vehicle width **1.942 m**, initial left/right buffers **4 m / 4 m**, and the **half-body** overlap denominator in Eq. (8).

APP handles static, known obstacles and forward driving. As the paper states, it does **not** guarantee exact goal-pose satisfaction or convergence in every feasible scene. The illustrated run ends **0.430 m** and **0.211 rad** from the requested goal. Inspect these reported errors before using a result in a task that needs precise docking.

`success` additionally requires all 0.01 s integration poses to pass footprint collision checks. This discrete audit is stricter than checking only the 1 s controller samples, but is not a proof of continuous swept-volume clearance. A rejected candidate remains available for diagnosis; a failure result must not be executed as a valid plan.

本章在路径规划之后讨论的独立速度规划不属于本论文 APP 核心。这里采用论文规定的恒定虚拟车速，不混入旧版 DP 速度规划、AMPL/NLP 对比实验或机器人平台工程。旧仓库的 ROS/DP 草稿可在 [历史版本](https://github.com/libai1943/Adaptive_Pure_Pursuit_Planner/tree/6f3ae6f4a1e2a1c007a7c577e88a8d3e78abda5e)中查看。

## Repository layout

```text
cpp/       C++17 planner and command-line demo
matlab/    native MATLAB planner, demo and core tests
data/      shared parameters and four text-format regression scenes
tests/     C++ analytic and behavior tests
tools/     regression, plotting, independent audit and data conversion
docs/      equation mapping, validation record and the generated figure
```

Generated CSVs, builds, executables, DLLs, original MAT collections, solver installations and scratch experiments are excluded from the source tree. See [data provenance](data/README.md) and [cleanup notes](docs/CLEANUP.md).

## Citation

If APP contributes to your research, please cite the original paper. This also helps readers find the algorithm derivation, comparative simulations, and physical experiments.

```bibtex
@article{li2023adaptive,
  author  = {Li, Bai and Wang, Yazhou and Ma, Siji and Bian, Xuepeng and
             Li, Hu and Zhang, Tantan and Li, Xiaohui and Zhang, Youmin},
  title   = {Adaptive Pure Pursuit: A Real-Time Path Planner Using Tracking
             Controllers to Plan Safe and Kinematically Feasible Paths},
  journal = {IEEE Transactions on Intelligent Vehicles},
  year    = {2023},
  volume  = {8},
  number  = {9},
  pages   = {4155--4168},
  doi     = {10.1109/TIV.2023.3296435}
}
```
