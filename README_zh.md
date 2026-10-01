# 第八章：自适应纯跟踪（APP）规划器

本仓库配套《非结构化场景自动驾驶轨迹规划技术》第八章“露天矿区狭窄大曲率道路场景中的轨迹规划方法”，提供 **C++17 和原生 MATLAB** 两个实质一致的版本。

算法依据为 Bai Li 等发表于 IEEE TIV 的论文：**Adaptive Pure Pursuit: A Real-Time Path Planner Using Tracking Controllers to Plan Safe and Kinematically Feasible Paths**，2023，8(9)：4155–4168。[论文 DOI](https://doi.org/10.1109/TIV.2023.3296435)。若在研究中使用本方法，欢迎引用原论文；完整 BibTeX 见[英文主页](README.md#citation)。

![实际运行结果](docs/images/app_benchmark.png)

图中为原始实验集合第 005 号场景在本仓库中的实际运行结果：外层迭代 5 次，冲突采样点数量依次为 28、6、1、1、0；独立几何检查覆盖 18,901 个密集车身位姿，未发现碰撞。本图不是论文原图，也不代表已经重新完成论文中的全部统计实验。

## 算法过程

1. 在按半车宽膨胀的栅格地图上使用 **A*** 搜索初始胡萝卜路径。
2. 纯跟踪控制器驱动虚拟车辆，前向积分自行车模型；同时约束前轮转角及转角变化率。
3. 检查车身碰撞，按路径累计长度扩展并合并局部冲突段。
4. 比较左右半车身与障碍物的面积重叠率，沿胡萝卜路径回溯后侧向微调对应点。
5. 将局部修改合入全局胡萝卜路径，重新模拟并逐轮扩大缓冲区。

两版均以论文为准：初始化不用 DP；碰撞率分母是半车身面积；回溯使用胡萝卜路径的累计长度；车辆宽度采用 1.942 m；左右初始缓冲长度均为 4 m；转角状态在积分与控制周期之间连续传递。[逐项公式对应说明](docs/ALGORITHM.md)。

## MATLAB 运行

在仓库根目录执行，已在 MATLAB R2021b 验证，无需额外工具箱：

```matlab
addpath('matlab');
result = run_demo();
assert(result.success);
plot_demo(result);
test_core;
```

结果写入 `output/matlab`。修改场景时，向 `run_demo` 传入场景文件和输出目录；使用 `plot_demo` 时也传入相同场景文件。

## C++ 运行

需要 C++17 编译器和 CMake 3.16 或以上版本：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
./build/app_demo data/benchmark_005.txt data/paper_parameters.txt output/cpp
```

Visual Studio 生成器对应的可执行文件通常在 `build/Release/app_demo.exe`。若本机 MinGW 不支持中文目录，请在纯英文目录下构建。也可直接编译：

```sh
g++ -std=c++17 -O2 cpp/app.cpp cpp/main.cpp -o app_demo
```

## 一致性检查与输出含义

两版共用 `data/paper_parameters.txt` 与相同场景文件。四个场景的本地回归结果中，A* 路径和迭代记录完全相同，状态及胡萝卜点最大差异小于 2.4e-13。比较命令：

```sh
python tools/compare_results.py output/cpp output/matlab
```

`summary.json` 给出成功标志、失败原因和终点误差；`dense_path.csv` 包含初始位姿与全部积分位姿；`path.csv` 保存控制周期采样点及其对应胡萝卜点。失败场景也会保留候选路径便于分析，但不能作为成功规划结果使用。

APP 不保证精确抵达目标位姿，也不保证在所有可行场景中收敛。示例终点位置误差约 0.430 m、航向误差约 0.211 rad，与论文讨论的能力边界一致。实现额外检查 0.01 s 积分位姿，仍不等于连续扫掠体的严格安全证明。[详细验证记录](docs/VALIDATION.md)。

本次整理去除了 AMPL/IPOPT 安装包、DLL、优化对比代码、旧 DP 初始化、平台工程及临时数据，只提交运行和验证 APP 所需的源码、场景、文档与示例图。独立速度规划不包含在本仓库的 APP 核心中。
