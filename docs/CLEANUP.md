# Source cleanup

The Chapter 8 source tree contains the two native planners, shared inputs, tests, documentation, and one generated result figure.

The supplied MATLAB directory mixed APP with AMPL/IPOPT executables and DLLs, numerical-optimization comparison models, DP velocity planning, archived solver outputs, ad hoc plotting scripts (`asd.m`, `dsa.m`), batch workspaces and a large MAT benchmark collection. These are not dependencies of the published APP path-planning loop and are not included in this source tree. Four scene inputs are exported to an auditable text format; the conversion utility is retained.

The former C++ tree was a ROS/catkin platform draft with a DP guide and small-robot parameters. The maintained C++ version is now a standalone C++17 implementation matching MATLAB and the paper's full-size vehicle. The stale `src/` tree and platform submodule are removed from the current branch; the former `app_hnu_planner.zip` distribution is no longer needed. Existing Git history preserves the old version.

Builds, runtime CSVs, temporary plots, local tools, solver binaries and original workspace backups are excluded from commits. The paper and unpublished book PDF are not uploaded.
