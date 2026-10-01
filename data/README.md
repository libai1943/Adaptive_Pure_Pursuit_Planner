# Shared input data

`paper_parameters.txt` is the only runtime parameter file for both language versions. Comments distinguish Table I settings from numerical choices.

`benchmark_001.txt`, `benchmark_002.txt`, `benchmark_003.txt`, and `benchmark_005.txt` are converted from the supplied original MATLAB `Benchmarks/1.mat`, `2.mat`, `3.mat`, and `5.mat` files. The source `old_params.environment.obs` entries include two road boundary regions plus respectively 16, 17, 13 and 14 interior obstacles. These scenes use the curvy-road layout underlying the original experimental collection; they are not identified as the exact two cases shown in paper Fig. 3.

Conversion takes the union of all obstacle polygons, then triangulates the union into disjoint counterclockwise triangles. This preserves filled obstacles, nonconvex road boundaries, holes and overlap without needing a geometry library in either planner. MATLAB's `polyshape` simplifies duplicate/degenerate vertices in the source polygons during this one-time conversion. No planner outputs, global workspaces, serialized objects, solver settings or user paths are retained. See `source_manifest.json` for source and converted file checksums.

Each whitespace-separated scene has this schema:

```text
xmin xmax ymin ymax
start_x start_y start_heading
goal_x goal_y goal_heading
number_of_triangles
x1 y1 x2 y2 x3 y3
... one counterclockwise triangle per line ...
```

The triangles must form a non-overlapping decomposition of the obstacle union; overlaps would double-count Eq. (8)'s area. All coordinates are in metres and headings in radians. Concave regions should be decomposed before loading. Keep 17 significant digits when exporting.

To convert a different original benchmark with MATLAB (the one-time exporter uses `polyshape`; the planner does not):

```matlab
addpath('tools');
export_benchmark('path/to/Benchmarks/5.mat', 'data/custom_scene.txt');
```
