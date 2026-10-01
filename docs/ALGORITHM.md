# Correspondence with the APP paper

The reference is Li et al., IEEE TIV 8(9), 2023, DOI [10.1109/TIV.2023.3296435](https://doi.org/10.1109/TIV.2023.3296435). The book companion is Chapter 8 of 《非结构化场景自动驾驶轨迹规划技术》. Where draft code differs, the paper determines the algorithm and Table I determines the published parameters.

| Paper | C++ (`cpp/app.cpp`) | MATLAB | Contract |
| --- | --- | --- | --- |
| Algorithm 1; Sec. III-B | `search_astar`, `plan` | `app_astar`, `app_plan` | Half-width map dilation; A*; bounded outer loop |
| Eqs. (2), (3), (5), (A4) | `sample_pure_pursuit` | `app_sample` | Bicycle model; bounded steering and slew rate; constant speed |
| Eqs. (6), (7) | `conflict_segments` | `app_segments` | Grow seeds by accumulated smooth-path distance; merge touching intervals |
| Eq. (8) | `collision_rates` | `app_geometry('rates',...)` | Overlap area divided by `length * width / 2` |
| Eq. (9) | `matched_carrot` | `app_matched_carrot` | Closest accumulated **carrot-path** distance to the requested traceback |
| Algorithm 2; Eq. (10) | `polish` | local `polish` in `app_plan` | Nudge right if left rate >= right rate; otherwise left |
| Algorithm 3 | segment replacement in `plan` | segment replacement in `app_plan` | Replace paired, fixed-size carrot segments; retrack globally |

## Published settings

The vehicle reference point is the rear-axle midpoint. Heading is counterclockwise; positive steering turns left; distances are metres, angles radians, time seconds. The footprint spans `[-rear_overhang, length-rear_overhang]` longitudinally and `[-width/2,width/2]` laterally.

Table I values are: wheelbase 2.8; width 1.942; length 4.689; rear overhang 0.929; maximum steering 0.7; maximum steering rate 2.5; 10 outer iterations; initial buffers 4 and 4; buffer growth 5 and 5; controller interval 1; constant speed 1; 200 inner iterations; maximum traceback 4; nudge 0.1. The paper's objective (4) motivates smooth, short paths; APP does not numerically optimize this objective.

The slew-limited steering is the analytic response `phi(t+h) = phi(t) + clamp(phi_target-phi(t), -omega_max*h, omega_max*h)`. Steering state persists between controller updates and a local simulation starts from the segment's actual steering state. The target is clipped to the mechanical steering bounds before this response.

## Numerical choices left open by the paper

These choices are explicit and identical in both languages; they are not claimed as additional published Table I entries.

- Grid resolution: 0.25 m. Occupancy uses triangle interiors and boundaries at grid nodes. A disk of `ceil((width/2)/resolution)` cells dilates the map. Grid boundaries are also blocked by that radius. Eight-neighbor moves have Euclidean costs and diagonal corner cutting is forbidden. The Euclidean heuristic is active; this is not Dijkstra with a renamed function or a DP lattice.
- A* prioritizes `round(1e9*f)`, then row-major node ID, then `g`; cost improvements require more than `1e-10`. This deterministic quantization only affects sub-nanometre cost ties. Start/goal coordinates replace their grid-node coordinates in the resulting guide. The guide is not itself certified as a vehicle trajectory.
- Carrot reference spacing: at most 0.1 m; lookahead: 3 m. Find the nearest remaining reference sample, advance monotonically, then take the first sample at least one lookahead away. Use the configured lookahead in (A4), as in the original MATLAB controller.
- Extend the reference 7 m along the requested terminal heading. Simulate until the extended reference cannot supply a carrot; keep the prefix ending at the controller sample nearest the goal position. Do not append an artificial exact-goal connection. A limit of 2000 controller steps guards nonterminating tracking.
- Integrate with 100 substeps per controller interval (0.01 s here). Evaluate steering at the substep midpoint, then update position at midpoint yaw. This is a numerical approximation of the continuous bicycle dynamics, not an exact symbolic integration.
- Each stored controller sample has exactly one associated tracked carrot. Local paired resampling uses nearest index with half-up rounding, preserving this association. It does not interpolate a new path and pretend that path was dynamically simulated. Only the final global retracking is returned as the candidate trajectory.
- Segment endpoints are inclusive, so the size is `end-start+1`. This resolves the paper pseudocode's mixed inclusive slicing and `end-start` notation. At zero traceback the current carrot can be selected; ties prefer the more recent point. Traceback saturates at the first local carrot.
- Obstacles are provided as non-overlapping, counterclockwise triangles of their geometric union. Convex clipping sums exact polygon intersection areas without counting overlapping obstacles twice. The separating-axis test detects both interior overlap and boundary contact. This implements the polygon collision constraint using a different geometric primitive from the triangle-area checker cited in the paper. AABB rejection accelerates both operations.
- Leaving the workspace is a collision; the outside area contributes to the corresponding half-body overlap rate. Initial and requested goal footprints are checked before A*.

## Success and failure semantics

Algorithm 1 checks controller samples for local repair. This implementation adds a final check of every integration pose, including the initial pose. The additional gate can reject a candidate that satisfies the coarser sample checks. It does not alter the paper's nudge law or silently introduce another optimizer. The reason is reported as `dense_collision`.

Other statuses are `success`, `invalid_endpoint`, `no_astar_route`, `tracking_failed`, and `iteration_limit`. On failure, candidate data are diagnostic, not a certified trajectory. A failed run does not establish that the scene is mathematically infeasible. No exact terminal pose, continuous swept-volume guarantee, global optimality, or complete-planner property is claimed.

## Corrections relative to the supplied drafts

The former C++ DP initializer and robot-specific scaling are replaced by the shared A* and full-size Table I vehicle. The old MATLAB overlap denominator used the whole vehicle area; Eq. (8) requires half of it. Its iteration-dependent extra traceback and smooth-path Euclidean traceback are replaced by Eq. (9) along the carrot path. Its steering integration did not return the updated steering state; both new implementations preserve it. Published buffer sizes replace the unequal draft buffer sizes. These changes intentionally mean that legacy run outputs need not be reproduced bit for bit.
