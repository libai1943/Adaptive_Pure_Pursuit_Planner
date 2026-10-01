"""Render an actual successful APP run for the README (NumPy + Matplotlib)."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import numpy as np


def render(scene_file, result_dir, output, parameters):
    c = dict(line.split() for line in parameters.read_text().splitlines() if line.strip() and not line.startswith('#'))
    c = {k: float(v) for k, v in c.items()}
    scene = np.fromstring(scene_file.read_text(), sep=' ')
    triangles = scene[11:].reshape(-1, 3, 2)
    report = json.loads((result_dir / 'summary.json').read_text())
    if not report['success']:
        raise ValueError('Refusing to illustrate a failed run as a successful result')
    path = np.loadtxt(result_dir / 'dense_path.csv', delimiter=',', skiprows=1)
    route = np.loadtxt(result_dir / 'initial_route.csv', delimiter=',', skiprows=1)
    history = np.loadtxt(result_dir / 'history.csv', delimiter=',', skiprows=1, ndmin=2)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False})
    fig = plt.figure(figsize=(14, 7.4), facecolor='#fafbfc')
    grid = fig.add_gridspec(2, 2, width_ratios=[1.6, 1], hspace=.55, wspace=.28)
    ax = fig.add_subplot(grid[:, 0]); curve = fig.add_subplot(grid[0, 1]); progress = fig.add_subplot(grid[1, 1])
    ax.add_collection(PolyCollection(triangles, facecolors='#c5cbd3', edgecolors='none', antialiaseds=False, rasterized=True))
    ax.plot(route[:, 0], route[:, 1], color='#dd9b28', lw=1.3, ls='--', label='A* guide')
    ax.plot(path[:, 1], path[:, 2], color='#087c9e', lw=2.2, label='APP rear-axle path')
    rear, front, half = c['rear_overhang'], c['length']-c['rear_overhang'], c['width']/2
    local = np.array([[-rear, -half], [front, -half], [front, half], [-rear, half]])
    footprints = []
    stride = max(1, round(4/(c['speed']*c['simulation_dt']/c['integration_steps'])))
    for row in path[::stride]:
        co, si = np.cos(row[3]), np.sin(row[3])
        footprints.append(local @ np.array([[co, si], [-si, co]]) + row[1:3])
    ax.add_collection(PolyCollection(footprints, facecolors='none', edgecolors='#087c9e', linewidths=.5, alpha=.65))
    ax.scatter(*scene[4:6], color='#194a57', marker='o', s=45, zorder=6, label='Start')
    ax.scatter(*scene[7:9], color='#c44f37', marker='*', s=120, zorder=6, label='Requested goal')
    ax.set(xlim=scene[:2], ylim=scene[2:4], aspect='equal', xlabel='x (m)', ylabel='y (m)', title='Curvy-road benchmark 005')
    ax.legend(loc='lower left', framealpha=.95, fontsize=9)
    shift = path[:, 0]*c['speed']
    curvature = np.tan(path[:, 4]) / c['wheelbase']
    limit = np.tan(c['max_steer']) / c['wheelbase']
    curve.plot(shift, curvature, color='#087c9e', lw=1.5)
    curve.axhline(limit, color='#c44f37', ls='--', lw=1, label='Steering bound')
    curve.axhline(-limit, color='#c44f37', ls='--', lw=1)
    curve.set(xlabel='Travelled distance (m)', ylabel='Curvature (1/m)', title='Rate-limited steering, bounded curvature')
    curve.grid(alpha=.18); curve.legend(fontsize=9, loc='upper right')
    progress.plot(history[:, 0], history[:, 1], '-o', color='#087c9e', lw=2, ms=5)
    progress.set(xlabel='Outer iteration', ylabel='Conflicting sampled poses', title='Adaptive local repair')
    progress.set_xticks(history[:, 0].astype(int)); progress.set_ylim(bottom=-1); progress.grid(alpha=.18)
    fig.suptitle('ADAPTIVE PURE PURSUIT', x=.075, ha='left', y=.98, fontsize=22, fontweight='bold', color='#18364a')
    fig.text(.075, .916, 'Chapter 8  |  A* initialization + virtual tracking + collision-driven carrot refinement', color='#506070', fontsize=11)
    dt = c['simulation_dt']/c['integration_steps']
    fig.text(.075, .037, f"Reproduction run: {len(path):,} dense poses checked at {dt:g} s  |  goal error {report['goal_position_error_m']:.3f} m, {report['goal_heading_error_rad']:.3f} rad", color='#506070', fontsize=10)
    fig.subplots_adjust(left=.07, right=.97, top=.855, bottom=.14)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, facecolor=fig.get_facecolor())
    plt.close(fig)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('scene', type=Path); p.add_argument('result', type=Path); p.add_argument('output', type=Path)
    p.add_argument('--parameters',type=Path,default=Path(__file__).resolve().parents[1]/'data'/'paper_parameters.txt')
    a = p.parse_args(); render(a.scene, a.result, a.output, a.parameters)
