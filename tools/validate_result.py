"""Independent dense-footprint audit using GEOS/Shapely, not planner geometry.

Requires numpy and shapely. Discrete checks are not a continuous safety proof.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from shapely.geometry import Polygon, box
from shapely.ops import unary_union
from shapely.prepared import prep


def validate(scene_file, result_dir, config_file):
    values = np.fromstring(scene_file.read_text(), sep=' ')
    triangles = values[11:].reshape(-1, 3, 2)
    obstacles = prep(unary_union([Polygon(t) for t in triangles]))
    bounds = box(values[0], values[2], values[1], values[3])
    c = dict(line.split() for line in config_file.read_text().splitlines() if line.strip() and not line.startswith('#'))
    c = {k: float(v) for k, v in c.items()}
    p = np.loadtxt(result_dir / 'dense_path.csv', delimiter=',', skiprows=1, ndmin=2)
    assert np.isfinite(p).all() and len(p)>1
    dt = np.diff(p[:, 0]); assert (dt>0).all()
    assert np.max(np.abs(p[:, 4])) <= c['max_steer']+1e-10
    assert np.max(np.abs(np.diff(p[:, 4]))/dt) <= c['max_steer_rate']+1e-8
    step = np.hypot(np.diff(p[:, 1]), np.diff(p[:, 2]))
    assert np.max(np.abs(step-c['speed']*dt)) < 1e-9
    # Check model consistency independently at midpoint yaw.
    yaw_mid = (p[1:, 3]+p[:-1, 3])/2
    assert np.max(np.abs(np.diff(p[:, 1])-c['speed']*np.cos(yaw_mid)*dt))<1e-9
    assert np.max(np.abs(np.diff(p[:, 2])-c['speed']*np.sin(yaw_mid)*dt))<1e-9
    assert np.max(np.abs(np.diff(p[:, 3]))/dt) <= c['speed']/c['wheelbase']*np.tan(c['max_steer'])+1e-8
    assert np.max(np.abs(p[0, 1:4]-values[4:7]))<1e-9
    rear=c['rear_overhang']; front=c['length']-rear; half=c['width']/2
    local=np.array([[-rear,-half],[front,-half],[front,half],[-rear,half]])
    collisions=[]
    for i,row in enumerate(p):
        co,si=np.cos(row[3]),np.sin(row[3]); footprint=Polygon(local @ np.array([[co,si],[-si,co]]) + row[1:3])
        if not bounds.covers(footprint) or obstacles.intersects(footprint):
            collisions.append(i)
    summary=json.loads((result_dir/'summary.json').read_text())
    if summary['success']:
        assert not collisions, f"Planner reported success with {len(collisions)} dense collisions"
    return {'poses_checked':len(p), 'dense_collisions':len(collisions),
            'max_abs_curvature':float(np.max(np.abs(np.tan(p[:,4])/c['wheelbase']))),
            'max_abs_steering_rate':float(np.max(np.abs(np.diff(p[:,4]))/dt)),
            'path_length_m':float(step.sum())}


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('scene',type=Path);p.add_argument('result',type=Path);p.add_argument('config',type=Path)
    a=p.parse_args();print(json.dumps(validate(a.scene,a.result,a.config),indent=2))
