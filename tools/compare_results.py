"""Compare independently executed C++ and MATLAB results (Python stdlib only)."""
import argparse
import csv
import json
import math
from pathlib import Path


def compare(first, second, tolerance=1e-9):
    a = json.loads((first / 'summary.json').read_text())
    b = json.loads((second / 'summary.json').read_text())
    for key in ('success', 'status', 'outer_iterations'):
        assert a[key] == b[key], (key, a[key], b[key])
    for key in ('goal_position_error_m', 'goal_heading_error_rad'):
        assert abs(a[key] - b[key]) <= tolerance, key
    errors = {}
    for filename in ('initial_route.csv', 'path.csv', 'dense_path.csv', 'history.csv'):
        with (first / filename).open() as f, (second / filename).open() as g:
            aa, bb = list(csv.reader(f)), list(csv.reader(g))
        assert aa[0] == bb[0] and len(aa) == len(bb), (filename, 'shape/header mismatch')
        error = 0.0
        for row_a, row_b in zip(aa[1:], bb[1:]):
            assert len(row_a) == len(row_b), filename
            for x, y in zip(map(float, row_a), map(float, row_b)):
                assert math.isfinite(x) and math.isfinite(y), (filename, 'nonfinite')
                error = max(error, abs(x-y))
        limit = 0 if filename in ('initial_route.csv', 'history.csv') else tolerance
        assert error <= limit, (filename, error, limit)
        errors[filename] = error
    return errors


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cpp', type=Path)
    parser.add_argument('matlab', type=Path)
    args = parser.parse_args()
    print(json.dumps(compare(args.cpp, args.matlab), indent=2))
