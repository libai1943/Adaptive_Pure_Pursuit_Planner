"""Run curated scenes, preserve expected failures, optionally compare MATLAB."""
import argparse
import json
from pathlib import Path
import subprocess

from compare_results import compare

EXPECTED = {1: ('dense_collision', 8), 2: ('iteration_limit', 10),
            3: ('dense_collision', 2), 5: ('success', 5)}


def matlab_string(path):
    return "'" + str(path).replace('\\', '/').replace("'", "''") + "'"


def run(binary, output, matlab=None, compare_existing=False):
    root = Path(__file__).resolve().parents[1]
    binary = binary.resolve(); output = output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if matlab:
        statements = [f"addpath({matlab_string(root/'matlab')}); test_core;"]
        for case in EXPECTED:
            statements.append(f"run_demo({matlab_string(root/'data'/f'benchmark_{case:03d}.txt')},"
                              f"{matlab_string(output/'matlab'/str(case))});")
        subprocess.run([matlab, '-batch', ' '.join(statements)], check=True)
    report = {}
    for case, (status, iterations) in EXPECTED.items():
        target = output/'cpp'/str(case)
        process = subprocess.run([str(binary), str(root/'data'/f'benchmark_{case:03d}.txt'),
                                  str(root/'data'/'paper_parameters.txt'), str(target)], check=False)
        expected_code = 0 if status == 'success' else 1
        assert process.returncode == expected_code, (case, process.returncode)
        summary = json.loads((target/'summary.json').read_text())
        assert summary['status'] == status and summary['outer_iterations'] == iterations, (case, summary)
        assert summary['success'] == (status == 'success'), case
        report[str(case)] = {'status': status, 'outer_iterations': iterations}
        if matlab or compare_existing:
            report[str(case)]['max_abs_errors'] = compare(target, output/'matlab'/str(case))
    (output/'regression.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cpp', type=Path, required=True)
    p.add_argument('--output', type=Path, default=Path('output/regression'))
    p.add_argument('--matlab', help='MATLAB executable to launch (optional)')
    p.add_argument('--compare-existing-matlab', action='store_true')
    a = p.parse_args(); run(a.cpp, a.output, a.matlab, a.compare_existing_matlab)
