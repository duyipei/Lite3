"""Compile the real generated .cu as host C++ under explicit Thrust doubles."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.gpu_ndt.p2d_optimization import patch


def sources(variant):
    return patch.apply(patch.load_sources(patch.HERE/'baseline'), variant)


def compile_host(directory, variant, sanitizer=False):
    directory = Path(directory)
    directory.mkdir()
    files = sources(variant)
    for name, content in files.items():
        p = directory/name;p.parent.mkdir(parents=True, exist_ok=True);p.write_text(content)
    # Use explicit reference declarations, never label them installed headers.
    for p in (patch.HERE/'baseline/include').rglob('*'):
        if p.is_file():
            target = directory/p.relative_to(patch.HERE/'baseline')
            if not target.exists():target.parent.mkdir(parents=True, exist_ok=True);target.write_bytes(p.read_bytes())
    for name in ['device_vector.h', 'host_vector.h', 'transform_reduce.h', 'functional.h', 'copy.h', 'pair.h']:
        p = directory/'include/thrust'/name;p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text('#include "p2d_thrust_double.hpp"\n')
    # Keep the tested LM math/control body, with observation-only tracing.
    lm = files['include/fast_gicp/gicp/impl/lsq_registration_impl.hpp']
    from experiments.gpu_ndt.prepare import method_bounds
    methods = []
    for signature, declaration in [
        ('bool LsqRegistration<PointTarget, PointSource>::is_converged(', 'bool is_converged('),
        ('bool LsqRegistration<PointTarget, PointSource>::step_lm(', 'bool step_lm(')]:
        a,b = method_bounds(lm,signature)
        body = lm[a:b].replace(signature,declaration,1)
        if 'step_lm' in signature:
            needle = '    double rho = (y0 - yi) / (d.dot(lm_lambda_ * d - b));'
            assert body.count(needle) == 1
            body = body.replace(needle,needle+'\n    decisions.push_back(rho < 0.0 ? (is_converged(delta) ? 2 : -1) : 1);')
        methods.append(body)
    (directory/'lm_methods.inc').write_text('\n'.join(methods))
    source = directory/'host_check.cpp'
    source.write_bytes((ROOT/'tests/p2d_host_check.cpp').read_bytes())
    binary = directory/'host_check'
    command = ['g++','-std=c++14','-O1','-I'+str(directory/'include'),'-I'+str(ROOT/'tests'),
               '-I/usr/include/eigen3','-I'+str(directory),str(source),'-o',str(binary)]
    if sanitizer:command[1:1]=['-fsanitize=address,undefined','-fno-omit-frame-pointer']
    process = subprocess.run(command,capture_output=True,text=True,timeout=120)
    if process.returncode:raise AssertionError(process.stdout+process.stderr)
    return binary
