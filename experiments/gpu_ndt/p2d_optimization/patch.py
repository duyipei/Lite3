#!/usr/bin/env python3
"""Materialize a NEW A/B/C source tree; never patches an installed GPU library."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
BASELINE = json.loads((HERE/'baseline.json').read_text())
CORE = 'src/fast_gicp/cuda/ndt_cuda.cu'
DERIV = 'src/fast_gicp/cuda/ndt_compute_derivatives.cu'
CORE_HEADER = 'include/fast_gicp/cuda/ndt_cuda.cuh'
DERIV_HEADER = 'include/fast_gicp/cuda/ndt_compute_derivatives.cuh'
IGNORED = {'.git','build','install','log','__pycache__'}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('source layout mismatch: '+old[:90])
    return text.replace(old, new, 1)


def snippet(name):
    return (HERE/'snippets'/name).read_text().rstrip()+'\n'


def validate_baseline(files):
    for name, expected in BASELINE['files'].items():
        if name not in files or sha(files[name].encode()) != expected:
            raise ValueError('tested Jetson baseline changed: '+name)
    for name, fragment in [(CORE_HEADER, 'std::unique_ptr<Correspondences> correspondences;'),
                           (DERIV_HEADER, 'double p2d_ndt_compute_derivatives(')]:
        if name not in files or files[name].count(fragment) != 1:
            raise ValueError('unverified/incompatible header: '+name)


def apply(files, variant):
    if variant not in ('A', 'B', 'C'):
        raise ValueError('variant must be A, B or C')
    validate_baseline(files)
    result = dict(files)
    if variant == 'A':
        return result
    result[DERIV] = replace_once(result[DERIV], '#include <thrust/transform_reduce.h>',
                                 '#include <thrust/transform_reduce.h>\n#include <thrust/functional.h>')
    result[DERIV] = replace_once(result[DERIV], 'struct d2d_ndt_compute_derivatives_kernel {',
                                 snippet('error_functor.inc')+'\nstruct d2d_ndt_compute_derivatives_kernel {')
    result[DERIV] = replace_once(result[DERIV], 'double d2d_ndt_compute_derivatives(',
                                 snippet('error_function.inc')+'\ndouble d2d_ndt_compute_derivatives(')
    result[DERIV_HEADER] = replace_once(result[DERIV_HEADER], 'double p2d_ndt_compute_derivatives(',
                                        snippet('error_declaration.inc')+'\ndouble p2d_ndt_compute_derivatives(')
    call = '      return p2d_ndt_compute_derivatives(*target_voxelmap, *source_points, *correspondences, trans_ptr.data(), trans_ptr.data() + 1, H, b);'
    dispatch = '''      if (H == nullptr && b == nullptr) {
        return p2d_ndt_compute_error_only(*target_voxelmap, *source_points, *correspondences, trans_ptr.data() + 1);
      }
'''
    result[CORE] = replace_once(result[CORE], call, dispatch+call)
    if variant == 'C':
        result[CORE_HEADER] = replace_once(result[CORE_HEADER], 'class GaussianVoxelMap;',
                                          'class GaussianVoxelMap;\nstruct P2DWorkspace;')
        result[CORE_HEADER] = replace_once(result[CORE_HEADER],
                                          '  std::unique_ptr<Correspondences> correspondences;',
                                          '  std::unique_ptr<Correspondences> correspondences;\n\n'
                                          'private:\n  mutable std::unique_ptr<P2DWorkspace> p2d_workspace_;')
        result[CORE] = replace_once(result[CORE], '#include <thrust/device_vector.h>',
                                   '#include <thrust/device_vector.h>\n#include <thrust/copy.h>')
        result[CORE] = replace_once(result[CORE], 'NDTCudaCore::NDTCudaCore() {',
                                   snippet('workspace.inc')+'\nNDTCudaCore::NDTCudaCore() {')
        sig = 'void NDTCudaCore::update_correspondences(const Eigen::Isometry3d& trans) {'
        result[CORE] = replace_once(result[CORE], sig, sig+'\n'+snippet('workspace_search.inc'))
        sig = ('double NDTCudaCore::compute_error(const Eigen::Isometry3d& trans, '
               'Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) const {')
        result[CORE] = replace_once(result[CORE], sig, sig+'\n'+snippet('workspace_error.inc'))
    return result


def load_sources(base):
    names = list(BASELINE['files'])+[CORE_HEADER, DERIV_HEADER]
    return {name: (Path(base)/name).read_text() for name in names}


def inventory(base):
    result={}
    for p in sorted(Path(base).rglob('*')):
        name=p.relative_to(base)
        if any(part in IGNORED for part in name.parts):continue
        if p.is_symlink():raise ValueError('source symlinks are not accepted in the private rebuild: '+str(name))
        if p.is_file() and name.as_posix()!='lite3_p2d_variant.json':result[name.as_posix()]=sha(p.read_bytes())
    return result


def prepare(base, output, variant):
    base, output = Path(base).resolve(), Path(output).resolve()
    if output.exists() or base == output or base in output.parents or output in base.parents:
        raise ValueError('output must be a NEW independent source directory')
    sources = load_sources(base)
    changed = apply(sources, variant)
    before=inventory(base)
    shutil.copytree(base, output, ignore=shutil.ignore_patterns('.git', 'build', 'install', 'log', '__pycache__'))
    if inventory(output)!=before or inventory(base)!=before:
        raise ValueError('source changed during private copy; do not build this incomplete tree')
    for name, content in changed.items():
        if content != sources[name]:
            (output/name).write_text(content)
    manifest = dict(schema_version=1, variant=variant, baseline_source=str(base),
                    scope='independent_GPU_P2D_only', cpu_lm_unchanged=True,
                    d2d_algorithm_unchanged=True, native_validated=False,
                    baseline_hashes={n:sha(t.encode()) for n,t in sources.items()},
                    generated_hashes={n:sha(t.encode()) for n,t in changed.items()},
                    baseline_inventory=before,generated_inventory=inventory(output),
                    modified_files=[n for n in changed if changed[n] != sources[n]],
                    abi_requires_complete_private_rebuild=variant == 'C')
    (output/'lite3_p2d_variant.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-fast-gicp', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--variant', choices=('A', 'B', 'C'), required=True)
    args = parser.parse_args()
    try:
        print(json.dumps(prepare(args.base_fast_gicp, args.output, args.variant), indent=2))
    except (ValueError, OSError, KeyError) as error:
        parser.exit(2, str(error)+'\n')


if __name__ == '__main__':
    main()
