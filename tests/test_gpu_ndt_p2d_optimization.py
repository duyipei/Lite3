"""Offline P2D checks. Host C++/Thrust doubles are NOT native CUDA tests."""
import json
import shutil
import subprocess
from pathlib import Path

import pytest

from p2d_host_support import compile_host, sources
from experiments.gpu_ndt.p2d_optimization import patch
from experiments.gpu_ndt.prepare import method_bounds


def method(text, signature):
    a,b = method_bounds(text, signature)
    return text[a:b]


@pytest.fixture(scope='module')
def binaries(tmp_path_factory):
    if not shutil.which('g++') or not Path('/usr/include/eigen3/Eigen/Core').is_file():
        pytest.skip('host C++ compiler/Eigen unavailable; no CUDA result implied')
    root = tmp_path_factory.mktemp('p2d_host')
    return {v:compile_host(root/v,v) for v in 'ABC'}


def execute(binary, *args):
    p = subprocess.run([str(binary),*map(str,args)],capture_output=True,text=True,check=True,timeout=20)
    return json.loads(p.stdout)


def test_a_is_exact_baseline():
    before = patch.load_sources(patch.HERE/'baseline')
    assert sources('A') == before


@pytest.mark.parametrize('variant', ['B','C'])
def test_cpu_lm_and_correspondence_search_are_unchanged(variant):
    a,b = sources('A'),sources(variant)
    for name in patch.BASELINE['files']:
        if name not in (patch.CORE,patch.DERIV):assert a[name] == b[name]


@pytest.mark.parametrize('variant', ['B','C'])
def test_full_derivative_and_d2d_kernels_unchanged(variant):
    a,b = sources('A'),sources(variant)
    for signature in ['double p2d_ndt_compute_derivatives(', 'double d2d_ndt_compute_derivatives(']:
        assert method(a[patch.DERIV],signature) == method(b[patch.DERIV],signature)
    assert a[patch.DERIV].split('struct p2d_ndt_compute_derivatives_kernel {',1)[1].split('struct d2d_',1)[0].strip() in b[patch.DERIV]
    assert a[patch.DERIV].split('struct d2d_ndt_compute_derivatives_kernel {',1)[1].split('double p2d_',1)[0].strip() in b[patch.DERIV]


def test_error_only_preserves_scalar_expression_without_derivatives_or_search():
    original = sources('A')[patch.DERIV]
    scalar = patch.snippet('error_functor.inc')
    for expression in ['if (correspondence.second < 0)', 'if (num_points <= 6)',
                       'R * mean_A + t', 'cov_B.inverse()', 'mean_B - transed_mean_A',
                       'cauchy(thrust::raw_pointer_cast(voxelmap_info_ptr)->voxel_resolution, error.norm())',
                       'w * error.transpose() * RCR_inv * error']:
        assert expression in original and expression in scalar
    for forbidden in ['Matrix<float, 6', 'find_voxel_correspondences(', 'linearized_x', 'Hessian']:
        assert forbidden not in scalar
    assert '0.0f, thrust::plus<float>()' in patch.snippet('error_function.inc')


@pytest.mark.parametrize('variant', ['B','C'])
def test_error_only_requires_both_derivatives_null(variant):
    code = sources(variant)[patch.CORE]
    assert 'if (H == nullptr && b == nullptr)' in code
    assert 'if (H == nullptr || b == nullptr)' not in code


def test_workspace_is_per_instance_p2d_only_and_requires_abi_rebuild():
    code = sources('C')
    assert 'mutable std::unique_ptr<P2DWorkspace> p2d_workspace_' in code[patch.CORE_HEADER]
    assert 'static P2DWorkspace' not in code[patch.CORE]
    assert 'distance_mode == fast_gicp::NDTDistanceMode::P2D' in patch.snippet('workspace_search.inc')
    assert 'distance_mode == fast_gicp::NDTDistanceMode::P2D' in patch.snippet('workspace_error.inc')
    assert 'reserve(source_points->size() * offsets->size())' in patch.snippet('workspace_search.inc')


def test_correspondence_reserve_does_not_override_existing_growth_policy_every_iteration():
    search=patch.snippet('workspace_search.inc')
    reservation=search.index('correspondences->reserve(')
    creation=search.index('if (!correspondences) {')
    close=search.index('\n    }',creation)
    assert creation < reservation < close


@pytest.mark.parametrize('variant', ['B','C'])
def test_existing_synchronization_is_retained(variant):
    code = sources(variant)
    assert code[patch.CORE].count('cudaDeviceSynchronize()') == sources('A')[patch.CORE].count('cudaDeviceSynchronize()')
    search='src/fast_gicp/cuda/find_voxel_correspondences.cu'
    assert code[search] == sources('A')[search]
    assert '.wait()' in code[search]


@pytest.mark.parametrize('variant', list('ABC'))
def test_real_scalar_and_derivative_code_agree_under_host_double(binaries,variant):
    result = execute(binaries[variant],'compare')
    assert result['max_abs_error'] == 0
    assert result['cuda_executed'] is False


@pytest.mark.parametrize('seed', [0,1,-2,4,8])
def test_original_lm_decisions_iterations_and_pose_unchanged_under_host_double(binaries,seed):
    result = [execute(binaries[v],'lm',seed) for v in 'ABC']
    assert result[0] == result[1] == result[2]
    assert result[0]['full'] > 0 and result[0]['errors'] > 0


@pytest.mark.parametrize('seed',[0,1,-2])
def test_original_lm_rejected_trials_and_converged_negative_rho_branch(binaries,seed):
    results=[execute(binaries[v],'lm',seed,'hard') for v in 'ABC']
    assert results[0]==results[1]==results[2]
    assert -1 in results[0]['decisions']
    if seed==-2:assert 2 in results[0]['decisions']


@pytest.mark.parametrize('variant', list('ABC'))
def test_repeated_sizes_two_instances_and_d2d_switch(binaries,variant):
    result = execute(binaries[variant],'lifecycle')
    assert result['lifecycle_pass'] is True
    assert result['cuda_executed'] is False


def test_workspace_eliminates_only_explicit_pose_allocations_in_host_double(binaries):
    result = {v:execute(binaries[v],'compare') for v in 'ABC'}
    # 25 iterations x (one search-pose + two evaluation-pose temporaries).
    # These are NOT cudaMalloc/Nsight measurements or a GPU speedup estimate.
    assert result['A']['emulated_allocations'] == result['B']['emulated_allocations'] == 75
    assert result['C']['emulated_allocations'] == result['C']['emulated_frees'] == 0


def test_changed_baseline_refused():
    files = patch.load_sources(patch.HERE/'baseline')
    files[patch.CORE] += '\n// unexpected installed source\n'
    with pytest.raises(ValueError,match='baseline changed'):patch.apply(files,'B')


def test_incompatible_header_refused():
    files = patch.load_sources(patch.HERE/'baseline')
    files[patch.CORE_HEADER] = ''
    with pytest.raises(ValueError,match='incompatible header'):patch.apply(files,'C')


@pytest.mark.parametrize('variant', list('ABC'))
def test_prepare_new_tree_keeps_input_and_manifest(tmp_path,variant):
    base = patch.HERE/'baseline'
    before = patch.load_sources(base)
    out = tmp_path/variant
    manifest = patch.prepare(base,out,variant)
    assert patch.load_sources(base) == before
    assert manifest['native_validated'] is False
    assert manifest['cpu_lm_unchanged'] and manifest['d2d_algorithm_unchanged']
    assert manifest['abi_requires_complete_private_rebuild'] == (variant=='C')
    assert set(manifest['modified_files']) <= {patch.CORE,patch.CORE_HEADER,patch.DERIV,patch.DERIV_HEADER}
    assert patch.load_sources(out) == sources(variant)


def test_existing_output_and_overlapping_tree_refused(tmp_path):
    base=patch.HERE/'baseline'
    for target in [tmp_path,base,base/'would_overwrite']:
        with pytest.raises(ValueError,match='NEW independent'):patch.prepare(base,target,'C')
